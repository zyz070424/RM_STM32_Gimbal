#include "Gimbal.h"
#include "gimbal_sentry_target.h"
#include "gimbal_sentry_control.h"
#include "dvc_motor_protect.h"
#include "alg_pid.h"
#include <stdbool.h>
#include <stdint.h>

#define GIMBAL_CTRL_PERIOD_TICK       1
#define GIMBAL_CTRL_DT                0.001f
#define GIMBAL_MAHONY_KP              0.5f
#define GIMBAL_MAHONY_KI              0.001f

// IMU 实际 dt 估计参数（优先 DWT，失败回退 HAL tick）
#define GIMBAL_IMU_DT_DEFAULT_S       GIMBAL_CTRL_DT
#define GIMBAL_IMU_DT_MIN_S           0.0002f
#define GIMBAL_IMU_DT_MAX_S           0.0100f

// IMU 数据就绪中断开关：0=纯任务轮询，1=中断唤醒任务（中断里不读SPI）
#define GIMBAL_IMU_DRDY_ENABLE        0
// 中断模式下任务等待超时（tick），超时后回退一次轮询读取
#define GIMBAL_IMU_WAIT_TIMEOUT_TICK  2

// 机械限位（单位：度），用于保护线束
#define GIMBAL_PITCH_MIN_ANGLE        (-42.0f)
#define GIMBAL_PITCH_MAX_ANGLE        (42.0f)
#define GIMBAL_YAW_MIN_ANGLE          (-120.0f)
#define GIMBAL_YAW_MAX_ANGLE          (120.0f)

// 电机输出限幅（GM6020电压模式常用范围）
#define GIMBAL_MOTOR_CMD_LIMIT        10000.0f

// ============ 哨兵扫描参数：绕 0 度中心摆动 ============
#define GIMBAL_SCAN_YAW_AMPLITUDE_DEG   50.0f
#define GIMBAL_SCAN_PITCH_AMPLITUDE_DEG 37.0f
#define GIMBAL_SCAN_YAW_FREQ_HZ         0.8f
#define GIMBAL_SCAN_PITCH_FREQ_HZ       0.3f

// ============ 视觉跟踪参数 ============
#define GIMBAL_VISION_TRACK_TIMEOUT_MS    120u
#define GIMBAL_VISION_TARGET_FILTER_TAU_S 0.010f

// ============ 丢失目标回扫处理 ============
#define GIMBAL_LOST_RETURN_SPEED_DEG_S 110.0f
#define GIMBAL_LOST_RETURN_NEAR_DEG    1.0f

// ============ 哨兵模式固定整形参数（低通+固定斜率） ============
#define GIMBAL_SENTRY_SHAPER_FILTER_TAU_S    0.003f
#define GIMBAL_PITCH_SENTRY_SHAPER_SLEW_RATE 60.0f
#define GIMBAL_YAW_SENTRY_SHAPER_SLEW_RATE   80.0f

// Yaw外环参数（角度环 -> 速度目标）
#define GIMBAL_YAW_ANGLE_KP           1.0f
#define GIMBAL_YAW_ANGLE_KI           0.0f
#define GIMBAL_YAW_ANGLE_KD           0.00f
#define GIMBAL_YAW_ANGLE_FEEDFORWARD  0.025f
#define GIMBAL_YAW_ANGLE_OUT_LIMIT    10.0f
#define GIMBAL_YAW_ANGLE_I_LIMIT      1.5f
#define GIMBAL_YAW_ANGLE_DEADBAND_DEG 0.0f

// Yaw内环参数（速度环 -> 电机控制量）
#define GIMBAL_YAW_SPEED_KP      1400.0f
#define GIMBAL_YAW_SPEED_KI      600.0f
#define GIMBAL_YAW_SPEED_KD      0.00f
#define GIMBAL_YAW_SPEED_I_LIMIT 2000.0f

// Pitch外环参数（角度环 -> 速度目标）
#define GIMBAL_PITCH_ANGLE_KP           1.0f
#define GIMBAL_PITCH_ANGLE_KI           0.0f
#define GIMBAL_PITCH_ANGLE_KD           0.00f
#define GIMBAL_PITCH_ANGLE_FEEDFORWARD  0.025f
#define GIMBAL_PITCH_ANGLE_OUT_LIMIT    10.0f
#define GIMBAL_PITCH_ANGLE_I_LIMIT      1.5f
#define GIMBAL_PITCH_ANGLE_DEADBAND_DEG 0.4f

// Pitch内环参数（速度环 -> 电机控制量）
#define GIMBAL_PITCH_SPEED_KP      1000.0f
#define GIMBAL_PITCH_SPEED_KI      300.0f
#define GIMBAL_PITCH_SPEED_KD      0.00f
#define GIMBAL_PITCH_SPEED_I_LIMIT 2000.0f

// 哨兵目标生成配置仍放在 Gimbal.cpp，便于结合本云台机械范围统一调参
static const Gimbal_Sentry_Target_Config_TypeDef g_gimbal_sentry_target_config =
{
    .control_dt_s = GIMBAL_CTRL_DT,
    .vision_track_timeout_ms = GIMBAL_VISION_TRACK_TIMEOUT_MS,
    .vision_target_filter_tau_s = GIMBAL_VISION_TARGET_FILTER_TAU_S,
    .sentry_config =
    {
        .dt_s = GIMBAL_CTRL_DT,
        .pitch_min_deg = GIMBAL_PITCH_MIN_ANGLE,
        .pitch_max_deg = GIMBAL_PITCH_MAX_ANGLE,
        .yaw_min_deg = GIMBAL_YAW_MIN_ANGLE,
        .yaw_max_deg = GIMBAL_YAW_MAX_ANGLE,
        .scan_pitch_amplitude_deg = GIMBAL_SCAN_PITCH_AMPLITUDE_DEG,
        .scan_yaw_amplitude_deg = GIMBAL_SCAN_YAW_AMPLITUDE_DEG,
        .scan_pitch_frequency_hz = GIMBAL_SCAN_PITCH_FREQ_HZ,
        .scan_yaw_frequency_hz = GIMBAL_SCAN_YAW_FREQ_HZ,
        .lost_return_speed_deg_s = GIMBAL_LOST_RETURN_SPEED_DEG_S,
        .lost_return_near_deg = GIMBAL_LOST_RETURN_NEAR_DEG
    }
};

static const Gimbal_Sentry_Control_Config_TypeDef g_gimbal_sentry_control_config =
{
    .pitch_angle_feedforward = GIMBAL_PITCH_ANGLE_FEEDFORWARD,
    .yaw_angle_feedforward = GIMBAL_YAW_ANGLE_FEEDFORWARD,
    .sentry_shaper_filter_tau_s = GIMBAL_SENTRY_SHAPER_FILTER_TAU_S,
    .pitch_sentry_shaper_slew_rate = GIMBAL_PITCH_SENTRY_SHAPER_SLEW_RATE,
    .yaw_sentry_shaper_slew_rate = GIMBAL_YAW_SENTRY_SHAPER_SLEW_RATE,
    .vision_shaper_filter_tau_s = GIMBAL_VISION_SHAPER_FILTER_TAU_S,
    .pitch_vision_shaper_slew_rate = GIMBAL_PITCH_VISION_SHAPER_SLEW_RATE,
    .yaw_vision_shaper_slew_rate = GIMBAL_YAW_VISION_SHAPER_SLEW_RATE,
    .target_reset_pitch_deg = GIMBAL_TARGET_RESET_PITCH_DEG,
    .target_reset_yaw_deg = GIMBAL_TARGET_RESET_YAW_DEG
};

Class_Gimbal Gimbal_Object = {};

/**
 * @brief 对输入角度做上下限钳位。
 * @param value 输入角度值，单位 deg。
 * @param min_value 最小允许角度，单位 deg。
 * @param max_value 最大允许角度，单位 deg。
 * @return 限幅后的角度值。
 */
float Class_Gimbal::Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief 选择角度环重置后的输出整形初值。
 * @param pid 需要读取旧输出状态的角度环 PID 对象。
 * @param target_deg 当前目标角度，单位 deg。
 * @param feedback_deg 当前反馈角度，单位 deg。
 * @param error_deadband_deg 误差死区阈值，单位 deg。
 * @param output_limit 角度环输出限幅。
 * @return 保留旧输出方向时返回旧输出，否则返回 0。
 * @note 旧输出方向仍朝向新目标时保留，否则清零，避免换向时继承错误速度。
 */
float Class_Gimbal::SelectAngleResetOutput(const Class_PID *pid,
                                           float target_deg,
                                           float feedback_deg,
                                           float error_deadband_deg,
                                           float output_limit)
{
    float old_output;
    float new_error;
    float limit_abs;

    if (pid == NULL)
    {
        return 0.0f;
    }

    limit_abs = fabsf(output_limit);
    old_output = Clamp(pid->output, -limit_abs, limit_abs);
    new_error = target_deg - feedback_deg;

    if ((fabsf(new_error) <= fabsf(error_deadband_deg)) ||
        ((old_output * new_error) <= 0.0f))
    {
        return 0.0f;
    }

    return old_output;
}

/**
 * @brief 复位当前控制目标。
 * @return 无。
 */
void Class_Gimbal::ResetControlTargets()
{
    Gimbal_Sentry_Target_Object.ClearOutput();
}

/**
 * @brief 清空 IMU 对外输出。
 * @return 无。
 */
void Class_Gimbal::ResetImuOutput()
{
    BMI088_Manage_Object.YawContinuousReset();
    Euler_Angle_To_Send.roll = 0.0f;
    Euler_Angle_To_Send.pitch = 0.0f;
    Euler_Angle_To_Send.yaw = 0.0f;
}

/**
 * @brief 处理 CAN 在线状态变化。
 * @param online 0 表示离线，1 表示在线。
 * @return 无。
 */
void Class_Gimbal::HandleCanAliveChange(uint8_t online)
{
    if (online != 0u)
    {
        return;
    }

    Motor_Pitch.ClearRuntime();
    Motor_Yaw.ClearRuntime();
    ResetControlTargets();
}

/**
 * @brief 处理 SPI 在线状态变化。
 * @param online 0 表示离线，1 表示在线。
 * @return 无。
 */
void Class_Gimbal::HandleSpiAliveChange(uint8_t online)
{
    if (online == 0u)
    {
        ResetImuOutput();
        return;
    }

    BMI088_Manage_Object.YawContinuousReset();
}

/**
 * @brief 处理 USB 在线状态变化。
 * @param online 0 表示离线，1 表示在线。
 * @return 无。
 * @note USB 离线时同时清空协议层目标和上层有效目标缓存，避免恢复在线后沿用旧目标。
 */
void Class_Gimbal::HandleUsbAliveChange(uint8_t online)
{
    if (online != 0u)
    {
        return;
    }

    Manifold_Manage_Object.ClearTarget();
    Gimbal_Sentry_Target_Object.ResetVision();
}

/**
 * @brief 将浮点输出转换为 16 位 CAN 控制量。
 * @param value 输入浮点控制量。
 * @return 限幅后的 16 位有符号整数。
 */
int16_t Class_Gimbal::FloatToInt16Sat(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)value;
}

/**
 * @brief 把任意输出映射为离线时的零命令。
 * @param value 任意输入值。
 * @return 固定返回 0。
 */
int16_t Class_Gimbal::OutputToCanZero(float value)
{
    (void)value;
    return 0;
}

/**
 * @brief 把浮点输出映射为在线时的 CAN 控制量。
 * @param value PID 浮点输出。
 * @return 转换后的 16 位控制量。
 */
int16_t Class_Gimbal::OutputToCanNormal(float value)
{
    return FloatToInt16Sat(value);
}

/**
 * @brief 合帧发送 Pitch 和 Yaw 电机控制量。
 * @param pitch_cmd Pitch 电机控制量。
 * @param yaw_cmd Yaw 电机控制量。
 * @return 无。
 * @details 当 Pitch 和 Yaw 位于同一路 CAN 且共用发送 ID 时，优先走合帧缓存发送，
 *          否则回退为逐电机发送。
 */
void Class_Gimbal::SendPitchYawCan(int16_t pitch_cmd, int16_t yaw_cmd)
{
    uint32_t pitch_send_id;
    uint32_t yaw_send_id;

    pitch_send_id = Motor_Pitch.GetCanSendId();
    yaw_send_id = Motor_Yaw.GetCanSendId();

    if ((Motor_Pitch.can == Motor_Yaw.can) &&
        (pitch_send_id != 0u) &&
        (pitch_send_id == yaw_send_id))
    {
        Motor_Pitch.UpdateCanCache(pitch_cmd);
        Motor_Yaw.UpdateCanCache(yaw_cmd);
        Class_Motor::SendCanFrameById(Motor_Pitch.can, pitch_send_id);
        return;
    }

    Motor_Pitch.SendCanData(pitch_cmd);
    Motor_Yaw.SendCanData(yaw_cmd);
}

/**
 * @brief 初始化云台模块。
 * @param params 任务入口透传参数，当前未使用。
 * @return 无。
 */
void Class_Gimbal::Init(void *params)
{
    (void)params;

    Imu_Task_Handle = NULL;
    Imu_Last_Dt_S = GIMBAL_IMU_DT_DEFAULT_S;
    Imu_Last_Dt_From_Dwt = 0u;
    Yaw_Test_Target_Deg = 0.0f;
    Pitch_Test_Target_Deg = 0.0f;
    Imu_Data = {};
    Euler_Angle_To_Send = {};
    Tx_Data = {};

    Manifold_Manage_Object.Init(&Tx_Data, 0xFE, 0xFF, Manifold_Sentry_Mode_ENABLE);

    CAN2_Manage_Object.Init(&hcan2);
    CAN2_Manage_Object.Start();

    if (BMI088_Manage_Object.Init(&hspi1) != HAL_OK)
    {
        return;
    }

    Motor_Pitch.Init(4, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    Motor_Pitch.SetPidParams(1,
                             GIMBAL_PITCH_ANGLE_KP,
                             GIMBAL_PITCH_ANGLE_KI,
                             GIMBAL_PITCH_ANGLE_KD,
                             GIMBAL_PITCH_ANGLE_FEEDFORWARD,
                             -GIMBAL_PITCH_ANGLE_OUT_LIMIT,
                             GIMBAL_PITCH_ANGLE_OUT_LIMIT,
                             -GIMBAL_PITCH_ANGLE_I_LIMIT,
                             GIMBAL_PITCH_ANGLE_I_LIMIT);
    Motor_Pitch.SetPidParams(0,
                             GIMBAL_PITCH_SPEED_KP,
                             GIMBAL_PITCH_SPEED_KI,
                             GIMBAL_PITCH_SPEED_KD,
                             0.00f,
                             -GIMBAL_MOTOR_CMD_LIMIT,
                             GIMBAL_MOTOR_CMD_LIMIT,
                             -GIMBAL_PITCH_SPEED_I_LIMIT,
                             GIMBAL_PITCH_SPEED_I_LIMIT);

    Motor_Yaw.Init(2, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    Motor_Yaw.SetPidParams(1,
                           GIMBAL_YAW_ANGLE_KP,
                           GIMBAL_YAW_ANGLE_KI,
                           GIMBAL_YAW_ANGLE_KD,
                           GIMBAL_YAW_ANGLE_FEEDFORWARD,
                           -GIMBAL_YAW_ANGLE_OUT_LIMIT,
                           GIMBAL_YAW_ANGLE_OUT_LIMIT,
                           -GIMBAL_YAW_ANGLE_I_LIMIT,
                           GIMBAL_YAW_ANGLE_I_LIMIT);
    Motor_Yaw.SetPidParams(0,
                           GIMBAL_YAW_SPEED_KP,
                           GIMBAL_YAW_SPEED_KI,
                           GIMBAL_YAW_SPEED_KD,
                           0.00f,
                           -GIMBAL_MOTOR_CMD_LIMIT,
                           GIMBAL_MOTOR_CMD_LIMIT,
                           -GIMBAL_YAW_SPEED_I_LIMIT,
                           GIMBAL_YAW_SPEED_I_LIMIT);

    Motor_Yaw.PID[0].FrictionCompensationEnable(true, 1200.0f, 0.0f, 0.3f, false);
    Motor_Pitch.PID[0].FrictionCompensationEnable(true, 1600.0f, 0.0f, 0.3f, false);

    Motor_Protect_Pitch_Object.Init(GIMBAL_PITCH_MIN_ANGLE,
                                    GIMBAL_PITCH_MAX_ANGLE,
                                    1u);

    Gimbal_Sentry_Control_Object.Init(&g_gimbal_sentry_control_config);
    Gimbal_Sentry_Control_Object.ApplyModeParams(&Motor_Pitch,
                                                 &Motor_Yaw,
                                                 GIMBAL_SENTRY_STATE_SCAN);

    Gimbal_Sentry_Target_Object.Init(&g_gimbal_sentry_target_config);
}

/**
 * @brief 执行云台双轴控制任务。
 * @param params 任务入口透传参数，当前未使用。
 * @return 无。
 * @note 当前使用 SCAN / TRACK_ARMOR / LOST_TARGET_RETURN_SCAN 三态目标生成。
 */
void Class_Gimbal::MotorControlTask(void *params)
{
    TickType_t time;
    TickType_t now_tick;
    uint8_t can_online;
    uint8_t yaw_feedback_ready;
    float pitch_output;
    float pitch_feedback_deg;
    float pitch_target_deg;
    float pitch_reset_output;
    float pitch_protect_reset_target_deg;
    float pitch_target_speed;
    float yaw_output;
    float yaw_target_deg;
    float yaw_reset_output;
    float yaw_target_speed;
    Gimbal_Sentry_State_TypeDef sentry_state;
    int16_t pitch_can_cmd;
    int16_t yaw_can_cmd;

    (void)params;

    time = xTaskGetTickCount();

    Gimbal_Sentry_Target_Object.ResetMode();

    pitch_output = 0.0f;
    yaw_output = 0.0f;
    while (1)
    {
        Motor_Pitch.CanDataReceive();
        Motor_Yaw.CanDataReceive();

        yaw_feedback_ready = (Motor_Yaw.RxData.Encoder_Initialized != 0u);
        if (yaw_feedback_ready == 0u)
        {
            can_online = CAN2_Manage_Object.AliveIsOnline();
            pitch_can_cmd = (can_online != 0u) ? OutputToCanNormal(0.0f) : OutputToCanZero(0.0f);
            yaw_can_cmd = (can_online != 0u) ? OutputToCanNormal(0.0f) : OutputToCanZero(0.0f);
            SendPitchYawCan(pitch_can_cmd, yaw_can_cmd);
            vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
            continue;
        }

        now_tick = xTaskGetTickCount();
        Gimbal_Sentry_Target_Object.Update(now_tick);

        pitch_target_deg = Gimbal_Sentry_Target_Object.GetPitch();
        yaw_target_deg = Gimbal_Sentry_Target_Object.GetYaw();
        pitch_feedback_deg = -Euler_Angle_To_Send.pitch;
        sentry_state = Gimbal_Sentry_Target_Object.GetState();

        Motor_Protect_Pitch_Object.SetActive(sentry_state != GIMBAL_SENTRY_STATE_TRACK_ARMOR);
        pitch_target_deg = Motor_Protect_Pitch_Object.ApplyTarget(pitch_target_deg);

        Gimbal_Sentry_Control_Object.ApplyModeParams(&Motor_Pitch,
                                                     &Motor_Yaw,
                                                     sentry_state);
        if (Gimbal_Sentry_Control_Object.AnglePidResetEventCheck(sentry_state,
                                                                 pitch_target_deg,
                                                                 yaw_target_deg) != 0u)
        {
            Motor_Protect_Pitch_Object.Blank();
            pitch_reset_output = SelectAngleResetOutput(&Motor_Pitch.PID[1],
                                                        pitch_target_deg,
                                                        pitch_feedback_deg,
                                                        GIMBAL_PITCH_ANGLE_DEADBAND_DEG,
                                                        GIMBAL_PITCH_ANGLE_OUT_LIMIT);
            yaw_reset_output = SelectAngleResetOutput(&Motor_Yaw.PID[1],
                                                      yaw_target_deg,
                                                      Euler_Angle_To_Send.yaw,
                                                      GIMBAL_YAW_ANGLE_DEADBAND_DEG,
                                                      GIMBAL_YAW_ANGLE_OUT_LIMIT);
            Gimbal_Sentry_Control_Object.ResetAnglePidDynamicState(&Motor_Pitch.PID[1],
                                                                   pitch_target_deg,
                                                                   pitch_reset_output);
            Gimbal_Sentry_Control_Object.ResetAnglePidDynamicState(&Motor_Yaw.PID[1],
                                                                   yaw_target_deg,
                                                                   yaw_reset_output);
        }

        pitch_target_speed = Motor_Pitch.PidCalculateAngle(pitch_target_deg,
                                                           pitch_feedback_deg,
                                                           GIMBAL_CTRL_DT);
        pitch_output = Motor_Pitch.PidCalculateSpeed(pitch_target_speed,
                                                     Motor_Pitch.RxData.Speed,
                                                     GIMBAL_CTRL_DT);

        yaw_target_speed = Motor_Yaw.PidCalculateAngle(yaw_target_deg,
                                                       Euler_Angle_To_Send.yaw,
                                                       GIMBAL_CTRL_DT);
        yaw_output = Motor_Yaw.PidCalculateSpeed(yaw_target_speed,
                                                 Motor_Yaw.RxData.Speed,
                                                 GIMBAL_CTRL_DT);

        pitch_output = Clamp(pitch_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);
        yaw_output = Clamp(yaw_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);

        pitch_output = Motor_Protect_Pitch_Object.UpdateOutput(pitch_target_deg,
                                                               pitch_feedback_deg,
                                                               Motor_Pitch.RxData.Speed,
                                                               pitch_output,
                                                               Motor_Pitch.RxData.Torque,
                                                               GIMBAL_CTRL_PERIOD_TICK);
        if (Motor_Protect_Pitch_Object.TakeResetRequest(&pitch_protect_reset_target_deg) != 0u)
        {
            Gimbal_Sentry_Control_Object.ResetAnglePidDynamicState(&Motor_Pitch.PID[1],
                                                                   pitch_protect_reset_target_deg,
                                                                   0.0f);
        }

        can_online = CAN2_Manage_Object.AliveIsOnline();
        pitch_can_cmd = (can_online != 0u) ? OutputToCanNormal(pitch_output) : OutputToCanZero(pitch_output);
        yaw_can_cmd = (can_online != 0u) ? OutputToCanNormal(yaw_output) : OutputToCanZero(yaw_output);
        SendPitchYawCan(pitch_can_cmd, yaw_can_cmd);
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}

/**
 * @brief 处理 IMU 数据就绪外部中断回调。
 * @param gpio_pin 触发中断的 GPIO 引脚编号。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_Gimbal::ImuExtiCallback(uint16_t gpio_pin)
{
#if GIMBAL_IMU_DRDY_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((gpio_pin == ACCEL_INT_Pin) || (gpio_pin == GYRO_INT_Pin))
    {
        if (Imu_Task_Handle != NULL)
        {
            vTaskNotifyGiveFromISR(Imu_Task_Handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
#else
    (void)gpio_pin;
#endif
}

/**
 * @brief 执行 IMU 欧拉角解算任务。
 * @param params 任务入口透传参数，当前未使用。
 * @return 无。
 */
void Class_Gimbal::EulerTask(void *params)
{
    float imu_dt;
    euler_t euler_raw;

    (void)params;

    Imu_Timebase.Init(GIMBAL_IMU_DT_DEFAULT_S);
    BMI088_Manage_Object.YawContinuousReset();

#if GIMBAL_IMU_DRDY_ENABLE
    Imu_Task_Handle = xTaskGetCurrentTaskHandle();
#else
    TickType_t time;
    time = xTaskGetTickCount();
#endif

    while (1)
    {
#if GIMBAL_IMU_DRDY_ENABLE
        (void)ulTaskNotifyTake(pdTRUE, GIMBAL_IMU_WAIT_TIMEOUT_TICK);
#else
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
#endif

        BMI088_Manage_Object.ReadGyro(&hspi1, &Imu_Data);
        BMI088_Manage_Object.ReadAccel(&hspi1, &Imu_Data);
        BMI088_Manage_Object.ReadTemp(&hspi1, &Imu_Data);
        imu_dt = Imu_Timebase.GetDtS(GIMBAL_IMU_DT_DEFAULT_S,
                                     GIMBAL_IMU_DT_MIN_S,
                                     GIMBAL_IMU_DT_MAX_S);
        Imu_Last_Dt_S = Imu_Timebase.last_dt_s;
        Imu_Last_Dt_From_Dwt = Imu_Timebase.last_dt_from_dwt;
        Imu_Data.dt = imu_dt;
        euler_raw = BMI088_Manage_Object.ComplementaryFilter(&Imu_Data,
                                                             imu_dt,
                                                             GIMBAL_MAHONY_KP,
                                                             GIMBAL_MAHONY_KI);
        Euler_Angle_To_Send = euler_raw;
    }
}

/**
 * @brief 执行发往视觉的姿态发送任务。
 * @param params 任务入口透传参数，当前未使用。
 * @return 无。
 */
void Class_Gimbal::ManifoldControlTask(void *params)
{
    TickType_t time;

    (void)params;

    time = xTaskGetTickCount();

    while (1)
    {
        Manifold_Manage_Object.SendData(&Tx_Data, Euler_Angle_To_Send);
        vTaskDelayUntil(&time, 10);
    }
}

/**
 * @brief 执行云台设备在线检测任务。
 * @param params 任务入口透传参数，当前未使用。
 * @return 无。
 */
void Class_Gimbal::TaskLoop(void *params)
{
    TickType_t time;
    uint16_t alive_check_div = 0u;
    uint8_t can_online_changed;
    uint8_t spi_online_changed;
    uint8_t usb_online_changed;

    (void)params;

    time = xTaskGetTickCount();

    while (1)
    {
        if (++alive_check_div >= 100u)
        {
            alive_check_div = 0u;
            CAN2_Manage_Object.AliveCheck100ms();
            SPI1_Manage_Object.AliveCheck100ms();
            USB_Manage_Object.AliveCheck100ms();
        }

        if (CAN2_Manage_Object.AliveTryConsumeChanged(&can_online_changed) != 0u)
        {
            HandleCanAliveChange(can_online_changed);
        }

        if (SPI1_Manage_Object.AliveTryConsumeChanged(&spi_online_changed) != 0u)
        {
            HandleSpiAliveChange(spi_online_changed);
        }

        if (USB_Manage_Object.AliveTryConsumeChanged(&usb_online_changed) != 0u)
        {
            HandleUsbAliveChange(usb_online_changed);
        }

        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}

extern "C" {
/**
 * @brief IMU 数据就绪外部中断桥接入口。
 * @param GPIO_Pin 触发中断的 GPIO 引脚编号。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin)
{
    Gimbal_Object.ImuExtiCallback(GPIO_Pin);
}
}
