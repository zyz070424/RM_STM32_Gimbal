#include "Gimbal.h"
#include "gimbal_sentry_target.h"
#include "gimbal_sentry_control.h"
#include "alg_pid.h"
#include "yaw_fusion_cfg.h"
#include <stdbool.h>
#include <stdint.h>
#define PI   3.1415926f
#define GIMBAL_CTRL_PERIOD_TICK     1
#define GIMBAL_CTRL_DT              0.001f
#define GIMBAL_MAHONY_KP            0.5f
#define GIMBAL_MAHONY_KI            0.001f

// IMU 实际 dt 估计参数（优先 DWT，失败回退 HAL tick）
#define GIMBAL_IMU_DT_DEFAULT_S     GIMBAL_CTRL_DT
#define GIMBAL_IMU_DT_MIN_S         0.0002f
#define GIMBAL_IMU_DT_MAX_S         0.0100f

// IMU 数据就绪中断开关：0=纯任务轮询，1=中断唤醒任务（中断里不读SPI）
#define GIMBAL_IMU_DRDY_ENABLE      0
// 中断模式下任务等待超时（tick），超时后回退一次轮询读取
#define GIMBAL_IMU_WAIT_TIMEOUT_TICK 2

// 机械限位（单位：度），用于保护线束
#define GIMBAL_PITCH_MIN_ANGLE     (-42.0f)
#define GIMBAL_PITCH_MAX_ANGLE     (42.0f)
#define GIMBAL_YAW_MIN_ANGLE       (-120.0f)
#define GIMBAL_YAW_MAX_ANGLE       (120.0f)

// 电机输出限幅（GM6020电压模式常用范围）
#define GIMBAL_MOTOR_CMD_LIMIT      10000.0f

// ============ 哨兵扫描参数：绕 0 度中心摆动 ============
#define GIMBAL_SCAN_YAW_AMPLITUDE_DEG        50.0f
#define GIMBAL_SCAN_PITCH_AMPLITUDE_DEG      37.0f
#define GIMBAL_SCAN_YAW_FREQ_HZ              0.8f
#define GIMBAL_SCAN_PITCH_FREQ_HZ            0.3f

// ============ 视觉跟踪参数 ============
#define GIMBAL_VISION_TRACK_TIMEOUT_MS       120u
#define GIMBAL_VISION_TARGET_FILTER_TAU_S    0.010f

// ============ 丢失目标回扫处理 ============
#define GIMBAL_LOST_RETURN_SPEED_DEG_S       110.0f
#define GIMBAL_LOST_RETURN_NEAR_DEG          1.0f

// ============ 哨兵模式固定整形参数（低通+固定斜率） ============
#define GIMBAL_SENTRY_SHAPER_FILTER_TAU_S      0.003f
#define GIMBAL_PITCH_SENTRY_SHAPER_SLEW_RATE   60.0f
#define GIMBAL_YAW_SENTRY_SHAPER_SLEW_RATE     80.0f

// Yaw外环参数（角度环 -> 速度目标）
#define GIMBAL_YAW_ANGLE_KP                  1.0f
#define GIMBAL_YAW_ANGLE_KI                  0.0f
#define GIMBAL_YAW_ANGLE_KD                  0.00f
#define GIMBAL_YAW_ANGLE_FEEDFORWARD         0.025f
#define GIMBAL_YAW_ANGLE_OUT_LIMIT           10.0f
#define GIMBAL_YAW_ANGLE_I_LIMIT             1.5f
#define GIMBAL_YAW_ANGLE_DEADBAND_DEG        0.0f

// Yaw内环参数（速度环 -> 电机控制量）
#define GIMBAL_YAW_SPEED_KP                  1400.0f
#define GIMBAL_YAW_SPEED_KI                  600.0f
#define GIMBAL_YAW_SPEED_KD                  0.00f
#define GIMBAL_YAW_SPEED_I_LIMIT             2000.0f

// #define GIMBAL_YAW_SPEED_KP                  0.0f
// #define GIMBAL_YAW_SPEED_KI                  0.0f
// #define GIMBAL_YAW_SPEED_KD                  0.00f
// #define GIMBAL_YAW_SPEED_I_LIMIT             2000.0f


//sin测试函数
#define GIMBAL_TEST_TARGET          30.0f
#define GIMBAL_TEST_FREQ_HZ         1.25f

//Pitch外环参数（角度环 -> 速度目标）
#define GIMBAL_PITCH_ANGLE_KP                  1.0f
#define GIMBAL_PITCH_ANGLE_KI                  0.0f
#define GIMBAL_PITCH_ANGLE_KD                  0.00f
#define GIMBAL_PITCH_ANGLE_FEEDFORWARD         0.025f
#define GIMBAL_PITCH_ANGLE_OUT_LIMIT           10.0f
#define GIMBAL_PITCH_ANGLE_I_LIMIT             1.5f
#define GIMBAL_PITCH_ANGLE_DEADBAND_DEG        0.4f




// Pitch内环参数（速度环 -> 电机控制量）
#define GIMBAL_PITCH_SPEED_KP                  1000.0f
#define GIMBAL_PITCH_SPEED_KI                  300.0f
#define GIMBAL_PITCH_SPEED_KD                  0.00f
#define GIMBAL_PITCH_SPEED_I_LIMIT             2000.0f
//这个是调试使用的
// #define GIMBAL_PITCH_SPEED_KP                  0.0f
// #define GIMBAL_PITCH_SPEED_KI                  0.0f
// #define GIMBAL_PITCH_SPEED_KD                  0.00f
//  #define GIMBAL_PITCH_SPEED_I_LIMIT             2000.0f
Motor_TypeDef Gimbal_Motor_Pitch;
Motor_TypeDef Gimbal_Motor_Yaw;
imu_data_t Gimbal_IMU_Data;
euler_t Gimbal_Euler_Angle_to_send;
euler_t Gimbal_Euler_Angle_Final_Observe;
Manifold_UART_Tx_Data Tx_Data;
Gimbal_Yaw_Fusion_Observe_TypeDef Gimbal_Yaw_Fusion_Observe;

#if GIMBAL_IMU_DRDY_ENABLE
static TaskHandle_t g_gimbal_imu_task_handle = NULL;
#endif
static alg_dwt_timebase_t g_gimbal_imu_timebase;
volatile float g_gimbal_imu_last_dt_s = GIMBAL_IMU_DT_DEFAULT_S;
volatile uint8_t g_gimbal_imu_last_dt_from_dwt = 0;
static YawFusion_t g_gimbal_yaw_fusion;
static YawFusionConfig_t g_gimbal_yaw_fusion_cfg = YAW_FUSION_CFG_DEFAULT_INITIALIZER;
static uint8_t g_gimbal_yaw_fusion_need_align = 0u;




float yaw_output = 1;

// 哨兵目标生成配置仍放在 Gimbal.c，便于结合本云台机械范围统一调参
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
/**
 * @brief   云台角度限幅函数
 * @param  value: 输入角度值（单位：度）
 * @param  min_value: 最小角度值（单位：度）
 * @param  max_value: 最大角度值（单位：度）
 * @retval 限幅后的角度值（单位：度）
 */
static float Gimbal_Clamp(float value, float min_value, float max_value)
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
 * @brief   复位当前控制目标
 * @retval 无
 */
void Gimbal_Reset_Control_Targets(void)
{
    Gimbal_Sentry_Target_Clear_Output();
}

/**
 * @brief   清空IMU对外输出
 * @retval 无
 */
void Gimbal_Reset_Imu_Output(void)
{
    BMI088_Yaw_Continuous_Reset();
    Gimbal_Euler_Angle_to_send.roll = 0.0f;
    Gimbal_Euler_Angle_to_send.pitch = 0.0f;
    Gimbal_Euler_Angle_to_send.yaw = 0.0f;
    Gimbal_Euler_Angle_Final_Observe.roll = 0.0f;
    Gimbal_Euler_Angle_Final_Observe.pitch = 0.0f;
    Gimbal_Euler_Angle_Final_Observe.yaw = 0.0f;
    YawFusion_Reset(&g_gimbal_yaw_fusion);
    memset(&Gimbal_Yaw_Fusion_Observe, 0, sizeof(Gimbal_Yaw_Fusion_Observe));
    g_gimbal_yaw_fusion_need_align = 1u;
}

/**
 * @brief CAN 在线状态变化处理
 * @param online 0=离线 1=在线
 */
static void Gimbal_Device_Check_Handle_CAN_Change(uint8_t online)
{
    if (online != 0u)
    {
        return;
    }

    Motor_Clear_Runtime(&Gimbal_Motor_Pitch);
    Motor_Clear_Runtime(&Gimbal_Motor_Yaw);
    Gimbal_Reset_Control_Targets();
    YawFusion_Reset(&g_gimbal_yaw_fusion);
    memset(&Gimbal_Yaw_Fusion_Observe, 0, sizeof(Gimbal_Yaw_Fusion_Observe));
    g_gimbal_yaw_fusion_need_align = 1u;
}

/**
 * @brief SPI 在线状态变化处理
 * @param online 0=离线 1=在线
 */
static void Gimbal_Device_Check_Handle_SPI_Change(uint8_t online)
{
    if (online == 0u)
    {
        Gimbal_Reset_Imu_Output();
        return;
    }

    BMI088_Yaw_Continuous_Reset();
}

/**
 * @brief USB 在线状态变化处理。
 * @param online 0=离线 1=在线
 * @note  USB 离线时同时清空协议层目标和上层有效目标缓存，避免恢复在线后沿用旧目标。
 */
static void Gimbal_Device_Check_Handle_USB_Change(uint8_t online)
{
    if (online != 0u)
    {
        return;
    }

    Manifold_Clear_Target();
    Gimbal_Sentry_Target_Reset_Vision();
}

/**
 * @brief   浮点数到16位有符号整数的转换（带限幅）
 * @param  value: 输入浮点数（范围：-32768.0f ~ 32767.0f）
 * @retval 转换后的16位有符号整数（范围：-32768 ~ 32767）
 */
static int16_t Gimbal_FloatToInt16_Sat(float value)
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
 * @brief   CAN离线时的输出转换函数
 * @param   value: 任意输入值
 * @retval  固定返回0，确保离线期间电机输出为0
 */
static int16_t Gimbal_Output_To_CAN_Zero(float value)
{
    (void)value;
    return 0;
}

/**
 * @brief   CAN在线时的输出转换函数
 * @param   value: PID输出（浮点）
 * @retval  转换后的16位控制量
 */
static int16_t Gimbal_Output_To_CAN_Normal(float value)
{
    return Gimbal_FloatToInt16_Sat(value);
}

/**
 * @brief   Pitch/Yaw 共帧时合帧发送，否则回退为原来的逐电机发送
 * @param   pitch_cmd: Pitch 电机控制量
 * @param   yaw_cmd: Yaw 电机控制量
 * @retval  无
 */
static void Gimbal_Send_Pitch_Yaw_CAN(int16_t pitch_cmd, int16_t yaw_cmd)
{
    uint32_t pitch_send_id;
    uint32_t yaw_send_id;

    pitch_send_id = Motor_Get_CAN_Send_Id(&Gimbal_Motor_Pitch);
    yaw_send_id = Motor_Get_CAN_Send_Id(&Gimbal_Motor_Yaw);

    if ((Gimbal_Motor_Pitch.can == Gimbal_Motor_Yaw.can) &&
        (pitch_send_id != 0u) &&
        (pitch_send_id == yaw_send_id))
    {
        Motor_Update_CAN_Cache(&Gimbal_Motor_Pitch, pitch_cmd);
        Motor_Update_CAN_Cache(&Gimbal_Motor_Yaw, yaw_cmd);
        Motor_Send_CAN_Frame_By_Id(Gimbal_Motor_Pitch.can, pitch_send_id);
        return;
    }

    Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, pitch_cmd);
    Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, yaw_cmd);
}

/**
 * @brief   云台初始化函数
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Init(void* pramas)
{
    (void)pramas;

    // 初始化视觉通信
    Manifold_Init(&Tx_Data, 0xFE, 0xFF, Manifold_Sentry_Mode_ENABLE);

    // 启动电机所在CAN总线
    CAN_Start(&hcan2);

    // 初始化IMU
    if (BMI088_Init(&hspi1) != HAL_OK)
    {
        return;
    }
    
    // 俯仰电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Pitch, 4, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    // PID[1]: 角度外环，输出目标速度（小限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 1,
                         GIMBAL_PITCH_ANGLE_KP, GIMBAL_PITCH_ANGLE_KI, GIMBAL_PITCH_ANGLE_KD, GIMBAL_PITCH_ANGLE_FEEDFORWARD,
                         -GIMBAL_PITCH_ANGLE_OUT_LIMIT, GIMBAL_PITCH_ANGLE_OUT_LIMIT,
                         -GIMBAL_PITCH_ANGLE_I_LIMIT, GIMBAL_PITCH_ANGLE_I_LIMIT);
    // PID[0]: 速度内环，输出最终电机控制量（大限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Pitch, 0,
                         GIMBAL_PITCH_SPEED_KP, GIMBAL_PITCH_SPEED_KI, GIMBAL_PITCH_SPEED_KD, 0.00f,
                         -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT,
                         -GIMBAL_PITCH_SPEED_I_LIMIT, GIMBAL_PITCH_SPEED_I_LIMIT);
    // 偏航电机：角度外环 + 速度内环
    Motor_Init(&Gimbal_Motor_Yaw, 2, GM6020_Voltage, &hcan2, DJI_Control_Method_Angle);
    // PID[1]: 角度外环，输出目标速度（小限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 1,
                         GIMBAL_YAW_ANGLE_KP, GIMBAL_YAW_ANGLE_KI, GIMBAL_YAW_ANGLE_KD, GIMBAL_YAW_ANGLE_FEEDFORWARD,
                         -GIMBAL_YAW_ANGLE_OUT_LIMIT, GIMBAL_YAW_ANGLE_OUT_LIMIT,
                         -GIMBAL_YAW_ANGLE_I_LIMIT, GIMBAL_YAW_ANGLE_I_LIMIT);
    // PID[0]: 速度内环，输出最终电机控制量（大限幅）
    Motor_Set_PID_Params(&Gimbal_Motor_Yaw, 0,
                         GIMBAL_YAW_SPEED_KP, GIMBAL_YAW_SPEED_KI, GIMBAL_YAW_SPEED_KD, 0.00f,
                         -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT,
                         -GIMBAL_YAW_SPEED_I_LIMIT, GIMBAL_YAW_SPEED_I_LIMIT);
    //速度环的阻力补偿
    PID_Friction_Compensation_Enable(&Gimbal_Motor_Yaw.PID[0], true ,1200.0f,0.0f, 0.3, false );
    PID_Friction_Compensation_Enable(&Gimbal_Motor_Pitch.PID[0], true, 1600.0f, 0.0f, 0.3f, false);
    Gimbal_Sentry_Control_Init(&g_gimbal_sentry_control_config);
    Gimbal_Sentry_Control_Apply_Mode_Params(&Gimbal_Motor_Pitch,
                                            &Gimbal_Motor_Yaw,
                                            GIMBAL_SENTRY_STATE_SCAN);
    // 死区测试
    // PID_Deadband_Enable(&Gimbal_Motor_Yaw.PID[1], true, 0.2);
    // PID_Deadband_Enable(&Gimbal_Motor_Pitch.PID[1], true, 0.4);
   
    Gimbal_Sentry_Target_Init(&g_gimbal_sentry_target_config);
    memset(&Gimbal_Euler_Angle_Final_Observe, 0, sizeof(Gimbal_Euler_Angle_Final_Observe));
    YawFusion_Init(&g_gimbal_yaw_fusion, &g_gimbal_yaw_fusion_cfg);
    memset(&Gimbal_Yaw_Fusion_Observe, 0, sizeof(Gimbal_Yaw_Fusion_Observe));
    g_gimbal_yaw_fusion_need_align = 1u;
}



/**
 * @brief   云台双轴控制任务（1kHz）
 * @param  params: 无
 * @retval 无
 * @note    当前使用 SCAN / TRACK_ARMOR / LOST_TARGET_RETURN_SCAN 三态目标生成
 */
void Gimbal_Motor_Control_ALL_Test(void* params)
{
    TickType_t time;
    TickType_t now_tick;
    uint8_t can_online;
    uint8_t yaw_feedback_ready;
    float pitch_output;
    float pitch_target_deg;
    float pitch_target_speed ;
    float yaw_output;
    float yaw_target_deg;
    float yaw_target_speed;
    Gimbal_Sentry_State_TypeDef sentry_state;
    static int16_t (* const can_output_map[2])(float) =
    {
        Gimbal_Output_To_CAN_Zero,
        Gimbal_Output_To_CAN_Normal
    };

    
    (void)params;

    time = xTaskGetTickCount();

    Gimbal_Sentry_Target_Reset_Mode();

    pitch_output = 0.0f;
    yaw_output = 0.0f;
    while (1)
    {
        

        // 1) 刷新电机反馈
        Motor_CAN_Data_Receive(&Gimbal_Motor_Pitch);
        Motor_CAN_Data_Receive(&Gimbal_Motor_Yaw);

        // Pitch/Yaw外环都基于IMU姿态；这里只要求Yaw电机反馈ready后再闭环
        yaw_feedback_ready = (Gimbal_Motor_Yaw.RxData.Encoder_Initialized != 0u);
        if (yaw_feedback_ready == 0u)
        {
            can_online = CAN_Alive_IsOnline(&hcan2);
            Gimbal_Send_Pitch_Yaw_CAN(can_output_map[can_online](0.0f),
                                      can_output_map[can_online](0.0f));
            vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
            continue;
        }
        now_tick = xTaskGetTickCount();
        // 2) 更新目标和控制参数，不获取当前目标角度
        Gimbal_Sentry_Target_Update(now_tick);

        // 2.1) 根据当前Sentry状态应用控制参数
        pitch_target_deg = Gimbal_Sentry_Target_Get_Pitch();
        yaw_target_deg = Gimbal_Sentry_Target_Get_Yaw();
        // 2.2) 检查是否需要重置角度PID动态状态
        sentry_state = Gimbal_Sentry_Target_Get_State();
        // 应用当前状态对应的控制参数（如整形器斜率、前馈增益等）
        Gimbal_Sentry_Control_Apply_Mode_Params(&Gimbal_Motor_Pitch,
                                                &Gimbal_Motor_Yaw,
                                                sentry_state);
        // 2.3) 检查是否需要重置角度PID动态状态
        if (Gimbal_Sentry_Control_Angle_PID_Reset_Event_Check(sentry_state,
                                                              pitch_target_deg,
                                                              yaw_target_deg) != 0u)
        {
            Gimbal_Sentry_Control_Reset_Angle_PID_Dynamic_State(&Gimbal_Motor_Pitch.PID[1],
                                                                pitch_target_deg,
                                                                0.0f);
            Gimbal_Sentry_Control_Reset_Angle_PID_Dynamic_State(&Gimbal_Motor_Yaw.PID[1],
                                                                yaw_target_deg,
                                                                0.0f);
     
        }
        // 3) 计算PID输出
        pitch_target_speed = Motor_PID_Calculate_Angle(&Gimbal_Motor_Pitch, pitch_target_deg, -Gimbal_Euler_Angle_to_send.pitch, GIMBAL_CTRL_DT);
        pitch_output = Motor_PID_Calculate_Speed(&Gimbal_Motor_Pitch, pitch_target_speed, Gimbal_Motor_Pitch.RxData.Speed, GIMBAL_CTRL_DT);
        
        yaw_target_speed = Motor_PID_Calculate_Angle(&Gimbal_Motor_Yaw, yaw_target_deg, Gimbal_Euler_Angle_to_send.yaw, GIMBAL_CTRL_DT);
        yaw_output = Motor_PID_Calculate_Speed(&Gimbal_Motor_Yaw, yaw_target_speed, Gimbal_Motor_Yaw.RxData.Speed, GIMBAL_CTRL_DT);
        
        //输出限幅
        pitch_output = Gimbal_Clamp(pitch_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);
        yaw_output = Gimbal_Clamp(yaw_output, -GIMBAL_MOTOR_CMD_LIMIT, GIMBAL_MOTOR_CMD_LIMIT);

        can_online = CAN_Alive_IsOnline(&hcan2);
        Gimbal_Send_Pitch_Yaw_CAN(can_output_map[can_online](pitch_output),
                                can_output_map[can_online](yaw_output));
        // Motor_Send_CAN_Data(&Gimbal_Motor_Pitch, (int16_t)pitch_output);
        // Motor_Send_CAN_Data(&Gimbal_Motor_Yaw, (int16_t)yaw_output);
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}
/**
 * @brief   IMU数据就绪外部中断回调（仅做任务通知，不在中断中读SPI）
 * @param  GPIO_Pin: 触发中断的GPIO引脚
 * @note   非官方
 * @retval 无
 */
void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin)
{
#if GIMBAL_IMU_DRDY_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((GPIO_Pin == ACCEL_INT_Pin) || (GPIO_Pin == GYRO_INT_Pin))
    {
        if (g_gimbal_imu_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(g_gimbal_imu_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
#else
    (void)GPIO_Pin;
#endif
}

/**
 * @brief   云台欧拉角任务（1kHz）
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Euler(void *pramas)
{
    float imu_dt;
    float acc_norm_g;
    float imu_gyro_z_rad_s;
    euler_t euler_raw;
    uint16_t encoder_raw_count;
    uint8_t encoder_feedback_valid;

    (void)pramas;

    ALG_DWT_Timebase_Init(&g_gimbal_imu_timebase, GIMBAL_IMU_DT_DEFAULT_S);
    BMI088_Yaw_Continuous_Reset();
    
#if GIMBAL_IMU_DRDY_ENABLE
    g_gimbal_imu_task_handle = xTaskGetCurrentTaskHandle();
    
#else
    TickType_t time;
    time = xTaskGetTickCount();
#endif

    while (1)
    {
#if GIMBAL_IMU_DRDY_ENABLE
        // 中断模式：等待IMU DRDY通知，超时则回退一次轮询，避免中断异常导致任务停摆
        (void)ulTaskNotifyTake(pdTRUE, GIMBAL_IMU_WAIT_TIMEOUT_TICK);
#else
        // 纯任务轮询模式：固定1kHz节拍
        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
#endif

        // 周期采集IMU并更新欧拉角，供视觉发送使用
        BMI088_ReadGyro(&hspi1, &Gimbal_IMU_Data);
        BMI088_ReadAccel(&hspi1, &Gimbal_IMU_Data);
        BMI088_ReadTemp(&hspi1, &Gimbal_IMU_Data);
        imu_dt = ALG_DWT_Timebase_GetDtS(&g_gimbal_imu_timebase,
                                         GIMBAL_IMU_DT_DEFAULT_S,
                                         GIMBAL_IMU_DT_MIN_S,
                                         GIMBAL_IMU_DT_MAX_S);
        g_gimbal_imu_last_dt_s = g_gimbal_imu_timebase.last_dt_s;
        g_gimbal_imu_last_dt_from_dwt = g_gimbal_imu_timebase.last_dt_from_dwt;
        Gimbal_IMU_Data.dt = imu_dt;
        euler_raw = BMI088_Complementary_Filter(&Gimbal_IMU_Data, imu_dt, GIMBAL_MAHONY_KP, GIMBAL_MAHONY_KI);
        Gimbal_Euler_Angle_to_send = euler_raw;

        /* 并行 yaw 融合比对链路：边界统一换成 rad/rad/s/s，不影响现有 IMU 欧拉角输出。 */
        encoder_raw_count = Gimbal_Motor_Yaw.RxData.Last_encoder_angle;
        encoder_feedback_valid = Gimbal_Motor_Yaw.RxData.Encoder_Initialized;
        imu_gyro_z_rad_s = Gimbal_IMU_Data.gyro.z * PI / 180.0f;
        acc_norm_g = sqrtf(Gimbal_IMU_Data.acc.x * Gimbal_IMU_Data.acc.x +
                           Gimbal_IMU_Data.acc.y * Gimbal_IMU_Data.acc.y +
                           Gimbal_IMU_Data.acc.z * Gimbal_IMU_Data.acc.z);

        if (encoder_feedback_valid != 0u)
        {
            YawFusion_UpdateEncoderRaw(&g_gimbal_yaw_fusion, encoder_raw_count);
        }

        YawFusion_UpdateImu(&g_gimbal_yaw_fusion, imu_gyro_z_rad_s, acc_norm_g, imu_dt);
        YawFusion_UpdateTemperature(&g_gimbal_yaw_fusion, Gimbal_IMU_Data.temp);

        /* 编码器首次 ready 或掉线恢复后，先与编码器连续角对齐，避免对比输出起步跳变。 */
        if ((encoder_feedback_valid != 0u) && (g_gimbal_yaw_fusion_need_align != 0u))
        {
            YawFusion_AlignToEncoder(&g_gimbal_yaw_fusion);
            g_gimbal_yaw_fusion_need_align = 0u;
        }

        YawFusion_Run(&g_gimbal_yaw_fusion);
        YawFusion_GetState(&g_gimbal_yaw_fusion, &Gimbal_Yaw_Fusion_Observe.fusion_state);
        Gimbal_Yaw_Fusion_Observe.encoder_raw_count = encoder_raw_count;
        Gimbal_Yaw_Fusion_Observe.encoder_feedback_valid = encoder_feedback_valid;
        Gimbal_Yaw_Fusion_Observe.imu_gyro_z_rad_s = imu_gyro_z_rad_s;
        Gimbal_Yaw_Fusion_Observe.acc_norm_g = acc_norm_g;
        Gimbal_Yaw_Fusion_Observe.temp_c = Gimbal_IMU_Data.temp;
        Gimbal_Yaw_Fusion_Observe.dt_s = imu_dt;
        Gimbal_Yaw_Fusion_Observe.update_count++;

        /* 统一输出一份便于观察的最终欧拉角：roll/pitch 取 IMU，yaw 取并行融合结果。 */
        Gimbal_Euler_Angle_Final_Observe.roll = euler_raw.roll;
        Gimbal_Euler_Angle_Final_Observe.pitch = euler_raw.pitch;
        if (g_gimbal_yaw_fusion_need_align == 0u)
        {
            Gimbal_Euler_Angle_Final_Observe.yaw =
                Gimbal_Yaw_Fusion_Observe.fusion_state.yaw_est_rad * 180.0f / PI;
        }
        else
        {
            Gimbal_Euler_Angle_Final_Observe.yaw = euler_raw.yaw;
        }
    }
}

/**
 * @brief   云台manifold 控制任务
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Manifold_Control(void *pramas)
{
    TickType_t time;

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        // 周期发送当前欧拉角到视觉
        //USB_SendString("Gimbal Euler Angles: ");
        Manifold_USB_SendData(&Tx_Data, Gimbal_Euler_Angle_to_send);
        vTaskDelayUntil(&time, 10);
    }
}

/**
 * @brief   云台任务
 * @param  pramas: 无
 * @retval 无
 */
void Gimbal_Task(void *pramas)
{
    TickType_t time;
    uint16_t alive_check_div = 0u;
    uint8_t can_online_changed;
    uint8_t spi_online_changed;
    uint8_t usb_online_changed;

    (void)pramas;

    time = xTaskGetTickCount();

    while (1)
    {
        // 每100ms执行一次 CAN / SPI / USB 存活判断
        if (++alive_check_div >= 100u)
        {
            alive_check_div = 0u;
            CAN_Alive_Check_100ms(&hcan2);
            SPI_Alive_Check_100ms();
            USB_Alive_Check_100ms();
        }

        if (CAN_Alive_TryConsumeChanged(&hcan2, &can_online_changed) != 0u)
        {
            Gimbal_Device_Check_Handle_CAN_Change(can_online_changed);
        }

        if (SPI_Alive_TryConsumeChanged(&spi_online_changed) != 0u)
        {
            Gimbal_Device_Check_Handle_SPI_Change(spi_online_changed);
        }

        if (USB_Alive_TryConsumeChanged(&usb_online_changed) != 0u)
        {
            Gimbal_Device_Check_Handle_USB_Change(usb_online_changed);
        }

        vTaskDelayUntil(&time, GIMBAL_CTRL_PERIOD_TICK);
    }
}
