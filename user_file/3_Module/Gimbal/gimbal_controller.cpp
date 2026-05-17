#include "gimbal_controller.h"

#include "Gimbal.h"
#include "dvc_motor_protect.h"
#include "gimbal_debug.h"
#include "gimbal_fault.h"
#include "gimbal_sentry_control.h"
#include "gimbal_sentry_target.h"
#include <math.h>

#define GIMBAL_CONTROLLER_CTRL_DT                0.001f
#define GIMBAL_CONTROLLER_PERIOD_TICK            1u
#define GIMBAL_CONTROLLER_MOTOR_CMD_LIMIT        10000.0f
#define GIMBAL_CONTROLLER_YAW_ANGLE_OUT_LIMIT    10.0f
#define GIMBAL_CONTROLLER_YAW_ANGLE_DEADBAND_DEG 0.0f
#define GIMBAL_CONTROLLER_PITCH_ANGLE_OUT_LIMIT  10.0f
#define GIMBAL_CONTROLLER_PITCH_ANGLE_DEADBAND_DEG 0.4f

/**
 * @brief 选择角度环重置后的输出整形初值。
 * @param pid 需要读取旧输出状态的角度环 PID 对象。
 * @param target_deg 当前目标角度，单位 deg。
 * @param feedback_deg 当前反馈角度，单位 deg。
 * @param error_deadband_deg 误差死区阈值，单位 deg。
 * @param output_limit 角度环输出限幅。
 * @return 保留旧输出方向时返回旧输出，否则返回 0。
 */
static float Gimbal_Controller_SelectAngleResetOutput(const Class_PID *pid,
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

Class_Gimbal_Controller Gimbal_Controller_Object = {};

/**
 * @brief 初始化云台最终控制裁决对象。
 * @return 无。
 */
void Class_Gimbal_Controller::Init()
{
    Pitch_Target_Deg = 0.0f;
    Pitch_Target_Speed = 0.0f;
    Pitch_Output = 0.0f;
    Yaw_Target_Deg = 0.0f;
    Yaw_Target_Speed = 0.0f;
    Yaw_Output = 0.0f;
    Pitch_Can_Cmd = 0;
    Yaw_Can_Cmd = 0;
    Sentry_State = GIMBAL_SENTRY_STATE_SCAN;
}

/**
 * @brief 把任意输出映射为离线时的零命令。
 * @param value 任意输入值。
 * @return 固定返回 0。
 */
int16_t Class_Gimbal_Controller::OutputToCanZero(float value)
{
    (void)value;
    return 0;
}

/**
 * @brief 把浮点输出映射为在线时的 CAN 控制量。
 * @param value PID 浮点输出。
 * @return 转换后的 16 位控制量。
 */
int16_t Class_Gimbal_Controller::OutputToCanNormal(float value)
{
    return FloatToInt16Sat(value);
}

/**
 * @brief 推进一拍云台最终控制裁决。
 * @param gimbal 云台主对象。
 * @param now_tick 当前系统 tick。
 * @return 无。
 * @details
 * 第一版骨架只保留调用顺序，不在本文件内真正搬运控制逻辑。
 */
void Class_Gimbal_Controller::Update(Class_Gimbal *gimbal, uint32_t now_tick)
{
    if (gimbal == nullptr)
    {
        return;
    }

    UpdateSentryTarget(now_tick);
    ApplyProtectAndMode(gimbal);
    HandleAnglePidReset(gimbal);
    CalculateOutput(gimbal, now_tick);
    BuildCanCmd(gimbal);

    gimbal->Pitch_Current_Target_Deg = Pitch_Target_Deg;
    gimbal->Pitch_Current_Target_Speed = Pitch_Target_Speed;
    gimbal->Pitch_Current_Output = Pitch_Output;
    gimbal->Yaw_Current_Target_Deg = Yaw_Target_Deg;
    gimbal->Yaw_Current_Target_Speed = Yaw_Target_Speed;
    gimbal->Yaw_Current_Output = Yaw_Output;
}

/**
 * @brief 更新 sentry 原始目标与状态缓存。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Controller::UpdateSentryTarget(uint32_t now_tick)
{
    Gimbal_Sentry_Target_Object.Update(now_tick);
    Pitch_Target_Deg = Gimbal_Sentry_Target_Object.GetPitch();
    Yaw_Target_Deg = Gimbal_Sentry_Target_Object.GetYaw();
    Sentry_State = Gimbal_Sentry_Target_Object.GetState();
}

/**
 * @brief 处理保护开关与模式参数入口。
 * @param gimbal 云台主对象。
 * @return 无。
 */
void Class_Gimbal_Controller::ApplyProtectAndMode(Class_Gimbal *gimbal)
{
    if (Gimbal_Debug_IsPitchProtectForceDisabled() != 0u)
    {
        Motor_Protect_Pitch_Object.SetActive(0u);
    }
    else
    {
        Motor_Protect_Pitch_Object.SetActive(Sentry_State != GIMBAL_SENTRY_STATE_TRACK_ARMOR);
    }

    Pitch_Target_Deg = Motor_Protect_Pitch_Object.ApplyTarget(Pitch_Target_Deg);
    Gimbal_Sentry_Control_Object.ApplyModeParams(&gimbal->Motor_Pitch,
                                                 &gimbal->Motor_Yaw,
                                                 Sentry_State);
}

/**
 * @brief 处理角度环 reset 入口。
 * @param gimbal 云台主对象。
 * @return 无。
 */
void Class_Gimbal_Controller::HandleAnglePidReset(Class_Gimbal *gimbal)
{
    float pitch_feedback_deg;
    float pitch_reset_output;
    float yaw_reset_output;

    pitch_feedback_deg = -gimbal->Euler_Angle_Ekf_To_Send.pitch;

    if (Gimbal_Sentry_Control_Object.AnglePidResetEventCheck(Sentry_State,
                                                             Pitch_Target_Deg,
                                                             Yaw_Target_Deg) == 0u)
    {
        return;
    }

    Motor_Protect_Pitch_Object.Blank();
    pitch_reset_output = Gimbal_Controller_SelectAngleResetOutput(&gimbal->Motor_Pitch.PID[1],
                                                                  Pitch_Target_Deg,
                                                                  pitch_feedback_deg,
                                                                  GIMBAL_CONTROLLER_PITCH_ANGLE_DEADBAND_DEG,
                                                                  GIMBAL_CONTROLLER_PITCH_ANGLE_OUT_LIMIT);
    yaw_reset_output = Gimbal_Controller_SelectAngleResetOutput(&gimbal->Motor_Yaw.PID[1],
                                                                Yaw_Target_Deg,
                                                                gimbal->Euler_Angle_To_Send.yaw,
                                                                GIMBAL_CONTROLLER_YAW_ANGLE_DEADBAND_DEG,
                                                                GIMBAL_CONTROLLER_YAW_ANGLE_OUT_LIMIT);
    Gimbal_Sentry_Control_Object.ResetAnglePidDynamicState(&gimbal->Motor_Pitch.PID[1],
                                                           Pitch_Target_Deg,
                                                           pitch_reset_output);
    Gimbal_Sentry_Control_Object.ResetAnglePidDynamicState(&gimbal->Motor_Yaw.PID[1],
                                                           Yaw_Target_Deg,
                                                           yaw_reset_output);
}

/**
 * @brief 计算当前拍最终速度目标与输出。
 * @param gimbal 云台主对象。
 * @return 无。
 */
void Class_Gimbal_Controller::CalculateOutput(Class_Gimbal *gimbal, uint32_t now_tick)
{
    float pitch_feedback_deg;
    float pitch_protect_reset_target_deg;

    pitch_feedback_deg = -gimbal->Euler_Angle_Ekf_To_Send.pitch;

    Pitch_Target_Speed = gimbal->Motor_Pitch.PidCalculateAngle(Pitch_Target_Deg,
                                                               pitch_feedback_deg,
                                                               GIMBAL_CONTROLLER_CTRL_DT);
    Pitch_Output = gimbal->Motor_Pitch.PidCalculateSpeed(Pitch_Target_Speed,
                                                         gimbal->Motor_Pitch.RxData.Speed,
                                                         GIMBAL_CONTROLLER_CTRL_DT);

    Yaw_Target_Speed = gimbal->Motor_Yaw.PidCalculateAngle(Yaw_Target_Deg,
                                                           gimbal->Euler_Angle_Ekf_To_Send.yaw,
                                                           GIMBAL_CONTROLLER_CTRL_DT);
    Yaw_Output = gimbal->Motor_Yaw.PidCalculateSpeed(Yaw_Target_Speed,
                                                     gimbal->Motor_Yaw.RxData.Speed,
                                                     GIMBAL_CONTROLLER_CTRL_DT);

    Pitch_Output = Clamp(Pitch_Output,
                         -GIMBAL_CONTROLLER_MOTOR_CMD_LIMIT,
                         GIMBAL_CONTROLLER_MOTOR_CMD_LIMIT);
    Yaw_Output = Clamp(Yaw_Output,
                       -GIMBAL_CONTROLLER_MOTOR_CMD_LIMIT,
                       GIMBAL_CONTROLLER_MOTOR_CMD_LIMIT);

    Pitch_Output = Motor_Protect_Pitch_Object.UpdateOutput(Pitch_Target_Deg,
                                                           pitch_feedback_deg,
                                                           gimbal->Motor_Pitch.RxData.Speed,
                                                           Pitch_Output,
                                                           gimbal->Motor_Pitch.RxData.Torque,
                                                           GIMBAL_CONTROLLER_PERIOD_TICK);
    Gimbal_Fault_Object.SyncBit(GIMBAL_FAULT_PITCH_PROTECT,
                                Motor_Protect_Pitch_Object.IsFault(),
                                0u,
                                now_tick);

    if (Motor_Protect_Pitch_Object.TakeResetRequest(&pitch_protect_reset_target_deg) != 0u)
    {
        Gimbal_Sentry_Control_Object.ResetAnglePidDynamicState(&gimbal->Motor_Pitch.PID[1],
                                                               pitch_protect_reset_target_deg,
                                                               0.0f);
    }
}

/**
 * @brief 生成当前拍最终 CAN 控制量。
 * @param gimbal 云台主对象。
 * @return 无。
 */
void Class_Gimbal_Controller::BuildCanCmd(Class_Gimbal *gimbal)
{
    uint8_t can_online;

    (void)gimbal;

    can_online = CAN2_Manage_Object.AliveIsOnline();
    Pitch_Can_Cmd = (can_online != 0u) ? OutputToCanNormal(Pitch_Output)
                                       : OutputToCanZero(Pitch_Output);
    Yaw_Can_Cmd = (can_online != 0u) ? OutputToCanNormal(Yaw_Output)
                                     : OutputToCanZero(Yaw_Output);
}
