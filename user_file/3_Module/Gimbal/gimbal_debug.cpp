/**
 * @file gimbal_debug.cpp
 * @brief 云台调试观察量与调试命令实现。
 */
#include "gimbal_debug.h"

#include "Gimbal.h"
#include "dvc_manifold.h"
#include "dvc_motor_protect.h"
#include "drv_usb.h"
#include "gimbal_fault.h"
#include "gimbal_status.h"
#include "gimbal_sentry_target.h"
#include <string.h>

volatile Gimbal_Debug_View_TypeDef Gimbal_Debug_View = {};
volatile Gimbal_Debug_Cmd_TypeDef Gimbal_Debug_Cmd = {};

static uint8_t g_gimbal_debug_pitch_protect_force_disabled = 0u;
static uint8_t g_gimbal_debug_last_pid_target = 0u;

/**
 * @brief 根据调试目标索引获取对应 PID 对象。
 * @param gimbal 云台对象指针。
 * @param pid_target PID 调试对象索引。
 * @return 成功时返回 PID 指针，失败返回 nullptr。
 */
static Class_PID *Gimbal_Debug_GetPid(Class_Gimbal *gimbal, uint8_t pid_target)
{
    if (gimbal == nullptr)
    {
        return nullptr;
    }

    switch (pid_target)
    {
    case GIMBAL_DEBUG_PID_PITCH_SPEED:
        return &gimbal->Motor_Pitch.PID[0];
    case GIMBAL_DEBUG_PID_PITCH_ANGLE:
        return &gimbal->Motor_Pitch.PID[1];
    case GIMBAL_DEBUG_PID_YAW_SPEED:
        return &gimbal->Motor_Yaw.PID[0];
    case GIMBAL_DEBUG_PID_YAW_ANGLE:
        return &gimbal->Motor_Yaw.PID[1];
    default:
        return nullptr;
    }
}

/**
 * @brief 按当前调试命令参数覆盖选中的 PID 环参数。
 * @param gimbal 云台对象指针。
 * @return 无。
 */
static void Gimbal_Debug_WritePid(Class_Gimbal *gimbal)
{
    Class_Motor *motor = nullptr;
    Class_PID *pid;
    uint8_t pid_index = 0u;

    pid = Gimbal_Debug_GetPid(gimbal, Gimbal_Debug_Cmd.pid_target);
    if (pid == nullptr)
    {
        return;
    }

    switch (Gimbal_Debug_Cmd.pid_target)
    {
    case GIMBAL_DEBUG_PID_PITCH_SPEED:
    case GIMBAL_DEBUG_PID_PITCH_ANGLE:
        motor = &gimbal->Motor_Pitch;
        break;
    case GIMBAL_DEBUG_PID_YAW_SPEED:
    case GIMBAL_DEBUG_PID_YAW_ANGLE:
        motor = &gimbal->Motor_Yaw;
        break;
    default:
        return;
    }
    //  4 个选项中提取"电机内部的 PID 环索引
    pid_index = (uint8_t)(Gimbal_Debug_Cmd.pid_target & 0x01u);
    motor->SetPidParams(pid_index,
                        Gimbal_Debug_Cmd.pid_kp,
                        Gimbal_Debug_Cmd.pid_ki,
                        Gimbal_Debug_Cmd.pid_kd,
                        Gimbal_Debug_Cmd.FeedForward,
                        pid->out_min,
                        pid->out_max,
                        pid->integral_min,
                        pid->integral_max);

}

/**
 * @brief 初始化云台调试观察量和调试命令区。
 * @param gimbal 云台对象指针。
 * @return 无。
 */
void Gimbal_Debug_Init(Class_Gimbal *gimbal)
{
    Class_PID *pid;

    memset((void *)&Gimbal_Debug_View, 0, sizeof(Gimbal_Debug_View));
    memset((void *)&Gimbal_Debug_Cmd, 0, sizeof(Gimbal_Debug_Cmd));
    g_gimbal_debug_pitch_protect_force_disabled = 0u;

    Gimbal_Debug_Cmd.pid_target = GIMBAL_DEBUG_PID_PITCH_SPEED;
    g_gimbal_debug_last_pid_target = Gimbal_Debug_Cmd.pid_target;
    pid = Gimbal_Debug_GetPid(gimbal, Gimbal_Debug_Cmd.pid_target);
    if (pid != nullptr)
    {
        Gimbal_Debug_Cmd.pid_kp = pid->Kp;
        Gimbal_Debug_Cmd.pid_ki = pid->Ki;
        Gimbal_Debug_Cmd.pid_kd = pid->Kd;
        Gimbal_Debug_Cmd.FeedForward = pid->FeedForward;
    }
}

/**
 * @brief 处理一拍调试命令。
 * @param gimbal 云台对象指针。
 * @return 无。
 */
void Gimbal_Debug_HandleCmd(Class_Gimbal *gimbal)
{
    Class_PID *pid;

    if (gimbal == nullptr)
    {
        return;
    }

    pid = Gimbal_Debug_GetPid(gimbal, Gimbal_Debug_Cmd.pid_target);
    if ((Gimbal_Debug_Cmd.pid_target != g_gimbal_debug_last_pid_target) &&
        (pid != nullptr))
    {
        // 切换调试目标时，先把新目标当前参数刷回命令区，避免沿用上一个环的参数。
        Gimbal_Debug_Cmd.pid_kp = pid->Kp;
        Gimbal_Debug_Cmd.pid_ki = pid->Ki;
        Gimbal_Debug_Cmd.pid_kd = pid->Kd;
        Gimbal_Debug_Cmd.FeedForward = pid->FeedForward;
        g_gimbal_debug_last_pid_target = Gimbal_Debug_Cmd.pid_target;
    }

    if ((Gimbal_Debug_Cmd.write_pid == 0u) && (pid != nullptr))
    {
        // 未开启连续写入时，命令区持续镜像当前选中 PID 参数，便于直接观察与修改。
        Gimbal_Debug_Cmd.pid_kp = pid->Kp;
        Gimbal_Debug_Cmd.pid_ki = pid->Ki;
        Gimbal_Debug_Cmd.pid_kd = pid->Kd;
        Gimbal_Debug_Cmd.FeedForward = pid->FeedForward;
    }
    else if (pid == nullptr)
    {
        g_gimbal_debug_last_pid_target = Gimbal_Debug_Cmd.pid_target;
    }

    if (Gimbal_Debug_Cmd.enable == 0u)
    {
        g_gimbal_debug_pitch_protect_force_disabled = 0u;
        return;
    }

    if (Gimbal_Debug_Cmd.clear_fault != 0u)
    {
        Gimbal_Fault_Object.ClearBits(0xFFFFFFFFu, 1u);
        Motor_Protect_Pitch_Object.ClearFault();
    }

    g_gimbal_debug_pitch_protect_force_disabled =
        (Gimbal_Debug_Cmd.disable_pitch_protect != 0u) ? 1u : 0u;

    if (Gimbal_Debug_Cmd.write_pid != 0u)
    {
        Gimbal_Debug_WritePid(gimbal);
    }
}

/**
 * @brief 更新视觉联调观察量。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Gimbal_Debug_UpdateVisionView(TickType_t now_tick)
{
    uint8_t target_valid = 0u;

    if ((Gimbal_Sentry_Target_Object.initialized != 0u) &&
        (Gimbal_Sentry_Target_Object.visual_has_valid_target != 0u) &&
        (USB_Manage_Object.AliveIsOnline() != 0u))
    {
        if ((now_tick - Gimbal_Sentry_Target_Object.visual_last_valid_tick) <=
            pdMS_TO_TICKS(Gimbal_Sentry_Target_Object.config.vision_track_timeout_ms))
        {
            target_valid = 1u;
        }
    }

    Gimbal_Debug_View.vision.target_valid = target_valid;
    Gimbal_Debug_View.vision.target_ever_valid = Gimbal_Sentry_Target_Object.visual_has_valid_target;
    Gimbal_Debug_View.vision.filter_tracking = Gimbal_Sentry_Target_Object.visual_filter_tracking;
    Gimbal_Debug_View.vision.sentry_state = (uint8_t)Gimbal_Sentry_Target_Object.GetState();
    Gimbal_Debug_View.vision.rx_frame_seq = Manifold_USB_Rx_Frame_Seq;
    Gimbal_Debug_View.vision.last_valid_tick = (uint32_t)Gimbal_Sentry_Target_Object.visual_last_valid_tick;
    Gimbal_Debug_View.vision.raw_pitch_deg = Rx_Data.Taget_Pitch;
    Gimbal_Debug_View.vision.raw_yaw_deg = Rx_Data.Taget_Yaw;
    Gimbal_Debug_View.vision.cached_pitch_deg = Gimbal_Sentry_Target_Object.visual_last_valid_pitch_deg;
    Gimbal_Debug_View.vision.cached_yaw_deg = Gimbal_Sentry_Target_Object.visual_last_valid_yaw_deg;
    Gimbal_Debug_View.vision.filtered_pitch_deg = Gimbal_Sentry_Target_Object.visual_filtered_pitch_deg;
    Gimbal_Debug_View.vision.filtered_yaw_deg = Gimbal_Sentry_Target_Object.visual_filtered_yaw_deg;
}

/**
 * @brief 更新通信与系统观察量。
 * @param gimbal 云台对象指针。
 * @return 无。
 */
void Gimbal_Debug_UpdateCommView(const Class_Gimbal *gimbal)
{
    if (gimbal == nullptr)
    {
        return;
    }

    Gimbal_Debug_View.comm.can_online = Gimbal_Status_Object.IsCanOnline();
    Gimbal_Debug_View.comm.spi_online = Gimbal_Status_Object.IsSpiOnline();
    Gimbal_Debug_View.comm.usb_online = Gimbal_Status_Object.IsUsbOnline();
    Gimbal_Debug_View.comm.fault_bits = Gimbal_Fault_Object.Active_Bits;
    Gimbal_Debug_View.comm.fault_count = Gimbal_Fault_Object.Fault_Count;
    Gimbal_Debug_View.comm.last_fault_tick = Gimbal_Fault_Object.Last_Fault_Tick;
    Gimbal_Debug_View.comm.imu_dt_s = gimbal->Imu_Last_Dt_S;
    Gimbal_Debug_View.comm.imu_dt_from_dwt = gimbal->Imu_Last_Dt_From_Dwt;
}

/**
 * @brief 更新电机与控制观察量。
 * @param gimbal 云台对象指针。
 * @return 无。
 */
void Gimbal_Debug_UpdateMotorView(const Class_Gimbal *gimbal)
{
    if (gimbal == nullptr)
    {
        return;
    }

    Gimbal_Debug_View.motor.pitch_protect_enabled = Motor_Protect_Pitch_Object.IsActive();
    Gimbal_Debug_View.motor.pitch_protect_fault = Motor_Protect_Pitch_Object.IsFault();
    Gimbal_Debug_View.motor.pitch_protect_force_disabled = g_gimbal_debug_pitch_protect_force_disabled;

    Gimbal_Debug_View.motor.pitch_angle_target = gimbal->Pitch_Current_Target_Deg;
    Gimbal_Debug_View.motor.pitch_angle_feedback = -gimbal->Euler_Angle_Ekf_To_Send.pitch;
    Gimbal_Debug_View.motor.pitch_speed_target = gimbal->Pitch_Current_Target_Speed;
    Gimbal_Debug_View.motor.pitch_speed_feedback = gimbal->Motor_Pitch.RxData.Speed;
    Gimbal_Debug_View.motor.pitch_output = gimbal->Pitch_Current_Output;

    Gimbal_Debug_View.motor.yaw_angle_target = gimbal->Yaw_Current_Target_Deg;
    Gimbal_Debug_View.motor.yaw_angle_feedback = gimbal->Euler_Angle_Ekf_To_Send.yaw;
    Gimbal_Debug_View.motor.yaw_speed_target = gimbal->Yaw_Current_Target_Speed;
    Gimbal_Debug_View.motor.yaw_speed_feedback = gimbal->Motor_Yaw.RxData.Speed;
    Gimbal_Debug_View.motor.yaw_output = gimbal->Yaw_Current_Output;
}

/**
 * @brief 查询当前是否强制关闭 pitch 保护。
 * @return 强制关闭返回 1，否则返回 0。
 */
uint8_t Gimbal_Debug_IsPitchProtectForceDisabled()
{
    return g_gimbal_debug_pitch_protect_force_disabled;
}
