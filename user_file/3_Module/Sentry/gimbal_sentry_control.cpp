/**
 * @file gimbal_sentry_control.cpp
 * @brief 哨兵模式控制参数切换实现。
 * @details
 * 本文件实现 `Class_Gimbal_Sentry_Control` 的成员函数。
 */
#include "gimbal_sentry_control.h"

#include <math.h>

Class_Gimbal_Sentry_Control Gimbal_Sentry_Control_Object = {};

/**
 * @brief 初始化 Sentry 控制模块。
 * @param config Sentry 控制模块配置结构体指针。
 * @return 无。
 */
void Class_Gimbal_Sentry_Control::Init(const Gimbal_Sentry_Control_Config_TypeDef *config_value)
{
    if (config_value == nullptr)
    {
        return;
    }

    config = *config_value;
    initialized = 1u;
    params_applied = 0u;
    last_param_state = GIMBAL_SENTRY_STATE_SCAN;
    last_reset_state = GIMBAL_SENTRY_STATE_SCAN;
    last_pitch_target_deg = 0.0f;
    last_yaw_target_deg = 0.0f;
}

/**
 * @brief 按当前状态应用 Sentry 控制参数。
 * @param pitch_motor 俯仰电机结构体指针。
 * @param yaw_motor 偏航电机结构体指针。
 * @param state 当前 Sentry 状态。
 * @return 无。
 */
void Class_Gimbal_Sentry_Control::ApplyModeParams(Class_Motor *pitch_motor,
                                                  Class_Motor *yaw_motor,
                                                  Gimbal_Sentry_State_TypeDef state_value)
{
    float pitch_feedforward;
    float yaw_feedforward;
    float filter_tau_s;
    float pitch_slew_rate;
    float yaw_slew_rate;

    if ((initialized == 0u) || (pitch_motor == nullptr) || (yaw_motor == nullptr))
    {
        return;
    }

    if ((params_applied != 0u) && (state_value == last_param_state))
    {
        return;
    }

    if (state_value == GIMBAL_SENTRY_STATE_TRACK_ARMOR)
    {
        pitch_feedforward = 0.0f;
        yaw_feedforward = 0.0f;
        filter_tau_s = config.vision_shaper_filter_tau_s;
        pitch_slew_rate = config.pitch_vision_shaper_slew_rate;
        yaw_slew_rate = config.yaw_vision_shaper_slew_rate;
    }
    else
    {
        pitch_feedforward = config.pitch_angle_feedforward;
        yaw_feedforward = config.yaw_angle_feedforward;
        filter_tau_s = config.sentry_shaper_filter_tau_s;
        pitch_slew_rate = config.pitch_sentry_shaper_slew_rate;
        yaw_slew_rate = config.yaw_sentry_shaper_slew_rate;
    }

    pitch_motor->PID[1].FeedForward = pitch_feedforward;
    yaw_motor->PID[1].FeedForward = yaw_feedforward;
    pitch_motor->PID[1].OutputFilterEnable(true, filter_tau_s);
    pitch_motor->PID[1].OutputSlewEnable(true, pitch_slew_rate);
    yaw_motor->PID[1].OutputFilterEnable(true, filter_tau_s);
    yaw_motor->PID[1].OutputSlewEnable(true, yaw_slew_rate);

    last_param_state = state_value;
    params_applied = 1u;
}

/**
 * @brief 检查 Sentry 控制模块是否需要重置角度 PID。
 * @param state 当前 Sentry 状态。
 * @param pitch_target_deg 当前俯仰目标角度，单位：deg。
 * @param yaw_target_deg 当前偏航目标角度，单位：deg。
 * @return `1` 表示需要重置，`0` 表示不需要。
 */
uint8_t Class_Gimbal_Sentry_Control::AnglePidResetEventCheck(Gimbal_Sentry_State_TypeDef state_value,
                                                             float pitch_target_deg_value,
                                                             float yaw_target_deg_value)
{
    uint8_t reset_needed = 0u;

    if (initialized == 0u)
    {
        return 0u;
    }

    if (state_value == GIMBAL_SENTRY_STATE_TRACK_ARMOR)
    {
        if (last_reset_state != GIMBAL_SENTRY_STATE_TRACK_ARMOR)
        {
            reset_needed = 1u;
        }
        else if ((fabsf(yaw_target_deg_value - last_yaw_target_deg) > config.target_reset_yaw_deg) ||
                 (fabsf(pitch_target_deg_value - last_pitch_target_deg) > config.target_reset_pitch_deg))
        {
            reset_needed = 1u;
        }
    }

    last_reset_state = state_value;
    last_pitch_target_deg = pitch_target_deg_value;
    last_yaw_target_deg = yaw_target_deg_value;

    return reset_needed;
}

/**
 * @brief 重置 Sentry 控制模块的角度 PID 动态状态。
 * @param pid 要重置的角度 PID 结构体指针。
 * @param target_now 当前目标角度，单位：deg。
 * @param output_now 当前输出角度，单位：deg。
 * @return 无。
 */
void Class_Gimbal_Sentry_Control::ResetAnglePidDynamicState(Class_PID *pid,
                                                            float target_now,
                                                            float output_now)
{
    if (pid == nullptr)
    {
        return;
    }

    pid->target = target_now;
    pid->prev_target = target_now;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->P_out = 0.0f;
    pid->I_out = 0.0f;
    pid->D_out = 0.0f;
    pid->FeedForward_out = 0.0f;
    pid->Friction_Compensation_out = 0.0f;
    pid->Gravity_Compensation_out = 0.0f;
    pid->output = output_now;
    pid->output_shaper_state = output_now;
    pid->output_shaper_inited = true;
}
