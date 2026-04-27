/**
 * @file gimbal_sentry_control.c
 * @brief 哨兵模式控制参数切换实现。
 */
#include "gimbal_sentry_control.h"
#include <math.h>
#include <stdbool.h>

static Gimbal_Sentry_Control_Config_TypeDef g_sentry_control_config; /**< 控制模块配置缓存。 */
static uint8_t g_sentry_control_initialized = 0u; /**< 控制模块初始化完成标志。 */
static uint8_t g_sentry_control_params_applied = 0u; /**< 当前状态对应参数是否已下发。 */
static Gimbal_Sentry_State_TypeDef g_sentry_control_last_param_state = GIMBAL_SENTRY_STATE_SCAN; /**< 上次下发参数时的状态。 */

static Gimbal_Sentry_State_TypeDef g_sentry_control_last_reset_state = GIMBAL_SENTRY_STATE_SCAN; /**< 上次判定重置事件时的状态。 */
static float g_sentry_control_last_pitch_target_deg = 0.0f; /**< 上次判定重置事件时的 pitch 目标角。 */
static float g_sentry_control_last_yaw_target_deg = 0.0f; /**< 上次判定重置事件时的 yaw 目标角。 */

/**
 * @brief 初始化 Sentry 控制模块。
 * @param config Sentry 控制模块配置结构体指针。
 */
void Gimbal_Sentry_Control_Init(const Gimbal_Sentry_Control_Config_TypeDef *config)
{
    if (config == NULL)
    {
        return;
    }

    g_sentry_control_config = *config;
    g_sentry_control_initialized = 1u;
    g_sentry_control_params_applied = 0u;
    g_sentry_control_last_param_state = GIMBAL_SENTRY_STATE_SCAN;
    g_sentry_control_last_reset_state = GIMBAL_SENTRY_STATE_SCAN;
    g_sentry_control_last_pitch_target_deg = 0.0f;
    g_sentry_control_last_yaw_target_deg = 0.0f;
}

/**
 * @brief 按当前状态应用 Sentry 控制参数。
 * @param pitch_motor 俯仰电机结构体指针。
 * @param yaw_motor 偏航电机结构体指针。
 * @param state 当前 Sentry 状态。
 */
void Gimbal_Sentry_Control_Apply_Mode_Params(Motor_TypeDef *pitch_motor,
                                             Motor_TypeDef *yaw_motor,
                                             Gimbal_Sentry_State_TypeDef state)
{
    float pitch_feedforward;
    float yaw_feedforward;
    float filter_tau_s;
    float pitch_slew_rate;
    float yaw_slew_rate;

    if ((g_sentry_control_initialized == 0u) || (pitch_motor == NULL) || (yaw_motor == NULL))
    {
        return;
    }

    if ((g_sentry_control_params_applied != 0u) &&
        (state == g_sentry_control_last_param_state))
    {
        return;
    }

    if (state == GIMBAL_SENTRY_STATE_TRACK_ARMOR)
    {
        pitch_feedforward = 0.0f;
        yaw_feedforward = 0.0f;
        filter_tau_s = g_sentry_control_config.vision_shaper_filter_tau_s;
        pitch_slew_rate = g_sentry_control_config.pitch_vision_shaper_slew_rate;
        yaw_slew_rate = g_sentry_control_config.yaw_vision_shaper_slew_rate;
    }
    else
    {
        pitch_feedforward = g_sentry_control_config.pitch_angle_feedforward;
        yaw_feedforward = g_sentry_control_config.yaw_angle_feedforward;
        filter_tau_s = g_sentry_control_config.sentry_shaper_filter_tau_s;
        pitch_slew_rate = g_sentry_control_config.pitch_sentry_shaper_slew_rate;
        yaw_slew_rate = g_sentry_control_config.yaw_sentry_shaper_slew_rate;
    }

    pitch_motor->PID[1].FeedForward = pitch_feedforward;
    yaw_motor->PID[1].FeedForward = yaw_feedforward;
    PID_Output_Filter_Enable(&pitch_motor->PID[1], true, filter_tau_s);
    PID_Output_Slew_Enable(&pitch_motor->PID[1], true, pitch_slew_rate);
    PID_Output_Filter_Enable(&yaw_motor->PID[1], true, filter_tau_s);
    PID_Output_Slew_Enable(&yaw_motor->PID[1], true, yaw_slew_rate);

    g_sentry_control_last_param_state = state;
    g_sentry_control_params_applied = 1u;
}

/**
 * @brief 检查 Sentry 控制模块是否需要重置角度 PID。
 * @param state 当前 Sentry 状态。
 * @param pitch_target_deg 当前俯仰目标角度，单位：deg。
 * @param yaw_target_deg 当前偏航目标角度，单位：deg。
 * @return `1` 表示需要重置，`0` 表示不需要。
 */
uint8_t Gimbal_Sentry_Control_Angle_PID_Reset_Event_Check(Gimbal_Sentry_State_TypeDef state,
                                                          float pitch_target_deg,
                                                          float yaw_target_deg)
{
    uint8_t reset_needed = 0u;

    if (g_sentry_control_initialized == 0u)
    {
        return 0u;
    }

    if (state == GIMBAL_SENTRY_STATE_TRACK_ARMOR)
    {
        if (g_sentry_control_last_reset_state != GIMBAL_SENTRY_STATE_TRACK_ARMOR)
        {
            reset_needed = 1u;
        }
        else if ((fabsf(yaw_target_deg - g_sentry_control_last_yaw_target_deg) >
                  g_sentry_control_config.target_reset_yaw_deg) ||
                 (fabsf(pitch_target_deg - g_sentry_control_last_pitch_target_deg) >
                  g_sentry_control_config.target_reset_pitch_deg))
        {
            reset_needed = 1u;
        }
    }

    g_sentry_control_last_reset_state = state;
    g_sentry_control_last_pitch_target_deg = pitch_target_deg;
    g_sentry_control_last_yaw_target_deg = yaw_target_deg;

    return reset_needed;
}

/**
 * @brief 重置 Sentry 控制模块的角度 PID 动态状态。
 * @param pid 要重置的角度 PID 结构体指针。
 * @param target_now 当前目标角度，单位：deg。
 * @param output_now 当前输出角度，单位：deg。
 */
void Gimbal_Sentry_Control_Reset_Angle_PID_Dynamic_State(PID_TypeDef *pid,
                                                         float target_now,
                                                         float output_now)
{
    if (pid == NULL)
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
    pid->output = output_now;
    pid->output_shaper_state = output_now;
    pid->output_shaper_inited = true;
}
