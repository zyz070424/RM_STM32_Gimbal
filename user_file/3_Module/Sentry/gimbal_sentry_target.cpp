/**
 * @file gimbal_sentry_target.cpp
 * @brief 哨兵目标生成模块实现。
 * @details
 * 本文件实现 `Class_Gimbal_Sentry_Target` 的成员函数。
 */
#include "gimbal_sentry_target.h"

#include "dvc_manifold.h"
#include "drv_usb.h"
#include <math.h>

extern Manifold_UART_Rx_Data Rx_Data; /**< 视觉链路最新一帧接收缓存。 */

namespace
{
constexpr float GimbalSentryTwoPi = 6.283185307f;

float Gimbal_Sentry_Wrap_Phase(float phase_rad)
{
    while (phase_rad >= GimbalSentryTwoPi)
    {
        phase_rad -= GimbalSentryTwoPi;
    }

    while (phase_rad < 0.0f)
    {
        phase_rad += GimbalSentryTwoPi;
    }

    return phase_rad;
}

float Gimbal_Sentry_Scan_Phase_Step(float phase_rad, float frequency_hz, float dt_s)
{
    phase_rad += GimbalSentryTwoPi * frequency_hz * dt_s;
    return Gimbal_Sentry_Wrap_Phase(phase_rad);
}

float Gimbal_Sentry_Scan_Target_Sine(float amplitude_deg, float phase_rad)
{
    return amplitude_deg * sinf(phase_rad);
}
}

Class_Gimbal_Sentry_Target Gimbal_Sentry_Target_Object = {};

/**
 * @brief 刷新视觉输入缓存。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::UpdateCache(TickType_t now_tick)
{
    uint32_t rx_frame_seq = Manifold_USB_Rx_Frame_Seq;

    if (rx_frame_seq != visual_last_rx_frame_seq)
    {
        visual_last_rx_frame_seq = rx_frame_seq;
        if (Rx_Data.Target_Valid != 0u)
        {
            visual_last_valid_pitch_deg = Rx_Data.Taget_Pitch;
            visual_last_valid_yaw_deg = Rx_Data.Taget_Yaw;
            visual_last_valid_tick = now_tick;
            visual_has_valid_target = 1u;
        }
    }
}

/**
 * @brief 判断当前拍是否存在可用视觉目标。
 * @param now_tick 当前系统 tick。
 * @return `1` 表示视觉目标可用，`0` 表示不可用。
 */
uint8_t Class_Gimbal_Sentry_Target::IsAvailable(TickType_t now_tick)
{
    UpdateCache(now_tick);

    if ((visual_has_valid_target == 0u) || (USB_Manage_Object.AliveIsRxOnline() == 0u))
    {
        return 0u;
    }

    if ((now_tick - visual_last_valid_tick) > pdMS_TO_TICKS(config.vision_track_timeout_ms))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 根据目标有效性更新视觉滤波输出。
 * @param target_available 当前拍视觉目标是否可用。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::UpdateFilter(uint8_t target_available)
{
    float alpha;

    if (target_available == 0u)
    {
        visual_filter_tracking = 0u;
        return;
    }

    if (visual_filter_tracking == 0u)
    {
        visual_filtered_pitch_deg = -pitch_target;
        visual_filtered_yaw_deg = yaw_target;
        visual_filter_tracking = 1u;
    }

    if (config.vision_target_filter_tau_s <= 0.0f)
    {
        alpha = 1.0f;
    }
    else
    {
        alpha = config.control_dt_s /
                (config.vision_target_filter_tau_s + config.control_dt_s);
    }

    alpha = Clamp(alpha, 0.0f, 1.0f);
    visual_filtered_pitch_deg = FirstOrderLPF(visual_last_valid_pitch_deg,
                                              visual_filtered_pitch_deg,
                                              alpha);
    visual_filtered_yaw_deg = FirstOrderLPF(visual_last_valid_yaw_deg,
                                            visual_filtered_yaw_deg,
                                            alpha);
}

/**
 * @brief 初始化哨兵目标生成模块。
 * @param config_value 目标生成模块配置。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::Init(const Gimbal_Sentry_Target_Config_TypeDef *config_value)
{
    if (config_value == nullptr)
    {
        return;
    }

    config = *config_value;
    initialized = 1u;
    ResetMode();
}

/**
 * @brief 复位哨兵模式状态。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::ResetMode()
{
    if (initialized == 0u)
    {
        return;
    }

    sentry_handle.Reset();
    last_state = GIMBAL_SENTRY_STATE_SCAN;
    scan_yaw_phase_rad = 0.0f;
    scan_pitch_phase_rad = 0.0f;
    scan_yaw_target_deg = 0.0f;
    scan_pitch_target_deg = 0.0f;
    raw_pitch_target = 0.0f;
    raw_yaw_target = 0.0f;
    ResetVision();
    ClearOutput();
}

/**
 * @brief 清空视觉目标缓存与滤波状态。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::ResetVision()
{
    visual_last_rx_frame_seq = Manifold_USB_Rx_Frame_Seq;
    visual_last_valid_tick = 0u;
    visual_last_valid_pitch_deg = 0.0f;
    visual_last_valid_yaw_deg = 0.0f;
    visual_filtered_pitch_deg = 0.0f;
    visual_filtered_yaw_deg = 0.0f;
    visual_has_valid_target = 0u;
    visual_filter_tracking = 0u;
}

/**
 * @brief 清空当前输出的目标角。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::ClearOutput()
{
    raw_pitch_target = 0.0f;
    raw_yaw_target = 0.0f;
    pitch_target = 0.0f;
    yaw_target = 0.0f;
}

/**
 * @brief 结合视觉缓存、状态机和模式目标生成当前拍目标角。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Sentry_Target::Update(TickType_t now_tick)
{
    Gimbal_Sentry_Input_TypeDef sentry_input = {};
    Gimbal_Sentry_Output_TypeDef sentry_output = {};
    Gimbal_Sentry_State_TypeDef state;
    float return_step;

    if (initialized == 0u)
    {
        return;
    }

    sentry_input.vision_target_available = IsAvailable(now_tick);
    UpdateFilter(sentry_input.vision_target_available);
    sentry_input.vision_pitch_deg = visual_filtered_pitch_deg;
    sentry_input.vision_yaw_deg = visual_filtered_yaw_deg;

    scan_yaw_phase_rad = Gimbal_Sentry_Scan_Phase_Step(scan_yaw_phase_rad,
                                                       config.sentry_config.scan_yaw_frequency_hz,
                                                       config.sentry_config.dt_s);
    scan_pitch_phase_rad = Gimbal_Sentry_Scan_Phase_Step(scan_pitch_phase_rad,
                                                         config.sentry_config.scan_pitch_frequency_hz,
                                                         config.sentry_config.dt_s);
    scan_yaw_target_deg = Clamp(
        Gimbal_Sentry_Scan_Target_Sine(config.sentry_config.scan_yaw_amplitude_deg, scan_yaw_phase_rad),
        config.sentry_config.yaw_min_deg,
        config.sentry_config.yaw_max_deg);
    scan_pitch_target_deg = Clamp(
        Gimbal_Sentry_Scan_Target_Sine(config.sentry_config.scan_pitch_amplitude_deg, scan_pitch_phase_rad),
        config.sentry_config.pitch_min_deg,
        config.sentry_config.pitch_max_deg);

    sentry_input.lost_return_finished = 0u;
    if (sentry_handle.GetState() == GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN)
    {
        if ((fabsf(raw_pitch_target - scan_pitch_target_deg) <= config.sentry_config.lost_return_near_deg) &&
            (fabsf(raw_yaw_target - scan_yaw_target_deg) <= config.sentry_config.lost_return_near_deg))
        {
            sentry_input.lost_return_finished = 1u;
        }
    }

    sentry_handle.Update(&config.sentry_config, &sentry_input, &sentry_output);
    state = sentry_output.state;
    return_step = config.sentry_config.lost_return_speed_deg_s * config.sentry_config.dt_s;

    switch (state)
    {
    case GIMBAL_SENTRY_STATE_SCAN:
        raw_pitch_target = scan_pitch_target_deg;
        raw_yaw_target = scan_yaw_target_deg;
        break;

    case GIMBAL_SENTRY_STATE_TRACK_ARMOR:
        raw_pitch_target = Clamp(visual_filtered_pitch_deg,
                                 config.sentry_config.pitch_min_deg,
                                 config.sentry_config.pitch_max_deg);
        raw_yaw_target = Clamp(visual_filtered_yaw_deg,
                               config.sentry_config.yaw_min_deg,
                               config.sentry_config.yaw_max_deg);
        break;

    case GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN:
        raw_pitch_target = SlewLimit(scan_pitch_target_deg,
                                     raw_pitch_target,
                                     return_step);
        raw_yaw_target = SlewLimit(scan_yaw_target_deg,
                                   raw_yaw_target,
                                   return_step);
        break;

    default:
        sentry_handle.Reset();
        raw_pitch_target = scan_pitch_target_deg;
        raw_yaw_target = scan_yaw_target_deg;
        state = GIMBAL_SENTRY_STATE_SCAN;
        break;
    }

    raw_pitch_target = Clamp(raw_pitch_target,
                             config.sentry_config.pitch_min_deg,
                             config.sentry_config.pitch_max_deg);
    raw_yaw_target = Clamp(raw_yaw_target,
                           config.sentry_config.yaw_min_deg,
                           config.sentry_config.yaw_max_deg);
    pitch_target = -raw_pitch_target;
    yaw_target = raw_yaw_target;
    last_state = state;
}

/**
 * @brief 获取当前输出的 pitch 目标角。
 * @return pitch 目标角，单位：deg。
 */
float Class_Gimbal_Sentry_Target::GetPitch() const
{
    return pitch_target;
}

/**
 * @brief 获取当前输出的 yaw 目标角。
 * @return yaw 目标角，单位：deg。
 */
float Class_Gimbal_Sentry_Target::GetYaw() const
{
    return yaw_target;
}

/**
 * @brief 获取当前哨兵状态。
 * @return 当前状态机状态；模块未初始化时返回扫描态。
 */
Gimbal_Sentry_State_TypeDef Class_Gimbal_Sentry_Target::GetState() const
{
    if (initialized == 0u)
    {
        return GIMBAL_SENTRY_STATE_SCAN;
    }

    return sentry_handle.GetState();
}
