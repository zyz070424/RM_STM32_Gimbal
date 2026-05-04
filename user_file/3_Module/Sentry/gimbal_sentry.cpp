/**
 * @file gimbal_sentry.cpp
 * @brief 哨兵状态机实现。
 * @details
 * 本文件实现 `Class_Gimbal_Sentry` 的成员函数。
 */
#include "gimbal_sentry.h"

#include <math.h>
#include <stdint.h>

namespace
{
constexpr float GimbalSentryTwoPi = 6.283185307f;

/**
 * @brief 限幅输入值。
 * @param value 待限幅的值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 * @return 限幅后的值。
 */
float Gimbal_Sentry_Clamp(float value, float min_value, float max_value)
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
 * @brief 以固定最大步长逼近目标值。
 * @param current 当前值。
 * @param target 目标值。
 * @param max_step 单次允许的最大变化量。
 * @return 逼近后的结果。
 */
float Gimbal_Sentry_Move_Towards(float current, float target, float max_step)
{
    if (target > current + max_step)
    {
        return current + max_step;
    }

    if (target < current - max_step)
    {
        return current - max_step;
    }

    return target;
}

/**
 * @brief 将相位约束到 `[0, 2pi)` 区间。
 * @param phase_rad 原始相位，单位：rad。
 * @return 包装后的相位，单位：rad。
 */
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

/**
 * @brief 推进单轴扫描正弦波相位。
 * @param phase_rad 当前相位，单位：rad。
 * @param frequency_hz 扫描频率，单位：Hz。
 * @param dt_s 控制周期，单位：s。
 * @return 推进后的相位，单位：rad。
 */
float Gimbal_Sentry_Scan_Phase_Step(float phase_rad, float frequency_hz, float dt_s)
{
    phase_rad += GimbalSentryTwoPi * frequency_hz * dt_s;
    return Gimbal_Sentry_Wrap_Phase(phase_rad);
}

/**
 * @brief 根据正弦波相位生成扫描目标角。
 * @param amplitude_deg 扫描幅值，单位：deg。
 * @param phase_rad 当前相位，单位：rad。
 * @return 扫描目标角，单位：deg。
 */
float Gimbal_Sentry_Scan_Target_Sine(float amplitude_deg, float phase_rad)
{
    return amplitude_deg * sinf(phase_rad);
}

/**
 * @brief 将内部目标缓存拷贝到输出结构。
 * @param handle 哨兵状态机句柄。
 * @param output 输出结构体指针。
 * @return 无。
 */
void Gimbal_Sentry_Fill_Output(const Class_Gimbal_Sentry *handle,
                               Gimbal_Sentry_Output_TypeDef *output)
{
    if ((handle == nullptr) || (output == nullptr))
    {
        return;
    }

    output->pitch_target_deg = handle->pitch_target_deg;
    output->yaw_target_deg = handle->yaw_target_deg;
    output->state = handle->state;
}
}

/**
 * @brief 初始化哨兵状态机。
 * @return 无。
 */
void Class_Gimbal_Sentry::Init()
{
    Reset();
}

/**
 * @brief 复位哨兵状态机到默认扫描态。
 * @return 无。
 */
void Class_Gimbal_Sentry::Reset()
{
    state = GIMBAL_SENTRY_STATE_SCAN;
    scan_yaw_phase_rad = 0.0f;
    scan_pitch_phase_rad = 0.0f;
    scan_yaw_target_deg = 0.0f;
    scan_pitch_target_deg = 0.0f;
    lost_return_yaw_target_deg = 0.0f;
    lost_return_pitch_target_deg = 0.0f;
    yaw_target_deg = 0.0f;
    pitch_target_deg = 0.0f;
}

/**
 * @brief 根据当前状态和输入计算输出目标。
 * @param config 哨兵配置参数。
 * @param input 哨兵输入，包含整理后的视觉目标信息。
 * @param output 哨兵输出，包含目标角与当前状态。
 * @return 无。
 */
void Class_Gimbal_Sentry::Update(const Gimbal_Sentry_Config_TypeDef *config,
                                 const Gimbal_Sentry_Input_TypeDef *input,
                                 Gimbal_Sentry_Output_TypeDef *output)
{
    float return_step;

    if ((config == nullptr) || (input == nullptr))
    {
        return;
    }

    scan_yaw_phase_rad = Gimbal_Sentry_Scan_Phase_Step(scan_yaw_phase_rad,
                                                       config->scan_yaw_frequency_hz,
                                                       config->dt_s);
    scan_pitch_phase_rad = Gimbal_Sentry_Scan_Phase_Step(scan_pitch_phase_rad,
                                                         config->scan_pitch_frequency_hz,
                                                         config->dt_s);
    scan_yaw_target_deg = Gimbal_Sentry_Clamp(
        Gimbal_Sentry_Scan_Target_Sine(config->scan_yaw_amplitude_deg, scan_yaw_phase_rad),
        config->yaw_min_deg,
        config->yaw_max_deg);
    scan_pitch_target_deg = Gimbal_Sentry_Clamp(
        Gimbal_Sentry_Scan_Target_Sine(config->scan_pitch_amplitude_deg, scan_pitch_phase_rad),
        config->pitch_min_deg,
        config->pitch_max_deg);

    return_step = config->lost_return_speed_deg_s * config->dt_s;

    switch (state)
    {
    case GIMBAL_SENTRY_STATE_SCAN:
        if (input->vision_target_available != 0u)
        {
            state = GIMBAL_SENTRY_STATE_TRACK_ARMOR;
            pitch_target_deg = Gimbal_Sentry_Clamp(input->vision_pitch_deg,
                                                   config->pitch_min_deg,
                                                   config->pitch_max_deg);
            yaw_target_deg = Gimbal_Sentry_Clamp(input->vision_yaw_deg,
                                                 config->yaw_min_deg,
                                                 config->yaw_max_deg);
        }
        else
        {
            yaw_target_deg = scan_yaw_target_deg;
            pitch_target_deg = scan_pitch_target_deg;
        }
        break;

    case GIMBAL_SENTRY_STATE_TRACK_ARMOR:
        if (input->vision_target_available != 0u)
        {
            pitch_target_deg = Gimbal_Sentry_Clamp(input->vision_pitch_deg,
                                                   config->pitch_min_deg,
                                                   config->pitch_max_deg);
            yaw_target_deg = Gimbal_Sentry_Clamp(input->vision_yaw_deg,
                                                 config->yaw_min_deg,
                                                 config->yaw_max_deg);
        }
        else
        {
            state = GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN;
            lost_return_yaw_target_deg = scan_yaw_target_deg;
            lost_return_pitch_target_deg = scan_pitch_target_deg;
        }
        break;

    case GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN:
        if (input->vision_target_available != 0u)
        {
            state = GIMBAL_SENTRY_STATE_TRACK_ARMOR;
            pitch_target_deg = Gimbal_Sentry_Clamp(input->vision_pitch_deg,
                                                   config->pitch_min_deg,
                                                   config->pitch_max_deg);
            yaw_target_deg = Gimbal_Sentry_Clamp(input->vision_yaw_deg,
                                                 config->yaw_min_deg,
                                                 config->yaw_max_deg);
            break;
        }

        pitch_target_deg = Gimbal_Sentry_Move_Towards(pitch_target_deg,
                                                      scan_pitch_target_deg,
                                                      return_step);
        yaw_target_deg = Gimbal_Sentry_Move_Towards(yaw_target_deg,
                                                    scan_yaw_target_deg,
                                                    return_step);

        if ((fabsf(pitch_target_deg - scan_pitch_target_deg) <= config->lost_return_near_deg) &&
            (fabsf(yaw_target_deg - scan_yaw_target_deg) <= config->lost_return_near_deg))
        {
            state = GIMBAL_SENTRY_STATE_SCAN;
            pitch_target_deg = scan_pitch_target_deg;
            yaw_target_deg = scan_yaw_target_deg;
        }
        break;

    default:
        Reset();
        break;
    }

    pitch_target_deg = Gimbal_Sentry_Clamp(pitch_target_deg,
                                           config->pitch_min_deg,
                                           config->pitch_max_deg);
    yaw_target_deg = Gimbal_Sentry_Clamp(yaw_target_deg,
                                         config->yaw_min_deg,
                                         config->yaw_max_deg);
    Gimbal_Sentry_Fill_Output(this, output);
}

/**
 * @brief 获取当前哨兵状态。
 * @return 当前状态机状态。
 */
Gimbal_Sentry_State_TypeDef Class_Gimbal_Sentry::GetState() const
{
    return state;
}
