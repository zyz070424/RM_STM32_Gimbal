/**
 * @file gimbal_sentry.c
 * @brief 哨兵状态机实现。
 */
#include "gimbal_sentry.h"
#include <math.h>
#include <stdint.h>

/** @brief 扫描相位使用的 2pi 常量。 */
#define GIMBAL_SENTRY_TWO_PI 6.283185307f

/**
 * @brief 限幅输入值。
 * @param value 待限幅的值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 * @return 限幅后的值。
 */
static float Gimbal_Sentry_Clamp(float value, float min_value, float max_value)
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
 * @note 用在丢目标回扫阶段，避免从跟踪目标瞬间跳回扫描轨迹。
 */
static float Gimbal_Sentry_Move_Towards(float current, float target, float max_step)
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
static float Gimbal_Sentry_Wrap_Phase(float phase_rad)
{
    while (phase_rad >= GIMBAL_SENTRY_TWO_PI)
    {
        phase_rad -= GIMBAL_SENTRY_TWO_PI;
    }

    while (phase_rad < 0.0f)
    {
        phase_rad += GIMBAL_SENTRY_TWO_PI;
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
static float Gimbal_Sentry_Scan_Phase_Step(float phase_rad, float frequency_hz, float dt_s)
{
    phase_rad += GIMBAL_SENTRY_TWO_PI * frequency_hz * dt_s;
    return Gimbal_Sentry_Wrap_Phase(phase_rad);
}

/**
 * @brief 根据正弦波相位生成扫描目标角。
 * @param amplitude_deg 扫描幅值，单位：deg。
 * @param phase_rad 当前相位，单位：rad。
 * @return 扫描目标角，单位：deg。
 */
static float Gimbal_Sentry_Scan_Target_Sine(float amplitude_deg, float phase_rad)
{
    return amplitude_deg * sinf(phase_rad);
}

/**
 * @brief 将内部目标缓存拷贝到输出结构。
 * @param handle 哨兵状态机句柄。
 * @param output 输出结构体指针。
 */
static void Gimbal_Sentry_Fill_Output(const Gimbal_Sentry_Handle_TypeDef *handle,
                                      Gimbal_Sentry_Output_TypeDef *output)
{
    if (handle == NULL || output == NULL)
    {
        return;
    }

    output->pitch_target_deg = handle->pitch_target_deg;
    output->yaw_target_deg = handle->yaw_target_deg;
    output->state = handle->state;
}

/**
 * @brief 初始化哨兵状态机。
 * @param handle 哨兵状态机句柄。
 */
void Gimbal_Sentry_Init(Gimbal_Sentry_Handle_TypeDef *handle)
{
    Gimbal_Sentry_Reset(handle);
}

/**
 * @brief 复位哨兵状态机到默认扫描态。
 * @param handle 哨兵状态机句柄。
 */
void Gimbal_Sentry_Reset(Gimbal_Sentry_Handle_TypeDef *handle)
{
    if (handle == NULL)
    {
        return;
    }

    handle->state = GIMBAL_SENTRY_STATE_SCAN;
    handle->scan_yaw_phase_rad = 0.0f;
    handle->scan_pitch_phase_rad = 0.0f;
    handle->scan_yaw_target_deg = 0.0f;
    handle->scan_pitch_target_deg = 0.0f;
    handle->lost_return_yaw_target_deg = 0.0f;
    handle->lost_return_pitch_target_deg = 0.0f;
    handle->yaw_target_deg = 0.0f;
    handle->pitch_target_deg = 0.0f;
}

/**
 * @brief 根据当前状态和输入计算输出目标。
 * @param handle 哨兵状态机句柄。
 * @param config 哨兵配置参数。
 * @param input 哨兵输入，包含整理后的视觉目标信息。
 * @param output 哨兵输出，包含目标角与当前状态。
 * @note  1. 扫描轨迹持续推进，跟踪态丢目标后冻结轨迹点并进入回扫。
 * @note  2. 回扫态若重新捕获目标则优先恢复跟踪，否则平滑贴回扫描轨迹。
 * @note  3. 所有状态最终都会做一次物理/安全限幅。
 */
void Gimbal_Sentry_Update(Gimbal_Sentry_Handle_TypeDef *handle,
                          const Gimbal_Sentry_Config_TypeDef *config,
                          const Gimbal_Sentry_Input_TypeDef *input,
                          Gimbal_Sentry_Output_TypeDef *output)
{
    float return_step;

    if (handle == NULL || config == NULL || input == NULL)
    {
        return;
    }

    // 扫描相位始终推进：跟踪目标时也推进，丢目标后才能回到“此刻应该扫到的位置”
    handle->scan_yaw_phase_rad = Gimbal_Sentry_Scan_Phase_Step(handle->scan_yaw_phase_rad,
                                                                config->scan_yaw_frequency_hz,
                                                                config->dt_s);
    handle->scan_pitch_phase_rad = Gimbal_Sentry_Scan_Phase_Step(handle->scan_pitch_phase_rad,
                                                                  config->scan_pitch_frequency_hz,
                                                                  config->dt_s);
    handle->scan_yaw_target_deg = Gimbal_Sentry_Clamp(Gimbal_Sentry_Scan_Target_Sine(config->scan_yaw_amplitude_deg,
                                                                                      handle->scan_yaw_phase_rad),
                                                      config->yaw_min_deg,
                                                      config->yaw_max_deg);
    handle->scan_pitch_target_deg = Gimbal_Sentry_Clamp(Gimbal_Sentry_Scan_Target_Sine(config->scan_pitch_amplitude_deg,
                                                                                        handle->scan_pitch_phase_rad),
                                                        config->pitch_min_deg,
                                                        config->pitch_max_deg);

    return_step = config->lost_return_speed_deg_s * config->dt_s;

    switch (handle->state)
    {
        case GIMBAL_SENTRY_STATE_SCAN:
            // 搜索态：没有目标就输出扫描轨迹；有新鲜目标立即切跟踪
            if (input->vision_target_available != 0u)
            {
                handle->state = GIMBAL_SENTRY_STATE_TRACK_ARMOR;
                handle->pitch_target_deg = Gimbal_Sentry_Clamp(input->vision_pitch_deg,
                                                               config->pitch_min_deg,
                                                               config->pitch_max_deg);
                handle->yaw_target_deg = Gimbal_Sentry_Clamp(input->vision_yaw_deg,
                                                             config->yaw_min_deg,
                                                             config->yaw_max_deg);
            }
            else
            {
                handle->yaw_target_deg = handle->scan_yaw_target_deg;
                handle->pitch_target_deg = handle->scan_pitch_target_deg;
            }
            break;

        case GIMBAL_SENTRY_STATE_TRACK_ARMOR:
            // 跟踪态：目标还新鲜就直接跟随；目标超时则冻结扫描轨迹点并进入回扫
            if (input->vision_target_available != 0u)
            {
                handle->pitch_target_deg = Gimbal_Sentry_Clamp(input->vision_pitch_deg,
                                                               config->pitch_min_deg,
                                                               config->pitch_max_deg);
                handle->yaw_target_deg = Gimbal_Sentry_Clamp(input->vision_yaw_deg,
                                                             config->yaw_min_deg,
                                                             config->yaw_max_deg);
            }
            else
            {
                handle->state = GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN;
                handle->lost_return_yaw_target_deg = handle->scan_yaw_target_deg;
                handle->lost_return_pitch_target_deg = handle->scan_pitch_target_deg;
            }
            break;

        case GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN:
            // 回扫态：如果视觉重新捕获目标，优先回到跟踪；否则平滑贴回扫描轨迹
            if (input->vision_target_available != 0u)
            {
                handle->state = GIMBAL_SENTRY_STATE_TRACK_ARMOR;
                handle->pitch_target_deg = Gimbal_Sentry_Clamp(input->vision_pitch_deg,
                                                               config->pitch_min_deg,
                                                               config->pitch_max_deg);
                handle->yaw_target_deg = Gimbal_Sentry_Clamp(input->vision_yaw_deg,
                                                             config->yaw_min_deg,
                                                             config->yaw_max_deg);
                break;
            }

            handle->pitch_target_deg = Gimbal_Sentry_Move_Towards(handle->pitch_target_deg,
                                                                  handle->lost_return_pitch_target_deg,
                                                                  return_step);
            handle->yaw_target_deg = Gimbal_Sentry_Move_Towards(handle->yaw_target_deg,
                                                                handle->lost_return_yaw_target_deg,
                                                                return_step);

            if ((fabsf(handle->pitch_target_deg - handle->lost_return_pitch_target_deg) <= config->lost_return_near_deg) &&
                (fabsf(handle->yaw_target_deg - handle->lost_return_yaw_target_deg) <= config->lost_return_near_deg))
            {
                handle->state = GIMBAL_SENTRY_STATE_SCAN;
                handle->pitch_target_deg = handle->scan_pitch_target_deg;
                handle->yaw_target_deg = handle->scan_yaw_target_deg;
            }
            break;

        default:
            // 防御式处理：状态异常时回到安全的扫描状态
            Gimbal_Sentry_Reset(handle);
            break;
    }

    // 所有状态最终都做一次物理/安全限幅，避免上层或视觉目标越界
    handle->pitch_target_deg = Gimbal_Sentry_Clamp(handle->pitch_target_deg,
                                                   config->pitch_min_deg,
                                                   config->pitch_max_deg);
    handle->yaw_target_deg = Gimbal_Sentry_Clamp(handle->yaw_target_deg,
                                                 config->yaw_min_deg,
                                                 config->yaw_max_deg);
    Gimbal_Sentry_Fill_Output(handle, output);
}

/**
 * @brief 获取当前哨兵状态。
 * @param handle 哨兵状态机句柄。
 * @return 当前状态枚举值；当句柄为空时返回扫描态。
 */
Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Get_State(const Gimbal_Sentry_Handle_TypeDef *handle)
{
    if (handle == NULL)
    {
        return GIMBAL_SENTRY_STATE_SCAN;
    }

    return handle->state;
}
