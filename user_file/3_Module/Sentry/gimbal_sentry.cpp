/**
 * @file gimbal_sentry.cpp
 * @brief 哨兵状态机实现。
 * @details
 * 本文件实现 `Class_Gimbal_Sentry` 的成员函数。
 */
#include "gimbal_sentry.h"

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
}

/**
 * @brief 根据当前输入推进状态机。
 * @param config 哨兵配置参数，当前状态机阶段未直接使用。
 * @param input 哨兵输入，包含视觉可用性与回扫完成标志。
 * @param output 哨兵输出，包含当前状态。
 * @return 无。
 */
void Class_Gimbal_Sentry::Update(const Gimbal_Sentry_Config_TypeDef *config,
                                 const Gimbal_Sentry_Input_TypeDef *input,
                                 Gimbal_Sentry_Output_TypeDef *output)
{
    (void)config;

    if (input == nullptr)
    {
        return;
    }

    switch (state)
    {
    case GIMBAL_SENTRY_STATE_SCAN:
        if (input->vision_target_available != 0u)
        {
            state = GIMBAL_SENTRY_STATE_TRACK_ARMOR;
        }
        break;

    case GIMBAL_SENTRY_STATE_TRACK_ARMOR:
        if (input->vision_target_available == 0u)
        {
            state = GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN;
        }
        break;

    case GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN:
        if (input->vision_target_available != 0u)
        {
            state = GIMBAL_SENTRY_STATE_TRACK_ARMOR;
        }
        else if (input->lost_return_finished != 0u)
        {
            state = GIMBAL_SENTRY_STATE_SCAN;
        }
        break;

    default:
        Reset();
        break;
    }

    if (output != nullptr)
    {
        output->state = state;
    }
}

/**
 * @brief 获取当前哨兵状态。
 * @return 当前状态机状态。
 */
Gimbal_Sentry_State_TypeDef Class_Gimbal_Sentry::GetState() const
{
    return state;
}
