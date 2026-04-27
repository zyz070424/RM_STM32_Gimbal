/**
 * @file gimbal_sentry_target.h
 * @brief 哨兵目标生成模块接口定义。
 */
#ifndef __GIMBAL_SENTRY_TARGET_H__
#define __GIMBAL_SENTRY_TARGET_H__

#include "FreeRTOS.h"
#include "gimbal_sentry.h"
#include <stdint.h>

/**
 * @brief 哨兵目标生成模块配置。
 */
typedef struct
{
    float control_dt_s;                   /**< 控制周期，单位：s */
    uint32_t vision_track_timeout_ms;     /**< 视觉目标保持超时，单位：ms */
    float vision_target_filter_tau_s;     /**< 视觉目标一阶低通时间常数，单位：s */
    Gimbal_Sentry_Config_TypeDef sentry_config; /**< 底层状态机配置 */
} Gimbal_Sentry_Target_Config_TypeDef;

/**
 * @brief 初始化哨兵目标生成模块。
 * @param config 目标生成模块配置。
 */
void Gimbal_Sentry_Target_Init(const Gimbal_Sentry_Target_Config_TypeDef *config);

/**
 * @brief 复位哨兵模式状态。
 * @note 会同步复位视觉缓存和当前输出目标。
 */
void Gimbal_Sentry_Target_Reset_Mode(void);

/**
 * @brief 清空视觉目标缓存与滤波状态。
 */
void Gimbal_Sentry_Target_Reset_Vision(void);

/**
 * @brief 清空当前输出的云台目标角。
 */
void Gimbal_Sentry_Target_Clear_Output(void);

/**
 * @brief 按当前系统时刻更新哨兵目标。
 * @param now_tick 当前系统 tick。
 */
void Gimbal_Sentry_Target_Update(TickType_t now_tick);

/**
 * @brief 获取当前输出的 pitch 目标角。
 * @return pitch 目标角，单位：deg。
 */
float Gimbal_Sentry_Target_Get_Pitch(void);

/**
 * @brief 获取当前输出的 yaw 目标角。
 * @return yaw 目标角，单位：deg。
 */
float Gimbal_Sentry_Target_Get_Yaw(void);

/**
 * @brief 获取当前哨兵状态。
 * @return 当前状态机状态；模块未初始化时返回扫描态。
 */
Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Target_Get_State(void);

#endif /* __GIMBAL_SENTRY_TARGET_H__ */
