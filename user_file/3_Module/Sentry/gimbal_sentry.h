/**
 * @file gimbal_sentry.h
 * @brief 哨兵状态机接口与管理对象定义。
 * @details
 * 本文件定义哨兵状态机配置、输入、输出与
 * `Class_Gimbal_Sentry` 管理对象。
 */
#ifndef __GIMBAL_SENTRY_H__
#define __GIMBAL_SENTRY_H__

#include "common_math.h"
#include <stdint.h>

/**
 * @brief 哨兵状态机状态枚举。
 */
typedef enum
{
    GIMBAL_SENTRY_STATE_SCAN = 0,
    GIMBAL_SENTRY_STATE_TRACK_ARMOR,
    GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN,
} Gimbal_Sentry_State_TypeDef;

/**
 * @brief 哨兵状态机配置。
 */
typedef struct
{
    float dt_s;                      /**< 控制周期，单位秒 */
    float pitch_min_deg;             /**< pitch 最小角度 */
    float pitch_max_deg;             /**< pitch 最大角度 */
    float yaw_min_deg;               /**< yaw 最小角度 */
    float yaw_max_deg;               /**< yaw 最大角度 */
    float scan_pitch_amplitude_deg;  /**< 扫描 pitch 幅值 */
    float scan_yaw_amplitude_deg;    /**< 扫描 yaw 幅值 */
    float scan_pitch_frequency_hz;   /**< 扫描 pitch 频率 */
    float scan_yaw_frequency_hz;     /**< 扫描 yaw 频率 */
    float lost_return_speed_deg_s;   /**< 丢目标回扫角速度 */
    float lost_return_near_deg;      /**< 回扫到达判定阈值 */
} Gimbal_Sentry_Config_TypeDef;

/**
 * @brief 哨兵状态机输入。
 */
typedef struct
{
    uint8_t vision_target_available; /**< 当前是否存在视觉目标 */
    float vision_pitch_deg;          /**< 当前视觉 pitch */
    float vision_yaw_deg;            /**< 当前视觉 yaw */
    uint8_t lost_return_finished;    /**< 当前丢目标回扫是否已回到扫描轨迹 */
} Gimbal_Sentry_Input_TypeDef;

/**
 * @brief 哨兵状态机输出。
 */
typedef struct
{
    Gimbal_Sentry_State_TypeDef state;   /**< 当前状态机状态 */
} Gimbal_Sentry_Output_TypeDef;

#ifdef __cplusplus
/**
 * @class Class_Gimbal_Sentry
 * @brief 哨兵状态机管理对象。
 * @details
 * 只负责维护扫描、跟踪与丢目标回扫之间的状态切换，
 * 不直接生成目标角输出。
 */
class Class_Gimbal_Sentry
{
public:
    Gimbal_Sentry_State_TypeDef state;   /**< 当前状态机状态 */

    void Init();
    void Reset();
    void Update(const Gimbal_Sentry_Config_TypeDef *config,
                const Gimbal_Sentry_Input_TypeDef *input,
                Gimbal_Sentry_Output_TypeDef *output);
    Gimbal_Sentry_State_TypeDef GetState() const;
};
#endif

#endif /* __GIMBAL_SENTRY_H__ */
