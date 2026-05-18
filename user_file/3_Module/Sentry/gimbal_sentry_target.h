/**
 * @file gimbal_sentry_target.h
 * @brief 哨兵目标生成模块接口与管理对象定义。
 * @details
 * 本文件定义哨兵目标生成配置与
 * `Class_Gimbal_Sentry_Target` 管理对象。
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
    float control_dt_s;                     /**< 控制周期，单位秒 */
    uint32_t vision_track_timeout_ms;       /**< 视觉跟踪超时阈值 */
    float vision_target_filter_tau_s;       /**< 视觉目标一阶滤波时间常数 */
    Gimbal_Sentry_Config_TypeDef sentry_config; /**< 内部状态机配置 */
} Gimbal_Sentry_Target_Config_TypeDef;

#ifdef __cplusplus
/**
 * @class Class_Gimbal_Sentry_Target
 * @brief 哨兵目标生成管理对象。
 * @details
 * 负责维护视觉目标缓存与滤波状态，并根据当前哨兵状态生成当前目标角。
 */
class Class_Gimbal_Sentry_Target
{
public:
    Gimbal_Sentry_Target_Config_TypeDef config;  /**< 模块配置缓存 */
    uint8_t initialized;                         /**< 模块初始化完成标志 */
    Class_Gimbal_Sentry sentry_handle;           /**< 底层哨兵状态机实例 */
    Gimbal_Sentry_State_TypeDef last_state;      /**< 上一拍哨兵状态缓存 */
    uint32_t visual_last_rx_frame_seq;           /**< 最近处理过的视觉帧序号 */
    TickType_t visual_last_valid_tick;           /**< 最近一次收到有效目标的时间戳 */
    float visual_last_valid_pitch_deg;           /**< 最近一次有效视觉 pitch，单位：deg */
    float visual_last_valid_yaw_deg;             /**< 最近一次有效视觉 yaw，单位：deg */
    float visual_filtered_pitch_deg;             /**< 视觉滤波后的 pitch，单位：deg */
    float visual_filtered_yaw_deg;               /**< 视觉滤波后的 yaw，单位：deg */
    uint8_t visual_has_valid_target;             /**< 是否已经缓存过有效视觉目标 */
    uint8_t visual_filter_tracking;              /**< 视觉滤波器是否已进入跟踪状态 */
    float scan_yaw_phase_rad;                    /**< yaw 扫描正弦相位，单位：rad */
    float scan_pitch_phase_rad;                  /**< pitch 扫描正弦相位，单位：rad */
    float scan_yaw_target_deg;                   /**< 当前扫描轨迹上的 yaw 目标角，单位：deg */
    float scan_pitch_target_deg;                 /**< 当前扫描轨迹上的 pitch 目标角，单位：deg */
    float raw_pitch_target;                      /**< sentry 内部 pitch 目标角，单位：deg */
    float raw_yaw_target;                        /**< sentry 内部 yaw 目标角，单位：deg */
    float pitch_target;                          /**< 输出给云台的 pitch 目标角，单位：deg */
    float yaw_target;                            /**< 输出给云台的 yaw 目标角，单位：deg */

    void Init(const Gimbal_Sentry_Target_Config_TypeDef *config);
    void ResetMode();
    void ResetVision();
    void ClearOutput();
    void Update(TickType_t now_tick);
    float GetPitch() const;
    float GetYaw() const;
    Gimbal_Sentry_State_TypeDef GetState() const;

private:
    void UpdateCache(TickType_t now_tick);
    uint8_t IsAvailable(TickType_t now_tick);
    void UpdateFilter(uint8_t target_available);
};
extern Class_Gimbal_Sentry_Target Gimbal_Sentry_Target_Object;
#endif

#endif /* __GIMBAL_SENTRY_TARGET_H__ */
