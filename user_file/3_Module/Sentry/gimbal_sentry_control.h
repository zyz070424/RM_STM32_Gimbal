/**
 * @file gimbal_sentry_control.h
 * @brief 哨兵模式控制参数管理对象定义。
 * @details
 * 本文件定义哨兵模式控制配置与
 * `Class_Gimbal_Sentry_Control` 管理对象。
 */
#ifndef __GIMBAL_SENTRY_CONTROL_H__
#define __GIMBAL_SENTRY_CONTROL_H__

#include "alg_pid.h"
#include "dvc_motor.h"
#include "gimbal_sentry.h"
#include <stdint.h>

#define GIMBAL_VISION_SHAPER_FILTER_TAU_S       0.003f
#define GIMBAL_YAW_VISION_SHAPER_SLEW_RATE      250.0f
#define GIMBAL_PITCH_VISION_SHAPER_SLEW_RATE    180.0f

#define GIMBAL_TARGET_RESET_YAW_DEG             10.0f
#define GIMBAL_TARGET_RESET_PITCH_DEG           8.0f

/**
 * @brief 哨兵模式控制链配置。
 */
typedef struct
{
    float pitch_angle_feedforward;       /**< 扫描/回扫态 pitch 角度环前馈 */
    float yaw_angle_feedforward;         /**< 扫描/回扫态 yaw 角度环前馈 */
    float sentry_shaper_filter_tau_s;    /**< 扫描/回扫态输出整形低通常数 */
    float pitch_sentry_shaper_slew_rate; /**< 扫描/回扫态 pitch 输出斜率限制 */
    float yaw_sentry_shaper_slew_rate;   /**< 扫描/回扫态 yaw 输出斜率限制 */
    float vision_shaper_filter_tau_s;    /**< 跟踪态输出整形低通常数 */
    float pitch_vision_shaper_slew_rate; /**< 跟踪态 pitch 输出斜率限制 */
    float yaw_vision_shaper_slew_rate;   /**< 跟踪态 yaw 输出斜率限制 */
    float target_reset_pitch_deg;        /**< 触发 pitch 角度环重置的目标突变量阈值 */
    float target_reset_yaw_deg;          /**< 触发 yaw 角度环重置的目标突变量阈值 */
} Gimbal_Sentry_Control_Config_TypeDef;

#ifdef __cplusplus
/**
 * @class Class_Gimbal_Sentry_Control
 * @brief 哨兵模式控制参数管理对象。
 * @details
 * 负责在扫描态、跟踪态和丢目标回扫态之间切换 PID 前馈与输出整形参数，
 * 同时维护角度环 PID 动态状态重置事件判定缓存。
 */
class Class_Gimbal_Sentry_Control
{
public:
    Gimbal_Sentry_Control_Config_TypeDef config;   /**< 控制模块配置缓存 */
    uint8_t initialized;                           /**< 控制模块初始化完成标志 */
    uint8_t params_applied;                        /**< 当前状态对应参数是否已下发 */
    Gimbal_Sentry_State_TypeDef last_param_state;  /**< 上次下发参数时的状态 */
    Gimbal_Sentry_State_TypeDef last_reset_state;  /**< 上次判定重置事件时的状态 */
    float last_pitch_target_deg;                   /**< 上次判定重置事件时的 pitch 目标角 */
    float last_yaw_target_deg;                     /**< 上次判定重置事件时的 yaw 目标角 */

    void Init(const Gimbal_Sentry_Control_Config_TypeDef *config);
    void ApplyModeParams(Class_Motor *pitch_motor,
                         Class_Motor *yaw_motor,
                         Gimbal_Sentry_State_TypeDef state);
    uint8_t AnglePidResetEventCheck(Gimbal_Sentry_State_TypeDef state,
                                    float pitch_target_deg,
                                    float yaw_target_deg);
    void ResetAnglePidDynamicState(Class_PID *pid,
                                   float target_now,
                                   float output_now);
};
extern Class_Gimbal_Sentry_Control Gimbal_Sentry_Control_Object;
#endif

#endif /* __GIMBAL_SENTRY_CONTROL_H__ */
