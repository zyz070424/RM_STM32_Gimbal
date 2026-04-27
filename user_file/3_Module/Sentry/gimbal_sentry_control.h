/**
 * @file gimbal_sentry_control.h
 * @brief 哨兵模式控制参数接口定义。
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
    float sentry_shaper_filter_tau_s;    /**< 扫描/回扫态输出整形低通时间常数，单位：s */
    float pitch_sentry_shaper_slew_rate; /**< 扫描/回扫态 pitch 输出斜率限制，单位：deg/s */
    float yaw_sentry_shaper_slew_rate;   /**< 扫描/回扫态 yaw 输出斜率限制，单位：deg/s */
    float vision_shaper_filter_tau_s;    /**< 跟踪态输出整形低通时间常数，单位：s */
    float pitch_vision_shaper_slew_rate; /**< 跟踪态 pitch 输出斜率限制，单位：deg/s */
    float yaw_vision_shaper_slew_rate;   /**< 跟踪态 yaw 输出斜率限制，单位：deg/s */
    float target_reset_pitch_deg;        /**< 触发 pitch 角度环重置的目标突变量阈值，单位：deg */
    float target_reset_yaw_deg;          /**< 触发 yaw 角度环重置的目标突变量阈值，单位：deg */
} Gimbal_Sentry_Control_Config_TypeDef;

/**
 * @brief 初始化哨兵控制模块。
 * @param config 哨兵控制配置。
 */
void Gimbal_Sentry_Control_Init(const Gimbal_Sentry_Control_Config_TypeDef *config);

/**
 * @brief 按当前哨兵状态切换控制链参数。
 * @param pitch_motor pitch 电机对象。
 * @param yaw_motor yaw 电机对象。
 * @param state 当前哨兵状态。
 */
void Gimbal_Sentry_Control_Apply_Mode_Params(Motor_TypeDef *pitch_motor,
                                             Motor_TypeDef *yaw_motor,
                                             Gimbal_Sentry_State_TypeDef state);

/**
 * @brief 检查当前拍是否需要重置角度环 PID 动态状态。
 * @param state 当前哨兵状态。
 * @param pitch_target_deg 当前 pitch 目标角，单位：deg。
 * @param yaw_target_deg 当前 yaw 目标角，单位：deg。
 * @return `1` 表示需要重置，`0` 表示不需要。
 */
uint8_t Gimbal_Sentry_Control_Angle_PID_Reset_Event_Check(Gimbal_Sentry_State_TypeDef state,
                                                          float pitch_target_deg,
                                                          float yaw_target_deg);

/**
 * @brief 重置角度环 PID 的动态状态。
 * @param pid 需要重置的 PID 对象。
 * @param target_now 当前目标角。
 * @param output_now 当前实际输出角。
 */
void Gimbal_Sentry_Control_Reset_Angle_PID_Dynamic_State(PID_TypeDef *pid,
                                                         float target_now,
                                                         float output_now);

#endif /* __GIMBAL_SENTRY_CONTROL_H__ */
