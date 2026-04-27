/**
 * @file gimbal_sentry.h
 * @brief 哨兵状态机接口定义。
 */
#ifndef __GIMBAL_SENTRY_H__
#define __GIMBAL_SENTRY_H__

#include <stdint.h>

/**
 * @brief 哨兵状态机运行状态。
 */
typedef enum
{
    GIMBAL_SENTRY_STATE_SCAN = 0,                /**< 无目标时按扫描轨迹搜索 */
    GIMBAL_SENTRY_STATE_TRACK_ARMOR,             /**< 视觉目标有效时直接跟踪目标角 */
    GIMBAL_SENTRY_STATE_LOST_TARGET_RETURN_SCAN, /**< 丢目标后平滑回到扫描轨迹 */
} Gimbal_Sentry_State_TypeDef;

/**
 * @brief 哨兵状态机配置参数。
 */
typedef struct
{
    float dt_s;                     /**< 控制周期，单位：s */
    float pitch_min_deg;            /**< pitch 最小安全角，单位：deg */
    float pitch_max_deg;            /**< pitch 最大安全角，单位：deg */
    float yaw_min_deg;              /**< yaw 最小安全角，单位：deg */
    float yaw_max_deg;              /**< yaw 最大安全角，单位：deg */
    float scan_pitch_amplitude_deg; /**< 扫描 pitch 正弦幅值，单位：deg */
    float scan_yaw_amplitude_deg;   /**< 扫描 yaw 正弦幅值，单位：deg */
    float scan_pitch_frequency_hz;  /**< 扫描 pitch 正弦频率，单位：Hz */
    float scan_yaw_frequency_hz;    /**< 扫描 yaw 正弦频率，单位：Hz */
    float lost_return_speed_deg_s;  /**< 丢目标回扫最大角速度，单位：deg/s */
    float lost_return_near_deg;     /**< 判断已贴近回扫目标的阈值，单位：deg */
} Gimbal_Sentry_Config_TypeDef;

/**
 * @brief 哨兵状态机的视觉输入。
 * @note  上层已经把 Target_Valid、最后一次有效目标保持和通信在线判断
 *        统一折叠为 vision_target_available；状态机只消费整理后的输入。
 */
typedef struct
{
    uint8_t vision_target_available; /**< 上层整理后的视觉目标有效标志 */
    float vision_pitch_deg;          /**< 当前视觉目标 pitch 角，单位：deg */
    float vision_yaw_deg;            /**< 当前视觉目标 yaw 角，单位：deg */
} Gimbal_Sentry_Input_TypeDef;

/**
 * @brief 哨兵状态机输出结果。
 */
typedef struct
{
    float pitch_target_deg;            /**< 输出给上层的 pitch 目标角，单位：deg */
    float yaw_target_deg;              /**< 输出给上层的 yaw 目标角，单位：deg */
    Gimbal_Sentry_State_TypeDef state; /**< 当前状态机状态 */
} Gimbal_Sentry_Output_TypeDef;

/**
 * @brief 哨兵状态机内部运行句柄。
 */
typedef struct
{
    Gimbal_Sentry_State_TypeDef state; /**< 当前状态机状态 */
    float scan_yaw_phase_rad;          /**< yaw 扫描正弦相位，单位：rad */
    float scan_pitch_phase_rad;        /**< pitch 扫描正弦相位，单位：rad */
    float scan_yaw_target_deg;         /**< 当前扫描轨迹上的 yaw 目标角，单位：deg */
    float scan_pitch_target_deg;       /**< 当前扫描轨迹上的 pitch 目标角，单位：deg */
    float lost_return_yaw_target_deg;   /**< 丢目标后冻结的 yaw 回扫目标角，单位：deg */
    float lost_return_pitch_target_deg; /**< 丢目标后冻结的 pitch 回扫目标角，单位：deg */
    float yaw_target_deg;               /**< 当前输出缓存中的 yaw 目标角，单位：deg */
    float pitch_target_deg;             /**< 当前输出缓存中的 pitch 目标角，单位：deg */
} Gimbal_Sentry_Handle_TypeDef;

/**
 * @brief 初始化哨兵状态机。
 * @param handle 哨兵状态机句柄。
 * @note  初始化后默认进入扫描态，并清零内部缓存。
 */
void Gimbal_Sentry_Init(Gimbal_Sentry_Handle_TypeDef *handle);

/**
 * @brief 复位哨兵状态机。
 * @param handle 哨兵状态机句柄。
 * @note  复位后默认进入扫描态，并清零内部缓存。
 */
void Gimbal_Sentry_Reset(Gimbal_Sentry_Handle_TypeDef *handle);

/**
 * @brief 更新哨兵状态机并生成当前拍目标角。
 * @param handle 哨兵状态机句柄。
 * @param config 状态机配置参数。
 * @param input 当前拍整理后的视觉输入。
 * @param output 输出的 pitch/yaw 目标角与状态。
 * @note  扫描相位会持续推进，丢目标后优先回到冻结的扫描点，再重新接回扫描轨迹。
 */
void Gimbal_Sentry_Update(Gimbal_Sentry_Handle_TypeDef *handle,
                          const Gimbal_Sentry_Config_TypeDef *config,
                          const Gimbal_Sentry_Input_TypeDef *input,
                          Gimbal_Sentry_Output_TypeDef *output);

/**
 * @brief 获取当前哨兵状态。
 * @param handle 哨兵状态机句柄。
 * @return 当前状态机状态；当句柄为空时返回扫描态。
 */
Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Get_State(const Gimbal_Sentry_Handle_TypeDef *handle);

#endif /* __GIMBAL_SENTRY_H__ */
