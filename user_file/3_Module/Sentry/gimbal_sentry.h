/**
 * @file gimbal_sentry.h
 * @brief 哨兵状态机接口与管理对象定义。
 * @details
 * 本文件定义哨兵状态机配置、输入、输出与
 * `Class_Gimbal_Sentry` 管理对象。
 */
#ifndef __GIMBAL_SENTRY_H__
#define __GIMBAL_SENTRY_H__

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
} Gimbal_Sentry_Input_TypeDef;

/**
 * @brief 哨兵状态机输出。
 */
typedef struct
{
    float pitch_target_deg;              /**< 输出 pitch 目标角 */
    float yaw_target_deg;                /**< 输出 yaw 目标角 */
    Gimbal_Sentry_State_TypeDef state;   /**< 当前状态机状态 */
} Gimbal_Sentry_Output_TypeDef;

#ifdef __cplusplus
/**
 * @class Class_Gimbal_Sentry
 * @brief 哨兵状态机管理对象。
 * @details
 * 负责维护扫描相位、跟踪与丢目标回扫状态切换，
 * 并在每个控制周期内生成当前目标角输出。
 */
class Class_Gimbal_Sentry
{
public:
    Gimbal_Sentry_State_TypeDef state;   /**< 当前状态机状态 */
    float scan_yaw_phase_rad;            /**< yaw 扫描正弦相位，单位：rad */
    float scan_pitch_phase_rad;          /**< pitch 扫描正弦相位，单位：rad */
    float scan_yaw_target_deg;           /**< 当前扫描轨迹上的 yaw 目标角，单位：deg */
    float scan_pitch_target_deg;         /**< 当前扫描轨迹上的 pitch 目标角，单位：deg */
    float lost_return_yaw_target_deg;    /**< 丢目标后冻结的 yaw 回扫目标角，单位：deg */
    float lost_return_pitch_target_deg;  /**< 丢目标后冻结的 pitch 回扫目标角，单位：deg */
    float yaw_target_deg;                /**< 当前输出缓存中的 yaw 目标角，单位：deg */
    float pitch_target_deg;              /**< 当前输出缓存中的 pitch 目标角，单位：deg */

    void Init();
    void Reset();
    void Update(const Gimbal_Sentry_Config_TypeDef *config,
                const Gimbal_Sentry_Input_TypeDef *input,
                Gimbal_Sentry_Output_TypeDef *output);
    Gimbal_Sentry_State_TypeDef GetState() const;
};
#endif

#endif /* __GIMBAL_SENTRY_H__ */
