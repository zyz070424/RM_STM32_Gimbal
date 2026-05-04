/**
 * @file dvc_motor_protect.h
 * @brief 电机堵转保护接口与 Pitch 保护包装器定义。
 * @details
 * 本文件定义通用堵转保护状态机所需的配置、输入与管理对象类型。
 */
#ifndef __DVC_MOTOR_PROTECT_H__
#define __DVC_MOTOR_PROTECT_H__

#include <stdint.h>

/**
 * @brief 堵转保护状态枚举。
 */
typedef enum
{
    MOTOR_PROTECT_STATE_NORMAL = 0,
    MOTOR_PROTECT_STATE_BACKOFF,
    MOTOR_PROTECT_STATE_COOLDOWN,
    MOTOR_PROTECT_STATE_FAULT,
} Motor_Protect_State_TypeDef;

/**
 * @brief 堵转保护配置结构。
 * @details
 * 保存堵转判定阈值、回退窗口时长、冷却时间和重试次数限制。
 */
typedef struct Motor_Protect_Config_TypeDef
{
    uint8_t enable;          /**< 是否启用堵转保护 */
    uint8_t allow_backoff;   /**< 是否允许进入回退状态 */
    uint8_t has_mech_limit;  /**< 是否存在机械限位 */
    float cmd_th;            /**< 指令绝对值阈值 */
    float torque_th;         /**< 力矩绝对值阈值 */
    float speed_th_rad_s;    /**< 速度绝对值阈值 */
    float err_th_deg;        /**< 位置误差绝对值阈值 */
    uint16_t blank_ms;       /**< 初始化后空白保护时间 */
    uint16_t confirm_ms;     /**< 堵转确认时间 */
    uint16_t backoff_ms;     /**< 回退持续时间 */
    uint16_t cooldown_ms;    /**< 冷却持续时间 */
    uint16_t retry_reset_ms; /**< 自动清零重试计数的时间窗口 */
    float backoff_cmd_limit; /**< 回退阶段输出限幅 */
    uint8_t retry_limit;     /**< 最大重试次数 */
} Motor_Protect_Config_TypeDef;

/**
 * @brief 堵转保护输入结构。
 * @details
 * 每次更新时传入的运行反馈，用于投票判定是否发生堵转。
 */
typedef struct Motor_Protect_Input_TypeDef
{
    uint16_t dt_ms;          /**< 本次更新的时间步长 */
    float pos_err_deg;       /**< 当前角度误差 */
    float speed_rad_s;       /**< 当前转速 */
    float cmd;               /**< 当前控制指令 */
    int16_t torque_raw;      /**< 当前力矩反馈原始值 */
    uint8_t near_limit;      /**< 当前是否靠近机械限位 */
    uint8_t pushing_outward; /**< 当前是否继续向外顶机械限位 */
} Motor_Protect_Input_TypeDef;

#ifdef __cplusplus
/**
 * @class Class_Motor_Protect
 * @brief 通用电机堵转保护管理对象。
 * @details
 * 负责维护堵转检测、回退、冷却和故障状态机，
 * 并提供输出限幅和回退请求消费接口。
 */
class Class_Motor_Protect
{
public:
    Motor_Protect_State_TypeDef state;  /**< 当前保护状态 */
    uint16_t state_ms;                  /**< 当前状态已持续的时间 */
    uint16_t stall_ms;                  /**< 当前堵转累计判定时间 */
    uint8_t retry_count;                /**< 当前累计重试次数 */
    uint8_t backoff_request;            /**< 是否请求生成回退目标 */
    float last_cmd;                     /**< 最近一次输入控制量 */

    void Init();
    void Reset();
    void Blank();
    void ClearFault();
    void Update(const Motor_Protect_Config_TypeDef *config,
                const Motor_Protect_Input_TypeDef *input);
    float ApplyOutput(const Motor_Protect_Config_TypeDef *config, float raw_cmd) const;
    uint8_t ConsumeBackoffRequest();
    uint8_t IsFault() const;
};
#endif

#ifdef __cplusplus
/**
 * @class Class_Motor_Protect_Pitch
 * @brief Pitch 轴堵转保护包装器。
 * @details
 * 该对象在通用堵转保护状态机基础上增加 Pitch 机械限位判断、
 * 回退目标生成和复位请求管理。
 */
class Class_Motor_Protect_Pitch
{
public:
    void Init(float min_angle_deg,
              float max_angle_deg,
              uint8_t cmd_positive_to_angle_positive_value);
    void SetActive(uint8_t active_value);
    uint8_t IsActive() const;
    float ApplyTarget(float raw_target_deg) const;
    float UpdateOutput(float target_deg,
                       float feedback_deg,
                       float speed_rad_s,
                       float raw_cmd,
                       int16_t torque_raw,
                       uint16_t dt_ms);
    uint8_t TakeResetRequest(float *reset_target_deg);
    void Blank();
    void ClearFault();
    uint8_t IsFault() const;

private:
    uint8_t NearLimit(float feedback_deg) const;
    uint8_t PushingOutward(float feedback_deg, float raw_cmd) const;
    float MakeBackoffTarget(float feedback_deg, float raw_cmd) const;

    Class_Motor_Protect core;                   /**< 通用堵转保护核心 */
    Motor_Protect_Config_TypeDef cfg;           /**< Pitch 轴堵转保护配置 */
    uint8_t initialized;                        /**< 是否已完成初始化 */
    uint8_t active;                             /**< 当前是否启用 Pitch 保护 */
    uint8_t cmd_positive_to_angle_positive;     /**< 指令正方向是否对应角度正方向 */
    uint8_t reset_request;                      /**< 是否请求外层复位角度环 */
    float min_angle_deg;                        /**< Pitch 最小机械角度 */
    float max_angle_deg;                        /**< Pitch 最大机械角度 */
    float backoff_target_deg;                   /**< 回退阶段使用的目标角度 */
};

extern Class_Motor_Protect_Pitch Motor_Protect_Pitch_Object;
#endif

#endif /* __DVC_MOTOR_PROTECT_H__ */
