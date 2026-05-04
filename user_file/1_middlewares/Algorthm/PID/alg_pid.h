/**
 * @file alg_pid.h
 * @brief PID 控制器类定义。
 * @details
 * 本文件定义 `Class_PID` PID 控制器对象。
 */
#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "main.h"
#include <stdbool.h>
#include <math.h>
#include <sys/types.h>


#ifdef __cplusplus
/**
 * @class Class_PID
 * @brief PID 控制器对象。
 * @details
 * 该对象维护 PID 配置参数、运行时状态以及控制计算能力。
 */
class Class_PID
{
public:
    float Kp;                        /**< 比例系数 */
    float Ki;                        /**< 积分系数 */
    float Kd;                        /**< 微分系数 */
    float FeedForward;               /**< 前馈系数 */
    float P_out;                     /**< 本次计算得到的比例输出 */
    float I_out;                     /**< 本次计算得到的积分输出 */
    float D_out;                     /**< 本次计算得到的微分输出 */
    float FeedForward_out;           /**< 本次计算得到的前馈输出 */
    float Friction_Compensation_out; /**< 本次计算得到的摩擦补偿输出 */
    float Gravity_Compensation_out;  /**< 本次计算得到的重力补偿输出 */
    float target;                    /**< 当前控制目标值 */
    float prev_target;               /**< 上一次控制目标值 */
    float output;                    /**< 当前总输出 */
    float Input;                     /**< 当前输入值 */
    float error;                     /**< 当前误差 */
    float prev_error;                /**< 上一次误差 */
    float integral;                  /**< 当前积分累计值 */
    float out_min;                   /**< 输出最小限幅 */
    float out_max;                   /**< 输出最大限幅 */
    float integral_min;              /**< 积分项最小限幅 */
    float integral_max;              /**< 积分项最大限幅 */
    bool target_limit_enable;        /**< 是否启用目标值限幅 */
    float target_limit_min;          /**< 目标值下限 */
    float target_limit_max;          /**< 目标值上限 */
    bool integral_separation_enable; /**< 是否启用变速积分 */
    float integral_separation_threshold_A; /**< 变速积分强抑制阈值 */
    float integral_separation_threshold_B; /**< 变速积分弱抑制阈值 */
    bool differential_enable;        /**< 是否启用微分先行 */
    bool deadband_enable;            /**< 是否启用死区 */
    float deadband;                  /**< 死区阈值 */
    bool friction_comp_enable;       /**< 是否启用摩擦补偿 */
    float friction_fc;               /**< 库仑摩擦补偿系数 */
    float friction_bv;               /**< 粘性摩擦补偿系数 */
    float friction_w_eps;            /**< 摩擦补偿平滑参数 */
    bool friction_use_target_omega;  /**< 摩擦补偿是否使用目标速度 */
    bool gravity_comp_enable;        /**< 是否启用重力补偿 */
    float gravity_comp_kg;           /**< 重力补偿系数 */
    float gravity_comp_angle_deg;    /**< 重力补偿使用的姿态角，单位度 */
    bool gravity_comp_reverse;       /**< 是否反向输出重力补偿 */
    bool output_filter_enable;       /**< 是否启用输出低通整形 */
    float output_filter_tau_s;       /**< 输出低通时间常数，单位秒 */
    bool output_slew_enable;         /**< 是否启用输出斜率限制 */
    float output_slew_rate;          /**< 输出最大变化率，单位每秒 */
    bool output_shaper_inited;       /**< 输出整形状态是否已初始化 */
    float output_shaper_state;       /**< 当前输出整形内部状态 */
    float dt;                        /**< 本次控制周期，单位秒 */

    void Init();
    void DifferentialEnable(bool enable);
    void DeadbandEnable(bool enable, float deadband_value);
    void FrictionCompensationEnable(bool enable, float fc, float bv, float w_eps, bool use_target_omega);
    void GravityCompensationEnable(bool enable, float kg, bool reverse_output);
    void GravityCompensationSetAngle(float angle_deg);
    void OutputFilterEnable(bool enable, float tau_s);
    void OutputSlewEnable(bool enable, float slew_rate);
    void OutputShaperReset(float init_output);
    void IntegralSeparationEnable(bool enable, float threshold_a, float threshold_b);
    void TargetLimitEnable(bool enable, float min, float max);
    void SetParameters(float p, float i, float d, float feed_forward,
                       float integral_min_value, float integral_max_value,
                       float out_min_value, float out_max_value);
    float Calculate(float input_value, float target_value, float dt_value);
};
#endif

#endif /* __ALG_PID_H__ */
