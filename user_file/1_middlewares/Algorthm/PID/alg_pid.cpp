/**
 * @file alg_pid.cpp
 * @brief PID 控制器实现。
 * @details
 * 本文件实现 `Class_PID` 的成员函数。
 */
#include "alg_pid.h"

#define PID_OUTPUT_SLEW_RELEASE_GAIN 8.0f
#define PID_DEG_TO_RAD 0.01745329252f

namespace
{
/**
 * @brief 计算摩擦补偿输出。
 * @param pid PID 控制器对象指针。
 * @return 当前摩擦补偿输出值。
 * @details
 * 补偿模型为 `fc * tanh(omega / w_eps) + bv * omega`。
 * 当未启用摩擦补偿或对象指针为空时，直接返回零。
 */
float PID_Get_Friction_Compensation(const Class_PID *pid)
{
    float omega;
    float w_eps;

    if ((pid == NULL) || (pid->friction_comp_enable == false))
    {
        return 0.0f;
    }

    omega = pid->friction_use_target_omega ? pid->target : pid->Input;
    w_eps = fabsf(pid->friction_w_eps);
    if (w_eps < 1e-6f)
    {
        w_eps = 1e-6f;
    }

    return pid->friction_fc * tanhf(omega / w_eps) + pid->friction_bv * omega;
}

/**
 * @brief 计算重力补偿输出。
 * @param pid PID 控制器对象指针。
 * @return 当前重力补偿输出值。
 * @details
 * 补偿模型为 `kg * sin(theta)`。
 * 当未启用重力补偿或对象指针为空时，直接返回零。
 */
float PID_Get_Gravity_Compensation(const Class_PID *pid)
{
    float gravity_ff;

    if ((pid == NULL) || (pid->gravity_comp_enable == false))
    {
        return 0.0f;
    }

    gravity_ff = pid->gravity_comp_kg * sinf(pid->gravity_comp_angle_deg * PID_DEG_TO_RAD);
    if (pid->gravity_comp_reverse != false)
    {
        gravity_ff = -gravity_ff;
    }

    return gravity_ff;
}
}

/**
 * @brief 初始化 PID 对象。
 * @return 无。
 * @details
 * 清空全部参数与运行时状态，使控制器回到未配置初始状态。
 */
void Class_PID::Init()
{
    Kp = 0.0f;
    Ki = 0.0f;
    Kd = 0.0f;
    FeedForward = 0.0f;

    P_out = 0.0f;
    I_out = 0.0f;
    D_out = 0.0f;
    FeedForward_out = 0.0f;
    Friction_Compensation_out = 0.0f;
    Gravity_Compensation_out = 0.0f;

    target = 0.0f;
    prev_target = 0.0f;
    output = 0.0f;
    Input = 0.0f;

    error = 0.0f;
    prev_error = 0.0f;
    integral = 0.0f;

    out_min = 0.0f;
    out_max = 0.0f;

    integral_min = 0.0f;
    integral_max = 0.0f;

    target_limit_enable = false;
    target_limit_min = 0.0f;
    target_limit_max = 0.0f;

    integral_separation_enable = false;
    integral_separation_threshold_A = 0.0f;
    integral_separation_threshold_B = 0.0f;

    differential_enable = false;

    deadband_enable = false;
    deadband = 0.0f;

    friction_comp_enable = false;
    friction_fc = 0.0f;
    friction_bv = 0.0f;
    friction_w_eps = 1.0f;
    friction_use_target_omega = true;

    gravity_comp_enable = false;
    gravity_comp_kg = 0.0f;
    gravity_comp_angle_deg = 0.0f;
    gravity_comp_reverse = false;

    output_filter_enable = false;
    output_filter_tau_s = 0.0f;
    output_slew_enable = false;
    output_slew_rate = 0.0f;
    output_shaper_inited = false;
    output_shaper_state = 0.0f;

    dt = 0.0f;
}

/**
 * @brief 配置微分先行开关。
 * @param enable true 表示启用微分先行，false 表示关闭。
 * @return 无。
 * @details 启用后微分项对目标变化做响应，而不是对误差变化做响应。
 */
void Class_PID::DifferentialEnable(bool enable)
{
    differential_enable = enable;
}

/**
 * @brief 配置死区处理。
 * @param enable true 表示启用死区。
 * @param deadband_value 死区阈值。
 * @return 无。
 * @details 当误差绝对值不超过死区阈值时，控制器输出会被直接压为零。
 */
void Class_PID::DeadbandEnable(bool enable, float deadband_value)
{
    deadband_enable = enable;
    deadband = deadband_value;
}

/**
 * @brief 配置摩擦补偿。
 * @param enable true 表示启用摩擦补偿。
 * @param fc 库仑摩擦补偿系数。
 * @param bv 粘性摩擦补偿系数。
 * @param w_eps 平滑参数。
 * @param use_target_omega_value true 表示使用目标值作为补偿输入。
 * @return 无。
 * @details
 * 该配置只更新摩擦补偿参数，不会主动修改主 PID 的运行时状态。
 */
void Class_PID::FrictionCompensationEnable(bool enable, float fc, float bv, float w_eps, bool use_target_omega_value)
{
    friction_comp_enable = enable;
    friction_fc = fc;
    friction_bv = bv;
    friction_w_eps = fabsf(w_eps);
    if (friction_w_eps < 1e-6f)
    {
        friction_w_eps = 1e-6f;
    }
    friction_use_target_omega = use_target_omega_value;
}

/**
 * @brief 配置重力补偿。
 * @param enable true 表示启用重力补偿。
 * @param kg 重力补偿系数。
 * @param reverse_output true 表示反向输出补偿量。
 * @return 无。
 * @details 该配置只更新重力补偿参数，不直接影响主 PID 误差计算。
 */
void Class_PID::GravityCompensationEnable(bool enable, float kg, bool reverse_output)
{
    gravity_comp_enable = enable;
    gravity_comp_kg = kg;
    gravity_comp_reverse = reverse_output;
}

/**
 * @brief 更新重力补偿姿态角。
 * @param angle_deg 当前姿态角，单位为度。
 * @return 无。
 * @details 该角度仅用于重力补偿模型计算。
 */
void Class_PID::GravityCompensationSetAngle(float angle_deg)
{
    gravity_comp_angle_deg = angle_deg;
}

/**
 * @brief 配置输出低通整形。
 * @param enable true 表示启用输出低通。
 * @param tau_s 低通时间常数，单位为秒。
 * @return 无。
 * @details 当时间常数不大于零时，会自动退化为关闭低通效果。
 */
void Class_PID::OutputFilterEnable(bool enable, float tau_s)
{
    output_filter_enable = enable;
    output_filter_tau_s = (tau_s > 0.0f) ? tau_s : 0.0f;
}

/**
 * @brief 配置输出斜率限制。
 * @param enable true 表示启用输出斜率限制。
 * @param slew_rate 输出增大时的最大变化率，单位为每秒。
 * @return 无。
 * @details 斜率值会取绝对值保存，用于后续输出整形。
 */
void Class_PID::OutputSlewEnable(bool enable, float slew_rate)
{
    output_slew_enable = enable;
    output_slew_rate = fabsf(slew_rate);
}

/**
 * @brief 重置输出整形内部状态。
 * @param init_output 重置后的初始输出值。
 * @return 无。
 * @details 该接口通常用于模式切换或目标突变后的整形状态同步。
 */
void Class_PID::OutputShaperReset(float init_output)
{
    output_shaper_state = init_output;
    output_shaper_inited = true;
}

/**
 * @brief 配置变速积分。
 * @param enable true 表示启用变速积分。
 * @param threshold_a 强抑制阈值。
 * @param threshold_b 弱抑制阈值。
 * @return 无。
 * @details
 * 当误差绝对值大于等于 threshold_a 时关闭积分，
 * 当误差绝对值小于等于 threshold_b 时全量积分。
 */
void Class_PID::IntegralSeparationEnable(bool enable, float threshold_a, float threshold_b)
{
    integral_separation_enable = enable;
    integral_separation_threshold_A = threshold_a;
    integral_separation_threshold_B = threshold_b;
}

/**
 * @brief 配置目标值限幅。
 * @param enable true 表示启用目标值限幅。
 * @param min 目标值下限。
 * @param max 目标值上限。
 * @return 无。
 * @details 目标值限幅在误差计算前执行。
 */
void Class_PID::TargetLimitEnable(bool enable, float min, float max)
{
    target_limit_enable = enable;
    target_limit_min = min;
    target_limit_max = max;
}

/**
 * @brief 设置 PID 参数与限幅参数。
 * @param p 比例系数。
 * @param i 积分系数。
 * @param d 微分系数。
 * @param feed_forward 前馈系数。
 * @param integral_min_value 积分项下限。
 * @param integral_max_value 积分项上限。
 * @param out_min_value 输出下限。
 * @param out_max_value 输出上限。
 * @return 无。
 * @details 该接口只更新参数配置，不会主动清空已有运行时状态。
 */
void Class_PID::SetParameters(float p, float i, float d, float feed_forward,
                              float integral_min_value, float integral_max_value,
                              float out_min_value, float out_max_value)
{
    Kp = p;
    Ki = i;
    Kd = d;
    FeedForward = feed_forward;
    integral_min = integral_min_value;
    integral_max = integral_max_value;
    out_min = out_min_value;
    out_max = out_max_value;
}

/**
 * @brief 执行一次 PID 控制计算。
 * @param input_value 当前输入值。
 * @param target_value 当前目标值。
 * @param dt_value 当前控制周期，单位为秒。
 * @return 本次计算得到的控制输出。
 * @details
 * 该接口会按顺序执行目标限幅、误差计算、死区处理、积分更新、
 * 微分计算、前馈补偿、摩擦补偿、重力补偿以及输出整形与限幅。
 */
float Class_PID::Calculate(float input_value, float target_value, float dt_value)
{
    const float dt_min = 1e-6f;
    float integral_coef = 1.0f;
    float integral_candidate;
    float output_unsat_candidate;
    float output_unsat;
    float output_limited;
    float output_shaped;
    float alpha;
    float delta;
    float delta_max;
    float output_no_extra_comp;

    if (dt_value < dt_min)
    {
        dt_value = dt_min;
    }

    dt = dt_value;
    Input = input_value;
    target = target_value;

    if (target_limit_enable)
    {
        if (target < target_limit_min)
        {
            target = target_limit_min;
        }
        else if (target > target_limit_max)
        {
            target = target_limit_max;
        }
    }

    error = target - Input;

    if (deadband_enable && (fabsf(error) <= deadband))
    {
        output = 0.0f;
        P_out = 0.0f;
        I_out = 0.0f;
        D_out = 0.0f;
        FeedForward_out = 0.0f;
        Friction_Compensation_out = 0.0f;
        Gravity_Compensation_out = 0.0f;
        output_shaper_state = 0.0f;
        output_shaper_inited = true;
        prev_error = error;
        prev_target = target;
        return 0.0f;
    }

    P_out = Kp * error;

    if (integral_separation_enable)
    {
        float threshold_a = integral_separation_threshold_A;
        float threshold_b = integral_separation_threshold_B;
        float abs_err = fabsf(error);

        if (threshold_a <= threshold_b)
        {
            threshold_a = threshold_b + 1e-6f;
        }

        if (abs_err >= threshold_a)
        {
            integral_coef = 0.0f;
        }
        else if (abs_err > threshold_b)
        {
            integral_coef = (threshold_a - abs_err) / (threshold_a - threshold_b);
        }
    }

    if (differential_enable)
    {
        D_out = Kd * (-(target - prev_target) / dt);
    }
    else
    {
        D_out = Kd * ((error - prev_error) / dt);
    }

    FeedForward_out = FeedForward * ((target - prev_target) / dt);

    integral_candidate = integral + error * dt * integral_coef;
    if (integral_candidate < integral_min)
    {
        integral_candidate = integral_min;
    }
    else if (integral_candidate > integral_max)
    {
        integral_candidate = integral_max;
    }

    output_no_extra_comp = P_out + Ki * integral_candidate + D_out + FeedForward_out;
    Friction_Compensation_out = PID_Get_Friction_Compensation(this);
    Gravity_Compensation_out = PID_Get_Gravity_Compensation(this);
    output_unsat_candidate = output_no_extra_comp + Friction_Compensation_out + Gravity_Compensation_out;

    if (((output_unsat_candidate > out_max) && (error > 0.0f)) ||
        ((output_unsat_candidate < out_min) && (error < 0.0f)))
    {
    }
    else
    {
        integral = integral_candidate;
    }

    I_out = Ki * integral;
    output_unsat = P_out + I_out + D_out + FeedForward_out +
                   Friction_Compensation_out + Gravity_Compensation_out;
    output_limited = output_unsat;

    if (output_limited < out_min)
    {
        output_limited = out_min;
    }
    else if (output_limited > out_max)
    {
        output_limited = out_max;
    }

    if (output_filter_enable || output_slew_enable)
    {
        if (output_shaper_inited == false)
        {
            output_shaper_state = output_limited;
            output_shaper_inited = true;
        }

        output_shaped = output_limited;
        if (output_filter_enable && (output_filter_tau_s > 0.0f))
        {
            alpha = dt / (output_filter_tau_s + dt);
            if (alpha < 0.0f)
            {
                alpha = 0.0f;
            }
            else if (alpha > 1.0f)
            {
                alpha = 1.0f;
            }

            output_shaped = output_shaper_state + alpha * (output_limited - output_shaper_state);
        }

        if (output_slew_enable && (output_slew_rate > 0.0f))
        {
            delta = output_shaped - output_shaper_state;
            delta_max = output_slew_rate * dt;
            if ((output_shaped * output_shaper_state < 0.0f) ||
                (fabsf(output_shaped) < fabsf(output_shaper_state)))
            {
                delta_max *= PID_OUTPUT_SLEW_RELEASE_GAIN;
            }

            if (delta > delta_max)
            {
                delta = delta_max;
            }
            else if (delta < -delta_max)
            {
                delta = -delta_max;
            }

            output_shaped = output_shaper_state + delta;
        }

        if (output_shaped < out_min)
        {
            output_shaped = out_min;
        }
        else if (output_shaped > out_max)
        {
            output_shaped = out_max;
        }

        output = output_shaped;
        output_shaper_state = output_shaped;
    }
    else
    {
        output = output_limited;
        output_shaper_state = output;
        output_shaper_inited = true;
    }

    prev_error = error;
    prev_target = target;

    return output;
}
