/**
 * @file dvc_motor_protect.cpp
 * @brief 电机堵转保护实现。
 * @details
 * 本文件实现 `Class_Motor_Protect` 与 `Class_Motor_Protect_Pitch` 的成员函数。
 */
#include "dvc_motor_protect.h"

#include <math.h>
#include <stdint.h>

namespace
{
constexpr float Motor_Protect_Pitch_Margin_Deg = 2.0f;
constexpr float Motor_Protect_Pitch_Backoff_Deg = 4.0f;

/**
 * @brief 对无符号 16 位计数执行饱和累加。
 * @param value 当前计数值。
 * @param delta 本次增量。
 * @return 累加后的饱和值。
 */
uint16_t Motor_Protect_Sat_Add_U16(uint16_t value, uint16_t delta)
{
    uint32_t sum = (uint32_t)value + (uint32_t)delta;

    if (sum > 65535u)
    {
        return 65535u;
    }

    return (uint16_t)sum;
}

/**
 * @brief 对浮点数执行区间限幅。
 * @param value 输入值。
 * @param min_value 最小允许值。
 * @param max_value 最大允许值。
 * @return 限幅后的结果。
 */
float Motor_Protect_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief 获取有效的时间步长。
 * @param input 当前保护输入。
 * @return 有效的时间步长，最小为 1ms。
 */
uint16_t Motor_Protect_Get_DtMs(const Motor_Protect_Input_TypeDef *input)
{
    if ((input == nullptr) || (input->dt_ms == 0u))
    {
        return 1u;
    }

    return input->dt_ms;
}

/**
 * @brief 切换堵转保护状态并重置状态相关计数。
 * @param handle 堵转保护对象。
 * @param next_state 目标状态。
 * @return 无。
 */
void Motor_Protect_Enter_State(Class_Motor_Protect *handle,
                               Motor_Protect_State_TypeDef next_state)
{
    handle->state = next_state;
    handle->state_ms = 0u;
    handle->stall_ms = 0u;

    if (next_state == MOTOR_PROTECT_STATE_BACKOFF)
    {
        handle->backoff_request = 1u;
    }
    else
    {
        handle->backoff_request = 0u;
    }
}

/**
 * @brief 执行一次堵转投票判定。
 * @param handle 堵转保护对象。
 * @param config 堵转保护配置。
 * @param input 当前保护输入。
 * @return 1 表示本次确认堵转，0 表示未确认。
 */
uint8_t Motor_Protect_Check_Stall(Class_Motor_Protect *handle,
                                  const Motor_Protect_Config_TypeDef *config,
                                  const Motor_Protect_Input_TypeDef *input)
{
    uint8_t vote = 0u;
    uint16_t dt_ms;

    if ((config == nullptr) || (input == nullptr))
    {
        return 0u;
    }

    if (handle->state_ms < config->blank_ms)
    {
        handle->stall_ms = 0u;
        return 0u;
    }

    dt_ms = Motor_Protect_Get_DtMs(input);

    if (fabsf(input->cmd) > config->cmd_th)
    {
        vote++;
    }

    if (fabsf((float)input->torque_raw) > config->torque_th)
    {
        vote++;
    }

    if (fabsf(input->speed_rad_s) < config->speed_th_rad_s)
    {
        vote++;
    }

    if (fabsf(input->pos_err_deg) > config->err_th_deg)
    {
        vote++;
    }

    if (vote >= 3u)
    {
        handle->stall_ms = Motor_Protect_Sat_Add_U16(handle->stall_ms, dt_ms);
    }
    else
    {
        handle->stall_ms = 0u;
    }

    if (handle->stall_ms >= config->confirm_ms)
    {
        return 1u;
    }

    return 0u;
}

}

Class_Motor_Protect_Pitch Motor_Protect_Pitch_Object = {};

/**
 * @brief 初始化堵转保护对象。
 * @return 无。
 */
void Class_Motor_Protect::Init()
{
    Reset();
}

/**
 * @brief 重置堵转保护对象到正常状态。
 * @return 无。
 */
void Class_Motor_Protect::Reset()
{
    state = MOTOR_PROTECT_STATE_NORMAL;
    state_ms = 0u;
    stall_ms = 0u;
    retry_count = 0u;
    backoff_request = 0u;
    last_cmd = 0.0f;
}

/**
 * @brief 清空本轮判定计时但保留当前状态。
 * @return 无。
 */
void Class_Motor_Protect::Blank()
{
    state_ms = 0u;
    stall_ms = 0u;
}

/**
 * @brief 清除故障并回到正常状态。
 * @return 无。
 */
void Class_Motor_Protect::ClearFault()
{
    Reset();
}

/**
 * @brief 更新一次堵转保护状态机。
 * @param config 堵转保护配置。
 * @param input 当前保护输入。
 * @return 无。
 */
void Class_Motor_Protect::Update(const Motor_Protect_Config_TypeDef *config,
                                 const Motor_Protect_Input_TypeDef *input)
{
    uint16_t dt_ms;

    if ((config == nullptr) || (input == nullptr))
    {
        return;
    }

    last_cmd = input->cmd;

    if (config->enable == 0u)
    {
        Reset();
        return;
    }

    dt_ms = Motor_Protect_Get_DtMs(input);

    switch (state)
    {
    case MOTOR_PROTECT_STATE_NORMAL:
        state_ms = Motor_Protect_Sat_Add_U16(state_ms, dt_ms);

        if ((config->retry_reset_ms > 0u) && (state_ms >= config->retry_reset_ms))
        {
            retry_count = 0u;
        }

        if (Motor_Protect_Check_Stall(this, config, input) != 0u)
        {
            if ((config->allow_backoff != 0u) &&
                (config->has_mech_limit != 0u) &&
                (input->near_limit != 0u) &&
                (input->pushing_outward != 0u))
            {
                if (retry_count < 255u)
                {
                    retry_count++;
                }

                Motor_Protect_Enter_State(this, MOTOR_PROTECT_STATE_BACKOFF);
            }
            else
            {
                Motor_Protect_Enter_State(this, MOTOR_PROTECT_STATE_FAULT);
            }
        }
        break;

    case MOTOR_PROTECT_STATE_BACKOFF:
        state_ms = Motor_Protect_Sat_Add_U16(state_ms, dt_ms);

        if (state_ms >= config->backoff_ms)
        {
            if (retry_count > config->retry_limit)
            {
                Motor_Protect_Enter_State(this, MOTOR_PROTECT_STATE_FAULT);
            }
            else
            {
                Motor_Protect_Enter_State(this, MOTOR_PROTECT_STATE_COOLDOWN);
            }
        }
        break;

    case MOTOR_PROTECT_STATE_COOLDOWN:
        state_ms = Motor_Protect_Sat_Add_U16(state_ms, dt_ms);

        if (state_ms >= config->cooldown_ms)
        {
            Motor_Protect_Enter_State(this, MOTOR_PROTECT_STATE_NORMAL);
        }
        break;

    case MOTOR_PROTECT_STATE_FAULT:
    default:
        break;
    }
}

/**
 * @brief 根据当前保护状态处理输出控制量。
 * @param config 堵转保护配置。
 * @param raw_cmd 原始控制量。
 * @return 处理后的控制量。
 */
float Class_Motor_Protect::ApplyOutput(const Motor_Protect_Config_TypeDef *config, float raw_cmd) const
{
    float limit;

    if ((config == nullptr) || (config->enable == 0u))
    {
        return raw_cmd;
    }

    switch (state)
    {
    case MOTOR_PROTECT_STATE_BACKOFF:
        limit = fabsf(config->backoff_cmd_limit);
        if (limit <= 0.0f)
        {
            return 0.0f;
        }
        return Motor_Protect_Clamp(raw_cmd, -limit, limit);

    case MOTOR_PROTECT_STATE_COOLDOWN:
    case MOTOR_PROTECT_STATE_FAULT:
        return 0.0f;

    case MOTOR_PROTECT_STATE_NORMAL:
    default:
        return raw_cmd;
    }
}

/**
 * @brief 读取并清除回退请求标志。
 * @return 1 表示本次存在新的回退请求，0 表示没有。
 */
uint8_t Class_Motor_Protect::ConsumeBackoffRequest()
{
    uint8_t request = backoff_request;

    backoff_request = 0u;
    return request;
}

/**
 * @brief 查询当前是否处于故障状态。
 * @return 1 表示故障，0 表示非故障。
 */
uint8_t Class_Motor_Protect::IsFault() const
{
    return (state == MOTOR_PROTECT_STATE_FAULT) ? 1u : 0u;
}

/**
 * @brief 初始化 Pitch 轴堵转保护包装器。
 * @param min_angle_deg Pitch 最小机械角度。
 * @param max_angle_deg Pitch 最大机械角度。
 * @param cmd_positive_to_angle_positive_value 指令正方向是否对应角度正方向。
 * @return 无。
 */
void Class_Motor_Protect_Pitch::Init(float min_angle_deg_value,
                                     float max_angle_deg_value,
                                     uint8_t cmd_positive_to_angle_positive_value)
{
    if (min_angle_deg_value > max_angle_deg_value)
    {
        float temp = min_angle_deg_value;

        min_angle_deg_value = max_angle_deg_value;
        max_angle_deg_value = temp;
    }

    cfg.enable = 1u;
    cfg.allow_backoff = 1u;
    cfg.has_mech_limit = 1u;
    cfg.cmd_th = 3500.0f;
    cfg.torque_th = 700.0f;
    cfg.speed_th_rad_s = 0.15f;
    cfg.err_th_deg = 2.5f;
    cfg.blank_ms = 120u;
    cfg.confirm_ms = 80u;
    cfg.backoff_ms = 120u;
    cfg.cooldown_ms = 200u;
    cfg.retry_reset_ms = 1000u;
    cfg.backoff_cmd_limit = 2500.0f;
    cfg.retry_limit = 2u;

    min_angle_deg = min_angle_deg_value;
    max_angle_deg = max_angle_deg_value;
    cmd_positive_to_angle_positive =
        (cmd_positive_to_angle_positive_value != 0u) ? 1u : 0u;
    backoff_target_deg = 0.0f;
    reset_request = 0u;
    active = 1u;
    initialized = 1u;

    core.Init();
}

/**
 * @brief 设置 Pitch 轴保护是否启用。
 * @param active_value 新的启用状态。
 * @return 无。
 */
void Class_Motor_Protect_Pitch::SetActive(uint8_t active_value)
{
    active_value = (active_value != 0u) ? 1u : 0u;

    if (initialized == 0u)
    {
        return;
    }

    if (active == active_value)
    {
        return;
    }

    active = active_value;
    reset_request = 0u;

    if (active_value == 0u)
    {
        core.Reset();
        backoff_target_deg = 0.0f;
    }
    else
    {
        core.Blank();
    }
}

/**
 * @brief 查询 Pitch 轴保护当前是否启用。
 * @return 1 表示启用，0 表示未启用。
 */
uint8_t Class_Motor_Protect_Pitch::IsActive() const
{
    if (initialized == 0u)
    {
        return 0u;
    }

    return active;
}

/**
 * @brief 根据当前 Pitch 保护状态处理目标角度。
 * @param raw_target_deg 原始目标角度。
 * @return 处理后的目标角度。
 */
float Class_Motor_Protect_Pitch::ApplyTarget(float raw_target_deg) const
{
    if ((initialized == 0u) || (active == 0u))
    {
        return raw_target_deg;
    }

    if (core.state == MOTOR_PROTECT_STATE_BACKOFF)
    {
        return backoff_target_deg;
    }

    return raw_target_deg;
}

/**
 * @brief 基于当前反馈更新 Pitch 保护并返回处理后的输出。
 * @param target_deg 当前目标角度。
 * @param feedback_deg 当前反馈角度。
 * @param speed_rad_s 当前角速度。
 * @param raw_cmd 原始控制输出。
 * @param torque_raw 当前力矩反馈原始值。
 * @param dt_ms 当前时间步长。
 * @return 处理后的控制输出。
 */
float Class_Motor_Protect_Pitch::UpdateOutput(float target_deg,
                                              float feedback_deg,
                                              float speed_rad_s,
                                              float raw_cmd,
                                              int16_t torque_raw,
                                              uint16_t dt_ms)
{
    Motor_Protect_Input_TypeDef input = {};

    if ((initialized == 0u) || (active == 0u))
    {
        return raw_cmd;
    }

    input.dt_ms = dt_ms;
    input.pos_err_deg = target_deg - feedback_deg;
    input.speed_rad_s = speed_rad_s;
    input.cmd = raw_cmd;
    input.torque_raw = torque_raw;
    input.near_limit = NearLimit(feedback_deg);
    input.pushing_outward = PushingOutward(feedback_deg, raw_cmd);

    core.Update(&cfg, &input);

    if (core.ConsumeBackoffRequest() != 0u)
    {
        backoff_target_deg = MakeBackoffTarget(feedback_deg, raw_cmd);
        reset_request = 1u;
    }

    return core.ApplyOutput(&cfg, raw_cmd);
}

/**
 * @brief 读取并清除角度环复位请求。
 * @param reset_target_deg 输出回退目标角度。
 * @return 1 表示存在新的复位请求，0 表示没有。
 */
uint8_t Class_Motor_Protect_Pitch::TakeResetRequest(float *reset_target_deg)
{
    uint8_t request = reset_request;

    reset_request = 0u;

    if ((request != 0u) && (reset_target_deg != nullptr))
    {
        *reset_target_deg = backoff_target_deg;
    }

    return request;
}

/**
 * @brief 清空 Pitch 轴当前轮的判定计时。
 * @return 无。
 */
void Class_Motor_Protect_Pitch::Blank()
{
    if (initialized == 0u)
    {
        return;
    }

    core.Blank();
}

/**
 * @brief 清除 Pitch 轴当前故障状态。
 * @return 无。
 */
void Class_Motor_Protect_Pitch::ClearFault()
{
    if (initialized == 0u)
    {
        return;
    }

    core.ClearFault();
    reset_request = 0u;
}

/**
 * @brief 查询 Pitch 轴当前是否处于故障状态。
 * @return 1 表示故障，0 表示非故障。
 */
uint8_t Class_Motor_Protect_Pitch::IsFault() const
{
    if (initialized == 0u)
    {
        return 0u;
    }

    return core.IsFault();
}

/**
 * @brief 判断当前反馈是否靠近机械限位。
 * @param feedback_deg 当前反馈角度。
 * @return 1 表示靠近限位，0 表示未靠近。
 */
uint8_t Class_Motor_Protect_Pitch::NearLimit(float feedback_deg) const
{
    if (feedback_deg >= (max_angle_deg - Motor_Protect_Pitch_Margin_Deg))
    {
        return 1u;
    }

    if (feedback_deg <= (min_angle_deg + Motor_Protect_Pitch_Margin_Deg))
    {
        return 1u;
    }

    return 0u;
}

/**
 * @brief 判断当前控制输出是否仍在向外顶机械限位。
 * @param feedback_deg 当前反馈角度。
 * @param raw_cmd 当前控制输出。
 * @return 1 表示仍在向外顶限位，0 表示没有。
 */
uint8_t Class_Motor_Protect_Pitch::PushingOutward(float feedback_deg, float raw_cmd) const
{
    if (feedback_deg >= (max_angle_deg - Motor_Protect_Pitch_Margin_Deg))
    {
        return (cmd_positive_to_angle_positive != 0u) ? (raw_cmd > 0.0f) : (raw_cmd < 0.0f);
    }

    if (feedback_deg <= (min_angle_deg + Motor_Protect_Pitch_Margin_Deg))
    {
        return (cmd_positive_to_angle_positive != 0u) ? (raw_cmd < 0.0f) : (raw_cmd > 0.0f);
    }

    return 0u;
}

/**
 * @brief 生成回退阶段使用的目标角度。
 * @param feedback_deg 当前反馈角度。
 * @param raw_cmd 当前控制输出。
 * @return 生成后的回退目标角度。
 */
float Class_Motor_Protect_Pitch::MakeBackoffTarget(float feedback_deg, float raw_cmd) const
{
    float target;

    if (feedback_deg >= (max_angle_deg - Motor_Protect_Pitch_Margin_Deg))
    {
        target = feedback_deg - Motor_Protect_Pitch_Backoff_Deg;
    }
    else if (feedback_deg <= (min_angle_deg + Motor_Protect_Pitch_Margin_Deg))
    {
        target = feedback_deg + Motor_Protect_Pitch_Backoff_Deg;
    }
    else if (cmd_positive_to_angle_positive != 0u)
    {
        target = feedback_deg + ((raw_cmd > 0.0f) ?
                                 -Motor_Protect_Pitch_Backoff_Deg :
                                  Motor_Protect_Pitch_Backoff_Deg);
    }
    else
    {
        target = feedback_deg + ((raw_cmd > 0.0f) ?
                                  Motor_Protect_Pitch_Backoff_Deg :
                                 -Motor_Protect_Pitch_Backoff_Deg);
    }

    return Motor_Protect_Clamp(target, min_angle_deg, max_angle_deg);
}
