/**
 * @file gimbal_sentry_target.c
 * @brief 哨兵目标生成模块实现。
 */
#include "gimbal_sentry_target.h"
#include "dvc_manifold.h"
#include "drv_usb.h"

extern Manifold_UART_Rx_Data Rx_Data; /**< 视觉链路最新一帧接收缓存。 */

static Gimbal_Sentry_Target_Config_TypeDef g_gimbal_sentry_target_config; /**< 模块配置缓存。 */
static uint8_t g_gimbal_sentry_target_initialized = 0u; /**< 模块初始化完成标志。 */
static Gimbal_Sentry_Handle_TypeDef g_gimbal_sentry; /**< 底层哨兵状态机实例。 */
static uint32_t g_gimbal_visual_last_rx_frame_seq = 0u; /**< 最近处理过的视觉帧序号。 */
static TickType_t g_gimbal_visual_last_valid_tick = 0u; /**< 最近一次收到有效目标的时间戳。 */
static float g_gimbal_visual_last_valid_pitch_deg = 0.0f; /**< 最近一次有效视觉 pitch，单位：deg。 */
static float g_gimbal_visual_last_valid_yaw_deg = 0.0f; /**< 最近一次有效视觉 yaw，单位：deg。 */
static float g_gimbal_visual_filtered_pitch_deg = 0.0f; /**< 视觉滤波后的 pitch，单位：deg。 */
static float g_gimbal_visual_filtered_yaw_deg = 0.0f; /**< 视觉滤波后的 yaw，单位：deg。 */
static uint8_t g_gimbal_visual_has_valid_target = 0u; /**< 是否已经缓存过有效视觉目标。 */
static uint8_t g_gimbal_visual_filter_tracking = 0u; /**< 视觉滤波器是否已进入跟踪状态。 */
static float g_gimbal_pitch_target = 0.0f; /**< 输出给云台的 pitch 目标角，单位：deg。 */
static float g_gimbal_yaw_target = 0.0f; /**< 输出给云台的 yaw 目标角，单位：deg。 */

/**
 * @brief 限幅输入值。
 * @param value 待限幅的值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 * @return 限幅后的值。
 */
static float Gimbal_Sentry_Target_Clamp(float value, float min_value, float max_value)
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
 * @brief 一阶低通滤波器。
 * @param current_value 当前滤波输出。
 * @param target_value 新的目标输入。
 * @param tau_s 滤波时间常数，单位：s。
 * @param dt_s 控制周期，单位：s。
 * @return 滤波后的输出值。
 */
static float Gimbal_Sentry_Target_First_Order_Low_Pass(float current_value,
                                                       float target_value,
                                                       float tau_s,
                                                       float dt_s)
{
    float alpha;

    if (tau_s <= 0.0f)
    {
        return target_value;
    }

    alpha = dt_s / (tau_s + dt_s);
    alpha = Gimbal_Sentry_Target_Clamp(alpha, 0.0f, 1.0f);
    return current_value + alpha * (target_value - current_value);
}

/**
 * @brief 刷新视觉输入缓存。
 * @param now_tick 当前系统 tick。
 * @note  只有在收到新帧且目标有效时，才会更新最近一次有效目标缓存。
 */
static void Gimbal_Sentry_Target_Update_Cache(TickType_t now_tick)
{
    uint32_t rx_frame_seq;

    rx_frame_seq = Manifold_USB_Rx_Frame_Seq;
    if (rx_frame_seq != g_gimbal_visual_last_rx_frame_seq)
    {
        g_gimbal_visual_last_rx_frame_seq = rx_frame_seq;
        if (Rx_Data.Target_Valid != 0u)
        {
            g_gimbal_visual_last_valid_pitch_deg = Rx_Data.Taget_Pitch;
            g_gimbal_visual_last_valid_yaw_deg = Rx_Data.Taget_Yaw;
            g_gimbal_visual_last_valid_tick = now_tick;
            g_gimbal_visual_has_valid_target = 1u;
        }
    }
}

/**
 * @brief 判断当前拍是否存在可用视觉目标。
 * @param now_tick 当前系统 tick。
 * @return `1` 表示视觉目标可用，`0` 表示不可用。
 */
static uint8_t Gimbal_Sentry_Target_IsAvailable(TickType_t now_tick)
{
    Gimbal_Sentry_Target_Update_Cache(now_tick);

    if ((g_gimbal_visual_has_valid_target == 0u) ||
        (USB_Alive_IsRxOnline() == 0u))
    {
        return 0u;
    }

    if ((now_tick - g_gimbal_visual_last_valid_tick) >
        pdMS_TO_TICKS(g_gimbal_sentry_target_config.vision_track_timeout_ms))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 根据目标有效性更新视觉滤波输出。
 * @param target_available 当前拍视觉目标是否可用。
 */
static void Gimbal_Sentry_Target_Update_Filter(uint8_t target_available)
{
    if (target_available == 0u)
    {
        g_gimbal_visual_filter_tracking = 0u;
        return;
    }

    if (g_gimbal_visual_filter_tracking == 0u)
    {
        g_gimbal_visual_filtered_pitch_deg = g_gimbal_visual_last_valid_pitch_deg;
        g_gimbal_visual_filtered_yaw_deg = g_gimbal_visual_last_valid_yaw_deg;
        g_gimbal_visual_filter_tracking = 1u;
        return;
    }

    g_gimbal_visual_filtered_pitch_deg = Gimbal_Sentry_Target_First_Order_Low_Pass(
        g_gimbal_visual_filtered_pitch_deg,
        g_gimbal_visual_last_valid_pitch_deg,
        g_gimbal_sentry_target_config.vision_target_filter_tau_s,
        g_gimbal_sentry_target_config.control_dt_s);
    g_gimbal_visual_filtered_yaw_deg = Gimbal_Sentry_Target_First_Order_Low_Pass(
        g_gimbal_visual_filtered_yaw_deg,
        g_gimbal_visual_last_valid_yaw_deg,
        g_gimbal_sentry_target_config.vision_target_filter_tau_s,
        g_gimbal_sentry_target_config.control_dt_s);
}

/**
 * @brief 初始化哨兵目标生成模块。
 * @param config 目标生成模块配置。
 */
void Gimbal_Sentry_Target_Init(const Gimbal_Sentry_Target_Config_TypeDef *config)
{
    if (config == NULL)
    {
        return;
    }

    g_gimbal_sentry_target_config = *config;
    g_gimbal_sentry_target_initialized = 1u;
    Gimbal_Sentry_Target_Reset_Mode();
}

/**
 * @brief 复位哨兵模式状态。
 */
void Gimbal_Sentry_Target_Reset_Mode(void)
{
    if (g_gimbal_sentry_target_initialized == 0u)
    {
        return;
    }

    Gimbal_Sentry_Reset(&g_gimbal_sentry);
    Gimbal_Sentry_Target_Reset_Vision();
    Gimbal_Sentry_Target_Clear_Output();
}

/**
 * @brief 清空视觉目标缓存与滤波状态。
 */
void Gimbal_Sentry_Target_Reset_Vision(void)
{
    g_gimbal_visual_last_rx_frame_seq = Manifold_USB_Rx_Frame_Seq;
    g_gimbal_visual_last_valid_tick = 0u;
    g_gimbal_visual_last_valid_pitch_deg = 0.0f;
    g_gimbal_visual_last_valid_yaw_deg = 0.0f;
    g_gimbal_visual_filtered_pitch_deg = 0.0f;
    g_gimbal_visual_filtered_yaw_deg = 0.0f;
    g_gimbal_visual_has_valid_target = 0u;
    g_gimbal_visual_filter_tracking = 0u;
}

/**
 * @brief 清空当前输出的目标角。
 */
void Gimbal_Sentry_Target_Clear_Output(void)
{
    g_gimbal_pitch_target = 0.0f;
    g_gimbal_yaw_target = 0.0f;
}

/**
 * @brief 结合视觉缓存与状态机更新当前拍目标角。
 * @param now_tick 当前系统 tick。
 */
void Gimbal_Sentry_Target_Update(TickType_t now_tick)
{
    Gimbal_Sentry_Input_TypeDef sentry_input;
    Gimbal_Sentry_Output_TypeDef sentry_output;

    if (g_gimbal_sentry_target_initialized == 0u)
    {
        return;
    }

    sentry_input.vision_target_available = Gimbal_Sentry_Target_IsAvailable(now_tick);
    Gimbal_Sentry_Target_Update_Filter(sentry_input.vision_target_available);
    sentry_input.vision_pitch_deg = g_gimbal_visual_filtered_pitch_deg;
    sentry_input.vision_yaw_deg = g_gimbal_visual_filtered_yaw_deg;

    Gimbal_Sentry_Update(&g_gimbal_sentry,
                         &g_gimbal_sentry_target_config.sentry_config,
                         &sentry_input,
                         &sentry_output);
    g_gimbal_pitch_target = -sentry_output.pitch_target_deg;
    g_gimbal_yaw_target = sentry_output.yaw_target_deg;
}

/**
 * @brief 获取当前输出的 pitch 目标角。
 * @return pitch 目标角，单位：deg。
 * @note  返回值已经做过与云台控制链一致的符号转换。
 */
float Gimbal_Sentry_Target_Get_Pitch(void)
{
    return g_gimbal_pitch_target;
}

/**
 * @brief 获取当前输出的 yaw 目标角。
 * @return yaw 目标角，单位：deg。
 */
float Gimbal_Sentry_Target_Get_Yaw(void)
{
    return g_gimbal_yaw_target;
}

/**
 * @brief 获取当前哨兵状态。
 * @return 当前状态机状态；模块未初始化时返回扫描态。
 */
Gimbal_Sentry_State_TypeDef Gimbal_Sentry_Target_Get_State(void)
{
    if (g_gimbal_sentry_target_initialized == 0u)
    {
        return GIMBAL_SENTRY_STATE_SCAN;
    }

    return Gimbal_Sentry_Get_State(&g_gimbal_sentry);
}
