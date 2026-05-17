/**
 * @file dvc_dr16.cpp
 * @brief DR16 遥控接收设备实现。
 * @details
 * 本文件实现 `Class_DR16` 的成员函数。
 */
#include "dvc_dr16.h"

#include "common_math.h"
#include <string.h>

Class_DR16 DR16_Manage_Object = {};

namespace
{
/**
 * @brief 根据 HAL UART 句柄获取对应的软件管理对象。
 * @param huart HAL UART 句柄。
 * @return 成功时返回管理对象指针，失败返回 nullptr。
 */
Class_UART_Manage_Object *DR16_Get_Uart_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart == nullptr)
    {
        return nullptr;
    }

    if (huart->Instance == USART1)
    {
        return &UART1_Manage_Object;
    }

    if (huart->Instance == USART3)
    {
        return &UART3_Manage_Object;
    }

    if (huart->Instance == USART6)
    {
        return &UART6_Manage_Object;
    }

    return nullptr;
}

/**
 * @brief DR16 串口接收转发回调。
 * @param data 接收数据缓冲区指针。
 * @param len 接收数据长度。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void DR16_Receive_Callback_Bridge(uint8_t *data, uint16_t len)
{
    DR16_Manage_Object.RxCallback(data, len);
}
}

/**
 * @brief 将非法拨杆值修正到中位。
 * @param sw 输入拨杆原始值。
 * @return 合法化后的拨杆值。
 */
uint8_t Class_DR16::SanitizeSwitch(uint8_t sw) const
{
    if ((sw == DR16_SWITCH_UP) || (sw == DR16_SWITCH_MIDDLE) || (sw == DR16_SWITCH_DOWN))
    {
        return sw;
    }

    return DR16_SWITCH_MIDDLE;
}

/**
 * @brief 从双缓冲中取出一帧最新 DR16 数据。
 * @param out_frame 输出缓冲区指针。
 * @return 0 表示当前无新帧，1 表示成功获取一帧。
 * @details 该流程会在临界区内复制最新帧，并消费新帧标志。
 */
uint8_t Class_DR16::FetchLatestFrame(uint8_t *out_frame)
{
    uint32_t primask;
    uint8_t index;

    if (out_frame == NULL)
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (Has_New_Frame == 0u)
    {
        if (primask == 0u)
        {
            __enable_irq();
        }
        return 0;
    }

    index = Ready_Index;
    memcpy(out_frame, Frame_Buffer[index], DR16_FRAME_LEN);
    Has_New_Frame = 0u;

    if (primask == 0u)
    {
        __enable_irq();
    }

    return 1;
}

/**
 * @brief 根据当前值和上一值判断拨杆状态。
 * @param sw 输出拨杆状态指针。
 * @param now 当前拨杆原始值。
 * @param prev 上一周期拨杆原始值。
 * @return 无。
 */
void Class_DR16::JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev)
{
    now = SanitizeSwitch(now);
    prev = SanitizeSwitch(prev);

    switch (prev)
    {
    case DR16_SWITCH_UP:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_UP;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_UP_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        break;

    case DR16_SWITCH_DOWN:
        if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_DOWN;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        break;

    case DR16_SWITCH_MIDDLE:
    default:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        else if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_MIDDLE;
        }
        break;
    }
}

/**
 * @brief 根据当前值和上一值判断按键状态。
 * @param key 输出按键状态指针。
 * @param now 当前按键原始值。
 * @param prev 上一周期按键原始值。
 * @return 无。
 */
void Class_DR16::JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev)
{
    if (prev == DR16_KEY_FREE)
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_TRIG_PRESSED_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_PRESSED;
        }
    }
}

/**
 * @brief 初始化 DR16 接收管理对象。
 * @param huart 关联的 UART 句柄。
 * @return 无。
 * @details 初始化后会清空双缓冲状态，并注册串口接收回调。
 */
void Class_DR16::Init(UART_HandleTypeDef *huart)
{
    Class_UART_Manage_Object *uart_manage;

    if (huart == NULL)
    {
        return;
    }

    uart_manage = DR16_Get_Uart_Manage_Object(huart);
    if (uart_manage == nullptr)
    {
        return;
    }

    Huart = huart;
    Ready_Index = 0u;
    Has_New_Frame = 0u;
    memset(Frame_Buffer, 0, sizeof(Frame_Buffer));

    uart_manage->Init(huart, DR16_Receive_Callback_Bridge);
}

/**
 * @brief 处理一帧 DR16 原始数据并更新应用层数据结构。
 * @param dr16 DR16 应用层数据结构指针。
 * @return 无。
 * @details 无新帧时直接返回，不阻塞任务执行。
 */
void Class_DR16::Process(DR16_DataTypeDef *dr16)
{
    uint8_t raw[DR16_FRAME_LEN];
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    float rocker_denom;

    if (dr16 == NULL)
    {
        return;
    }

    if (FetchLatestFrame(raw) == 0u)
    {
        return;
    }

    ch0 = ((uint16_t)raw[0] | ((uint16_t)(raw[1] & 0x07u) << 8)) & 0x07FFu;
    ch1 = (((uint16_t)raw[1] >> 3) | ((uint16_t)(raw[2] & 0x3Fu) << 5)) & 0x07FFu;
    ch2 = (((uint16_t)raw[2] >> 6) | ((uint16_t)raw[3] << 2) | ((uint16_t)(raw[4] & 0x01u) << 10)) & 0x07FFu;
    ch3 = (((uint16_t)raw[4] >> 1) | ((uint16_t)(raw[5] & 0x0Fu) << 7)) & 0x07FFu;

    dr16->raw_s1 = (raw[5] >> 6) & 0x03u;
    dr16->raw_s2 = (raw[5] >> 4) & 0x03u;

    mouse_x = (int16_t)((uint16_t)raw[6] | ((uint16_t)raw[7] << 8));
    mouse_y = (int16_t)((uint16_t)raw[8] | ((uint16_t)raw[9] << 8));
    mouse_z = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

    dr16->raw_mouse_l = raw[12];
    dr16->raw_mouse_r = raw[13];
    dr16->raw_key = (uint16_t)raw[14] | ((uint16_t)raw[15] << 8);

    rocker_denom = DR16_ROCKER_RANGE / 2.0f;

    dr16->right_x = Clamp((ch0 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->right_y = Clamp((ch1 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_x = Clamp((ch2 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_y = Clamp((ch3 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);

    dr16->mouse_x = Clamp(mouse_x / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_y = Clamp(mouse_y / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_z = Clamp(mouse_z / 32768.0f, -1.0f, 1.0f);
}

/**
 * @brief 处理一次 DR16 串口接收完成事件。
 * @param data 接收数据缓冲区指针。
 * @param len 接收数据长度。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_DR16::RxCallback(uint8_t *data, uint16_t len)
{
    uint8_t write_index;

    if ((data == NULL) || (len < DR16_FRAME_LEN))
    {
        return;
    }

    write_index = (uint8_t)(Ready_Index ^ 1u);
    memcpy(Frame_Buffer[write_index], data, DR16_FRAME_LEN);
    __DMB();
    Ready_Index = write_index;
    Has_New_Frame = 1u;
}

/**
 * @brief 处理一次 DR16 1ms 周期状态更新。
 * @param dr16 DR16 应用层数据结构指针。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_DR16::Timer1msCallback(DR16_DataTypeDef *dr16)
{
    uint8_t i;
    uint8_t now_bit;
    uint8_t prev_bit;

    if (dr16 == NULL)
    {
        return;
    }

    JudgeSwitch(&dr16->left_switch, dr16->raw_s1, dr16->prev_raw_s1);
    JudgeSwitch(&dr16->right_switch, dr16->raw_s2, dr16->prev_raw_s2);

    JudgeKey(&dr16->mouse_left, dr16->raw_mouse_l, dr16->prev_raw_mouse_l);
    JudgeKey(&dr16->mouse_right, dr16->raw_mouse_r, dr16->prev_raw_mouse_r);

    for (i = 0u; i < 16u; i++)
    {
        now_bit = (dr16->raw_key >> i) & 0x01u;
        prev_bit = (dr16->prev_raw_key >> i) & 0x01u;
        JudgeKey(&dr16->key[i], now_bit, prev_bit);
    }

    dr16->prev_raw_s1 = dr16->raw_s1;
    dr16->prev_raw_s2 = dr16->raw_s2;
    dr16->prev_raw_mouse_l = dr16->raw_mouse_l;
    dr16->prev_raw_mouse_r = dr16->raw_mouse_r;
    dr16->prev_raw_key = dr16->raw_key;
}
