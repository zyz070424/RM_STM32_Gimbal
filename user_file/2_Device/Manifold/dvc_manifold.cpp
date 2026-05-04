/**
 * @file dvc_manifold.cpp
 * @brief Manifold 视觉链路设备实现。
 * @details
 * 本文件实现 `Class_Manifold` 的成员函数。
 */
#include "dvc_manifold.h"

#include <math.h>
#include <string.h>

#define MANIFOLD_RX_PITCH_ABS_MAX_DEG 90.0f
#define MANIFOLD_RX_YAW_ABS_MAX_DEG   360.0f

Manifold_UART_Rx_Data Rx_Data;
volatile uint8_t Manifold_USB_Tx_Debug_Frame[MANIFOLD_USB_TX_FRAME_LEN];
volatile uint16_t Manifold_USB_Tx_Debug_Len = 0u;
volatile uint8_t Manifold_USB_Rx_Debug_Raw[MANIFOLD_USB_RX_DEBUG_RAW_MAX];
volatile uint16_t Manifold_USB_Rx_Debug_Raw_Len = 0u;
volatile uint8_t Manifold_USB_Rx_Debug_Frame[MANIFOLD_USB_RX_FRAME_LEN];
volatile uint16_t Manifold_USB_Rx_Debug_Frame_Len = 0u;
volatile uint32_t Manifold_USB_Rx_Frame_Seq = 0u;
Class_Manifold Manifold_Manage_Object = {};

namespace
{
/**
 * @brief Manifold USB 接收桥接回调。
 * @param buf USB 接收缓冲区指针。
 * @param len USB 接收长度。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Manifold_Usb_Rx_Callback_Bridge(uint8_t *buf, uint32_t len)
{
    Manifold_Manage_Object.UsbRxCallback(buf, len);
}
}

/**
 * @brief 对当前字节流执行重同步。
 * @param byte 触发重同步时的当前字节。
 * @return 无。
 */
void Class_Manifold::RxResync(uint8_t byte)
{
    if (byte == Frame_Header)
    {
        Rx_Frame_Buffer[0] = byte;
        Rx_Index = 1u;
    }
    else
    {
        Rx_Index = 0u;
    }
}

/**
 * @brief 校验视觉目标角度是否处于允许范围内。
 * @param pitch 俯仰角。
 * @param yaw 偏航角。
 * @return 1 表示合法，0 表示非法。
 * @details 该函数只校验数值合法性，不处理 Target_Valid 语义。
 */
uint8_t Class_Manifold::AngleIsValid(float pitch, float yaw) const
{
    if ((isfinite(pitch) == 0) || (isfinite(yaw) == 0))
    {
        return 0u;
    }

    if ((fabsf(pitch) > MANIFOLD_RX_PITCH_ABS_MAX_DEG) ||
        (fabsf(yaw) > MANIFOLD_RX_YAW_ABS_MAX_DEG))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 解析一帧视觉发给电控的接收帧。
 * @param frame 帧起始地址，包含帧头。
 * @param pitch 输出俯仰角指针。
 * @param yaw 输出偏航角指针。
 * @param target_valid 输出目标有效标志指针。
 * @return 1 表示解析成功，0 表示解析失败。
 * @details 接收协议固定为 [Header][Yaw][Pitch][Target_Valid][Tail]。
 */
uint8_t Class_Manifold::DecodeRxFrame(const uint8_t *frame,
                                      float *pitch,
                                      float *yaw,
                                      uint8_t *target_valid) const
{
    float pitch_tmp;
    float yaw_tmp;
    uint8_t target_valid_tmp;

    if ((frame == NULL) || (pitch == NULL) || (yaw == NULL) || (target_valid == NULL))
    {
        return 0u;
    }

    if (frame[MANIFOLD_USB_RX_FRAME_LEN - 1u] != Frame_Tail)
    {
        return 0u;
    }

    memcpy(&yaw_tmp, frame + 1u, sizeof(float));
    memcpy(&pitch_tmp, frame + 1u + sizeof(float), sizeof(float));
    target_valid_tmp = frame[1u + sizeof(float) + sizeof(float)];

    if (AngleIsValid(pitch_tmp, yaw_tmp) == 0u)
    {
        return 0u;
    }

    *pitch = pitch_tmp;
    *yaw = yaw_tmp;
    *target_valid = (target_valid_tmp != 0u) ? 1u : 0u;
    return 1u;
}

/**
 * @brief 初始化 Manifold 协议管理对象。
 * @param data 发送数据结构体指针。
 * @param frame_header 协议帧头字节。
 * @param frame_end 协议帧尾字节。
 * @param sentry_mode 哨兵模式状态。
 * @return 无。
 * @details 初始化后会注册 USB 接收回调，并设置 USB 发送最小间隔。
 */
void Class_Manifold::Init(Manifold_UART_Tx_Data *data,
                          uint8_t frame_header,
                          uint8_t frame_end,
                          enum Enum_Manifold_Sentry_Mode sentry_mode)
{
    (void)sentry_mode;

    if (data == NULL)
    {
        return;
    }

    Frame_Header = frame_header;
    Frame_Tail = frame_end;
    Rx_Index = 0u;

    data->Frame_Header = frame_header;
    data->Frame_Tail = frame_end;

    Rx_Data.Frame_Header = frame_header;
    Rx_Data.Frame_Tail = frame_end;
    Rx_Data.Taget_Yaw = 0.0f;
    Rx_Data.Taget_Pitch = 0.0f;
    Rx_Data.Target_Valid = 0u;

    USB_Manage_Object.Init(Manifold_Usb_Rx_Callback_Bridge);
    USB_Manage_Object.SetTxMinInterval(2u);
}

/**
 * @brief 清空当前视觉目标缓存。
 * @return 无。
 * @details USB 掉线或上层复位时调用，确保角度缓存和 Target_Valid 同时失效。
 */
void Class_Manifold::ClearTarget()
{
    Rx_Data.Taget_Yaw = 0.0f;
    Rx_Data.Taget_Pitch = 0.0f;
    Rx_Data.Target_Valid = 0u;
}

/**
 * @brief 处理一次 Manifold USB 接收事件。
 * @param buf 接收数据缓冲区指针。
 * @param len 接收数据长度。
 * @return 无。
 * @details 每收到一帧新协议数据都会刷新帧序号。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_Manifold::UsbRxCallback(uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t raw_copy_len;

    if ((buf == NULL) || (len == 0u))
    {
        return;
    }

    raw_copy_len = len;
    if (raw_copy_len > MANIFOLD_USB_RX_DEBUG_RAW_MAX)
    {
        raw_copy_len = MANIFOLD_USB_RX_DEBUG_RAW_MAX;
    }

    memcpy((void *)Manifold_USB_Rx_Debug_Raw, buf, raw_copy_len);
    Manifold_USB_Rx_Debug_Raw_Len = (uint16_t)raw_copy_len;

    for (i = 0u; i < len; i++)
    {
        uint8_t byte;
        float pitch_tmp;
        float yaw_tmp;
        uint8_t target_valid_tmp;

        byte = buf[i];

        if (Rx_Index == 0u)
        {
            if (byte == Frame_Header)
            {
                Rx_Frame_Buffer[Rx_Index++] = byte;
            }
            continue;
        }

        Rx_Frame_Buffer[Rx_Index++] = byte;

        if (Rx_Index == MANIFOLD_USB_RX_FRAME_LEN)
        {
            if (DecodeRxFrame(Rx_Frame_Buffer, &pitch_tmp, &yaw_tmp, &target_valid_tmp) != 0u)
            {
                memcpy((void *)Manifold_USB_Rx_Debug_Frame, Rx_Frame_Buffer, MANIFOLD_USB_RX_FRAME_LEN);
                Manifold_USB_Rx_Debug_Frame_Len = MANIFOLD_USB_RX_FRAME_LEN;
                Rx_Data.Taget_Pitch = pitch_tmp;
                Rx_Data.Taget_Yaw = yaw_tmp;
                Rx_Data.Target_Valid = target_valid_tmp;
                Manifold_USB_Rx_Frame_Seq++;
                Rx_Index = 0u;
            }
            else
            {
                RxResync(byte);
            }
        }
    }
}

/**
 * @brief 发送一帧电控姿态数据到视觉端。
 * @param data 发送数据结构体指针。
 * @param euler 当前姿态欧拉角。
 * @return 无。
 * @details 发送协议固定为 [Header][Yaw][Pitch][Tail]。
 */
void Class_Manifold::SendData(Manifold_UART_Tx_Data *data, euler_t euler)
{
    uint8_t tx_buf[MANIFOLD_USB_TX_FRAME_LEN];

    if (data == NULL)
    {
        return;
    }

    data->Pitch = euler.pitch;
    data->Yaw = euler.yaw;
    tx_buf[0] = data->Frame_Header;
    memcpy(tx_buf + 1u, &data->Yaw, sizeof(float));
    memcpy(tx_buf + 1u + sizeof(float), &data->Pitch, sizeof(float));
    tx_buf[sizeof(tx_buf) - 1u] = data->Frame_Tail;

    memcpy((void *)Manifold_USB_Tx_Debug_Frame, tx_buf, sizeof(tx_buf));
    Manifold_USB_Tx_Debug_Len = (uint16_t)sizeof(tx_buf);

    (void)USB_Manage_Object.SendData(tx_buf, (uint16_t)sizeof(tx_buf));
}
