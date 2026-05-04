/**
 * @file drv_usb.cpp
 * @brief USB 驱动实现。
 * @details
 * 本文件实现 `Class_USB_Manage_Object` 的成员函数，以及兼容旧模块调用方式的
 * `USB_*` 桥接接口。
 */
#include "drv_usb.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

Class_USB_Manage_Object USB_Manage_Object = {};

namespace
{
/**
 * @brief 获取 CDC 类句柄。
 * @return CDC 已就绪时返回类句柄指针，否则返回 NULL。
 */
USBD_CDC_HandleTypeDef *USB_Get_CDC_Handle(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return NULL;
    }

    return (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
}

/**
 * @brief 进入临界区。
 * @return 进入前保存的 PRIMASK。
 * @details 该函数兼容调度器未启动阶段。
 */
uint32_t USB_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区。
 * @param primask 进入前保存的 PRIMASK。
 * @return 无。
 */
void USB_Exit_Critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}
}

/**
 * @brief USB RX 方向活动计数喂狗。
 * @return 无。
 */
void Class_USB_Manage_Object::AliveRxFeed()
{
    Alive_Flag++;
}

/**
 * @brief USB TX 方向活动计数喂狗。
 * @return 无。
 */
void Class_USB_Manage_Object::AliveTxFeed()
{
    Tx_Alive_Flag++;
}

/**
 * @brief 启动下一包 USB 接收。
 * @return 无。
 * @details 仅在 CDC 已就绪时重装接收缓冲区。
 */
void Class_USB_Manage_Object::StartReceive()
{
    if (USB_Get_CDC_Handle() == NULL)
    {
        return;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Rx_Buffer_Active);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/**
 * @brief 切换接收双缓冲。
 * @return 无。
 * @details 将当前 Active 缓冲区发布为 Ready，并切换下一次接收缓冲区。
 */
void Class_USB_Manage_Object::SwapRxBuffer()
{
    uint8_t *ready_buffer = Rx_Buffer_Active;

    Rx_Buffer_Active = (ready_buffer == UserRxBufferFS) ? Rx_Buffer_0 : UserRxBufferFS;
    Rx_Buffer_Ready = ready_buffer;
}

/**
 * @brief 尝试直接发送一段数据。
 * @param data 发送数据缓冲区。
 * @param len 发送长度。
 * @return USBD_OK、USBD_BUSY 或 USBD_FAIL。
 * @details 为保持旧行为一致，CDC 未就绪时返回 USBD_BUSY。
 */
uint8_t Class_USB_Manage_Object::TryTransmitNoCopy(uint8_t *data, uint16_t len)
{
    USBD_CDC_HandleTypeDef *hcdc = USB_Get_CDC_Handle();

    if (hcdc == NULL)
    {
        return USBD_BUSY;
    }

    if (hcdc->TxState != 0u)
    {
        return USBD_BUSY;
    }

    return CDC_Transmit_FS(data, len);
}

/**
 * @brief 设置 USB 最小发送间隔。
 * @param interval_tick 最小发送间隔，传 0 会按 1 tick 处理。
 * @return 无。
 */
void Class_USB_Manage_Object::SetTxMinInterval(uint16_t interval_tick)
{
    Tx_Min_Interval_Tick = (interval_tick == 0u) ? 1u : interval_tick;
}

/**
 * @brief 初始化 USB 管理对象。
 * @param callback 上层接收回调函数。
 * @return 无。
 * @details 初始化完成后会尝试启动一次接收。
 */
void Class_USB_Manage_Object::Init(USB_Callback callback)
{
    uint32_t now_tick = HAL_GetTick();

    Callback_Function = callback;
    Rx_Buffer_Active = UserRxBufferFS;
    Rx_Buffer_Ready = NULL;

    Tx_Busy = 0u;
    Tx_Min_Interval_Tick = USB_TX_MIN_INTERVAL_TICK_DEFAULT;
    Tx_Last_Transmit_Tick = now_tick - USB_TX_MIN_INTERVAL_TICK_DEFAULT;
    Tx_Busy_Start_Tick = now_tick;

    Alive_Flag = 0u;
    Alive_Pre_Flag = 0u;
    Tx_Alive_Flag = 0u;
    Tx_Alive_Pre_Flag = 0u;
    Alive_Rx_Online = 0u;
    Alive_Tx_Online = 0u;
    Alive_Online = 0u;
    Alive_Changed = 0u;

    StartReceive();
}

/**
 * @brief 发送一段 USB 数据。
 * @param data 待发送数据指针。
 * @param len 待发送字节数。
 * @return USBD_OK、USBD_BUSY 或 USBD_FAIL。
 * @details
 * 该接口只允许在任务上下文调用。
 * 为避免异步发送访问上层失效地址，发送前会先复制到内部发送缓存。
 */
uint8_t Class_USB_Manage_Object::SendData(const uint8_t *data, uint16_t len)
{
    uint8_t ret;
    uint32_t primask;
    uint32_t now_tick;
    USBD_CDC_HandleTypeDef *hcdc;

    if ((data == NULL) || (len == 0u) || (len > USB_DATA_Send_MAX))
    {
        return USBD_FAIL;
    }

    if (__get_IPSR() != 0u)
    {
        return USBD_FAIL;
    }

    now_tick = HAL_GetTick();
    if ((uint32_t)(now_tick - Tx_Last_Transmit_Tick) < Tx_Min_Interval_Tick)
    {
        return USBD_BUSY;
    }

    primask = USB_Enter_Critical();

    if ((Tx_Busy != 0u) &&
        ((uint32_t)(now_tick - Tx_Busy_Start_Tick) >= USB_TX_BUSY_TIMEOUT_TICK))
    {
        hcdc = USB_Get_CDC_Handle();
        if ((hcdc == NULL) || (hcdc->TxState == 0u))
        {
            Tx_Busy = 0u;
        }
    }

    if (Tx_Busy != 0u)
    {
        USB_Exit_Critical(primask);
        return USBD_BUSY;
    }

    Tx_Busy = 1u;
    Tx_Busy_Start_Tick = now_tick;
    USB_Exit_Critical(primask);

    memcpy(Tx_Buffer, data, len);

    ret = TryTransmitNoCopy(Tx_Buffer, len);
    if (ret == USBD_OK)
    {
        Tx_Last_Transmit_Tick = now_tick;
        return USBD_OK;
    }

    primask = USB_Enter_Critical();
    Tx_Busy = 0u;
    USB_Exit_Critical(primask);

    return ret;
}

/**
 * @brief 发送一段以零结尾的字符串。
 * @param str 待发送字符串指针。
 * @return USBD_OK、USBD_BUSY 或 USBD_FAIL。
 */
uint8_t Class_USB_Manage_Object::SendString(const char *str)
{
    uint16_t len;

    if (str == NULL)
    {
        return USBD_FAIL;
    }

    len = (uint16_t)strlen(str);
    if (len == 0u)
    {
        return USBD_OK;
    }

    return SendData((const uint8_t *)str, len);
}

/**
 * @brief 处理一次 USB 接收事件。
 * @param buf USB CDC 给出的接收缓冲区指针。
 * @param len 本次接收的有效字节数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_USB_Manage_Object::RxCallback(uint8_t *buf, uint32_t len)
{
    (void)buf;

    if (len > 0u)
    {
        SwapRxBuffer();
        AliveRxFeed();
        StartReceive();

        if (Callback_Function != NULL)
        {
            Callback_Function(Rx_Buffer_Ready, len);
        }
        return;
    }

    StartReceive();
}

/**
 * @brief 处理一次 USB 发送完成事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_USB_Manage_Object::TxCpltCallback()
{
    Tx_Busy = 0u;
    AliveTxFeed();
}

/**
 * @brief 执行一次 100ms 周期的 USB 在线状态检查。
 * @return 无。
 */
void Class_USB_Manage_Object::AliveCheck100ms()
{
    uint8_t rx_online_new;
    uint8_t tx_online_new;
    uint8_t online_new;

    rx_online_new = (uint8_t)(Alive_Flag != Alive_Pre_Flag);
    Alive_Pre_Flag = Alive_Flag;
    tx_online_new = (uint8_t)(Tx_Alive_Flag != Tx_Alive_Pre_Flag);
    Tx_Alive_Pre_Flag = Tx_Alive_Flag;

    Alive_Rx_Online = rx_online_new;
    Alive_Tx_Online = tx_online_new;
    online_new = (uint8_t)((rx_online_new != 0u) || (tx_online_new != 0u));

    if (online_new != Alive_Online)
    {
        Alive_Online = online_new;
        Alive_Changed = 1u;
    }
}

/**
 * @brief 获取当前 USB 链路在线状态。
 * @return 0 表示离线，1 表示在线。
 */
uint8_t Class_USB_Manage_Object::AliveIsOnline() const
{
    return Alive_Online;
}

/**
 * @brief 获取当前 USB RX 在线状态。
 * @return 0 表示离线，1 表示在线。
 */
uint8_t Class_USB_Manage_Object::AliveIsRxOnline() const
{
    return Alive_Rx_Online;
}

/**
 * @brief 获取当前 USB TX 在线状态。
 * @return 0 表示离线，1 表示在线。
 */
uint8_t Class_USB_Manage_Object::AliveIsTxOnline() const
{
    return Alive_Tx_Online;
}

/**
 * @brief 消费一次 USB 在线状态变化事件。
 * @param online 若非空，则返回当前在线状态。
 * @return 0 表示无变化，1 表示有变化。
 */
uint8_t Class_USB_Manage_Object::AliveTryConsumeChanged(uint8_t *online)
{
    uint8_t changed = Alive_Changed;

    if (changed != 0u)
    {
        Alive_Changed = 0u;
        if (online != NULL)
        {
            *online = Alive_Online;
        }
    }

    return changed;
}

extern "C" {

/**
 * @brief 初始化 USB 软件管理对象。
 * @param callback 上层接收回调函数。
 * @return 无。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 Init 方法。
 */
void USB_Init(USB_Callback callback)
{
    USB_Manage_Object.Init(callback);
}

/**
 * @brief 发送一段 USB 数据。
 * @param data 待发送数据指针。
 * @param len 待发送字节数。
 * @return USBD_OK、USBD_BUSY 或 USBD_FAIL。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 SendData 方法。
 */
uint8_t USB_SendData(const uint8_t *data, uint16_t len)
{
    return USB_Manage_Object.SendData(data, len);
}

/**
 * @brief 发送一段以零结尾的字符串。
 * @param str 待发送字符串指针。
 * @return USBD_OK、USBD_BUSY 或 USBD_FAIL。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 SendString 方法。
 */
uint8_t USB_SendString(const char *str)
{
    return USB_Manage_Object.SendString(str);
}

/**
 * @brief 设置 USB 最小发送间隔。
 * @param interval_tick 最小发送间隔，传 0 会按 1 tick 处理。
 * @return 无。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 SetTxMinInterval 方法。
 */
void USB_Set_Tx_Min_Interval(uint16_t interval_tick)
{
    USB_Manage_Object.SetTxMinInterval(interval_tick);
}

/**
 * @brief 处理一次 USB 接收事件。
 * @param buf USB CDC 给出的接收缓冲区指针。
 * @param len 本次接收的有效字节数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 RxCallback 方法。
 */
void USB_Rx_Callback(uint8_t *buf, uint32_t len)
{
    USB_Manage_Object.RxCallback(buf, len);
}

/**
 * @brief 处理一次 USB 发送完成事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 TxCpltCallback 方法。
 */
void USB_TxCplt_Callback(void)
{
    USB_Manage_Object.TxCpltCallback();
}

/**
 * @brief 执行一次 100ms 周期的 USB 在线状态检查。
 * @return 无。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 AliveCheck100ms 方法。
 */
void USB_Alive_Check_100ms(void)
{
    USB_Manage_Object.AliveCheck100ms();
}

/**
 * @brief 获取当前 USB 链路在线状态。
 * @return 0 表示离线，1 表示在线。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 AliveIsOnline 方法。
 */
uint8_t USB_Alive_IsOnline(void)
{
    return USB_Manage_Object.AliveIsOnline();
}

/**
 * @brief 获取当前 USB RX 在线状态。
 * @return 0 表示离线，1 表示在线。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 AliveIsRxOnline 方法。
 */
uint8_t USB_Alive_IsRxOnline(void)
{
    return USB_Manage_Object.AliveIsRxOnline();
}

/**
 * @brief 获取当前 USB TX 在线状态。
 * @return 0 表示离线，1 表示在线。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 AliveIsTxOnline 方法。
 */
uint8_t USB_Alive_IsTxOnline(void)
{
    return USB_Manage_Object.AliveIsTxOnline();
}

/**
 * @brief 消费一次 USB 在线状态变化事件。
 * @param online 若非空，则返回当前在线状态。
 * @return 0 表示无变化，1 表示有变化。
 * @details 该接口为 C 风格兼容桥接，内部调用管理对象的 AliveTryConsumeChanged 方法。
 */
uint8_t USB_Alive_TryConsumeChanged(uint8_t *online)
{
    return USB_Manage_Object.AliveTryConsumeChanged(online);
}

}
