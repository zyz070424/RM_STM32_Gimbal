/**
 * @file drv_usb.h
 * @brief USB 驱动接口与管理对象定义。
 * @details
 * 本文件定义 `Class_USB_Manage_Object` USB 软件管理对象，
 * 以及给 `USB_DEVICE` 保留的 `USB_*` C 风格入口。
 */
#ifndef DRV_USB_H__
#define DRV_USB_H__

#include "main.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

#define USB_DATA_Send_MAX 256
#define USB_BUFFER_SIZE 256
#define USB_TX_MIN_INTERVAL_TICK_DEFAULT 1
#define USB_TX_BUSY_TIMEOUT_TICK 20

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*USB_Callback)(uint8_t *buffer, uint32_t length);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
/**
 * @class Class_USB_Manage_Object
 * @brief USB 软件管理对象。
 * @details
 * 负责维护 USB CDC 的接收缓冲、发送状态与链路在线检测状态。
 */
class Class_USB_Manage_Object
{
public:
    USB_Callback Callback_Function;       /**< 上层接收回调函数 */
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE]; /**< 接收缓冲区 0 */
    uint8_t *Rx_Buffer_Active;            /**< 当前由 USB CDC 写入的缓冲区 */
    uint8_t *Rx_Buffer_Ready;             /**< 最近一次接收完成的缓冲区 */
    uint8_t Tx_Buffer[USB_DATA_Send_MAX]; /**< 内部发送缓存，避免异步发送访问失效地址 */
    volatile uint8_t Tx_Busy;             /**< 当前是否存在发送任务 */
    uint16_t Tx_Min_Interval_Tick;        /**< 最小发送间隔，单位：tick */
    uint32_t Tx_Last_Transmit_Tick;       /**< 上一次成功发起发送的系统 tick */
    uint32_t Tx_Busy_Start_Tick;          /**< 当前忙状态开始的系统 tick */
    volatile uint32_t Alive_Flag;         /**< RX 活动计数 */
    uint32_t Alive_Pre_Flag;              /**< 上一次 RX 活动计数快照 */
    volatile uint32_t Tx_Alive_Flag;      /**< TX 活动计数 */
    uint32_t Tx_Alive_Pre_Flag;           /**< 上一次 TX 活动计数快照 */
    volatile uint8_t Alive_Rx_Online;     /**< RX 在线状态 */
    volatile uint8_t Alive_Tx_Online;     /**< TX 在线状态 */
    volatile uint8_t Alive_Online;        /**< 总链路在线状态 */
    volatile uint8_t Alive_Changed;       /**< 在线状态自上次消费后是否发生变化 */

    void Init(USB_Callback callback);
    uint8_t SendData(const uint8_t *data, uint16_t len);
    uint8_t SendString(const char *str);
    void SetTxMinInterval(uint16_t interval_tick);
    void RxCallback(uint8_t *buf, uint32_t len);
    void TxCpltCallback();
    void AliveCheck100ms();
    uint8_t AliveIsOnline() const;
    uint8_t AliveIsRxOnline() const;
    uint8_t AliveIsTxOnline() const;
    uint8_t AliveTryConsumeChanged(uint8_t *online);

private:
    void AliveRxFeed();
    void AliveTxFeed();
    void StartReceive();
    void SwapRxBuffer();
    uint8_t TryTransmitNoCopy(uint8_t *data, uint16_t len);
};
extern Class_USB_Manage_Object USB_Manage_Object;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C 风格兼容接口声明。
 * @details
 * 以下接口为 `Class_USB_Manage_Object` 的兼容桥接入口，
 * 用于保留旧模块的 `USB_*` 调用方式。
 */
void USB_Init(USB_Callback callback);
uint8_t USB_SendData(const uint8_t *data, uint16_t len);
uint8_t USB_SendString(const char *str);
void USB_Set_Tx_Min_Interval(uint16_t interval_tick);
void USB_Rx_Callback(uint8_t *buf, uint32_t len);
void USB_TxCplt_Callback(void);
void USB_Alive_Check_100ms(void);
uint8_t USB_Alive_IsOnline(void);
uint8_t USB_Alive_IsRxOnline(void);
uint8_t USB_Alive_IsTxOnline(void);
uint8_t USB_Alive_TryConsumeChanged(uint8_t *online);

#ifdef __cplusplus
}
#endif

#endif /* DRV_USB_H__ */
