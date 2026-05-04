/**
 * @file drv_usart.h
 * @brief USART 驱动接口与管理对象定义。
 * @details
 * 本文件定义 `Class_UART_Manage_Object` USART 软件管理对象。
 */
#ifndef DRV_USART_H__
#define DRV_USART_H__

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#define UART_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UART_Callback)(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
/**
 * @class Class_UART_Manage_Object
 * @brief UART 软件管理对象。
 * @details
 * 一个对象对应一路串口外设，负责保存 HAL 句柄、维护双缓冲接收区、
 * 维护 DMA 发送缓存，并在 HAL 官方回调到来时完成状态切换。
 */
class Class_UART_Manage_Object
{
public:
    UART_HandleTypeDef *huart;              /**< 关联的 HAL UART 句柄 */
    UART_Callback Callback_Function;        /**< 上层接收回调函数 */
    uint8_t Rx_Buffer_0[UART_BUFFER_SIZE];  /**< 接收缓冲区 0 */
    uint8_t Rx_Buffer_1[UART_BUFFER_SIZE];  /**< 接收缓冲区 1 */
    uint8_t *Rx_Buffer_Active;              /**< 当前由 HAL 写入的缓冲区 */
    uint8_t *Rx_Buffer_Ready;               /**< 最近一次接收完成的缓冲区 */
    uint8_t Tx_Buffer[UART_TX_BUFFER_SIZE]; /**< DMA 发送缓存 */
    volatile uint8_t Tx_Busy;               /**< 当前是否存在发送任务 */

    void Init(UART_HandleTypeDef *uart_handle, UART_Callback callback);
    HAL_StatusTypeDef SendData(const uint8_t *data, uint16_t len);
    void RxEventCallback(uint16_t size);
    void TxCpltCallback();
    void ErrorCallback();

private:
    HAL_StatusTypeDef StartRx();
    void Reinit();
};
extern Class_UART_Manage_Object UART1_Manage_Object;
extern Class_UART_Manage_Object UART3_Manage_Object;
extern Class_UART_Manage_Object UART6_Manage_Object;
#endif

#endif /* DRV_USART_H__ */
