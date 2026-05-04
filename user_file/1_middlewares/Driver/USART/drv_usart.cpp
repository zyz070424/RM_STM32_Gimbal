/**
 * @file drv_usart.cpp
 * @brief USART 驱动实现。
 * @details
 * 本文件实现 `Class_UART_Manage_Object` 的成员函数。
 */
#include "drv_usart.h"

#include <string.h>

Class_UART_Manage_Object UART1_Manage_Object = {};
Class_UART_Manage_Object UART3_Manage_Object = {};
Class_UART_Manage_Object UART6_Manage_Object = {};

namespace
{
/**
 * @brief 根据 HAL UART 句柄获取对应的软件管理对象。
 * @param huart HAL UART 句柄。
 * @return 成功时返回管理对象指针，失败返回 NULL。
 * @details 当前仅适配 USART1、USART3 和 USART6。
 */
Class_UART_Manage_Object *USART_Get_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return NULL;
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

    return NULL;
}

/**
 * @brief 进入临界区。
 * @return 进入前保存的 PRIMASK。
 * @details 该函数兼容调度器未启动阶段。
 */
uint32_t USART_Enter_Critical(void)
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
void USART_Exit_Critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}
}

/**
 * @brief 启动一次 ReceiveToIdle 接收。
 * @return HAL 状态码。
 * @details 有 DMA 时优先使用 DMA 接收，否则退化为中断接收。
 */
HAL_StatusTypeDef Class_UART_Manage_Object::StartRx()
{
    HAL_StatusTypeDef ret;

    if ((huart == NULL) || (Rx_Buffer_Active == NULL))
    {
        return HAL_ERROR;
    }

    if (huart->hdmarx != NULL)
    {
        ret = HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer_Active, UART_BUFFER_SIZE);
        if (ret != HAL_OK)
        {
            return ret;
        }

        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        return HAL_OK;
    }

    return HAL_UARTEx_ReceiveToIdle_IT(huart, Rx_Buffer_Active, UART_BUFFER_SIZE);
}

/**
 * @brief 重新初始化接收状态。
 * @return 无。
 * @details 主要用于错误回调中恢复接收路径。
 */
void Class_UART_Manage_Object::Reinit()
{
    Rx_Buffer_Active = Rx_Buffer_0;
    Rx_Buffer_Ready = Rx_Buffer_1;
    Tx_Busy = 0u;

    (void)StartRx();
}

/**
 * @brief 初始化 UART 管理对象。
 * @param uart_handle HAL UART 句柄。
 * @param callback 上层接收回调函数。
 * @return 无。
 * @details 初始化完成后会立即启动一次 ReceiveToIdle 接收。
 */
void Class_UART_Manage_Object::Init(UART_HandleTypeDef *uart_handle, UART_Callback callback)
{
    huart = uart_handle;
    Callback_Function = callback;
    Rx_Buffer_Active = Rx_Buffer_0;
    Rx_Buffer_Ready = Rx_Buffer_1;
    Tx_Busy = 0u;

    (void)StartRx();
}

/**
 * @brief 使用 DMA 发送一段数据。
 * @param data 待发送数据指针。
 * @param len 待发送字节数。
 * @return HAL 状态码。
 * @details
 * 该接口仅允许在任务上下文调用。
 * 为避免 DMA 访问上层栈内存，发送前会先复制到内部发送缓存。
 */
HAL_StatusTypeDef Class_UART_Manage_Object::SendData(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;
    uint32_t primask;

    if ((huart == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    if ((len == 0u) || (len > UART_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    if (huart->hdmatx == NULL)
    {
        return HAL_ERROR;
    }

    if (__get_IPSR() != 0u)
    {
        return HAL_ERROR;
    }

    primask = USART_Enter_Critical();
    if (Tx_Busy != 0u)
    {
        USART_Exit_Critical(primask);
        return HAL_BUSY;
    }
    Tx_Busy = 1u;
    USART_Exit_Critical(primask);

    memcpy(Tx_Buffer, data, len);

    ret = HAL_UART_Transmit_DMA(huart, Tx_Buffer, len);
    if (ret != HAL_OK)
    {
        primask = USART_Enter_Critical();
        Tx_Busy = 0u;
        USART_Exit_Critical(primask);
        return ret;
    }

    return HAL_OK;
}

/**
 * @brief 处理一次 ReceiveToIdle 接收完成事件。
 * @param size 本次接收的有效字节数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 * @details
 * 该接口会切换双缓冲区、重启接收，并在有有效数据时调用上层接收回调。
 */
void Class_UART_Manage_Object::RxEventCallback(uint16_t size)
{
    uint8_t *ready_buffer;

    ready_buffer = Rx_Buffer_Active;
    if (Rx_Buffer_Active == Rx_Buffer_0)
    {
        Rx_Buffer_Active = Rx_Buffer_1;
    }
    else
    {
        Rx_Buffer_Active = Rx_Buffer_0;
    }

    Rx_Buffer_Ready = ready_buffer;

    (void)StartRx();

    if ((size > 0u) && (Callback_Function != NULL))
    {
        Callback_Function(Rx_Buffer_Ready, size);
    }
}

/**
 * @brief 处理一次 DMA 发送完成事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 * @details 该接口用于释放发送忙状态。
 */
void Class_UART_Manage_Object::TxCpltCallback()
{
    Tx_Busy = 0u;
}

/**
 * @brief 处理一次 UART 错误事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 * @details 该接口会重置双缓冲状态并重新启动接收。
 */
void Class_UART_Manage_Object::ErrorCallback()
{
    Reinit();
}

extern "C" {

/**
 * @brief HAL ReceiveToIdle 接收事件回调。
 * @param huart HAL UART 句柄。
 * @param Size 本次接收的有效字节数。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Class_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->RxEventCallback(Size);
}

/**
 * @brief HAL UART DMA 发送完成回调。
 * @param huart HAL UART 句柄。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Class_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->TxCpltCallback();
}

/**
 * @brief HAL UART 错误回调。
 * @param huart HAL UART 句柄。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Class_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->ErrorCallback();
}

}
