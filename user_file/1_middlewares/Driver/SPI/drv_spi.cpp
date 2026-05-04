/**
 * @file drv_spi.cpp
 * @brief SPI 驱动实现。
 * @details
 * 本文件实现 `Class_SPI_Manage_Object` 的成员函数。
 */
#include "drv_spi.h"

#define SPI_DONE_TIMEOUT_TICKS pdMS_TO_TICKS(50)
#define SPI_TRANSFER_STATE_NONE 0
#define SPI_TRANSFER_STATE_OK 1
#define SPI_TRANSFER_STATE_ERROR 2
#define SPI_TRANSFER_STATE_TIMEOUT 3

Class_SPI_Manage_Object SPI1_Manage_Object = {};

namespace
{
/**
 * @brief 根据 HAL SPI 句柄获取对应的软件管理对象。
 * @param hspi HAL SPI 句柄。
 * @return 成功时返回管理对象指针，失败返回 NULL。
 * @details 当前驱动只维护一路 `SPI1_Manage_Object`。
 */
Class_SPI_Manage_Object *SPI_Get_Manage_Object(const SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL)
    {
        return NULL;
    }

    if (hspi->Instance == SPI1)
    {
        return &SPI1_Manage_Object;
    }

    return NULL;
}
}

/**
 * @brief 初始化 DMA 同步资源和软件状态。
 * @return 无。
 */
void Class_SPI_Manage_Object::InitDMA()
{
    if (DMA_Inited != 0u)
    {
        return;
    }

    Done_Sem = xSemaphoreCreateCounting(1, 0);
    if (Done_Sem == NULL)
    {
        DMA_Inited = 0u;
        return;
    }

    hspi = NULL;
    DMA_Inited = 1u;
    Transfer_State = SPI_TRANSFER_STATE_NONE;

    Alive_Flag = 0u;
    Alive_Pre_Flag = 0u;
    Alive_Online = 0u;
    Alive_Changed = 0u;

    Current_Transaction.device = ACCEL;
    Current_Transaction.user_rx_buf = NULL;
    Current_Transaction.valid_size = 0u;
}

/**
 * @brief 判断逻辑设备编号是否有效。
 * @param device 逻辑设备编号。
 * @return 1 表示有效，0 表示无效。
 */
uint8_t Class_SPI_Manage_Object::DeviceIsValid(uint8_t device) const
{
    return (device <= TEMP) ? 1u : 0u;
}

/**
 * @brief 获取逻辑设备读取返回值的偏移。
 * @param device 逻辑设备编号。
 * @return 有效数据在 DMA 接收缓冲区中的起始偏移。
 */
uint8_t Class_SPI_Manage_Object::DeviceGetReadOffset(uint8_t device) const
{
    switch (device)
    {
    case ACCEL:
    case TEMP:
        return 2u;
    case GYRO:
        return 1u;
    default:
        return 1u;
    }
}

/**
 * @brief 控制逻辑设备片选引脚。
 * @param device 逻辑设备编号。
 * @param level_high 0 表示拉低，1 表示拉高。
 * @return 无。
 */
void Class_SPI_Manage_Object::DeviceCSWrite(uint8_t device, uint8_t level_high)
{
    if (DeviceIsValid(device) == 0u)
    {
        return;
    }

    switch (device)
    {
    case ACCEL:
        if (level_high == 0u)
        {
            ACCEL_CS_LOW();
        }
        else
        {
            ACCEL_CS_HIGH();
        }
        break;
    case GYRO:
        if (level_high == 0u)
        {
            GYRO_CS_LOW();
        }
        else
        {
            GYRO_CS_HIGH();
        }
        break;
    case TEMP:
        if (level_high == 0u)
        {
            ACCEL_CS_LOW();
        }
        else
        {
            ACCEL_CS_HIGH();
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 判断当前是否处于中断上下文。
 * @return 1 表示当前处于 ISR 中，0 表示当前处于任务上下文中。
 */
uint8_t Class_SPI_Manage_Object::IsInISR() const
{
    return (__get_IPSR() != 0u) ? 1u : 0u;
}

/**
 * @brief 清空事务完成信号量中的残留计数。
 * @return 无。
 */
void Class_SPI_Manage_Object::ClearDoneSemaphore()
{
    if (Done_Sem == NULL)
    {
        return;
    }

    while (xSemaphoreTake(Done_Sem, 0) == pdTRUE)
    {
    }
}

/**
 * @brief 成功完成一次事务后喂在线检测计数。
 * @return 无。
 */
void Class_SPI_Manage_Object::AliveFeed()
{
    Alive_Flag++;
}

/**
 * @brief 等待当前 DMA 事务完成。
 * @return HAL_OK 表示事务成功，HAL_ERROR 表示事务失败，HAL_TIMEOUT 表示等待超时。
 */
HAL_StatusTypeDef Class_SPI_Manage_Object::WaitTransferDone()
{
    if ((Done_Sem == NULL) || (hspi == NULL))
    {
        return HAL_ERROR;
    }

    if (xSemaphoreTake(Done_Sem, SPI_DONE_TIMEOUT_TICKS) != pdTRUE)
    {
        Transfer_State = SPI_TRANSFER_STATE_TIMEOUT;
        DeviceCSWrite(Current_Transaction.device, 1u);
        (void)HAL_SPI_Abort(hspi);
        return HAL_TIMEOUT;
    }

    if (Transfer_State == SPI_TRANSFER_STATE_OK)
    {
        AliveFeed();
        return HAL_OK;
    }

    return HAL_ERROR;
}

/**
 * @brief 在中断上下文中统一完成事务收尾。
 * @param transfer_ok 1 表示事务成功，0 表示事务失败。
 * @param copy_rx 1 表示需要把接收数据拷贝到用户缓冲区。
 * @return 无。
 */
void Class_SPI_Manage_Object::TransferFinishFromISR(uint8_t transfer_ok, uint8_t copy_rx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t i;
    uint16_t offset;

    if (Done_Sem == NULL)
    {
        return;
    }

    DeviceCSWrite(Current_Transaction.device, 1u);

    if ((copy_rx != 0u) &&
        (Current_Transaction.user_rx_buf != NULL) &&
        (Current_Transaction.valid_size > 0u))
    {
        offset = DeviceGetReadOffset(Current_Transaction.device);
        if ((uint16_t)(Current_Transaction.valid_size + offset) > sizeof(Rx_Buffer))
        {
            transfer_ok = 0u;
        }
        else
        {
            for (i = 0u; i < Current_Transaction.valid_size; i++)
            {
                Current_Transaction.user_rx_buf[i] = Rx_Buffer[i + offset];
            }
        }
    }

    Transfer_State = (transfer_ok != 0u) ? SPI_TRANSFER_STATE_OK : SPI_TRANSFER_STATE_ERROR;

    xSemaphoreGiveFromISR(Done_Sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 向指定逻辑设备写寄存器。
 * @param spi_handle 当前使用的 HAL SPI 句柄。
 * @param device 逻辑设备编号。
 * @param reg 寄存器地址。
 * @param data 待写入的 1 字节数据。
 * @return HAL_OK 表示写入成功，HAL_ERROR 表示参数错误或事务失败，HAL_TIMEOUT 表示等待超时。
 */
HAL_StatusTypeDef Class_SPI_Manage_Object::WriteReg(SPI_HandleTypeDef *spi_handle, uint8_t device, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef ret;

    if (IsInISR() != 0u)
    {
        return HAL_ERROR;
    }

    if ((DMA_Inited == 0u) || (DeviceIsValid(device) == 0u) || (spi_handle == NULL))
    {
        return HAL_ERROR;
    }

    hspi = spi_handle;

    Tx_Buffer[0] = (uint8_t)(reg & 0x7Fu);
    Tx_Buffer[1] = data;

    Current_Transaction.device = device;
    Current_Transaction.user_rx_buf = NULL;
    Current_Transaction.valid_size = 0u;
    Transfer_State = SPI_TRANSFER_STATE_NONE;

    ClearDoneSemaphore();
    DeviceCSWrite(device, 0u);

    ret = HAL_SPI_Transmit_DMA(hspi, Tx_Buffer, 2u);
    if (ret != HAL_OK)
    {
        DeviceCSWrite(device, 1u);
        Transfer_State = SPI_TRANSFER_STATE_ERROR;
        return ret;
    }

    return WaitTransferDone();
}

/**
 * @brief 从指定逻辑设备读取寄存器。
 * @param spi_handle 当前使用的 HAL SPI 句柄。
 * @param device 逻辑设备编号。
 * @param reg 起始寄存器地址。
 * @param rx_data 用户接收缓冲区。
 * @param valid_size 需要读取的有效字节数。
 * @return HAL_OK 表示读取成功，HAL_ERROR 表示参数错误或事务失败，HAL_TIMEOUT 表示等待超时。
 */
HAL_StatusTypeDef Class_SPI_Manage_Object::ReadReg(SPI_HandleTypeDef *spi_handle, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size)
{
    uint16_t total_len;
    uint16_t i;
    HAL_StatusTypeDef ret;

    if (IsInISR() != 0u)
    {
        return HAL_ERROR;
    }

    if ((DMA_Inited == 0u) ||
        (DeviceIsValid(device) == 0u) ||
        (spi_handle == NULL) ||
        (rx_data == NULL) ||
        (valid_size == 0u))
    {
        return HAL_ERROR;
    }

    hspi = spi_handle;

    total_len = (uint16_t)(valid_size + DeviceGetReadOffset(device));
    if (total_len > sizeof(Tx_Buffer))
    {
        return HAL_ERROR;
    }

    Tx_Buffer[0] = (uint8_t)(reg | 0x80u);
    for (i = 1u; i < total_len; i++)
    {
        Tx_Buffer[i] = 0u;
    }

    Current_Transaction.device = device;
    Current_Transaction.user_rx_buf = rx_data;
    Current_Transaction.valid_size = valid_size;
    Transfer_State = SPI_TRANSFER_STATE_NONE;

    ClearDoneSemaphore();
    DeviceCSWrite(device, 0u);

    ret = HAL_SPI_TransmitReceive_DMA(hspi, Tx_Buffer, Rx_Buffer, total_len);
    if (ret != HAL_OK)
    {
        DeviceCSWrite(device, 1u);
        Transfer_State = SPI_TRANSFER_STATE_ERROR;
        return ret;
    }

    return WaitTransferDone();
}

/**
 * @brief 处理 SPI DMA 收发完成事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_SPI_Manage_Object::TxRxCpltCallback()
{
    TransferFinishFromISR(1u, 1u);
}

/**
 * @brief 处理 SPI DMA 发送完成事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_SPI_Manage_Object::TxCpltCallback()
{
    TransferFinishFromISR(1u, 0u);
}

/**
 * @brief 处理 SPI DMA 错误事件。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_SPI_Manage_Object::ErrorCallback()
{
    TransferFinishFromISR(0u, 0u);
}

/**
 * @brief 执行一次 100ms 周期的在线状态检查。
 * @return 无。
 */
void Class_SPI_Manage_Object::AliveCheck100ms()
{
    uint8_t online_new;

    if (DMA_Inited == 0u)
    {
        return;
    }

    online_new = (uint8_t)(Alive_Flag != Alive_Pre_Flag);
    Alive_Pre_Flag = Alive_Flag;

    if (online_new != Alive_Online)
    {
        Alive_Online = online_new;
        Alive_Changed = 1u;
    }
}

/**
 * @brief 获取当前 SPI 链路在线状态。
 * @return 0 表示离线，1 表示在线。
 */
uint8_t Class_SPI_Manage_Object::AliveIsOnline() const
{
    return Alive_Online;
}

/**
 * @brief 消费一次 SPI 在线状态变化事件。
 * @param online 若非空，则输出当前在线状态。
 * @return 0 表示无变化，1 表示有变化。
 */
uint8_t Class_SPI_Manage_Object::AliveTryConsumeChanged(uint8_t *online)
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
 * @brief HAL SPI DMA 收发完成回调。
 * @param hspi HAL SPI 句柄。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Class_SPI_Manage_Object *manage = SPI_Get_Manage_Object(hspi);

    if (manage == NULL)
    {
        return;
    }

    manage->TxRxCpltCallback();
}

/**
 * @brief HAL SPI DMA 发送完成回调。
 * @param hspi HAL SPI 句柄。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Class_SPI_Manage_Object *manage = SPI_Get_Manage_Object(hspi);

    if (manage == NULL)
    {
        return;
    }

    manage->TxCpltCallback();
}

/**
 * @brief HAL SPI 错误回调。
 * @param hspi HAL SPI 句柄。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    Class_SPI_Manage_Object *manage = SPI_Get_Manage_Object(hspi);

    if (manage == NULL)
    {
        return;
    }

    manage->ErrorCallback();
}

}
