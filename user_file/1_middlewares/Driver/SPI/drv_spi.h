/**
 * @file drv_spi.h
 * @brief SPI 驱动接口与管理对象定义。
 * @details
 * 本文件定义 `Class_SPI_Manage_Object` SPI 软件管理对象。
 */
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "main.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <stdint.h>

/**
 * @brief SPI 逻辑设备枚举。
 */
enum SPI_Device
{
    ACCEL = 0,
    GYRO = 1,
    TEMP = 2,
};

#define ACCEL_CS_HIGH() HAL_GPIO_WritePin(ACCEL_CSB1_GPIO_Port, ACCEL_CSB1_Pin, GPIO_PIN_SET)
#define ACCEL_CS_LOW() HAL_GPIO_WritePin(ACCEL_CSB1_GPIO_Port, ACCEL_CSB1_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH() HAL_GPIO_WritePin(GYRO_CSB2_GPIO_Port, GYRO_CSB2_Pin, GPIO_PIN_SET)
#define GYRO_CS_LOW() HAL_GPIO_WritePin(GYRO_CSB2_GPIO_Port, GYRO_CSB2_Pin, GPIO_PIN_RESET)

/**
 * @brief SPI 当前事务上下文。
 * @details
 * 保存当前逻辑设备编号、接收缓冲区地址和有效载荷长度，
 * 用于 DMA 异步完成后统一收尾。
 */
typedef struct Struct_SPI_Current_Transaction
{
    uint8_t device;         /**< 当前事务对应的逻辑设备编号 */
    uint8_t *user_rx_buf;   /**< 用户读操作接收缓冲区 */
    uint16_t valid_size;    /**< 用户期望接收的有效字节数 */
} Struct_SPI_Current_Transaction;

#ifdef __cplusplus
/**
 * @class Class_SPI_Manage_Object
 * @brief SPI 软件管理对象。
 * @details
 * 当前工程中该驱动只维护一路 `SPI1_Manage_Object`，主要负责管理 DMA 同步资源、
 * 固定长度收发缓冲区、当前事务上下文与 SPI 链路在线检测状态。
 */
class Class_SPI_Manage_Object
{
public:
    SPI_HandleTypeDef *hspi;                     /**< 关联的 HAL SPI 句柄 */
    SemaphoreHandle_t Done_Sem;                 /**< DMA 事务完成信号量 */
    uint8_t DMA_Inited;                         /**< DMA 同步资源是否已初始化 */
    uint8_t Tx_Buffer[16];                      /**< SPI 发送缓冲区 */
    uint8_t Rx_Buffer[16];                      /**< SPI 接收缓冲区 */
    Struct_SPI_Current_Transaction Current_Transaction; /**< 当前事务上下文 */
    volatile uint8_t Transfer_State;            /**< 当前事务状态 */
    volatile uint32_t Alive_Flag;               /**< 成功事务计数 */
    uint32_t Alive_Pre_Flag;                    /**< 上一次 100ms 检查时的计数值 */
    volatile uint8_t Alive_Online;              /**< 当前在线状态，0=离线，1=在线 */
    volatile uint8_t Alive_Changed;             /**< 在线状态是否发生过变化 */

    void InitDMA();
    HAL_StatusTypeDef WriteReg(SPI_HandleTypeDef *spi_handle, uint8_t device, uint8_t reg, uint8_t data);
    HAL_StatusTypeDef ReadReg(SPI_HandleTypeDef *spi_handle, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size);
    void TxRxCpltCallback();
    void TxCpltCallback();
    void ErrorCallback();
    void AliveCheck100ms();
    uint8_t AliveIsOnline() const;
    uint8_t AliveTryConsumeChanged(uint8_t *online);

private:
    uint8_t DeviceIsValid(uint8_t device) const;
    uint8_t DeviceGetReadOffset(uint8_t device) const;
    void DeviceCSWrite(uint8_t device, uint8_t level_high);
    uint8_t IsInISR() const;
    void ClearDoneSemaphore();
    void AliveFeed();
    HAL_StatusTypeDef WaitTransferDone();
    void TransferFinishFromISR(uint8_t transfer_ok, uint8_t copy_rx);
};
extern Class_SPI_Manage_Object SPI1_Manage_Object;
#endif

#endif /* __DRV_SPI_H__ */
