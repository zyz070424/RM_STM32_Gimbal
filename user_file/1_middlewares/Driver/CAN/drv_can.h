/**
 * @file drv_can.h
 * @brief CAN 驱动接口与管理对象定义。
 * @details
 * 本文件定义 `Class_CAN_Manage_Object` CAN 软件管理对象。
 */
#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include "main.h"
#include "stm32f4xx_hal_can.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define CAN_RX_BUFFER_SIZE 32

/**
 * @brief 一帧 CAN 接收消息的软件表示。
 * @details
 * 该结构体用于把 HAL 读出的帧头与 8 字节数据统一封装，
 * 便于中断与任务之间传递。
 */
typedef struct
{
    CAN_RxHeaderTypeDef rx_header; /**< HAL 提供的接收帧头信息 */
    uint8_t rx_data[8];            /**< 实际收到的 8 字节负载数据 */
} CAN_RX_MESSAGE;

#ifdef __cplusplus
/**
 * @class Class_CAN_Manage_Object
 * @brief CAN 软件管理对象。
 * @details
 * 一个对象对应一路物理 CAN 外设，负责维护 Active/Ready 双缓冲接收区、
 * 任务通知句柄和链路在线检测状态。
 */
class Class_CAN_Manage_Object
{
public:
    CAN_HandleTypeDef *hcan;                    /**< 关联的 HAL CAN 句柄 */
    CAN_RX_MESSAGE Rx_Buffer_0[CAN_RX_BUFFER_SIZE]; /**< 双缓冲区 0 */
    CAN_RX_MESSAGE Rx_Buffer_1[CAN_RX_BUFFER_SIZE]; /**< 双缓冲区 1 */
    CAN_RX_MESSAGE *Rx_Buffer_Active;           /**< 当前由中断写入的缓冲区 */
    uint16_t Rx_Length_Active;                  /**< Active 缓冲区已写入帧数 */
    CAN_RX_MESSAGE *Rx_Buffer_Ready;            /**< 当前可被任务读取的缓冲区 */
    uint16_t Rx_Length_Ready;                   /**< Ready 缓冲区当前有效帧数 */
    uint32_t Drop_Count;                        /**< 因软件缓冲区写满导致的累计丢帧数 */
    TaskHandle_t Notify_Task_Handle;            /**< Ready 缓冲区就绪时通知的任务句柄 */
    volatile uint32_t Alive_Flag;               /**< 每收到一帧加一，用于在线检测 */
    uint32_t Alive_Pre_Flag;                    /**< 上一次 100ms 检查时记录的计数值 */
    volatile uint8_t Alive_Online;              /**< 当前在线状态，0=离线，1=在线 */
    volatile uint8_t Alive_Changed;             /**< 在线状态自上次消费后是否发生变化 */

    void Init(CAN_HandleTypeDef *can_handle);
    void FilterConfig();
    void Start();
    void RegisterNotifyTask(TaskHandle_t task_handle);
    void Send(uint32_t send_id, uint8_t *data);
    void ReceiveCallback();
    HAL_StatusTypeDef ReadMessage(CAN_RX_MESSAGE *rx_message);
    HAL_StatusTypeDef ReadMessageByStdId(uint32_t std_id, CAN_RX_MESSAGE *rx_message);
    void AliveCheck100ms();
    uint8_t AliveIsOnline() const;
    uint8_t AliveTryConsumeChanged(uint8_t *online);

private:
    void PublishActiveToReady();
    void PromoteActiveToReadyIfNeeded();
};
extern Class_CAN_Manage_Object CAN1_Manage_Object;
extern Class_CAN_Manage_Object CAN2_Manage_Object;
#endif

#endif /* __DRV_CAN_H__ */
