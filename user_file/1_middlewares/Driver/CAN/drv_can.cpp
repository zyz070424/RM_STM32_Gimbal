/**
 * @file drv_can.cpp
 * @brief CAN 驱动实现。
 * @details
 * 本文件实现 `Class_CAN_Manage_Object` 的成员函数。
 */
#include "drv_can.h"

#include <stdint.h>

Class_CAN_Manage_Object CAN1_Manage_Object = {};
Class_CAN_Manage_Object CAN2_Manage_Object = {};

static uint8_t can_started[2] = {0};

namespace
{
/**
 * @brief 将 HAL CAN 句柄映射为本地启动状态数组下标。
 * @param hcan HAL CAN 句柄。
 * @return 0 对应 CAN1，1 对应 CAN2，-1 表示无效句柄。
 */
int CAN_Get_Index(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return -1;
    }

    if (hcan->Instance == CAN1)
    {
        return 0;
    }

    if (hcan->Instance == CAN2)
    {
        return 1;
    }

    return -1;
}

/**
 * @brief 根据 HAL CAN 句柄获取对应的软件管理对象。
 * @param hcan HAL CAN 句柄。
 * @return 成功时返回管理对象指针，失败返回 NULL。
 */
Class_CAN_Manage_Object *CAN_Get_Manage_Object(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return NULL;
    }

    if (hcan->Instance == CAN1)
    {
        return &CAN1_Manage_Object;
    }

    if (hcan->Instance == CAN2)
    {
        return &CAN2_Manage_Object;
    }

    return NULL;
}

/**
 * @brief 在首次使用时绑定 HAL 句柄到软件管理对象。
 * @param manage 软件管理对象。
 * @param hcan HAL CAN 句柄。
 * @return 无。
 */
void CAN_Bind_If_Needed(Class_CAN_Manage_Object *manage, CAN_HandleTypeDef *hcan)
{
    if ((manage == NULL) || (hcan == NULL))
    {
        return;
    }

    if (manage->hcan == NULL)
    {
        manage->Init(hcan);
    }
}
}

/**
 * @brief 初始化软件管理对象的运行时状态。
 * @param can_handle 对应的 HAL CAN 句柄。
 * @return 无。
 */
void Class_CAN_Manage_Object::Init(CAN_HandleTypeDef *can_handle)
{
    hcan = can_handle;
    Rx_Buffer_Active = Rx_Buffer_0;
    Rx_Length_Active = 0;
    Rx_Buffer_Ready = Rx_Buffer_1;
    Rx_Length_Ready = 0;
    Drop_Count = 0;
    Notify_Task_Handle = NULL;
    Alive_Flag = 0;
    Alive_Pre_Flag = 0;
    Alive_Online = 0;
    Alive_Changed = 0;
}

/**
 * @brief 当 Ready 缓冲区为空时，把 Active 缓冲区发布为 Ready。
 * @return 无。
 */
void Class_CAN_Manage_Object::PublishActiveToReady()
{
    if (Rx_Length_Active == 0)
    {
        return;
    }

    if (Rx_Length_Ready != 0)
    {
        return;
    }

    Rx_Buffer_Ready = Rx_Buffer_Active;
    Rx_Length_Ready = Rx_Length_Active;

    if (Rx_Buffer_Active == Rx_Buffer_0)
    {
        Rx_Buffer_Active = Rx_Buffer_1;
    }
    else
    {
        Rx_Buffer_Active = Rx_Buffer_0;
    }

    Rx_Length_Active = 0;
}

/**
 * @brief 在任务读取前，必要时把 Active 缓冲区提升为 Ready。
 * @return 无。
 */
void Class_CAN_Manage_Object::PromoteActiveToReadyIfNeeded()
{
    if ((Rx_Length_Ready == 0) && (Rx_Length_Active > 0))
    {
        Rx_Buffer_Ready = Rx_Buffer_Active;
        Rx_Length_Ready = Rx_Length_Active;

        if (Rx_Buffer_Active == Rx_Buffer_0)
        {
            Rx_Buffer_Active = Rx_Buffer_1;
        }
        else
        {
            Rx_Buffer_Active = Rx_Buffer_0;
        }

        Rx_Length_Active = 0;
    }
}

/**
 * @brief 配置当前 CAN 句柄的硬件滤波器。
 * @return 无。
 * @details 当前采用“全通 + 软件筛选”的策略，统一接收到 FIFO0。
 */
void Class_CAN_Manage_Object::FilterConfig()
{
    CAN_FilterTypeDef sFilterConfig = {};

    if (hcan == NULL)
    {
        return;
    }

    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterBank = (hcan->Instance == CAN2) ? 14 : 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    (void)HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
}

/**
 * @brief 启动 CAN 总线并使能 FIFO0 接收中断通知。
 * @return 无。
 */
void Class_CAN_Manage_Object::Start()
{
    int can_index;

    if (hcan == NULL)
    {
        return;
    }

    can_index = CAN_Get_Index(hcan);
    if (can_index < 0)
    {
        return;
    }

    if (Rx_Buffer_Active == NULL)
    {
        Init(hcan);
    }

    if (can_started[can_index] != 0u)
    {
        return;
    }

    FilterConfig();

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return;
    }

    can_started[can_index] = 1u;
}

/**
 * @brief 注册 Ready 缓冲区就绪时需要通知的任务。
 * @param task_handle 任务句柄。
 * @return 无。
 */
void Class_CAN_Manage_Object::RegisterNotifyTask(TaskHandle_t task_handle)
{
    taskENTER_CRITICAL();
    Notify_Task_Handle = task_handle;
    taskEXIT_CRITICAL();
}

/**
 * @brief 发送一帧标准 CAN 数据帧。
 * @param send_id 标准帧 ID。
 * @param data 指向 8 字节发送数据的指针。
 * @return 无。
 */
void Class_CAN_Manage_Object::Send(uint32_t send_id, uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if ((hcan == NULL) || (data == NULL))
    {
        return;
    }

    tx_header.StdId = send_id;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    (void)HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}

/**
 * @brief FIFO0 接收中断的软件处理流程。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Class_CAN_Manage_Object::ReceiveCallback()
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hcan == NULL)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        Alive_Flag++;

        if (Rx_Length_Active < CAN_RX_BUFFER_SIZE)
        {
            uint16_t write_index = Rx_Length_Active;
            Rx_Buffer_Active[write_index].rx_header = rx_header;
            memcpy(Rx_Buffer_Active[write_index].rx_data, rx_data, 8);
            Rx_Length_Active++;
        }
        else
        {
            Drop_Count++;
        }
    }

    PublishActiveToReady();

    if ((Rx_Length_Ready > 0u) && (Notify_Task_Handle != NULL))
    {
        vTaskNotifyGiveFromISR(Notify_Task_Handle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 以先进先出方式读取一帧 CAN 报文。
 * @param rx_message 输出报文缓存。
 * @return HAL_OK 表示读取成功，HAL_ERROR 表示无可读报文或参数无效。
 */
HAL_StatusTypeDef Class_CAN_Manage_Object::ReadMessage(CAN_RX_MESSAGE *rx_message)
{
    uint16_t i;

    if (rx_message == NULL)
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();

    PromoteActiveToReadyIfNeeded();

    if (Rx_Length_Ready == 0u)
    {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }

    *rx_message = Rx_Buffer_Ready[0];

    for (i = 1u; i < Rx_Length_Ready; i++)
    {
        Rx_Buffer_Ready[i - 1u] = Rx_Buffer_Ready[i];
    }

    Rx_Length_Ready--;

    taskEXIT_CRITICAL();

    return HAL_OK;
}

/**
 * @brief 按标准帧 ID 读取一帧匹配报文。
 * @param std_id 目标标准帧 ID。
 * @param rx_message 输出报文缓存。
 * @return HAL_OK 表示找到并读取成功，HAL_ERROR 表示未找到匹配报文或参数无效。
 */
HAL_StatusTypeDef Class_CAN_Manage_Object::ReadMessageByStdId(uint32_t std_id, CAN_RX_MESSAGE *rx_message)
{
    uint16_t i;
    uint16_t j;

    if (rx_message == NULL)
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();

    PromoteActiveToReadyIfNeeded();

    for (i = 0u; i < Rx_Length_Ready; i++)
    {
        if (Rx_Buffer_Ready[i].rx_header.StdId == std_id)
        {
            *rx_message = Rx_Buffer_Ready[i];

            for (j = i + 1u; j < Rx_Length_Ready; j++)
            {
                Rx_Buffer_Ready[j - 1u] = Rx_Buffer_Ready[j];
            }

            Rx_Length_Ready--;
            taskEXIT_CRITICAL();
            return HAL_OK;
        }
    }

    taskEXIT_CRITICAL();

    return HAL_ERROR;
}

/**
 * @brief 执行一次 100ms 周期的在线状态检查。
 * @return 无。
 */
void Class_CAN_Manage_Object::AliveCheck100ms()
{
    uint8_t online_new;

    online_new = (uint8_t)(Alive_Flag != Alive_Pre_Flag);
    Alive_Pre_Flag = Alive_Flag;

    if (online_new != Alive_Online)
    {
        Alive_Online = online_new;
        Alive_Changed = 1u;
    }
}

/**
 * @brief 获取当前链路在线状态。
 * @return 0 表示离线，1 表示在线。
 */
uint8_t Class_CAN_Manage_Object::AliveIsOnline() const
{
    return Alive_Online;
}

/**
 * @brief 消费一次“在线状态发生变化”的事件。
 * @param online 若非空，则返回当前在线状态。
 * @return 0 表示状态无变化，1 表示状态有变化。
 */
uint8_t Class_CAN_Manage_Object::AliveTryConsumeChanged(uint8_t *online)
{
    uint8_t changed;

    taskENTER_CRITICAL();

    changed = Alive_Changed;
    if (changed != 0u)
    {
        Alive_Changed = 0u;
        //这个是保护 online 指针不为空，避免段错误
        if (online != NULL)
        {
            *online = Alive_Online;
        }
    }

    taskEXIT_CRITICAL();

    return changed;
}

extern "C" {

/**
 * @brief HAL FIFO0 接收中断回调。
 * @param hcan HAL CAN 句柄。
 * @return 无。
 * @details 这是 HAL 官方回调入口，不使用 @novel 标记。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Class_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);

    if (manage == NULL)
    {
        return;
    }

    CAN_Bind_If_Needed(manage, hcan);
    manage->ReceiveCallback();
}

}
