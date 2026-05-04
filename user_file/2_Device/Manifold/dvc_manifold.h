/**
 * @file dvc_manifold.h
 * @brief Manifold 视觉链路设备接口与管理对象定义。
 * @details
 * 本文件定义 `Class_Manifold` Manifold 协议管理对象。
 */
#ifndef __DVC_MANIFOLD_H__
#define __DVC_MANIFOLD_H__

#include "drv_usb.h"
#include "alg_quaternion.h"
#include <stdint.h>

#define MANIFOLD_USB_RX_FRAME_LEN (1u + sizeof(float) + sizeof(float) + 1u + 1u)
#define MANIFOLD_USB_TX_FRAME_LEN (1u + sizeof(float) + sizeof(float) + 1u)
#define MANIFOLD_USB_RX_DEBUG_RAW_MAX USB_BUFFER_SIZE

/**
 * @brief 视觉链路总使能状态。
 */
enum Enum_Manifold_Status
{
    Manifold_Status_DISABLE = 0,
    Manifold_Status_ENABLE,
};

/**
 * @brief 视觉识别敌方颜色枚举。
 */
enum Enum_Manifold_Enemy_Color
{
    Manifold_Enemy_Color_RED = 0,
    Manifold_Enemy_Color_BLUE,
};

/**
 * @brief 视觉识别敌方目标编号枚举。
 */
enum Enum_Manifold_Enemy_ID
{
    Manifold_Enemy_ID_NONE_0 = 0,
    Manifold_Enemy_ID_HERO_1 = 1,
    Manifold_Enemy_ID_ENGINEER_2 = 2,
    Manifold_Enemy_ID_INFANTRY_3 = 3,
    Manifold_Enemy_ID_INFANTRY_4 = 4,
    Manifold_Enemy_ID_INFANTRY_5 = 5,
    Manifold_Enemy_ID_SENTRY_7 = 7,
    Manifold_Enemy_ID_OUTPOST = 8,
    Manifold_Enemy_ID_RUNE = 9,
};

/**
 * @brief 视觉链路哨兵模式枚举。
 */
enum Enum_Manifold_Sentry_Mode
{
    Manifold_Sentry_Mode_DISABLE = 0,
    Manifold_Sentry_Mode_ENABLE,
};

/**
 * @brief USB 接收目标帧。
 */
typedef struct
{
    uint8_t Frame_Header;                 /**< 帧头 */
    float Taget_Yaw;                      /**< 目标偏航角 */
    float Taget_Pitch;                    /**< 目标俯仰角 */
    uint8_t Target_Valid;                 /**< 当前目标是否有效 */
    uint8_t Frame_Tail;                   /**< 帧尾 */
    enum Enum_Manifold_Enemy_ID Enemy_ID; /**< 当前目标敌方编号 */
} Manifold_UART_Rx_Data;

/**
 * @brief USB 发送姿态帧。
 */
typedef struct
{
    uint8_t Frame_Header; /**< 帧头 */
    float Yaw;            /**< 输出偏航角 */
    float Pitch;          /**< 输出俯仰角 */
    uint8_t Frame_Tail;   /**< 帧尾 */
} Manifold_UART_Tx_Data;

#ifdef __cplusplus
/**
 * @class Class_Manifold
 * @brief Manifold 视觉链路管理对象。
 * @details
 * 负责 Manifold 协议初始化、USB 流式拼帧、目标数据解析和姿态数据回传。
 */
class Class_Manifold
{
public:
    uint8_t Frame_Header;                              /**< 当前协议帧头 */
    uint8_t Frame_Tail;                                /**< 当前协议帧尾 */
    uint8_t Rx_Frame_Buffer[MANIFOLD_USB_RX_FRAME_LEN];/**< 流式接收拼帧缓存 */
    uint8_t Rx_Index;                                  /**< 当前流式接收索引 */

    void Init(Manifold_UART_Tx_Data *data,
              uint8_t frame_header,
              uint8_t frame_end,
              enum Enum_Manifold_Sentry_Mode sentry_mode);
    void ClearTarget();
    void UsbRxCallback(uint8_t *buf, uint32_t len);
    void SendData(Manifold_UART_Tx_Data *data, euler_t euler);

private:
    void RxResync(uint8_t byte);
    uint8_t AngleIsValid(float pitch, float yaw) const;
    uint8_t DecodeRxFrame(const uint8_t *frame,
                         float *pitch,
                         float *yaw,
                         uint8_t *target_valid) const;
};
extern Class_Manifold Manifold_Manage_Object;
#endif

extern Manifold_UART_Rx_Data Rx_Data;
extern volatile uint8_t Manifold_USB_Tx_Debug_Frame[MANIFOLD_USB_TX_FRAME_LEN];
extern volatile uint16_t Manifold_USB_Tx_Debug_Len;
extern volatile uint8_t Manifold_USB_Rx_Debug_Raw[MANIFOLD_USB_RX_DEBUG_RAW_MAX];
extern volatile uint16_t Manifold_USB_Rx_Debug_Raw_Len;
extern volatile uint8_t Manifold_USB_Rx_Debug_Frame[MANIFOLD_USB_RX_FRAME_LEN];
extern volatile uint16_t Manifold_USB_Rx_Debug_Frame_Len;
extern volatile uint32_t Manifold_USB_Rx_Frame_Seq;

#endif /* __DVC_MANIFOLD_H__ */
