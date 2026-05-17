/**
 * @file gimbal_status.h
 * @brief 云台在线状态接口与管理对象定义。
 */
#ifndef __GIMBAL_STATUS_H__
#define __GIMBAL_STATUS_H__

#include <stdint.h>

#ifdef __cplusplus
class Class_Gimbal;
class Class_Gimbal_Fault;

/**
 * @class Class_Gimbal_Status
 * @brief 云台在线状态管理对象。
 * @details
 * 负责推进 CAN / SPI / USB 的 alive 检测，处理在线状态变化，并将结果同步到 fault。
 */
class Class_Gimbal_Status
{
public:
    void Init();
    void TaskUpdate(Class_Gimbal *gimbal,
                    Class_Gimbal_Fault *fault,
                    uint32_t now_tick);
    uint8_t IsCanOnline() const;
    uint8_t IsSpiOnline() const;
    uint8_t IsUsbOnline() const;

private:
    void HandleCanAliveChange(Class_Gimbal *gimbal,
                              Class_Gimbal_Fault *fault,
                              uint8_t online,
                              uint32_t now_tick);
    void HandleSpiAliveChange(Class_Gimbal *gimbal,
                              Class_Gimbal_Fault *fault,
                              uint8_t online,
                              uint32_t now_tick);
    void HandleUsbAliveChange(Class_Gimbal_Fault *fault,
                               uint8_t online,
                               uint32_t now_tick);

    uint16_t alive_check_div;  /**< alive 检测分频计数器 */
    uint8_t can_online;        /**< 当前 CAN 在线状态 */
    uint8_t spi_online;        /**< 当前 SPI 在线状态 */
    uint8_t usb_online;        /**< 当前 USB 在线状态 */
};

extern Class_Gimbal_Status Gimbal_Status_Object;
#endif

#endif /* __GIMBAL_STATUS_H__ */
