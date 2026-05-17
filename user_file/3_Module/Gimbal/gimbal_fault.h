/**
 * @file gimbal_fault.h
 * @brief 云台故障状态接口与管理对象定义。
 */
#ifndef __GIMBAL_FAULT_H__
#define __GIMBAL_FAULT_H__

#include <stdint.h>

/**
 * @brief 云台故障位定义。
 * @details
 * 使用位图表达当前云台的多个独立故障源，便于 Ozone 直接观察和按位扩展。
 */
typedef enum
{
    GIMBAL_FAULT_NONE = 0u,
    GIMBAL_FAULT_CAN_OFFLINE = (1u << 0),
    GIMBAL_FAULT_SPI_OFFLINE = (1u << 1),
    GIMBAL_FAULT_USB_OFFLINE = (1u << 2),
    GIMBAL_FAULT_PITCH_PROTECT = (1u << 3),
} Gimbal_Fault_Bit_TypeDef;

#ifdef __cplusplus
/**
 * @class Class_Gimbal_Fault
 * @brief 云台故障状态管理对象。
 * @details
 * 负责维护当前 active fault、latched fault、故障计数以及最近一次故障时间戳。
 */
class Class_Gimbal_Fault
{
public:
    uint32_t Active_Bits;      /**< 当前 active fault 位图 */
    uint32_t Latched_Bits;     /**< 当前 latched fault 位图 */
    uint32_t Fault_Count;      /**< 故障首次激活累计次数 */
    uint32_t Last_Fault_Tick;  /**< 最近一次故障激活时间戳 */

    void Init();
    void SetBits(uint32_t bits, uint8_t latched, uint32_t now_tick);
    void ClearBits(uint32_t bits, uint8_t clear_latched);
    void SyncBit(uint32_t bit, uint8_t active, uint8_t latched, uint32_t now_tick);
    uint8_t HasAny() const;
    uint8_t HasBit(uint32_t bit) const;
};

extern Class_Gimbal_Fault Gimbal_Fault_Object;
#endif

#endif /* __GIMBAL_FAULT_H__ */
