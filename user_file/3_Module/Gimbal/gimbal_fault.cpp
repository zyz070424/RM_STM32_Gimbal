/**
 * @file gimbal_fault.cpp
 * @brief 云台故障状态实现。但是还没有想好怎么样处理错误，还有通信的错误类型我还没有确定。这里先留白。
 */
#include "gimbal_fault.h"

Class_Gimbal_Fault Gimbal_Fault_Object = {};

/**
 * @brief 初始化云台故障状态。
 * @return 无。
 */
void Class_Gimbal_Fault::Init()
{
    Active_Bits = 0u;
    Latched_Bits = 0u;
    Fault_Count = 0u;
    Last_Fault_Tick = 0u;
}

/**
 * @brief 设置指定故障位。
 * @param bits 要设置的故障位掩码。
 * @param latched 是否同时写入锁存故障位。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Fault::SetBits(uint32_t bits, uint8_t latched, uint32_t now_tick)
{
    uint32_t new_bits;

    if (bits == 0u)
    {
        return;
    }
    // 只设置新增激活的故障位，避免重复计数。
    new_bits = bits & (~Active_Bits);
    // 仅在故障首次激活时累计次数，避免持续故障每拍都增加计数。
    if (new_bits != 0u)
    {
        Fault_Count++;
        Last_Fault_Tick = now_tick;
    }

    Active_Bits |= bits;
    if (latched != 0u)
    {
        Latched_Bits |= bits;
    }
}

/**
 * @brief 清除指定故障位。
 * @param bits 要清除的故障位掩码。
 * @param clear_latched 是否同时清除锁存故障位。
 * @return 无。
 */
void Class_Gimbal_Fault::ClearBits(uint32_t bits, uint8_t clear_latched)
{
    Active_Bits &= ~bits;
    if (clear_latched != 0u)
    {
        Latched_Bits &= ~bits;
    }
}

/**
 * @brief 按当前状态同步单个故障位。
 * @param bit 要同步的故障位。
 * @param active 当前是否激活。
 * @param latched 当前是否按锁存故障写入。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Fault::SyncBit(uint32_t bit, uint8_t active, uint8_t latched, uint32_t now_tick)
{
    if (active != 0u)
    {
        SetBits(bit, latched, now_tick);
    }
    else
    {
        // 这里只同步清除活动位；锁存位仍需要走显式清除流程。
        ClearBits(bit, 0u);
    }
}

/**
 * @brief 判断当前是否存在任意 active fault。
 * @return 存在返回 1，否则返回 0。
 */
uint8_t Class_Gimbal_Fault::HasAny() const
{
    return (Active_Bits != 0u) ? 1u : 0u;
}

/**
 * @brief 判断指定故障位当前是否激活。
 * @param bit 要查询的故障位。
 * @return 激活返回 1，否则返回 0。
 */
uint8_t Class_Gimbal_Fault::HasBit(uint32_t bit) const
{
    return ((Active_Bits & bit) != 0u) ? 1u : 0u;
}
