/**
 * @file gimbal_status.cpp
 * @brief 云台在线状态实现。
 */
#include "gimbal_status.h"

#include "Gimbal.h"
#include "dvc_manifold.h"
#include "dvc_bmi088.h"
#include "drv_can.h"
#include "drv_spi.h"
#include "drv_usb.h"
#include "gimbal_fault.h"
#include "gimbal_sentry_target.h"

Class_Gimbal_Status Gimbal_Status_Object = {};

/**
 * @brief 初始化云台在线状态管理对象。
 * @return 无。
 */
void Class_Gimbal_Status::Init()
{
    alive_check_div = 0u;
    can_online = CAN2_Manage_Object.AliveIsOnline();
    spi_online = SPI1_Manage_Object.AliveIsOnline();
    usb_online = USB_Manage_Object.AliveIsOnline();
}

/**
 * @brief 在任务循环中推进在线检测与状态同步。
 * @param gimbal 云台对象指针。
 * @param fault 云台故障对象指针。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Status::TaskUpdate(Class_Gimbal *gimbal,
                                     Class_Gimbal_Fault *fault,
                                     uint32_t now_tick)
{
    uint8_t online_changed;

    if ((gimbal == nullptr) || (fault == nullptr))
    {
        return;
    }

    if (++alive_check_div >= 100u)
    {
        alive_check_div = 0u;
        // TaskLoop 仍按 1ms 调度，但 alive 检测继续保持原先的 100ms 节奏。
        CAN2_Manage_Object.AliveCheck100ms();
        SPI1_Manage_Object.AliveCheck100ms();
        USB_Manage_Object.AliveCheck100ms();
    }

    if (CAN2_Manage_Object.AliveTryConsumeChanged(&online_changed) != 0u)
    {
        HandleCanAliveChange(gimbal, fault, online_changed, now_tick);
    }

    if (SPI1_Manage_Object.AliveTryConsumeChanged(&online_changed) != 0u)
    {
        HandleSpiAliveChange(gimbal, fault, online_changed, now_tick);
    }

    if (USB_Manage_Object.AliveTryConsumeChanged(&online_changed) != 0u)
    {
        HandleUsbAliveChange(fault, online_changed, now_tick);
    }

    // 即使本拍没有 changed 事件，也把当前在线状态同步进 fault，避免漏报。
    fault->SyncBit(GIMBAL_FAULT_CAN_OFFLINE,
                   (can_online == 0u) ? 1u : 0u,
                   0u,
                   now_tick);
    fault->SyncBit(GIMBAL_FAULT_SPI_OFFLINE,
                   (spi_online == 0u) ? 1u : 0u,
                   0u,
                   now_tick);
    fault->SyncBit(GIMBAL_FAULT_USB_OFFLINE,
                   (usb_online == 0u) ? 1u : 0u,
                   0u,
                   now_tick);
}

/**
 * @brief 获取当前 CAN 在线状态。
 * @return 在线返回 1，离线返回 0。
 */
uint8_t Class_Gimbal_Status::IsCanOnline() const
{
    return can_online;
}

/**
 * @brief 获取当前 SPI 在线状态。
 * @return 在线返回 1，离线返回 0。
 */
uint8_t Class_Gimbal_Status::IsSpiOnline() const
{
    return spi_online;
}

/**
 * @brief 获取当前 USB 在线状态。
 * @return 在线返回 1，离线返回 0。
 */
uint8_t Class_Gimbal_Status::IsUsbOnline() const
{
    return usb_online;
}

/**
 * @brief 处理 CAN 在线状态变化。
 * @param gimbal 云台对象指针。
 * @param fault 云台故障对象指针。
 * @param online 新的在线状态。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Status::HandleCanAliveChange(Class_Gimbal *gimbal,
                                               Class_Gimbal_Fault *fault,
                                               uint8_t online,
                                               uint32_t now_tick)
{
    can_online = (online != 0u) ? 1u : 0u;
    fault->SyncBit(GIMBAL_FAULT_CAN_OFFLINE,
                   (can_online == 0u) ? 1u : 0u,
                   0u,
                   now_tick);
    if (can_online != 0u)
    {
        return;
    }

    // CAN 离线时立即清理电机运行态和当前目标，避免恢复后继承旧状态。
    gimbal->Motor_Pitch.ClearRuntime();
    gimbal->Motor_Yaw.ClearRuntime();
    Gimbal_Sentry_Target_Object.ClearOutput();
}

/**
 * @brief 处理 SPI 在线状态变化。
 * @param gimbal 云台对象指针。
 * @param fault 云台故障对象指针。
 * @param online 新的在线状态。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Status::HandleSpiAliveChange(Class_Gimbal *gimbal,
                                               Class_Gimbal_Fault *fault,
                                               uint8_t online,
                                               uint32_t now_tick)
{
    spi_online = (online != 0u) ? 1u : 0u;
    fault->SyncBit(GIMBAL_FAULT_SPI_OFFLINE,
                   (spi_online == 0u) ? 1u : 0u,
                   0u,
                   now_tick);
    if (spi_online == 0u)
    {
        // SPI/IMU 离线时清空对外姿态输出，避免上层继续消费失效姿态。
        BMI088_Manage_Object.YawContinuousReset();
        BMI088_Manage_Object.QuaternionEkfReset();
        gimbal->Euler_Angle_To_Send.roll = 0.0f;
        gimbal->Euler_Angle_To_Send.pitch = 0.0f;
        gimbal->Euler_Angle_To_Send.yaw = 0.0f;
        gimbal->Euler_Angle_Ekf_To_Send.roll = 0.0f;
        gimbal->Euler_Angle_Ekf_To_Send.pitch = 0.0f;
        gimbal->Euler_Angle_Ekf_To_Send.yaw = 0.0f;
        return;
    }

    // SPI 恢复在线后只重置传感链状态，不在这里直接介入控制裁决。
    BMI088_Manage_Object.YawContinuousReset();
    BMI088_Manage_Object.QuaternionEkfReset();
}

/**
 * @brief 处理 USB 在线状态变化。
 * @param fault 云台故障对象指针。
 * @param online 新的在线状态。
 * @param now_tick 当前系统 tick。
 * @return 无。
 */
void Class_Gimbal_Status::HandleUsbAliveChange(Class_Gimbal_Fault *fault,
                                               uint8_t online,
                                               uint32_t now_tick)
{
    usb_online = (online != 0u) ? 1u : 0u;
    fault->SyncBit(GIMBAL_FAULT_USB_OFFLINE,
                   (usb_online == 0u) ? 1u : 0u,
                   0u,
                   now_tick);
    if (usb_online != 0u)
    {
        return;
    }

    // USB 视觉链路离线时清协议层目标和目标缓存，避免恢复后沿用旧目标。
    Manifold_Manage_Object.ClearTarget();
    Gimbal_Sentry_Target_Object.ResetVision();
}
