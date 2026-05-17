/**
 * @file gimbal_debug.h
 * @brief 云台调试观察量与调试命令接口定义。
 */
#ifndef __GIMBAL_DEBUG_H__
#define __GIMBAL_DEBUG_H__

#include "FreeRTOS.h"
#include <stdint.h>

/**
 * @brief 云台视觉联调观察量。
 */
typedef struct
{
    uint8_t target_valid;            /**< 当前视觉目标是否有效 */
    uint8_t target_ever_valid;       /**< 是否曾收到过有效视觉目标 */
    uint8_t filter_tracking;         /**< 当前视觉滤波器是否进入跟踪状态 */
    uint8_t sentry_state;            /**< 当前 sentry 状态 */
    uint32_t rx_frame_seq;           /**< 最近视觉帧序号 */
    uint32_t last_valid_tick;        /**< 最近一次有效视觉目标时间戳 */
    float raw_pitch_deg;             /**< 当前原始视觉 pitch，单位：deg */
    float raw_yaw_deg;               /**< 当前原始视觉 yaw，单位：deg */
    float cached_pitch_deg;          /**< 最近一次有效视觉 pitch，单位：deg */
    float cached_yaw_deg;            /**< 最近一次有效视觉 yaw，单位：deg */
    float filtered_pitch_deg;        /**< 当前滤波后视觉 pitch，单位：deg */
    float filtered_yaw_deg;          /**< 当前滤波后视觉 yaw，单位：deg */
} Gimbal_Debug_Vision_View_TypeDef;

/**
 * @brief 云台通信与系统观察量。
 */
typedef struct
{
    uint8_t can_online;              /**< 当前 CAN 在线状态 */
    uint8_t spi_online;              /**< 当前 SPI 在线状态 */
    uint8_t usb_online;              /**< 当前 USB 在线状态 */
    uint32_t fault_bits;             /**< 当前 active fault 位图 */
    uint32_t fault_count;            /**< 故障首次激活累计次数 */
    uint32_t last_fault_tick;        /**< 最近一次故障激活时间戳 */
    float imu_dt_s;                  /**< 最近一次 IMU dt，单位：s */
    uint8_t imu_dt_from_dwt;         /**< 最近一次 IMU dt 是否来自 DWT */
} Gimbal_Debug_Comm_View_TypeDef;

/**
 * @brief 云台电机与控制观察量。
 */
typedef struct
{
    uint8_t pitch_protect_enabled;         /**< 当前 pitch 保护是否启用 */
    uint8_t pitch_protect_fault;           /**< 当前 pitch 保护是否故障 */
    uint8_t pitch_protect_force_disabled;  /**< 当前是否被 debug 强制关闭 pitch 保护 */

    float pitch_angle_target;              /**< 当前 pitch 角度目标，单位：deg */
    float pitch_angle_feedback;            /**< 当前 pitch 角度反馈，单位：deg */
    float pitch_speed_target;              /**< 当前 pitch 速度目标 */
    float pitch_speed_feedback;            /**< 当前 pitch 速度反馈 */
    float pitch_output;                    /**< 当前 pitch 输出 */

    float yaw_angle_target;                /**< 当前 yaw 角度目标，单位：deg */
    float yaw_angle_feedback;              /**< 当前 yaw 角度反馈，单位：deg */
    float yaw_speed_target;                /**< 当前 yaw 速度目标 */
    float yaw_speed_feedback;              /**< 当前 yaw 速度反馈 */
    float yaw_output;                      /**< 当前 yaw 输出 */
} Gimbal_Debug_Motor_View_TypeDef;

/**
 * @brief 云台总调试观察窗口。
 */
typedef struct
{
    Gimbal_Debug_Vision_View_TypeDef vision;  /**< 视觉联调观察量 */
    Gimbal_Debug_Comm_View_TypeDef comm;      /**< 通信与系统观察量 */
    Gimbal_Debug_Motor_View_TypeDef motor;    /**< 电机与控制观察量 */
} Gimbal_Debug_View_TypeDef;

/**
 * @brief 云台 PID 调试对象枚举。
 */
typedef enum : uint8_t
{
    GIMBAL_DEBUG_PID_PITCH_SPEED = 0u,  /**< Pitch 速度环 */
    GIMBAL_DEBUG_PID_PITCH_ANGLE = 1u,  /**< Pitch 角度环 */
    GIMBAL_DEBUG_PID_YAW_SPEED   = 2u,  /**< Yaw 速度环 */
    GIMBAL_DEBUG_PID_YAW_ANGLE   = 3u,  /**< Yaw 角度环 */
} Gimbal_Debug_Pid_Target_TypeDef;

/**
 * @brief 云台调试命令区。
 */
typedef struct
{
    uint8_t enable;                   /**< 调试命令总开关 */
    uint8_t clear_fault;              /**< 清除 fault 与 pitch 保护故障 */
    uint8_t disable_pitch_protect;    /**< 强制关闭 pitch 保护 */
    uint8_t write_pid;                /**< 连续写入当前选中 PID 参数 */

    Gimbal_Debug_Pid_Target_TypeDef pid_target;               /**< 当前 PID 调试对象，见 `Gimbal_Debug_Pid_Target_TypeDef` */

    float pid_kp;                     /**< 待写入 PID kp */
    float pid_ki;                     /**< 待写入 PID ki */
    float pid_kd;                     /**< 待写入 PID kd */
    float FeedForward;                /**< 待写入 PID 前馈 */
} Gimbal_Debug_Cmd_TypeDef;

extern volatile Gimbal_Debug_View_TypeDef Gimbal_Debug_View;
extern volatile Gimbal_Debug_Cmd_TypeDef Gimbal_Debug_Cmd;

#ifdef __cplusplus
class Class_Gimbal;

void Gimbal_Debug_Init(Class_Gimbal *gimbal);
void Gimbal_Debug_HandleCmd(Class_Gimbal *gimbal);
void Gimbal_Debug_UpdateVisionView(TickType_t now_tick);
void Gimbal_Debug_UpdateCommView(const Class_Gimbal *gimbal);
void Gimbal_Debug_UpdateMotorView(const Class_Gimbal *gimbal);
uint8_t Gimbal_Debug_IsPitchProtectForceDisabled();
#endif

#endif /* __GIMBAL_DEBUG_H__ */
