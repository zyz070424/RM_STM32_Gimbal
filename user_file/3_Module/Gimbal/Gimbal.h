/**
 * @file Gimbal.h
 * @brief 云台模块接口与管理对象定义。
 * @details
 * 本文件定义 `Class_Gimbal` 云台管理对象，
 * 以及供 Core 中断入口继续使用的自定义回调声明。
 */
#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include <stdint.h>

#ifdef __cplusplus
#include "dvc_motor.h"
#include "dvc_manifold.h"
#include "dvc_bmi088.h"
#include "FreeRTOS.h"
#include "task.h"
#include "alg_quaternion.h"
#include "main.h"
#include "spi.h"
#include "can.h"
#include <string.h>
#include "alg_pid.h"
#include "gimbal_sentry.h"
#include "alg_dwt.h"
#include "gimbal_debug.h"
#include "common_math.h"
#include "portmacro.h"
#include "stm32f4xx_hal.h"
#include <math.h>


/**
 * @class Class_Gimbal
 * @brief 云台模块管理对象。
 * @details
 * 负责云台初始化、双轴控制任务、IMU 解算任务、视觉姿态发送任务、
 * 设备在线检测任务以及自定义 IMU 就绪中断回调处理。
 */
class Class_Gimbal
{
public:
    Class_Motor Motor_Pitch;                 /**< Pitch 轴电机对象 */
    Class_Motor Motor_Yaw;                   /**< Yaw 轴电机对象 */
    imu_data_t Imu_Data;                     /**< 当前 IMU 原始与解算输入数据 */
    euler_t Euler_Angle_To_Send;             /**< 当前对外发布的欧拉角 */
    euler_t Euler_Angle_Ekf_To_Send;         /**< 并行 EKF 路径输出的欧拉角 */
    Manifold_UART_Tx_Data Tx_Data;           /**< 发给视觉的姿态帧缓存 */
    TaskHandle_t Imu_Task_Handle;            /**< IMU 任务句柄 */
    Class_DWT_Timebase Imu_Timebase;         /**< IMU 周期估计时间基 */
    volatile float Imu_Last_Dt_S;            /**< 最近一次 IMU dt，单位 s */
    volatile uint8_t Imu_Last_Dt_From_Dwt;   /**< 最近一次 dt 是否来自 DWT */
    float Yaw_Test_Target_Deg;               /**< 调试用 Yaw 目标角 */
    float Pitch_Test_Target_Deg;             /**< 调试用 Pitch 目标角 */
    float Pitch_Current_Target_Deg;          /**< 当前 pitch 角度目标缓存 */
    float Pitch_Current_Target_Speed;        /**< 当前 pitch 速度目标缓存 */
    float Pitch_Current_Output;              /**< 当前 pitch 输出缓存 */
    float Yaw_Current_Target_Deg;            /**< 当前 yaw 角度目标缓存 */
    float Yaw_Current_Target_Speed;          /**< 当前 yaw 速度目标缓存 */
    float Yaw_Current_Output;                /**< 当前 yaw 输出缓存 */

    void Init(void *params);
    void EulerTask(void *params);
    void MotorControlTask(void *params);
    void TaskLoop(void *params);
    void ManifoldControlTask(void *params);
    void ImuExtiCallback(uint16_t gpio_pin);

private:
    static int16_t OutputToCanZero(float value);
    static int16_t OutputToCanNormal(float value);
};
extern Class_Gimbal Gimbal_Object;
#endif



#ifdef __cplusplus
extern "C" {
#endif

void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_H__ */
