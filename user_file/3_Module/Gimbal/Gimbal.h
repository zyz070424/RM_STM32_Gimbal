#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "drv_usb.h"
#include "dvc_motor.h"
#include "dvc_manifold.h"
#include "dvc_bmi088.h"
#include "FreeRTOS.h"
#include "alg_quaternion.h"
#include "yaw_fusion.h"
#include "main.h"
#include "spi.h"
#include "can.h"
#include <stdint.h>
#include <string.h>
#include "alg_pid.h"
#include "gimbal_sentry.h"
#include "alg_dwt.h"
#include "portmacro.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/**
 * @brief 云台 yaw 融合并行观测量。
 * @note  该结构体只用于和现有 IMU yaw 连续积分方案做对比，不接管主控制环。
 */
typedef struct
{
    YawFusionDebug_t fusion_state; /**< 当前 yaw 融合调试输出 */
    uint16_t encoder_raw_count;    /**< 当前拍原始编码器计数 */
    uint8_t encoder_feedback_valid;/**< 当前拍编码器反馈是否有效 */
    float imu_gyro_z_rad_s;        /**< 当前拍 gyro z 输入，单位：rad/s */
    float acc_norm_g;              /**< 当前拍加速度模长，单位：g */
    float temp_c;                  /**< 当前拍温度输入，单位：摄氏度 */
    float dt_s;                    /**< 当前拍融合使用的 dt，单位：s */
    uint32_t update_count;         /**< 当前拍更新计数 */
} Gimbal_Yaw_Fusion_Observe_TypeDef;

extern Gimbal_Yaw_Fusion_Observe_TypeDef Gimbal_Yaw_Fusion_Observe;

/**
 * @brief 云台并行比对方案最终欧拉角观测量。
 * @note  roll/pitch 保持当前 IMU 解算结果，yaw 使用并行 yaw 融合输出。
 *        该变量仅用于观察，不接管现有控制环。
 */
extern euler_t Gimbal_Euler_Angle_Final_Observe;

void Gimbal_Init(void* pramas);
void Gimbal_Euler(void *pramas);
void Gimbal_Motor_Control_ALL_Test(void* pramas);
void Gimbal_Task(void* pramas);
void Gimbal_Manifold_Control(void *pramas);
/**
 * @brief IMU 数据就绪外部中断回调
 * @note  非官方
 */
void Gimbal_IMU_EXTI_Callback(uint16_t GPIO_Pin);
#endif /* __GIMBAL_H__ */

