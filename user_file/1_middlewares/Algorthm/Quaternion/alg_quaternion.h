/**
 * @file alg_quaternion.h
 * @brief 四元数姿态算法接口与管理对象定义。
 * @details
 * 本文件定义四元数、向量、欧拉角与 IMU 数据结构，
 * 同时提供 Mahony 姿态解算管理对象。
 */
#ifndef __ALG_QUATERNION_H__
#define __ALG_QUATERNION_H__

#include <math.h>
#include <stdint.h>

/**
 * @brief 四元数数据结构。
 */
typedef struct
{
    float w; /**< 标量部 */
    float x; /**< X 虚部 */
    float y; /**< Y 虚部 */
    float z; /**< Z 虚部 */
} quat_t;

/**
 * @brief 三维向量数据结构。
 */
typedef struct
{
    float x; /**< X 分量 */
    float y; /**< Y 分量 */
    float z; /**< Z 分量 */
} vec3_t;

/**
 * @brief 欧拉角数据结构。
 */
typedef struct
{
    float roll;  /**< 横滚角 */
    float pitch; /**< 俯仰角 */
    float yaw;   /**< 偏航角 */
} euler_t;

/**
 * @brief IMU 输入数据结构。
 */
typedef struct
{
    vec3_t gyro; /**< 陀螺仪角速度 */
    vec3_t acc;  /**< 加速度计数据 */
    float temp;  /**< 温度 */
    float dt;    /**< 当前采样周期，单位秒 */
    float Kp;    /**< 姿态解算使用的比例增益 */
    float Ki;    /**< 姿态解算使用的积分增益 */
} imu_data_t;

/**
 * @brief Mahony 解算调试状态。
 */
typedef struct
{
    float acc_norm_raw;    /**< 当前加速度模长 */
    float acc_weight;      /**< 当前加速度权重 */
    float dt_used;         /**< 本次实际采用的 dt */
    uint8_t gyro_only_mode; /**< 是否退化为仅陀螺模式 */
} mahony_debug_t;

#ifdef __cplusplus
/**
 * @class Class_Mahony_Quaternion
 * @brief Mahony 四元数姿态解算管理对象。
 * @details
 * 负责维护积分反馈状态，并提供四元数转欧拉角与 Mahony 单步更新能力。
 */
class Class_Mahony_Quaternion
{
public:
    float Integral_FB_X; /**< X 轴积分反馈项 */
    float Integral_FB_Y; /**< Y 轴积分反馈项 */
    float Integral_FB_Z; /**< Z 轴积分反馈项 */

    euler_t QuatToEuler(quat_t q);
    void MahonyUpdate(quat_t *q, imu_data_t imu, float kp, float ki);

private:
    float InvSqrt(float x) const;
    float SanitizeDt(float dt_s) const;
    void ResetQuat(quat_t *q);
    void ResetIntegral();
};
extern Class_Mahony_Quaternion Mahony_Quaternion_Object;
#endif
extern volatile mahony_debug_t g_mahony_debug;

#endif /* __ALG_QUATERNION_H__ */
