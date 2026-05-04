/**
 * @file alg_quaternion.cpp
 * @brief 四元数姿态算法实现。
 * @details
 * 本文件实现 `Class_Mahony_Quaternion` 的成员函数。
 */
#include "alg_quaternion.h"

#define MAHONY_DT_DEFAULT_S         0.001f
#define MAHONY_DT_MIN_S             0.0002f
#define MAHONY_DT_MAX_S             0.0100f
#define MAHONY_ACC_NORM_VALID_MIN_G 0.05f
#define DEG_TO_RAD                  (M_PI / 180.0f)

Class_Mahony_Quaternion Mahony_Quaternion_Object = {};
volatile mahony_debug_t g_mahony_debug = {1.0f, 1.0f, MAHONY_DT_DEFAULT_S, 0u};

/**
 * @brief 计算输入数值的平方根倒数。
 * @param x 待计算的输入值。
 * @return 合法时返回平方根倒数，非法时返回 0。
 */
float Class_Mahony_Quaternion::InvSqrt(float x) const
{
    if ((isfinite(x) == 0) || (x <= 0.0f))
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(x);
}

/**
 * @brief 约束 Mahony 更新使用的时间步长。
 * @param dt_s 待检查的时间步长，单位：s。
 * @return 约束后的时间步长。
 */
float Class_Mahony_Quaternion::SanitizeDt(float dt_s) const
{
    if ((isfinite(dt_s) == 0) || (dt_s <= 0.0f))
    {
        return MAHONY_DT_DEFAULT_S;
    }

    if (dt_s < MAHONY_DT_MIN_S)
    {
        return MAHONY_DT_MIN_S;
    }

    if (dt_s > MAHONY_DT_MAX_S)
    {
        return MAHONY_DT_MAX_S;
    }

    return dt_s;
}

/**
 * @brief 将无效四元数重置为单位四元数。
 * @param q 待重置的四元数指针。
 * @return 无。
 */
void Class_Mahony_Quaternion::ResetQuat(quat_t *q)
{
    if (q == NULL)
    {
        return;
    }

    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

/**
 * @brief 清空跨周期积分反馈状态。
 * @return 无。
 */
void Class_Mahony_Quaternion::ResetIntegral()
{
    Integral_FB_X = 0.0f;
    Integral_FB_Y = 0.0f;
    Integral_FB_Z = 0.0f;
}

/**
 * @brief 四元数转欧拉角。
 * @param q 输入四元数。
 * @return 转换得到的欧拉角，单位：deg。
 * @details 按标准 ZYX 顺序计算 roll-x、pitch-y、yaw-z。
 */
euler_t Class_Mahony_Quaternion::QuatToEuler(quat_t q)
{
    euler_t euler;
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp);

    sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
    {
        euler.pitch = copysignf((float)(M_PI / 2.0f), sinp);
    }
    else
    {
        euler.pitch = asinf(sinp);
    }

    siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp);

    euler.roll *= 180.0f / (float)M_PI;
    euler.pitch *= 180.0f / (float)M_PI;
    euler.yaw *= 180.0f / (float)M_PI;

    return euler;
}

/**
 * @brief 执行一次 Mahony 姿态更新。
 * @param q 当前四元数，作为输入输出参数。
 * @param imu 输入传感器数据，gyro 单位为 deg/s，acc 单位为 g。
 * @param kp 比例增益。
 * @param ki 积分增益。
 * @return 无。
 * @details
 * 内部会先做参数有效性检查，再根据加速度有效性决定是否启用纠偏，
 * 最后完成四元数积分和单位化。
 */
void Class_Mahony_Quaternion::MahonyUpdate(quat_t *q, imu_data_t imu, float kp, float ki)
{
    float recip_norm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;
    float qa;
    float qb;
    float qc;
    float gx;
    float gy;
    float gz;
    float dt_s;
    float acc_norm_raw;
    uint8_t gyro_only_mode = 1u;

    if (q == NULL)
    {
        return;
    }

    if ((isfinite(q->w) == 0) || (isfinite(q->x) == 0) ||
        (isfinite(q->y) == 0) || (isfinite(q->z) == 0))
    {
        ResetQuat(q);
        ResetIntegral();
    }

    if ((isfinite(kp) == 0) || (kp < 0.0f))
    {
        kp = 0.0f;
    }

    if ((isfinite(ki) == 0) || (ki < 0.0f))
    {
        ki = 0.0f;
    }

    if ((isfinite(imu.gyro.x) == 0) || (isfinite(imu.gyro.y) == 0) || (isfinite(imu.gyro.z) == 0))
    {
        imu.gyro.x = 0.0f;
        imu.gyro.y = 0.0f;
        imu.gyro.z = 0.0f;
    }

    dt_s = SanitizeDt(imu.dt);
    gx = imu.gyro.x * DEG_TO_RAD;
    gy = imu.gyro.y * DEG_TO_RAD;
    gz = imu.gyro.z * DEG_TO_RAD;

    acc_norm_raw = sqrtf(imu.acc.x * imu.acc.x + imu.acc.y * imu.acc.y + imu.acc.z * imu.acc.z);
    if ((isfinite(acc_norm_raw) != 0) && (acc_norm_raw > MAHONY_ACC_NORM_VALID_MIN_G))
    {
        recip_norm = InvSqrt(imu.acc.x * imu.acc.x + imu.acc.y * imu.acc.y + imu.acc.z * imu.acc.z);
        if (recip_norm > 0.0f)
        {
            imu.acc.x *= recip_norm;
            imu.acc.y *= recip_norm;
            imu.acc.z *= recip_norm;

            halfvx = q->x * q->z - q->w * q->y;
            halfvy = q->w * q->x + q->y * q->z;
            halfvz = q->w * q->w - 0.5f + q->z * q->z;

            halfex = (imu.acc.y * halfvz - imu.acc.z * halfvy);
            halfey = (imu.acc.z * halfvx - imu.acc.x * halfvz);
            halfez = (imu.acc.x * halfvy - imu.acc.y * halfvx);

            if (ki > 0.0f)
            {
                Integral_FB_X += 2.0f * ki * halfex * dt_s;
                Integral_FB_Y += 2.0f * ki * halfey * dt_s;
                Integral_FB_Z += 2.0f * ki * halfez * dt_s;
                gx += Integral_FB_X;
                gy += Integral_FB_Y;
                gz += Integral_FB_Z;
            }
            else
            {
                ResetIntegral();
            }

            gx += 2.0f * kp * halfex;
            gy += 2.0f * kp * halfey;
            gz += 2.0f * kp * halfez;
            gyro_only_mode = 0u;
        }
    }

    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;
    qa = q->w;
    qb = q->x;
    qc = q->y;

    q->w += (-qb * gx - qc * gy - q->z * gz);
    q->x += (qa * gx + qc * gz - q->z * gy);
    q->y += (qa * gy - qb * gz + q->z * gx);
    q->z += (qa * gz + qb * gy - qc * gx);

    recip_norm = InvSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (recip_norm > 0.0f)
    {
        q->w *= recip_norm;
        q->x *= recip_norm;
        q->y *= recip_norm;
        q->z *= recip_norm;
    }
    else
    {
        ResetQuat(q);
    }

    g_mahony_debug.acc_norm_raw = (isfinite(acc_norm_raw) != 0) ? acc_norm_raw : 0.0f;
    g_mahony_debug.acc_weight = (gyro_only_mode == 0u) ? 1.0f : 0.0f;
    g_mahony_debug.dt_used = dt_s;
    g_mahony_debug.gyro_only_mode = gyro_only_mode;
}
