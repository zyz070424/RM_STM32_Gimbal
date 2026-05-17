/**
 * @file dvc_bmi088.cpp
 * @brief BMI088 设备驱动实现。
 * @details
 * 本文件实现 `Class_BMI088` 的成员函数。
 */
#include "dvc_bmi088.h"

#include "task.h"

#define BMI088_OK_OR_RETURN(x) do { if ((x) != HAL_OK) return HAL_ERROR; } while (0)
#define BMI088_DEG_TO_RAD (0.01745329251994329577f)
#define BMI088_G_TO_MS2   (9.81f)
#define BMI088_MAHONY_CALIB_SAMPLE_COUNT      800u
#define BMI088_MAHONY_CALIB_GYRO_LIMIT_DPS    3.0f
#define BMI088_MAHONY_CALIB_ACC_NORM_MIN_G    0.90f
#define BMI088_MAHONY_CALIB_ACC_NORM_MAX_G    1.10f
#define BMI088_MAHONY_ACC_USE_MIN_G           0.85f
#define BMI088_MAHONY_ACC_USE_MAX_G           1.15f
#define BMI088_MAHONY_BIAS_UPDATE_ALPHA       0.002f
#define BMI088_MAHONY_ACCEL_SCALE_UPDATE_ALPHA 0.001f
#define BMI088_EKF_CALIB_SAMPLE_COUNT    800u
#define BMI088_EKF_CALIB_GYRO_LIMIT_DPS        3.0f
#define BMI088_EKF_CALIB_ACC_NORM_MIN_G        0.90f
#define BMI088_EKF_CALIB_ACC_NORM_MAX_G        1.10f
#define BMI088_EKF_BIAS_UPDATE_ALPHA           0.002f
#define BMI088_EKF_ACCEL_SCALE_UPDATE_ALPHA    0.001f

Class_BMI088 BMI088_Manage_Object = {};

/**
 * @brief 将角度包裹到 [-180, 180) 区间。
 * @param angle_deg 待包裹角度，单位：deg。
 * @return 包裹后的角度。
 */
float Class_BMI088::Wrap180(float angle_deg) const
{
    while (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 将原始偏航角转换为连续偏航角。
 * @param raw_yaw_deg 原始偏航角，单位：deg。
 * @return 连续化后的偏航角，单位：deg。
 */
float Class_BMI088::YawToContinuous(float raw_yaw_deg)
{
    float yaw_rel_wrapped_deg;
    float dyaw_deg;

    if (isfinite(raw_yaw_deg) == 0)
    {
        return Yaw_Continuous_Deg;
    }

    if (Yaw_Continuous_Inited == 0u)
    {
        Yaw_Zero_Raw_Deg = raw_yaw_deg;
        Yaw_Last_Rel_Wrapped_Deg = 0.0f;
        Yaw_Continuous_Deg = 0.0f;
        Yaw_Continuous_Inited = 1u;
        return 0.0f;
    }

    yaw_rel_wrapped_deg = Wrap180(raw_yaw_deg - Yaw_Zero_Raw_Deg);
    dyaw_deg = yaw_rel_wrapped_deg - Yaw_Last_Rel_Wrapped_Deg;

    if (dyaw_deg > 180.0f)
    {
        dyaw_deg -= 360.0f;
    }
    else if (dyaw_deg < -180.0f)
    {
        dyaw_deg += 360.0f;
    }

    Yaw_Continuous_Deg += dyaw_deg;
    Yaw_Last_Rel_Wrapped_Deg = yaw_rel_wrapped_deg;

    return Yaw_Continuous_Deg;
}

/**
 * @brief 重置偏航连续化状态。
 * @return 无。
 */
void Class_BMI088::YawContinuousReset()
{
    Yaw_Continuous_Inited = 0u;
    Yaw_Zero_Raw_Deg = 0.0f;
    Yaw_Last_Rel_Wrapped_Deg = 0.0f;
    Yaw_Continuous_Deg = 0.0f;
}

/**
 * @brief 重置 Mahony 主路径输入校准状态。
 * @return 无。
 * @details
 * 该状态用于主路径的陀螺零偏估计与加速度模长修正，不阻塞姿态输出。
 */
void Class_BMI088::MahonyInputReset()
{
    Mahony_Input_Calibrated = 0u;
    Mahony_Calibration_Count = 0u;
    Mahony_Gyro_Bias_Dps.x = 0.0f;
    Mahony_Gyro_Bias_Dps.y = 0.0f;
    Mahony_Gyro_Bias_Dps.z = 0.0f;
    Mahony_Gyro_Bias_Sum_Dps.x = 0.0f;
    Mahony_Gyro_Bias_Sum_Dps.y = 0.0f;
    Mahony_Gyro_Bias_Sum_Dps.z = 0.0f;
    Mahony_Accel_Scale = 1.0f;
    Mahony_Accel_Norm_Sum_G = 0.0f;
}

/**
 * @brief 重置并行 EKF 姿态解算状态。
 * @return 无。
 */
void Class_BMI088::QuaternionEkfReset()
{
    IMU_QuaternionEKF_Reset();
    QuaternionEkfInputReset();
}

/**
 * @brief 将传感器坐标系映射到云台机体系。
 * @param imu 待映射的 IMU 数据指针。
 * @return 无。
 * @note 当前安装姿态等效于绕 +Z 旋转 90 度：Xb = Ys, Yb = -Xs, Zb = Zs。
 */
void Class_BMI088::MapSensorToBody(imu_data_t *imu)
{
    float sx;
    float sy;

    if (imu == NULL)
    {
        return;
    }

    sx = imu->gyro.x;
    sy = imu->gyro.y;
    imu->gyro.x = sy;
    imu->gyro.y = -sx;

    sx = imu->acc.x;
    sy = imu->acc.y;
    imu->acc.x = sy;
    imu->acc.y = -sx;
}

/**
 * @brief 判断当前样本是否可用于 Mahony 主路径的静止校准。
 * @param imu 已映射到机体系的 IMU 数据指针，gyro 单位 deg/s，acc 单位 g。
 * @param acc_norm_g 返回当前加速度模长，单位 g。
 * @return 1 表示样本稳定且可用于校准，0 表示当前样本不用于校准。
 */
uint8_t Class_BMI088::MahonySampleIsStable(const imu_data_t *imu, float *acc_norm_g) const
{
    float gyro_norm_dps;
    float acc_norm_local_g;

    if ((imu == NULL) || (acc_norm_g == NULL))
    {
        return 0u;
    }

    acc_norm_local_g = sqrtf((imu->acc.x * imu->acc.x) +
                             (imu->acc.y * imu->acc.y) +
                             (imu->acc.z * imu->acc.z));
    gyro_norm_dps = sqrtf((imu->gyro.x * imu->gyro.x) +
                          (imu->gyro.y * imu->gyro.y) +
                          (imu->gyro.z * imu->gyro.z));

    *acc_norm_g = acc_norm_local_g;

    if ((isfinite(acc_norm_local_g) == 0) ||
        (isfinite(gyro_norm_dps) == 0) ||
        (acc_norm_local_g < BMI088_MAHONY_CALIB_ACC_NORM_MIN_G) ||
        (acc_norm_local_g > BMI088_MAHONY_CALIB_ACC_NORM_MAX_G) ||
        (gyro_norm_dps > BMI088_MAHONY_CALIB_GYRO_LIMIT_DPS))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 对 Mahony 主路径执行后台零偏与加速度模长修正更新。
 * @param imu 已映射到机体系的 IMU 数据指针，gyro 单位 deg/s，acc 单位 g。
 * @return 无。
 * @details
 * 上电初期通过静止样本求平均值得到初始 bias，之后在静止窗口内慢速跟踪，
 * 保证主路径持续有输出，同时逐步改善零偏与模长误差。
 */
void Class_BMI088::MahonyInputUpdate(const imu_data_t *imu)
{
    float acc_norm_g;
    float target_acc_scale;

    if (MahonySampleIsStable(imu, &acc_norm_g) == 0u)
    {
        return;
    }

    if (Mahony_Input_Calibrated == 0u)
    {
        Mahony_Gyro_Bias_Sum_Dps.x += imu->gyro.x;
        Mahony_Gyro_Bias_Sum_Dps.y += imu->gyro.y;
        Mahony_Gyro_Bias_Sum_Dps.z += imu->gyro.z;
        Mahony_Accel_Norm_Sum_G += acc_norm_g;
        Mahony_Calibration_Count++;

        if (Mahony_Calibration_Count < BMI088_MAHONY_CALIB_SAMPLE_COUNT)
        {
            return;
        }

        Mahony_Gyro_Bias_Dps.x = Mahony_Gyro_Bias_Sum_Dps.x / (float)Mahony_Calibration_Count;
        Mahony_Gyro_Bias_Dps.y = Mahony_Gyro_Bias_Sum_Dps.y / (float)Mahony_Calibration_Count;
        Mahony_Gyro_Bias_Dps.z = Mahony_Gyro_Bias_Sum_Dps.z / (float)Mahony_Calibration_Count;

        if (Mahony_Accel_Norm_Sum_G > 1e-6f)
        {
            Mahony_Accel_Scale = (float)Mahony_Calibration_Count / Mahony_Accel_Norm_Sum_G;
        }
        else
        {
            Mahony_Accel_Scale = 1.0f;
        }

        Mahony_Input_Calibrated = 1u;
        return;
    }

    Mahony_Gyro_Bias_Dps.x += BMI088_MAHONY_BIAS_UPDATE_ALPHA * (imu->gyro.x - Mahony_Gyro_Bias_Dps.x);
    Mahony_Gyro_Bias_Dps.y += BMI088_MAHONY_BIAS_UPDATE_ALPHA * (imu->gyro.y - Mahony_Gyro_Bias_Dps.y);
    Mahony_Gyro_Bias_Dps.z += BMI088_MAHONY_BIAS_UPDATE_ALPHA * (imu->gyro.z - Mahony_Gyro_Bias_Dps.z);

    if (acc_norm_g > 1e-6f)
    {
        target_acc_scale = 1.0f / acc_norm_g;
        Mahony_Accel_Scale += BMI088_MAHONY_ACCEL_SCALE_UPDATE_ALPHA * (target_acc_scale - Mahony_Accel_Scale);
    }

    if (Mahony_Accel_Scale < 0.8f)
    {
        Mahony_Accel_Scale = 0.8f;
    }
    else if (Mahony_Accel_Scale > 1.2f)
    {
        Mahony_Accel_Scale = 1.2f;
    }
}

/**
 * @brief 重置并行 EKF 输入校准状态。
 * @return 无。
 * @details
 * 该状态仅服务于并行 EKF 路径，不影响原 Mahony 主路径的传感器数据使用方式。
 */
void Class_BMI088::QuaternionEkfInputReset()
{
    Ekf_Input_Calibrated = 0u;
    Ekf_Calibration_Count = 0u;
    Ekf_Gyro_Offset_Dps.x = 0.0f;
    Ekf_Gyro_Offset_Dps.y = 0.0f;
    Ekf_Gyro_Offset_Dps.z = 0.0f;
    Ekf_Gyro_Offset_Sum_Dps.x = 0.0f;
    Ekf_Gyro_Offset_Sum_Dps.y = 0.0f;
    Ekf_Gyro_Offset_Sum_Dps.z = 0.0f;
    Ekf_Accel_Scale = 1.0f;
    Ekf_Accel_Norm_Sum_G = 0.0f;
    Ekf_Has_Last_Calib_Sample = 0u;
    Ekf_Last_Calib_Gyro_Dps.x = 0.0f;
    Ekf_Last_Calib_Gyro_Dps.y = 0.0f;
    Ekf_Last_Calib_Gyro_Dps.z = 0.0f;
    Ekf_Last_Calib_Acc_Norm_G = 0.0f;
}

/**
 * @brief 对并行 EKF 输入执行后台零偏与加速度模长修正更新。
 * @param imu 已映射到机体系的 IMU 数据指针，gyro 单位 deg/s，acc 单位 g。
 * @return 无。
 * @details
 * 与 Mahony 主路径保持一致，EKF 也采用“前台不断流、后台缓慢校准”的方式。
 * 上电初期先通过静止样本求平均值得到初始 offset，后续在静止窗口内缓慢跟踪。
 */
void Class_BMI088::QuaternionEkfInputUpdate(const imu_data_t *imu)
{
    float acc_norm_g;
    float target_acc_scale;

    if (imu == NULL)
    {
        return;
    }

    if (MahonySampleIsStable(imu, &acc_norm_g) == 0u)
    {
        return;
    }

    if (Ekf_Input_Calibrated == 0u)
    {
        Ekf_Gyro_Offset_Sum_Dps.x += imu->gyro.x;
        Ekf_Gyro_Offset_Sum_Dps.y += imu->gyro.y;
        Ekf_Gyro_Offset_Sum_Dps.z += imu->gyro.z;
        Ekf_Accel_Norm_Sum_G += acc_norm_g;
        Ekf_Calibration_Count++;

        if (Ekf_Calibration_Count < BMI088_EKF_CALIB_SAMPLE_COUNT)
        {
            return;
        }

        Ekf_Gyro_Offset_Dps.x = Ekf_Gyro_Offset_Sum_Dps.x / (float)Ekf_Calibration_Count;
        Ekf_Gyro_Offset_Dps.y = Ekf_Gyro_Offset_Sum_Dps.y / (float)Ekf_Calibration_Count;
        Ekf_Gyro_Offset_Dps.z = Ekf_Gyro_Offset_Sum_Dps.z / (float)Ekf_Calibration_Count;

        if (Ekf_Accel_Norm_Sum_G > 1e-6f)
        {
            Ekf_Accel_Scale = (float)Ekf_Calibration_Count / Ekf_Accel_Norm_Sum_G;
        }
        else
        {
            Ekf_Accel_Scale = 1.0f;
        }

        Ekf_Input_Calibrated = 1u;
        IMU_QuaternionEKF_Reset();
        return;
    }

    Ekf_Gyro_Offset_Dps.x += BMI088_EKF_BIAS_UPDATE_ALPHA * (imu->gyro.x - Ekf_Gyro_Offset_Dps.x);
    Ekf_Gyro_Offset_Dps.y += BMI088_EKF_BIAS_UPDATE_ALPHA * (imu->gyro.y - Ekf_Gyro_Offset_Dps.y);
    Ekf_Gyro_Offset_Dps.z += BMI088_EKF_BIAS_UPDATE_ALPHA * (imu->gyro.z - Ekf_Gyro_Offset_Dps.z);

    if (acc_norm_g > 1e-6f)
    {
        target_acc_scale = 1.0f / acc_norm_g;
        Ekf_Accel_Scale += BMI088_EKF_ACCEL_SCALE_UPDATE_ALPHA * (target_acc_scale - Ekf_Accel_Scale);
    }

    if (Ekf_Accel_Scale < 0.8f)
    {
        Ekf_Accel_Scale = 0.8f;
    }
    else if (Ekf_Accel_Scale > 1.2f)
    {
        Ekf_Accel_Scale = 1.2f;
    }
}

/**
 * @brief 初始化 BMI088 传感器。
 * @param hspi SPI 句柄。
 * @return HAL_OK 表示初始化成功，否则返回 HAL_ERROR。
 * @details
 * 该流程会完成软复位、芯片 ID 校验、加速度计配置与陀螺仪配置，
 * 并重置姿态四元数和偏航连续化状态。
 */
HAL_StatusTypeDef Class_BMI088::Init(SPI_HandleTypeDef *hspi)
{
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t dummy = 0;

    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    Quat.w = 1.0f;
    Quat.x = 0.0f;
    Quat.y = 0.0f;
    Quat.z = 0.0f;
    YawContinuousReset();
    MahonyInputReset();
    QuaternionEkfReset();

    SPI1_Manage_Object.InitDMA();

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_SOFTRESET, BMI088_SOFT_RESET_CMD));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD));
    vTaskDelay(pdMS_TO_TICKS(50));

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &dummy, 1));
    vTaskDelay(pdMS_TO_TICKS(1));

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &acc_id, 1));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.ReadReg(hspi, GYRO, BMI088_REG_GYRO_CHIP_ID, &gyro_id, 1));
    if ((acc_id != BMI088_ACCEL_CHIP_ID) || (gyro_id != BMI088_GYRO_CHIP_ID))
    {
        return HAL_ERROR;
    }

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CONF, 0x00));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CTRL, BMI088_ACCEL_PWR_ENABLE));
    vTaskDelay(pdMS_TO_TICKS(10));

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_CONF, BMI088_ACCEL_CONF_800HZ_NORMAL));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_RANGE, BMI088_ACCEL_RANGE_6G));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT1_IO_CONF, BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_MAP));

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_LPM1, BMI088_GYRO_MODE_NORMAL));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_BANDWIDTH, BMI088_GYRO_BW_116_HZ));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_RANGE, BMI088_GYRO_RANGE_1000_DPS));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT_CTRL, BMI088_GYRO_INT_DRDY_ENABLE));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_DRDY_MAP));

    return HAL_OK;
}

/**
 * @brief 读取 BMI088 加速度计数据。
 * @param hspi SPI 句柄。
 * @param data 存储加速度数据的 IMU 结构体指针。
 * @return 无。
 */
void Class_BMI088::ReadAccel(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t accel_data_raw[6];
    int16_t accel_data[3];

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_X_LSB, accel_data_raw, 6) != HAL_OK)
    {
        return;
    }

    accel_data[0] = (int16_t)((uint16_t)accel_data_raw[1] << 8 | accel_data_raw[0]);
    accel_data[1] = (int16_t)((uint16_t)accel_data_raw[3] << 8 | accel_data_raw[2]);
    accel_data[2] = (int16_t)((uint16_t)accel_data_raw[5] << 8 | accel_data_raw[4]);

    data->acc.x = accel_data[0] / BMI088_ACCEL_SENSITIVITY_6G;
    data->acc.y = accel_data[1] / BMI088_ACCEL_SENSITIVITY_6G;
    data->acc.z = accel_data[2] / BMI088_ACCEL_SENSITIVITY_6G;
}

/**
 * @brief 读取 BMI088 陀螺仪数据。
 * @param hspi SPI 句柄。
 * @param data 存储陀螺仪数据的 IMU 结构体指针。
 * @return 无。
 */
void Class_BMI088::ReadGyro(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t gyro_data_raw[6];
    int16_t gyro_data[3];

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI1_Manage_Object.ReadReg(hspi, GYRO, BMI088_REG_GYRO_X_LSB, gyro_data_raw, 6) != HAL_OK)
    {
        return;
    }

    gyro_data[0] = (int16_t)((uint16_t)gyro_data_raw[1] << 8 | gyro_data_raw[0]);
    gyro_data[1] = (int16_t)((uint16_t)gyro_data_raw[3] << 8 | gyro_data_raw[2]);
    gyro_data[2] = (int16_t)((uint16_t)gyro_data_raw[5] << 8 | gyro_data_raw[4]);

    data->gyro.x = gyro_data[0] / BMI088_GYRO_SENSITIVITY_1000_DPS;
    data->gyro.y = gyro_data[1] / BMI088_GYRO_SENSITIVITY_1000_DPS;
    data->gyro.z = gyro_data[2] / BMI088_GYRO_SENSITIVITY_1000_DPS;
}

/**
 * @brief 读取 BMI088 温度数据。
 * @param hspi SPI 句柄。
 * @param data 存储温度数据的 IMU 结构体指针。
 * @return 无。
 */
void Class_BMI088::ReadTemp(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t temp_data_raw[2];
    int16_t temp_data;

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_TEMP_LSB, temp_data_raw, 2) != HAL_OK)
    {
        return;
    }

    temp_data = (int16_t)((uint16_t)temp_data_raw[1] << 8 | temp_data_raw[0]);
    data->temp = (temp_data / 326.8f) + 23.0f;
}

/**
 * @brief 基于当前 IMU 数据执行一次姿态互补滤波。
 * @param data 输入的 IMU 数据指针。
 * @param dt 时间间隔，单位：s。
 * @param kp 比例增益。
 * @param ki 积分增益。
 * @return 计算得到的欧拉角，单位：deg。
 * @details
 * 该流程会先复制原始 IMU 数据，再做坐标轴映射、Mahony 四元数更新和偏航连续化。
 */
euler_t Class_BMI088::ComplementaryFilter(imu_data_t *data, float dt, float kp, float ki)
{
    euler_t euler = {0};
    imu_data_t imu;
    float acc_norm_g;

    if (data == NULL)
    {
        return euler;
    }

    imu = *data;
    imu.dt = dt;
    MapSensorToBody(&imu);
    MahonyInputUpdate(&imu);

    imu.gyro.x -= Mahony_Gyro_Bias_Dps.x;
    imu.gyro.y -= Mahony_Gyro_Bias_Dps.y;
    imu.gyro.z -= Mahony_Gyro_Bias_Dps.z;

    imu.acc.x *= Mahony_Accel_Scale;
    imu.acc.y *= Mahony_Accel_Scale;
    imu.acc.z *= Mahony_Accel_Scale;

    acc_norm_g = sqrtf((imu.acc.x * imu.acc.x) +
                       (imu.acc.y * imu.acc.y) +
                       (imu.acc.z * imu.acc.z));
    if ((isfinite(acc_norm_g) == 0) ||
        (acc_norm_g < BMI088_MAHONY_ACC_USE_MIN_G) ||
        (acc_norm_g > BMI088_MAHONY_ACC_USE_MAX_G))
    {
        imu.acc.x = 0.0f;
        imu.acc.y = 0.0f;
        imu.acc.z = 0.0f;
    }

    Mahony_Quaternion_Object.MahonyUpdate(&Quat, imu, kp, ki);
    euler = Mahony_Quaternion_Object.QuatToEuler(Quat);
    euler.yaw = YawToContinuous(euler.yaw);

    return euler;
}

/**
 * @brief 基于当前 IMU 数据执行一次并行 EKF 姿态更新。
 * @param data 输入的 IMU 数据指针。
 * @param dt 时间间隔，单位：s。
 * @return 计算得到的欧拉角，单位：deg。
 * @details
 * 保留当前 BMI088 坐标轴映射规则，仅额外补齐 EKF 所需的单位转换。
 * 由于移植进来的 EKF 工程内部对 Pitch / Roll 的命名与本项目当前
 * Mahony 欧拉角定义不同，这里在适配层完成一次轴含义对齐，避免影响
 * 现有控制与联调习惯。
 */
euler_t Class_BMI088::QuaternionEkfFilter(imu_data_t *data, float dt)
{
    euler_t euler = {0};
    imu_data_t imu;
    float acc_norm_g;

    if (data == NULL)
    {
        return euler;
    }

    imu = *data;
    imu.dt = dt;
    MapSensorToBody(&imu);
    QuaternionEkfInputUpdate(&imu);

    imu.gyro.x -= Ekf_Gyro_Offset_Dps.x;
    imu.gyro.y -= Ekf_Gyro_Offset_Dps.y;
    imu.gyro.z -= Ekf_Gyro_Offset_Dps.z;

    imu.acc.x *= Ekf_Accel_Scale;
    imu.acc.y *= Ekf_Accel_Scale;
    imu.acc.z *= Ekf_Accel_Scale;

    acc_norm_g = sqrtf((imu.acc.x * imu.acc.x) +
                       (imu.acc.y * imu.acc.y) +
                       (imu.acc.z * imu.acc.z));
    if ((isfinite(acc_norm_g) == 0) ||
        (acc_norm_g < BMI088_MAHONY_ACC_USE_MIN_G) ||
        (acc_norm_g > BMI088_MAHONY_ACC_USE_MAX_G))
    {
        imu.acc.x = 0.0f;
        imu.acc.y = 0.0f;
        imu.acc.z = 1.0f;
    }

    IMU_QuaternionEKF_Update(imu.gyro.x * BMI088_DEG_TO_RAD,
                             imu.gyro.y * BMI088_DEG_TO_RAD,
                             imu.gyro.z * BMI088_DEG_TO_RAD,
                             imu.acc.x * BMI088_G_TO_MS2,
                             imu.acc.y * BMI088_G_TO_MS2,
                             imu.acc.z * BMI088_G_TO_MS2,
                             dt);

    // 对齐本项目当前 ZYX 欧拉角约定：
    // 当前 EKF 内部的 Pitch 对应本项目 roll，Roll 对应本项目 pitch。
    euler.roll = QEKF_INS.Pitch;
    euler.pitch = QEKF_INS.Roll;
    euler.yaw = QEKF_INS.YawTotalAngle;

    return euler;
}
