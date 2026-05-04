/**
 * @file dvc_bmi088.cpp
 * @brief BMI088 设备驱动实现。
 * @details
 * 本文件实现 `Class_BMI088` 的成员函数。
 */
#include "dvc_bmi088.h"

#include "task.h"

#define BMI088_OK_OR_RETURN(x) do { if ((x) != HAL_OK) return HAL_ERROR; } while (0)

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

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_CONF, BMI088_ACCEL_CONF_1600HZ_NORMAL));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_RANGE, BMI088_ACCEL_RANGE_24G));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT1_IO_CONF, BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_MAP));

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_LPM1, BMI088_GYRO_MODE_NORMAL));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_BANDWIDTH, BMI088_GYRO_BW_532_HZ));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_RANGE, BMI088_GYRO_RANGE_2000_DPS));
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

    data->acc.x = accel_data[0] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.y = accel_data[1] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.z = accel_data[2] / BMI088_ACCEL_SENSITIVITY_24G;
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

    data->gyro.x = gyro_data[0] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.y = gyro_data[1] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.z = gyro_data[2] / BMI088_GYRO_SENSITIVITY_2000_DPS;
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

    if (data == NULL)
    {
        return euler;
    }

    imu = *data;
    imu.dt = dt;
    MapSensorToBody(&imu);

    Mahony_Quaternion_Object.MahonyUpdate(&Quat, imu, kp, ki);
    euler = Mahony_Quaternion_Object.QuatToEuler(Quat);
    euler.yaw = YawToContinuous(euler.yaw);

    return euler;
}
