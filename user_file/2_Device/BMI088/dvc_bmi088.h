/**
 * @file dvc_bmi088.h
 * @brief BMI088 设备驱动接口与管理对象定义。
 * @details
 * 本文件定义 BMI088 姿态解算所需的 `Class_BMI088` 管理对象。
 */
#ifndef __BMI088_H__
#define __BMI088_H__

#include "drv_spi.h"
#include "alg_quaternion.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>


#ifdef __cplusplus
/**
 * @class Class_BMI088
 * @brief BMI088 设备管理对象。
 * @details
 * 负责 BMI088 初始化、三轴数据读取、温度读取以及姿态解算辅助状态维护。
 */
class Class_BMI088
{
public:
    quat_t Quat;                     /**< 当前姿态四元数 */
    uint8_t Yaw_Continuous_Inited;   /**< 偏航连续化是否已初始化 */
    float Yaw_Zero_Raw_Deg;          /**< 连续化零点对应的原始偏航角 */
    float Yaw_Last_Rel_Wrapped_Deg;  /**< 上一周期相对零点的包角偏航角 */
    float Yaw_Continuous_Deg;        /**< 当前连续偏航角 */

    HAL_StatusTypeDef Init(SPI_HandleTypeDef *hspi);
    void ReadAccel(SPI_HandleTypeDef *hspi, imu_data_t *data);
    void ReadGyro(SPI_HandleTypeDef *hspi, imu_data_t *data);
    void ReadTemp(SPI_HandleTypeDef *hspi, imu_data_t *data);
    void YawContinuousReset();
    euler_t ComplementaryFilter(imu_data_t *data, float dt, float kp, float ki);

private:
    float Wrap180(float angle_deg) const;
    float YawToContinuous(float raw_yaw_deg);
    void MapSensorToBody(imu_data_t *imu);
};
extern Class_BMI088 BMI088_Manage_Object;
#endif

//数据处理
#define BMI088_ACCEL_SENSITIVITY_24G 1365.0f
#define BMI088_ACCEL_SENSITIVITY_12G 2730.0f
#define BMI088_ACCEL_SENSITIVITY_6G  5460.0f
#define BMI088_ACCEL_SENSITIVITY_3G  10920.0f

// --- 陀螺仪 ---
#define BMI088_GYRO_SENSITIVITY_2000_DPS 16.384f
#define BMI088_GYRO_SENSITIVITY_1000_DPS 32.8f
#define BMI088_GYRO_SENSITIVITY_500_DPS  65.6f
#define BMI088_GYRO_SENSITIVITY_250_DPS  131.2f
#define BMI088_GYRO_SENSITIVITY_125_DPS  262.4f

// --- 芯片 ID ---
#define BMI088_ACCEL_CHIP_ID        0x1E
#define BMI088_GYRO_CHIP_ID         0x0F

// --- 软复位命令 ---
#define BMI088_SOFT_RESET_CMD       0xB6

// ACCEL
#define BMI088_REG_ACCEL_CHIP_ID       0x00
#define BMI088_REG_ACCEL_ERR_REG       0x02
#define BMI088_REG_ACCEL_STATUS        0x03
#define BMI088_REG_ACCEL_X_LSB         0x12
#define BMI088_REG_ACCEL_X_MSB         0x13
#define BMI088_REG_ACCEL_Y_LSB         0x14
#define BMI088_REG_ACCEL_Y_MSB         0x15
#define BMI088_REG_ACCEL_Z_LSB         0x16
#define BMI088_REG_ACCEL_Z_MSB         0x17
#define BMI088_REG_ACCEL_SENSORTIME_0  0x18
#define BMI088_REG_ACCEL_SENSORTIME_1  0x19
#define BMI088_REG_ACCEL_SENSORTIME_2  0x1A
#define BMI088_REG_ACCEL_INT_STAT_1    0x1D
#define BMI088_REG_ACCEL_TEMP_LSB      0x22
#define BMI088_REG_ACCEL_TEMP_MSB      0x23
#define BMI088_REG_ACCEL_CONF          0x40
#define BMI088_REG_ACCEL_RANGE         0x41
#define BMI088_REG_ACCEL_INT1_IO_CONF  0x53
#define BMI088_REG_ACCEL_INT2_IO_CONF  0x54
#define BMI088_REG_ACCEL_INT_MAP_DATA  0x58
#define BMI088_REG_ACCEL_SELF_TEST     0x6D
#define BMI088_REG_ACCEL_PWR_CONF      0x7C
#define BMI088_REG_ACCEL_PWR_CTRL      0x7D
#define BMI088_REG_ACCEL_SOFTRESET     0x7E

#define BMI088_ACCEL_ODR_1600_HZ    0x0C
#define BMI088_ACCEL_ODR_800_HZ     0x0B
#define BMI088_ACCEL_ODR_400_HZ     0x0A
#define BMI088_ACCEL_ODR_200_HZ     0x09
#define BMI088_ACCEL_ODR_100_HZ     0x08
#define BMI088_ACCEL_ODR_50_HZ      0x07

#define BMI088_ACCEL_BWP_NORMAL     0xA0
#define BMI088_ACCEL_BWP_OSR4       0x80
#define BMI088_ACCEL_BWP_OSR2       0x90
#define BMI088_ACCEL_BWP_CIC        0xB0

#define BMI088_ACCEL_CONF_1600HZ_NORMAL  (BMI088_ACCEL_ODR_1600_HZ | BMI088_ACCEL_BWP_NORMAL)

#define BMI088_ACCEL_RANGE_3G       0x00
#define BMI088_ACCEL_RANGE_6G       0x01
#define BMI088_ACCEL_RANGE_12G      0x02
#define BMI088_ACCEL_RANGE_24G      0x03

#define BMI088_ACCEL_PWR_ENABLE     0x04
#define BMI088_ACCEL_PWR_DISABLE    0x00

#define BMI088_ACCEL_INT1_DRDY_MAP  0x04
#define BMI088_ACCEL_INT2_DRDY_MAP  0x40

#define BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH  0x0A
#define BMI088_ACCEL_INT_CFG_PUSH_PULL_LOW   0x08

// GYRO
#define BMI088_REG_GYRO_CHIP_ID            0x00
#define BMI088_REG_GYRO_X_LSB              0x02
#define BMI088_REG_GYRO_X_MSB              0x03
#define BMI088_REG_GYRO_Y_LSB              0x04
#define BMI088_REG_GYRO_Y_MSB              0x05
#define BMI088_REG_GYRO_Z_LSB              0x06
#define BMI088_REG_GYRO_Z_MSB              0x07
#define BMI088_REG_GYRO_INT_STAT_1         0x0A
#define BMI088_REG_GYRO_RANGE              0x0F
#define BMI088_REG_GYRO_BANDWIDTH          0x10
#define BMI088_REG_GYRO_LPM1               0x11
#define BMI088_REG_GYRO_SOFTRESET          0x14
#define BMI088_REG_GYRO_INT_CTRL           0x15
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF  0x16
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP   0x18
#define BMI088_REG_GYRO_SELF_TEST          0x3C

#define BMI088_GYRO_BW_532_HZ       0x80
#define BMI088_GYRO_BW_230_HZ       0x81
#define BMI088_GYRO_BW_116_HZ       0x82
#define BMI088_GYRO_BW_47_HZ        0x83
#define BMI088_GYRO_BW_23_HZ        0x84
#define BMI088_GYRO_BW_12_HZ        0x85
#define BMI088_GYRO_BW_64_HZ        0x87

#define BMI088_GYRO_RANGE_2000_DPS  0x00
#define BMI088_GYRO_RANGE_1000_DPS  0x01
#define BMI088_GYRO_RANGE_500_DPS   0x02
#define BMI088_GYRO_RANGE_250_DPS   0x03
#define BMI088_GYRO_RANGE_125_DPS   0x04

#define BMI088_GYRO_MODE_NORMAL        0x00
#define BMI088_GYRO_MODE_DEEP_SUSPEND  0x20
#define BMI088_GYRO_MODE_SUSPEND       0x80

#define BMI088_GYRO_INT_DRDY_ENABLE 0x80

#define BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH 0x01
#define BMI088_GYRO_INT4_CFG_PUSH_PULL_HIGH 0x04

#define BMI088_GYRO_INT3_DRDY_MAP       0x01
#define BMI088_GYRO_INT4_DRDY_MAP       0x80
#define BMI088_GYRO_INT3_INT4_DRDY_MAP  0x81

#endif /* __BMI088_H__ */
