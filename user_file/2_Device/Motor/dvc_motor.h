/**
 * @file dvc_motor.h
 * @brief DJI 电机设备管理对象定义。
 * @details
 * 本文件定义电机接收数据结构与 `Class_Motor` 电机管理对象。
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "drv_can.h"
#include "alg_pid.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>

#define Encoder_Num_Per_Round 8192
#define M2508_Gearbox_Rate   268.0f / 17.0f
#define RPM_TO_RADS 0.104719755f

/**
 * @brief DJI 电机控制方法枚举。
 */
enum Motor_DJI_Control_Method
{
    DJI_Control_Method_Angle = 0,
    DJI_Control_Method_Speed = 1,
};

/**
 * @brief DJI 电机类型枚举。
 */
enum Motor_DJI_type
{
    GM6020_Current,
    GM6020_Voltage,
    M3508,
};

/**
 * @brief 电机反馈数据结构。
 * @details
 * 保存当前角度、速度、转矩、温度及编码器累计状态。
 */
typedef struct Motor_DataTypeDef
{
    float Angle;                 /**< 当前电机角度 */
    float Speed;                 /**< 当前电机速度 */
    int16_t Torque;              /**< 当前电机转矩反馈 */
    uint8_t Temperature;         /**< 当前电机温度 */
    int32_t Total_Encode;        /**< 编码器累计值 */
    uint16_t Last_encoder_angle; /**< 上一帧编码器角度 */
    int32_t Total_Round;         /**< 编码器累计圈数 */
    uint8_t Encoder_Initialized; /**< 首包是否已完成初始化 */
} Motor_DataTypeDef;

#ifdef __cplusplus
/**
 * @class Class_Motor
 * @brief DJI 电机管理对象。
 * @details
 * 负责电机初始化、PID 参数配置、CAN 反馈解析和控制帧发送。
 */
class Class_Motor
{
public:
    Class_PID PID[2];                         /**< PID 控制器数组，0=速度环，1=角度环 */
    uint8_t PID_Use_Count;                    /**< 当前启用的 PID 数量 */
    uint8_t ID;                               /**< 电机 ID */
    enum Motor_DJI_type type;                 /**< 电机类型 */
    CAN_HandleTypeDef *can;                   /**< 关联的 CAN 句柄 */
    enum Motor_DJI_Control_Method method;     /**< 当前控制方法 */
    Motor_DataTypeDef RxData;                 /**< 当前接收反馈数据 */

    void Init(uint8_t id,
              enum Motor_DJI_type motor_type,
              CAN_HandleTypeDef *can_handle,
              enum Motor_DJI_Control_Method control_method);
    void SetPidParams(uint8_t pid_index,
                      float p,
                      float i,
                      float d,
                      float feedforward,
                      float out_min,
                      float out_max,
                      float integral_min,
                      float integral_max);
    void ClearRuntime();
    float PidCalculateSpeed(float target, float feedback_speed, float dt);
    float PidCalculateAngle(float target, float feedback_angle, float dt);
    float PidCalculate(float target, float feedback_angle, float dt);
    void CanDataReceive();
    uint32_t GetCanSendId() const;
    void UpdateCanCache(int16_t data);
    void SendCanData(int16_t data);
    static void SendCanFrameById(CAN_HandleTypeDef *can, uint32_t send_id);

private:
    uint8_t GetSendFrameInfo(uint32_t *send_id, uint8_t *byte_index) const;
    void UpdateFrameData(uint32_t send_id, uint8_t byte_index, int16_t data);
    void UpdateFrameAndSend(uint32_t send_id, uint8_t byte_index, int16_t data);
    void Gm6020DataProcess(const uint8_t *data);
    void M3508DataProcess(const uint8_t *data);
};
#endif

#endif /* __MOTOR_H__ */
