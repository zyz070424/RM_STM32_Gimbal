/**
 * @file dvc_motor.cpp
 * @brief DJI 电机设备实现。
 * @details
 * 本文件实现 `Class_Motor` 的成员函数，以及兼容旧模块调用方式的
 * `Motor_*` 桥接接口。
 */
#include "dvc_motor.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace
{
/**
 * @brief CAN 发送缓存结构。
 * @details 每路 CAN 总线都维护一组控制帧缓存，供多电机共享同一发送帧。
 */
typedef struct Motor_CAN_Tx_Cache_TypeDef
{
    uint8_t frame_0x200[8]; /**< 0x200 控制帧缓存 */
    uint8_t frame_0x1FF[8]; /**< 0x1FF 控制帧缓存 */
    uint8_t frame_0x2FF[8]; /**< 0x2FF 控制帧缓存 */
    uint8_t frame_0x1FE[8]; /**< 0x1FE 控制帧缓存 */
    uint8_t frame_0x2FE[8]; /**< 0x2FE 控制帧缓存 */
} Motor_CAN_Tx_Cache_TypeDef;

Motor_CAN_Tx_Cache_TypeDef Motor_CAN1_Tx_Cache = {};
Motor_CAN_Tx_Cache_TypeDef Motor_CAN2_Tx_Cache = {};

/**
 * @brief 获取对应 CAN 总线的发送缓存。
 * @param can CAN 句柄。
 * @return 命中时返回缓存指针，否则返回 NULL。
 */
Motor_CAN_Tx_Cache_TypeDef *Motor_Get_Tx_Cache(CAN_HandleTypeDef *can)
{
    if (can == NULL)
    {
        return NULL;
    }

    if (can->Instance == CAN1)
    {
        return &Motor_CAN1_Tx_Cache;
    }

    if (can->Instance == CAN2)
    {
        return &Motor_CAN2_Tx_Cache;
    }

    return NULL;
}

/**
 * @brief 获取某个控制帧 ID 对应的 8 字节发送缓存。
 * @param can CAN 句柄。
 * @param send_id 控制帧 ID。
 * @return 命中时返回缓冲区指针，否则返回 NULL。
 */
uint8_t *Motor_Get_Tx_Frame_Buffer(CAN_HandleTypeDef *can, uint32_t send_id)
{
    Motor_CAN_Tx_Cache_TypeDef *cache = Motor_Get_Tx_Cache(can);

    if (cache == NULL)
    {
        return NULL;
    }

    switch (send_id)
    {
    case 0x200:
        return cache->frame_0x200;
    case 0x1FF:
        return cache->frame_0x1FF;
    case 0x2FF:
        return cache->frame_0x2FF;
    case 0x1FE:
        return cache->frame_0x1FE;
    case 0x2FE:
        return cache->frame_0x2FE;
    default:
        return NULL;
    }
}

/**
 * @brief 根据 HAL CAN 句柄获取对应的软件管理对象。
 * @param can HAL CAN 句柄。
 * @return 命中时返回管理对象指针，否则返回 NULL。
 */
Class_CAN_Manage_Object *Motor_Get_Can_Manage_Object(CAN_HandleTypeDef *can)
{
    if (can == NULL)
    {
        return NULL;
    }

    if (can->Instance == CAN1)
    {
        return &CAN1_Manage_Object;
    }

    if (can->Instance == CAN2)
    {
        return &CAN2_Manage_Object;
    }

    return NULL;
}
}

/**
 * @brief 获取当前电机对应的控制帧 ID 和槽位索引。
 * @param send_id 输出控制帧 ID 指针。
 * @param byte_index 输出当前电机对应的高字节索引指针。
 * @return 1 表示成功，0 表示失败。
 */
uint8_t Class_Motor::GetSendFrameInfo(uint32_t *send_id, uint8_t *byte_index) const
{
    if ((can == NULL) || (send_id == NULL) || (byte_index == NULL))
    {
        return 0u;
    }

    if ((ID < 1u) || (ID > 7u))
    {
        return 0u;
    }

    if (ID <= 4u)
    {
        *byte_index = (uint8_t)((ID - 1u) * 2u);

        switch (type)
        {
        case M3508:
            *send_id = 0x200;
            return 1u;
        case GM6020_Voltage:
            *send_id = 0x1FF;
            return 1u;
        case GM6020_Current:
            *send_id = 0x1FE;
            return 1u;
        default:
            return 0u;
        }
    }

    *byte_index = (uint8_t)((ID - 5u) * 2u);

    switch (type)
    {
    case M3508:
        *send_id = 0x1FF;
        return 1u;
    case GM6020_Voltage:
        *send_id = 0x2FF;
        return 1u;
    case GM6020_Current:
        *send_id = 0x2FE;
        return 1u;
    default:
        return 0u;
    }
}

/**
 * @brief 仅更新当前电机在控制帧中的两个字节。
 * @param send_id 控制帧 ID。
 * @param byte_index 当前电机对应的高字节索引。
 * @param data 电机控制量。
 * @return 无。
 */
void Class_Motor::UpdateFrameData(uint32_t send_id, uint8_t byte_index, int16_t data)
{
    uint8_t *tx_data;

    if (can == NULL)
    {
        return;
    }

    if (byte_index > 6u)
    {
        return;
    }

    tx_data = Motor_Get_Tx_Frame_Buffer(can, send_id);
    if (tx_data == NULL)
    {
        return;
    }

    tx_data[byte_index] = (uint8_t)(data >> 8);
    tx_data[byte_index + 1u] = (uint8_t)data;
}

/**
 * @brief 更新当前电机对应槽位并立即发送。
 * @param send_id 控制帧 ID。
 * @param byte_index 当前电机对应的高字节索引。
 * @param data 电机控制量。
 * @return 无。
 */
void Class_Motor::UpdateFrameAndSend(uint32_t send_id, uint8_t byte_index, int16_t data)
{
    if (can == NULL)
    {
        return;
    }

    UpdateFrameData(send_id, byte_index, data);
    SendCanFrameById(can, send_id);
}

/**
 * @brief 初始化电机对象。
 * @param id 电机 ID。
 * @param motor_type 电机类型。
 * @param can_handle 关联的 CAN 句柄。
 * @param control_method 控制方法。
 * @return 无。
 * @details 根据控制方法决定启用单级速度环还是级联角度环。
 */
void Class_Motor::Init(uint8_t id,
                       enum Motor_DJI_type motor_type,
                       CAN_HandleTypeDef *can_handle,
                       enum Motor_DJI_Control_Method control_method)
{
    can = can_handle;
    ID = id;
    type = motor_type;
    method = control_method;

    memset(&RxData, 0, sizeof(RxData));

    switch (control_method)
    {
    case DJI_Control_Method_Speed:
        PID_Use_Count = 1u;
        PID[0].Init();
        break;

    case DJI_Control_Method_Angle:
        PID_Use_Count = 2u;
        PID[0].Init();
        PID[1].Init();
        break;

    default:
        PID_Use_Count = 0u;
        break;
    }
}

/**
 * @brief 设置指定 PID 的参数和限幅。
 * @param pid_index PID 索引。
 * @param p 比例系数。
 * @param i 积分系数。
 * @param d 微分系数。
 * @param feedforward 前馈系数。
 * @param out_min 输出最小值。
 * @param out_max 输出最大值。
 * @param integral_min 积分最小值。
 * @param integral_max 积分最大值。
 * @return 无。
 */
void Class_Motor::SetPidParams(uint8_t pid_index,
                               float p,
                               float i,
                               float d,
                               float feedforward,
                               float out_min,
                               float out_max,
                               float integral_min,
                               float integral_max)
{
    if (pid_index >= 2u)
    {
        return;
    }

    PID[pid_index].SetParameters(p, i, d, feedforward,
                                 integral_min, integral_max, out_min, out_max);
}

/**
 * @brief 清空电机当前控制运行态，不改 PID 参数。
 * @return 无。
 */
void Class_Motor::ClearRuntime()
{
    uint8_t i;

    for (i = 0u; (i < PID_Use_Count) && (i < 2u); i++)
    {
        PID[i].integral = 0.0f;
        PID[i].error = 0.0f;
        PID[i].prev_error = 0.0f;
        PID[i].target = 0.0f;
        PID[i].prev_target = 0.0f;
        PID[i].output = 0.0f;
        PID[i].output_shaper_state = 0.0f;
        PID[i].output_shaper_inited = 1u;
    }
}

/**
 * @brief 计算速度环输出。
 * @param target 目标速度。
 * @param feedback_speed 当前速度反馈。
 * @param dt 控制周期。
 * @return 速度环输出。
 */
float Class_Motor::PidCalculateSpeed(float target, float feedback_speed, float dt)
{
    return PID[0].Calculate(feedback_speed, target, dt);
}

/**
 * @brief 计算角度环输出。
 * @param target 目标角度。
 * @param feedback_angle 当前角度反馈。
 * @param dt 控制周期。
 * @return 角度环输出。
 */
float Class_Motor::PidCalculateAngle(float target, float feedback_angle, float dt)
{
    return PID[1].Calculate(feedback_angle, target, dt);
}

/**
 * @brief 按当前控制方法计算电机控制输出。
 * @param target 目标值。
 * @param feedback_angle 当前角度反馈。
 * @param dt 控制周期。
 * @return 计算后的最终输出。
 * @details 速度模式直接运行单级 PID，角度模式则采用角度外环加速度度内环的级联控制。
 */
float Class_Motor::PidCalculate(float target, float feedback_angle, float dt)
{
    if (method == DJI_Control_Method_Speed)
    {
        return PID[0].Calculate((float)RxData.Speed, target, dt);
    }

    if (method == DJI_Control_Method_Angle)
    {
        float target_speed = PID[1].Calculate(feedback_angle, target, dt);
        return PID[0].Calculate((float)RxData.Speed, target_speed, dt);
    }

    return 0.0f;
}

/**
 * @brief 处理一帧 GM6020 原始反馈数据。
 * @param data 接收数据指针。
 * @return 无。
 */
void Class_Motor::Gm6020DataProcess(const uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t encoder_angle;

    if (data == NULL)
    {
        return;
    }

    encoder_angle = (uint16_t)data[0] << 8 | data[1];

    RxData.Speed = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS;
    RxData.Torque = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    RxData.Temperature = data[6];

    if (RxData.Encoder_Initialized == 0u)
    {
        RxData.Last_encoder_angle = encoder_angle;
        RxData.Total_Round = 0;
        RxData.Total_Encode = encoder_angle;
        RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round;
        RxData.Encoder_Initialized = 1u;
        return;
    }

    delta_encoder = encoder_angle - RxData.Last_encoder_angle;
    if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round--;
    }
    else if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round++;
    }

    RxData.Last_encoder_angle = encoder_angle;
    RxData.Total_Encode = RxData.Total_Round * Encoder_Num_Per_Round + encoder_angle;
    RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round;
}

/**
 * @brief 处理一帧 M3508 原始反馈数据。
 * @param data 接收数据指针。
 * @return 无。
 */
void Class_Motor::M3508DataProcess(const uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t encoder_angle;

    if (data == NULL)
    {
        return;
    }

    encoder_angle = (uint16_t)data[0] << 8 | data[1];

    RxData.Speed = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS / M2508_Gearbox_Rate;
    RxData.Torque = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    RxData.Temperature = data[6];

    if (RxData.Encoder_Initialized == 0u)
    {
        RxData.Last_encoder_angle = encoder_angle;
        RxData.Total_Round = 0;
        RxData.Total_Encode = encoder_angle;
        RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round / M2508_Gearbox_Rate;
        RxData.Encoder_Initialized = 1u;
        return;
    }

    delta_encoder = encoder_angle - RxData.Last_encoder_angle;
    if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round--;
    }
    else if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round++;
    }

    RxData.Last_encoder_angle = encoder_angle;
    RxData.Total_Encode = RxData.Total_Round * Encoder_Num_Per_Round + encoder_angle;
    RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round / M2508_Gearbox_Rate;
}

/**
 * @brief 按当前电机配置接收并解析 CAN 反馈数据。
 * @return 无。
 * @details 会按当前电机 ID 与类型映射标准反馈 ID，并尽量读空该 ID 的所有缓存数据。
 */
void Class_Motor::CanDataReceive()
{
    CAN_RX_MESSAGE rx_buffer;
    uint32_t feedback_id;
    Class_CAN_Manage_Object *manage;

    if (can == NULL)
    {
        return;
    }

    if ((ID < 1u) || (ID > 7u))
    {
        return;
    }

    switch (type)
    {
    case M3508:
        feedback_id = 0x200u + ID;
        break;

    case GM6020_Voltage:
    case GM6020_Current:
        feedback_id = 0x204u + ID;
        break;

    default:
        return;
    }

    manage = Motor_Get_Can_Manage_Object(can);
    if (manage == NULL)
    {
        return;
    }

    while (manage->ReadMessageByStdId(feedback_id, &rx_buffer) == HAL_OK)
    {
        switch (type)
        {
        case M3508:
            M3508DataProcess(rx_buffer.rx_data);
            break;

        case GM6020_Voltage:
        case GM6020_Current:
            Gm6020DataProcess(rx_buffer.rx_data);
            break;

        default:
            return;
        }
    }
}

/**
 * @brief 获取当前电机对应的控制帧发送 ID。
 * @return 控制帧 ID，失败时返回 0。
 */
uint32_t Class_Motor::GetCanSendId() const
{
    uint32_t send_id;
    uint8_t byte_index;

    if (GetSendFrameInfo(&send_id, &byte_index) == 0u)
    {
        return 0u;
    }

    return send_id;
}

/**
 * @brief 仅更新当前电机对应的 CAN 发送缓存，不立即发送。
 * @param data 要写入缓存的 16 位控制量。
 * @return 无。
 */
void Class_Motor::UpdateCanCache(int16_t data)
{
    uint32_t send_id;
    uint8_t byte_index;

    if (GetSendFrameInfo(&send_id, &byte_index) == 0u)
    {
        return;
    }

    UpdateFrameData(send_id, byte_index, data);
}

/**
 * @brief 联合发送两个电机控制量，必要时自动合帧。
 * @param motor_a 第一个电机对象。
 * @param cmd_a 第一个电机控制量。
 * @param motor_b 第二个电机对象。
 * @param cmd_b 第二个电机控制量。
 * @return 无。
 * @details 当两个电机位于同一路 CAN 且共用同一发送帧 ID 时，
 *          先分别写入同一帧缓存，再仅发送一次整帧；否则回退为逐电机发送。
 */
void Class_Motor::SendPair(Class_Motor *motor_a,
                           int16_t cmd_a,
                           Class_Motor *motor_b,
                           int16_t cmd_b)
{
    uint32_t send_id_a;
    uint32_t send_id_b;
    uint8_t byte_index_a;
    uint8_t byte_index_b;
    uint8_t motor_a_valid;
    uint8_t motor_b_valid;

    motor_a_valid = ((motor_a != NULL) &&
                     (motor_a->GetSendFrameInfo(&send_id_a, &byte_index_a) != 0u))
                        ? 1u
                        : 0u;
    motor_b_valid = ((motor_b != NULL) &&
                     (motor_b->GetSendFrameInfo(&send_id_b, &byte_index_b) != 0u))
                        ? 1u
                        : 0u;

    if ((motor_a_valid != 0u) &&
        (motor_b_valid != 0u) &&
        (motor_a->can == motor_b->can) &&
        (send_id_a == send_id_b))
    {
        motor_a->UpdateFrameData(send_id_a, byte_index_a, cmd_a);
        motor_b->UpdateFrameData(send_id_b, byte_index_b, cmd_b);
        SendCanFrameById(motor_a->can, send_id_a);
        return;
    }

    if (motor_a_valid != 0u)
    {
        motor_a->UpdateFrameAndSend(send_id_a, byte_index_a, cmd_a);
    }

    if (motor_b_valid != 0u)
    {
        motor_b->UpdateFrameAndSend(send_id_b, byte_index_b, cmd_b);
    }
}

/**
 * @brief 按控制帧 ID 发送对应缓存。
 * @param can CAN 句柄。
 * @param send_id 控制帧 ID。
 * @return 无。
 */
void Class_Motor::SendCanFrameById(CAN_HandleTypeDef *can, uint32_t send_id)
{
    uint8_t *tx_data;
    Class_CAN_Manage_Object *manage;

    if (can == NULL)
    {
        return;
    }

    tx_data = Motor_Get_Tx_Frame_Buffer(can, send_id);
    if (tx_data == NULL)
    {
        return;
    }

    manage = Motor_Get_Can_Manage_Object(can);
    if (manage == NULL)
    {
        return;
    }

    manage->Send(send_id, tx_data);
}

/**
 * @brief 根据当前电机类型和 ID 发送一帧控制数据。
 * @param data 要发送的 16 位控制量。
 * @return 无。
 */
void Class_Motor::SendCanData(int16_t data)
{
    uint32_t send_id;
    uint8_t byte_index;

    if (GetSendFrameInfo(&send_id, &byte_index) == 0u)
    {
        return;
    }

    UpdateFrameAndSend(send_id, byte_index, data);
}
