/**
 * @file gimbal_controller.h
 * @brief 云台最终控制裁决接口与管理对象定义。
 */
#ifndef __GIMBAL_CONTROLLER_H__
#define __GIMBAL_CONTROLLER_H__

#include <stdint.h>
#include "gimbal_sentry.h"

#ifdef __cplusplus
class Class_Gimbal;

/**
 * @class Class_Gimbal_Controller
 * @brief 云台最终控制裁决对象。
 * @details
 * 负责汇总 sentry、protect、debug 等输入，
 * 生成当前控制拍的最终目标、输出与 CAN 命令。
 */
class Class_Gimbal_Controller
{
public:
    float Pitch_Target_Deg;                /**< 当前拍最终 pitch 角度目标，单位：deg */
    float Pitch_Target_Speed;              /**< 当前拍最终 pitch 速度目标 */
    float Pitch_Output;                    /**< 当前拍最终 pitch 输出 */
    float Yaw_Target_Deg;                  /**< 当前拍最终 yaw 角度目标，单位：deg */
    float Yaw_Target_Speed;                /**< 当前拍最终 yaw 速度目标 */
    float Yaw_Output;                      /**< 当前拍最终 yaw 输出 */
    int16_t Pitch_Can_Cmd;                 /**< 当前拍最终 pitch CAN 控制量 */
    int16_t Yaw_Can_Cmd;                   /**< 当前拍最终 yaw CAN 控制量 */
    Gimbal_Sentry_State_TypeDef Sentry_State; /**< 当前拍 sentry 状态缓存 */

    void Init();
    void Update(Class_Gimbal *gimbal, uint32_t now_tick);

private:
    void ApplyProtectAndMode(Class_Gimbal *gimbal);
    void HandleAnglePidReset(Class_Gimbal *gimbal);
    void CalculateOutput(Class_Gimbal *gimbal);
    void BuildCanCmd();
};

extern Class_Gimbal_Controller Gimbal_Controller_Object;
#endif

#endif /* __GIMBAL_CONTROLLER_H__ */
