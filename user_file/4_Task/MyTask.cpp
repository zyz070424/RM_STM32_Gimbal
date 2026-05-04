#include "MyTask.h"
#include "Gimbal.h"

static uint8_t g_task_started = 0;

namespace
{
/**
 * @brief 云台双轴控制任务入口。
 * @param params 任务入口透传参数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Gimbal_Motor_Control_Task_Entry(void *params)
{
    Gimbal_Object.MotorControlTask(params);
}

/**
 * @brief 云台欧拉角解算任务入口。
 * @param params 任务入口透传参数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Gimbal_Euler_Task_Entry(void *params)
{
    Gimbal_Object.EulerTask(params);
}

/**
 * @brief 云台在线检测任务入口。
 * @param params 任务入口透传参数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Gimbal_Task_Entry(void *params)
{
    Gimbal_Object.TaskLoop(params);
}

/**
 * @brief 发往视觉的姿态发送任务入口。
 * @param params 任务入口透传参数。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void Gimbal_Manifold_Control_Task_Entry(void *params)
{
    Gimbal_Object.ManifoldControlTask(params);
}
}

/**
 * @brief 任务初始化函数。
 * @return 无。
 */
void Task_Init(void)
{
    // 设备/模块初始化统一放在任务层入口
    Gimbal_Object.Init(nullptr);
}
/**
 * @brief 任务循环函数。
 * @return 无。
 */
void Task_loop(void)
{
    TaskHandle_t task_gimbal_motor_control_handle;
    TaskHandle_t task_gimbal_euler_handle;
    TaskHandle_t task_gimbal_task_handle;
    TaskHandle_t task_gimbal_manifold_control_handle;

    if (g_task_started != 0)
    {
        return;
    }

    g_task_started = 1;

    xTaskCreate(Gimbal_Motor_Control_Task_Entry, "Task_Gimbal_Motor_Control_Test", 3000, NULL, osPriorityHigh, &task_gimbal_motor_control_handle);
    xTaskCreate(Gimbal_Euler_Task_Entry, "Task_Gimbal_Euler", 1000, NULL, osPriorityHigh, &task_gimbal_euler_handle);
    xTaskCreate(Gimbal_Task_Entry, "Task_Gimbal_Task", 2000, NULL, osPriorityNormal, &task_gimbal_task_handle);
    xTaskCreate(Gimbal_Manifold_Control_Task_Entry, "Task_Gimbal_Manifold_Control", 2000, NULL, osPriorityNormal, &task_gimbal_manifold_control_handle);
}
