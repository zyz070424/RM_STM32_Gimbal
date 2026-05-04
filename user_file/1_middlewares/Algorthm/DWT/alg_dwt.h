/**
 * @file alg_dwt.h
 * @brief DWT 时间基准接口与管理对象定义。
 * @details
 * 本文件定义 `Class_DWT_Timebase` DWT 时间基对象。
 */
#ifndef __ALG_DWT_H__
#define __ALG_DWT_H__

#include "main.h"
#include <stdint.h>


#ifdef __cplusplus
/**
 * @class Class_DWT_Timebase
 * @brief DWT 时间基准管理对象。
 * @details
 * 负责初始化 DWT 周期计数器，并在运行时优先使用 DWT 计算 dt，
 * 当 DWT 不可用时自动回退到 HAL Tick。
 */
class Class_DWT_Timebase
{
public:
    uint8_t dt_ready;          /**< 是否已经具备可用的上一周期时间戳 */
    uint8_t dwt_ready;         /**< DWT 周期计数器是否可用 */
    uint32_t last_cycle;       /**< 上一次记录的 DWT 周期计数 */
    uint32_t last_tick_ms;     /**< 上一次记录的系统 Tick，单位 ms */
    float last_dt_s;           /**< 上一次输出的 dt，单位 s */
    uint8_t last_dt_from_dwt;  /**< 上一次 dt 是否来自 DWT 周期计数 */

    void Init(float default_dt_s);
    float GetDtS(float default_dt_s, float dt_min_s, float dt_max_s);
};
#endif

#endif /* __ALG_DWT_H__ */
