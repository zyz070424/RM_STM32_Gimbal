/**
 * @file alg_dwt.cpp
 * @brief DWT 时间基准实现。
 * @details
 * 本文件实现 `Class_DWT_Timebase` 的成员函数。
 */
#include "alg_dwt.h"

#include <math.h>

namespace
{
constexpr float Alg_Dwt_Fallback_Default_Dt_S = 0.001f;

/**
 * @brief 消除无效时间间隔并进行限幅。
 * @param dt 输入时间间隔，单位 s。
 * @param default_dt_s 默认时间间隔，单位 s。
 * @param dt_min_s 最小允许时间间隔，单位 s。
 * @param dt_max_s 最大允许时间间隔，单位 s。
 * @return 限幅后的时间间隔，单位 s。
 */
float ALG_DWT_Sanitize_And_Clamp_Dt(float dt,
                                    float default_dt_s,
                                    float dt_min_s,
                                    float dt_max_s)
{
    if ((isfinite(default_dt_s) == 0) || (default_dt_s <= 0.0f))
    {
        default_dt_s = Alg_Dwt_Fallback_Default_Dt_S;
    }

    if ((isfinite(dt_min_s) == 0) || (dt_min_s <= 0.0f))
    {
        dt_min_s = default_dt_s;
    }

    if ((isfinite(dt_max_s) == 0) || (dt_max_s <= 0.0f))
    {
        dt_max_s = default_dt_s;
    }

    if (dt_min_s > dt_max_s)
    {
        float temp = dt_min_s;

        dt_min_s = dt_max_s;
        dt_max_s = temp;
    }

    if ((isfinite(dt) == 0) || (dt <= 0.0f))
    {
        dt = default_dt_s;
    }

    if (dt < dt_min_s)
    {
        dt = dt_min_s;
    }
    else if (dt > dt_max_s)
    {
        dt = dt_max_s;
    }

    return dt;
}
}

/**
 * @brief 初始化 DWT 时间基准对象。
 * @param default_dt_s 默认时间间隔，单位 s。
 * @return 无。
 */
void Class_DWT_Timebase::Init(float default_dt_s)
{
    default_dt_s = ALG_DWT_Sanitize_And_Clamp_Dt(default_dt_s,
                                                 default_dt_s,
                                                 default_dt_s,
                                                 default_dt_s);

    dt_ready = 0u;
    dwt_ready = 0u;
    last_cycle = 0u;
    last_tick_ms = HAL_GetTick();
    last_dt_s = default_dt_s;
    last_dt_from_dwt = 0u;

#if defined(CoreDebug) && defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55;
#endif
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u) ? 1u : 0u;
    last_cycle = DWT->CYCCNT;
#endif
}

/**
 * @brief 获取当前周期的时间间隔。
 * @param default_dt_s 默认时间间隔，单位 s。
 * @param dt_min_s 最小允许时间间隔，单位 s。
 * @param dt_max_s 最大允许时间间隔，单位 s。
 * @return 当前周期使用的时间间隔，单位 s。
 */
float Class_DWT_Timebase::GetDtS(float default_dt_s, float dt_min_s, float dt_max_s)
{
    float dt = default_dt_s;
    uint32_t now_tick_ms = HAL_GetTick();

#if defined(CoreDebug) && defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if ((dwt_ready != 0u) && (SystemCoreClock > 0u) &&
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u))
    {
        uint32_t now_cycle = DWT->CYCCNT;

        if (dt_ready != 0u)
        {
            uint32_t delta_cycle = now_cycle - last_cycle;

            dt = (float)delta_cycle / (float)SystemCoreClock;
            last_dt_from_dwt = 1u;
        }

        last_cycle = now_cycle;
    }
    else
#endif
    {
        if (dt_ready != 0u)
        {
            uint32_t delta_tick_ms = now_tick_ms - last_tick_ms;

            if (delta_tick_ms > 0u)
            {
                dt = (float)delta_tick_ms * 0.001f;
            }

            last_dt_from_dwt = 0u;
        }
    }

    last_tick_ms = now_tick_ms;
    dt_ready = 1u;
    dt = ALG_DWT_Sanitize_And_Clamp_Dt(dt, default_dt_s, dt_min_s, dt_max_s);
    last_dt_s = dt;

    return dt;
}
