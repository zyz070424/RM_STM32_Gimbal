#include "common_math.h"

/**
 * @brief  限幅函数，将值限制在指定范围内。
 * @param value 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @return 限制后的值。
 */
float Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief  绝对值限幅，当绝对值超过限制时，保持符号并限制绝对值。
 * @param value 输入值。
 * @param limit 绝对值限制（正数）。
 * @return 限制后的值。
 */
float AbsLimit(float value, float limit)
{
    if (limit < 0.0f)
    {
        limit = -limit;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

/**
 * @brief  符号函数，返回输入值的符号。
 * @param value 输入值。
 * @return 正数返回1.0f，负数返回-1.0f，零返回0.0f。
 */
float Sign(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }

    if (value < 0.0f)
    {
        return -1.0f;
    }

    return 0.0f;
}

/**
 * @brief  角度归一化（度），将角度归一化到 [-180, 180] 范围。
 * @param deg 输入角度（度）。
 * @return 归一化后的角度。
 */
float AngleWrapDeg(float deg)
{
    /* 先将角度归一化到 [0, 360) 范围 */
    deg = fmodf(deg, 360.0f);

    /* 处理负角度 */
    if (deg < 0.0f)
    {
        deg += 360.0f;
    }

    /* 转换到 [-180, 180] 范围 */
    if (deg > 180.0f)
    {
        deg -= 360.0f;
    }

    return deg;
}

/**
 * @brief  角度归一化（弧度），将角度归一化到 [-π, π] 范围。
 * @param rad 输入角度（弧度）。
 * @return 归一化后的角度。
 */
float AngleWrapRad(float rad)
{
    const float TWO_PI = 2.0f * M_PI;

    /* 先将角度归一化到 [0, 2π) 范围 */
    rad = fmodf(rad, TWO_PI);

    /* 处理负角度 */
    if (rad < 0.0f)
    {
        rad += TWO_PI;
    }

    /* 转换到 [-π, π] 范围 */
    if (rad > M_PI)
    {
        rad -= TWO_PI;
    }

    return rad;
}

/**
 * @brief  度转弧度。
 * @param deg 角度（度）。
 * @return 对应的弧度值。
 */
float DegToRad(float deg)
{
    return deg * (M_PI / 180.0f);
}

/**
 * @brief  弧度转度。
 * @param rad 弧度值。
 * @return 对应的角度（度）。
 */
float RadToDeg(float rad)
{
    return rad * (180.0f / M_PI);
}

/**
 * @brief  一阶低通滤波器，对输入信号进行平滑处理。
 * @param input 当前输入值。
 * @param prev_output 上一次的输出值。
 * @param alpha 滤波系数（0~1），越小滤波效果越强。
 * @return 滤波后的输出值。
 */
float FirstOrderLPF(float input, float prev_output, float alpha)
{
    /* 限制 alpha 在有效范围内 */
    alpha = Clamp(alpha, 0.0f, 1.0f);

    /* 一阶低通滤波公式: y[n] = alpha * x[n] + (1 - alpha) * y[n-1] */
    return alpha * input + (1.0f - alpha) * prev_output;
}

/**
 * @brief  斜率限制，限制输出值相对于上一次输出的最大变化率。
 * @param target 目标值。
 * @param prev_output 上一次的输出值。
 * @param max_delta 最大允许的变化量。
 * @return 限制变化率后的输出值。
 */
float SlewLimit(float target, float prev_output, float max_delta)
{
    float delta;

    /* 确保最大变化量为正数 */
    if (max_delta < 0.0f)
    {
        max_delta = -max_delta;
    }

    delta = target - prev_output;

    /* 限制变化量 */
    delta = AbsLimit(delta, max_delta);

    return prev_output + delta;
}

/**
 * @brief  判断浮点数是否为有限值。
 * @param value 输入值。
 * @return 有限值返回 true，否则返回 false。
 */
bool IsFiniteFloat(float value)
{
    return std::isfinite(value);
}

/**
 * @brief  将浮点数转换为 int16_t，溢出时做饱和截断。
 * @param  value 输入浮点数。
 * @return 饱和截断后的 int16_t 值。
 */
int16_t FloatToInt16Sat(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)value;
}

/**
 * @brief  将浮点数转换为 uint16_t，溢出时做饱和截断。
 * @param  value 输入浮点数。
 * @return 饱和截断后的 uint16_t 值。
 */
uint16_t FloatToUint16Sat(float value)
{
    if (value > 65535.0f)
    {
        return 65535u;
    }

    if (value < 0.0f)
    {
        return 0u;
    }

    return (uint16_t)value;
}

/**
 * @brief  安全除法，避免除零错误。
 * @param numerator 被除数。
 * @param denominator 除数。
 * @param default_value 除数为零时的默认返回值。
 * @return 除法结果，除数为零时返回默认值。
 */
float SafeDivide(float numerator, float denominator, float default_value)
{
    /* 检查除数是否接近零 */
    if (fabsf(denominator) < 1e-10f)
    {
        return default_value;
    }

    /* 检查结果是否有效 */
    float result = numerator / denominator;

    if (!IsFiniteFloat(result))
    {
        return default_value;
    }

    return result;
}

/**
 * @brief  范围检查，判断值是否在指定范围内。
 * @param value 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @return 在范围内返回true，否则返回false。
 */
bool RangeCheck(float value, float min_value, float max_value)
{
    return (value >= min_value) && (value <= max_value);
}
