#include "yaw_fusion.h"
#include "yaw_fusion_cfg.h"
#include <math.h>

#define TEST_PI            3.14159265358979323846f
#define TEST_TWO_PI        (2.0f * TEST_PI)
#define TEST_DT_S          0.001f
#define TEST_EPS_RAD       0.02f
#define TEST_EPS_RAD_S     0.02f

/**
 * @brief 浮点近似比较。
 * @param a 值 a。
 * @param b 值 b。
 * @param eps 允许误差。
 * @retval 1=近似相等，0=不相等。
 */
static int float_near(float a, float b, float eps)
{
    return (fabsf(a - b) <= eps) ? 1 : 0;
}

/**
 * @brief 构造默认测试配置。
 * @retval 默认测试配置。
 */
static YawFusionConfig_t make_default_cfg(void)
{
    YawFusionConfig_t cfg = YAW_FUSION_CFG_DEFAULT_INITIALIZER;

    return cfg;
}

/**
 * @brief 编码器跨圈连续性测试。
 * @retval 0=通过，非 0=失败。
 */
static int test_encoder_unwrap(void)
{
    YawFusion_t fusion;
    YawFusionConfig_t cfg = make_default_cfg();
    float step_rad;

    YawFusion_Init(&fusion, &cfg);
    YawFusion_UpdateEncoderRaw(&fusion, 8180u);
    YawFusion_UpdateEncoderRaw(&fusion, 8190u);
    YawFusion_UpdateEncoderRaw(&fusion, 5u);
    YawFusion_UpdateEncoderRaw(&fusion, 20u);

    step_rad = (32.0f * TEST_TWO_PI) / (float)cfg.enc_counts_per_rev;
    return float_near(fusion.yaw_enc_rad, step_rad, 0.001f) ? 0 : 1;
}

/**
 * @brief 静止时 bias 收敛测试。
 * @retval 0=通过，非 0=失败。
 */
static int test_static_bias_converge(void)
{
    YawFusion_t fusion;
    YawFusionConfig_t cfg = make_default_cfg();
    int32_t i;

    cfg.bias_alpha_static = 0.01f;
    cfg.static_confirm_samples = 20u;
    YawFusion_Init(&fusion, &cfg);
    YawFusion_UpdateEncoderRaw(&fusion, 100u);

    for (i = 0; i < 2000; i++)
    {
        YawFusion_UpdateEncoderRaw(&fusion, 100u);
        YawFusion_UpdateImu(&fusion, 0.02f, 1.0f, TEST_DT_S);
        YawFusion_UpdateTemperature(&fusion, 25.0f);
        YawFusion_Run(&fusion);
    }

    if (fusion.is_static == 0u)
    {
        return 1;
    }

    return float_near(fusion.gyro_bias_rad_s, 0.02f, 0.005f) ? 0 : 1;
}

/**
 * @brief 线性温漂补偿接口测试。
 * @retval 0=通过，非 0=失败。
 */
static int test_temperature_comp(void)
{
    YawFusion_t fusion;
    YawFusionConfig_t cfg = make_default_cfg();

    cfg.k_temp_rad_s_per_c = 0.002f;
    cfg.temp_ref_c = 25.0f;
    YawFusion_Init(&fusion, &cfg);
    YawFusion_UpdateImu(&fusion, 0.0f, 1.0f, TEST_DT_S);
    YawFusion_UpdateTemperature(&fusion, 35.0f);
    YawFusion_Run(&fusion);

    return float_near(fusion.gyro_temp_comp_rad_s, 0.02f, 1.0e-6f) ? 0 : 1;
}

/**
 * @brief 匀速转动跟踪测试。
 * @retval 0=通过，非 0=失败。
 */
static int test_constant_rate_tracking(void)
{
    YawFusion_t fusion;
    YawFusionConfig_t cfg = make_default_cfg();
    float yaw_cmd_rad_s = 1.0f;
    float yaw_true_rad = 0.0f;
    int32_t i;
    int32_t enc_count;

    cfg.k_enc = 0.05f;
    YawFusion_Init(&fusion, &cfg);

    for (i = 0; i < 3000; i++)
    {
        yaw_true_rad += yaw_cmd_rad_s * TEST_DT_S;
        enc_count = (int32_t)lroundf((yaw_true_rad / TEST_TWO_PI) * (float)cfg.enc_counts_per_rev);
        enc_count %= (int32_t)cfg.enc_counts_per_rev;
        if (enc_count < 0)
        {
            enc_count += (int32_t)cfg.enc_counts_per_rev;
        }

        YawFusion_UpdateEncoderRaw(&fusion, (uint16_t)enc_count);
        YawFusion_UpdateImu(&fusion, yaw_cmd_rad_s, 1.0f, TEST_DT_S);
        YawFusion_UpdateTemperature(&fusion, 25.0f);
        if ((i == 0) && (fusion.enc_ready != 0u))
        {
            YawFusion_AlignToEncoder(&fusion);
        }
        YawFusion_Run(&fusion);
    }

    return float_near(fusion.yaw_est_rad, fusion.yaw_enc_rad, TEST_EPS_RAD) ? 0 : 1;
}

/**
 * @brief 编码器固定且 gyro 有微小偏置时长时不漂移测试。
 * @retval 0=通过，非 0=失败。
 */
static int test_long_term_no_drift_with_fixed_encoder(void)
{
    YawFusion_t fusion;
    YawFusionConfig_t cfg = make_default_cfg();
    int32_t i;

    cfg.bias_alpha_static = 0.01f;
    cfg.static_confirm_samples = 20u;
    cfg.k_enc = 0.05f;
    YawFusion_Init(&fusion, &cfg);
    YawFusion_UpdateEncoderRaw(&fusion, 512u);
    YawFusion_AlignToEncoder(&fusion);

    for (i = 0; i < 8000; i++)
    {
        YawFusion_UpdateEncoderRaw(&fusion, 512u);
        YawFusion_UpdateImu(&fusion, 0.01f, 1.0f, TEST_DT_S);
        YawFusion_UpdateTemperature(&fusion, 25.0f);
        YawFusion_Run(&fusion);
    }

    if (fabsf(fusion.yaw_est_rad) > 0.05f)
    {
        return 1;
    }

    return (fabsf(fusion.gyro_z_corr_rad_s) < TEST_EPS_RAD_S) ? 0 : 1;
}

/**
 * @brief 运行全部 yaw 融合测试。
 * @retval 失败用例数量，0 表示全部通过。
 */
int YawFusion_RunAllTests(void)
{
    int failed = 0;

    failed += test_encoder_unwrap();
    failed += test_static_bias_converge();
    failed += test_temperature_comp();
    failed += test_constant_rate_tracking();
    failed += test_long_term_no_drift_with_fixed_encoder();

    return failed;
}

#ifdef YAW_FUSION_TEST_MAIN
int main(void)
{
    return YawFusion_RunAllTests();
}
#endif
