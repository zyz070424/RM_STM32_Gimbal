#include "yaw_fusion.h"
#include "yaw_fusion_cfg.h"

/*
 * YawFusion 打包示例。
 * 这个文件默认不加入主固件构建。
 */

static YawFusion_t g_example_fusion;
static uint8_t g_example_aligned = 0u;

void YawFusion_ExampleInit(void)
{
    YawFusionConfig_t cfg = YAW_FUSION_CFG_DEFAULT_INITIALIZER;

    YawFusion_Init(&g_example_fusion, &cfg);
    g_example_aligned = 0u;
}

void YawFusion_ExampleStep(uint16_t encoder_raw_count,
                           float gyro_z_rad_s,
                           float acc_norm_g,
                           float temp_c,
                           float dt_s)
{
    YawFusionDebug_t debug_state;

    YawFusion_UpdateEncoderRaw(&g_example_fusion, encoder_raw_count);
    YawFusion_UpdateImu(&g_example_fusion, gyro_z_rad_s, acc_norm_g, dt_s);
    YawFusion_UpdateTemperature(&g_example_fusion, temp_c);

    if ((g_example_aligned == 0u) && (g_example_fusion.enc_ready != 0u))
    {
        YawFusion_AlignToEncoder(&g_example_fusion);
        g_example_aligned = 1u;
    }

    YawFusion_Run(&g_example_fusion);
    YawFusion_GetState(&g_example_fusion, &debug_state);

    (void)debug_state;
}
