#include "yaw_fusion.h"
#include <math.h>
#include <string.h>

#define YAW_FUSION_PI            3.14159265358979323846f
#define YAW_FUSION_TWO_PI        (2.0f * YAW_FUSION_PI)
#define YAW_FUSION_DT_DEFAULT_S  0.001f

/**
 * @brief 将角度包裹到 [-pi, pi)。
 * @param angle_rad 输入角度，单位：rad。
 * @retval 包裹后的角度，单位：rad。
 */
static float YawFusion_WrapPi(float angle_rad)
{
    while (angle_rad >= YAW_FUSION_PI)
    {
        angle_rad -= YAW_FUSION_TWO_PI;
    }

    while (angle_rad < -YAW_FUSION_PI)
    {
        angle_rad += YAW_FUSION_TWO_PI;
    }

    return angle_rad;
}

/**
 * @brief 浮点限幅。
 * @param value 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @retval 限幅后的值。
 */
static float YawFusion_Clampf(float value, float min_value, float max_value)
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
 * @brief 清洗采样周期。
 * @param fusion 融合状态对象指针。
 * @param dt_s 输入采样周期，单位：s。
 * @retval 清洗后的采样周期，单位：s。
 */
static float YawFusion_SanitizeDt(const YawFusion_t *fusion, float dt_s)
{
    float dt_min_s;
    float dt_max_s;

    if (fusion == NULL)
    {
        return YAW_FUSION_DT_DEFAULT_S;
    }

    dt_min_s = fusion->cfg.dt_min_s;
    dt_max_s = fusion->cfg.dt_max_s;

    if ((dt_min_s <= 0.0f) || (dt_max_s <= dt_min_s))
    {
        dt_min_s = 0.0002f;
        dt_max_s = 0.0100f;
    }

    if ((isfinite(dt_s) == 0) || (dt_s <= 0.0f))
    {
        return YawFusion_Clampf(YAW_FUSION_DT_DEFAULT_S, dt_min_s, dt_max_s);
    }

    return YawFusion_Clampf(dt_s, dt_min_s, dt_max_s);
}

/**
 * @brief 根据半圈阈值解开编码器跨圈增量。
 * @param delta_count 原始计数增量。
 * @param counts_per_rev 编码器每圈计数。
 * @retval 解包后的连续计数增量。
 */
static int32_t YawFusion_UnwrapEncoderDelta(int32_t delta_count, uint16_t counts_per_rev)
{
    int32_t half_counts;

    if (counts_per_rev == 0u)
    {
        return 0;
    }

    half_counts = (int32_t)counts_per_rev / 2;
    if (delta_count > half_counts)
    {
        delta_count -= (int32_t)counts_per_rev;
    }
    else if (delta_count < -half_counts)
    {
        delta_count += (int32_t)counts_per_rev;
    }

    return delta_count;
}

/**
 * @brief 计算线性温漂补偿。
 * @param fusion 融合状态对象指针。
 * @retval 当前温漂补偿量，单位：rad/s。
 * @note  后续若替换为查表或分段线性，只需要改这一个 helper。
 */
static float YawFusion_ComputeTempCompLinear(const YawFusion_t *fusion)
{
    if (fusion == NULL)
    {
        return 0.0f;
    }

    return fusion->cfg.k_temp_rad_s_per_c * (fusion->temp_c - fusion->cfg.temp_ref_c);
}

/**
 * @brief 标准化配置，避免非法参数污染运行态。
 * @param cfg 配置对象指针。
 * @retval 无
 */
static void YawFusion_NormalizeConfig(YawFusionConfig_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    if (cfg->enc_counts_per_rev == 0u)
    {
        cfg->enc_counts_per_rev = 8192u;
    }

    cfg->enc_dir = (cfg->enc_dir >= 0) ? 1 : -1;
    cfg->gyro_dir = (cfg->gyro_dir >= 0) ? 1 : -1;
    cfg->k_enc = YawFusion_Clampf(cfg->k_enc, 0.0f, 1.0f);
    cfg->bias_alpha_static = YawFusion_Clampf(cfg->bias_alpha_static, 0.0f, 1.0f);

    if ((cfg->static_confirm_samples == 0u))
    {
        cfg->static_confirm_samples = 1u;
    }

    if ((cfg->dt_min_s <= 0.0f) || (cfg->dt_max_s <= cfg->dt_min_s))
    {
        cfg->dt_min_s = 0.0002f;
        cfg->dt_max_s = 0.0100f;
    }
}

void YawFusion_Init(YawFusion_t *fusion, const YawFusionConfig_t *cfg)
{
    if ((fusion == NULL) || (cfg == NULL))
    {
        return;
    }

    memset(fusion, 0, sizeof(YawFusion_t));
    fusion->cfg = *cfg;
    YawFusion_NormalizeConfig(&fusion->cfg);
    fusion->dt_s = YawFusion_SanitizeDt(fusion, YAW_FUSION_DT_DEFAULT_S);
}

void YawFusion_Reset(YawFusion_t *fusion)
{
    YawFusionConfig_t cfg_backup;

    if (fusion == NULL)
    {
        return;
    }

    cfg_backup = fusion->cfg;
    memset(fusion, 0, sizeof(YawFusion_t));
    fusion->cfg = cfg_backup;
    fusion->dt_s = YawFusion_SanitizeDt(fusion, YAW_FUSION_DT_DEFAULT_S);
}

void YawFusion_AlignToEncoder(YawFusion_t *fusion)
{
    if ((fusion == NULL) || (fusion->enc_ready == 0u))
    {
        return;
    }

    fusion->yaw_est_rad = fusion->yaw_enc_rad;
    fusion->yaw_gyro_rad = fusion->yaw_enc_rad;
    fusion->yaw_err_rad = 0.0f;
}

void YawFusion_UpdateEncoderRaw(YawFusion_t *fusion, uint16_t enc_raw_count)
{
    int32_t delta_count;

    if (fusion == NULL)
    {
        return;
    }

    if (fusion->enc_ready == 0u)
    {
        fusion->enc_raw_last_count = enc_raw_count;
        fusion->enc_unwrapped_count = 0;
        fusion->yaw_enc_rad = 0.0f;
        fusion->yaw_enc_last_run_rad = 0.0f;
        fusion->enc_vel_rad_s = 0.0f;
        fusion->enc_ready = 1u;
        return;
    }

    delta_count = (int32_t)enc_raw_count - (int32_t)fusion->enc_raw_last_count;
    delta_count = YawFusion_UnwrapEncoderDelta(delta_count, fusion->cfg.enc_counts_per_rev);
    fusion->enc_raw_last_count = enc_raw_count;
    fusion->enc_unwrapped_count += delta_count;
    fusion->yaw_enc_rad = (float)fusion->enc_unwrapped_count *
                          (YAW_FUSION_TWO_PI / (float)fusion->cfg.enc_counts_per_rev) *
                          (float)fusion->cfg.enc_dir;
}

void YawFusion_UpdateImu(YawFusion_t *fusion, float gyro_z_rad_s, float acc_norm_g, float dt_s)
{
    if (fusion == NULL)
    {
        return;
    }

    if (isfinite(gyro_z_rad_s) == 0)
    {
        gyro_z_rad_s = 0.0f;
    }

    if (isfinite(acc_norm_g) == 0)
    {
        acc_norm_g = 1.0f;
    }

    fusion->gyro_z_raw_rad_s = (float)fusion->cfg.gyro_dir * gyro_z_rad_s;
    fusion->acc_norm_g = acc_norm_g;
    fusion->dt_s = YawFusion_SanitizeDt(fusion, dt_s);
}

void YawFusion_UpdateTemperature(YawFusion_t *fusion, float temp_c)
{
    if (fusion == NULL)
    {
        return;
    }

    if (isfinite(temp_c) == 0)
    {
        temp_c = fusion->cfg.temp_ref_c;
    }

    fusion->temp_c = temp_c;
}

void YawFusion_Run(YawFusion_t *fusion)
{
    float gyro_z_corr_candidate_rad_s;
    float yaw_pred_rad;
    uint8_t static_candidate;

    if (fusion == NULL)
    {
        return;
    }

    if (fusion->enc_ready != 0u)
    {
        fusion->enc_vel_rad_s = (fusion->yaw_enc_rad - fusion->yaw_enc_last_run_rad) / fusion->dt_s;
        fusion->yaw_enc_last_run_rad = fusion->yaw_enc_rad;
    }
    else
    {
        fusion->enc_vel_rad_s = 0.0f;
    }

    fusion->gyro_temp_comp_rad_s = YawFusion_ComputeTempCompLinear(fusion);
    gyro_z_corr_candidate_rad_s = fusion->gyro_z_raw_rad_s - fusion->gyro_bias_rad_s - fusion->gyro_temp_comp_rad_s;

    static_candidate = ((fabsf(gyro_z_corr_candidate_rad_s) <= fusion->cfg.static_gyro_th_rad_s) &&
                        (fabsf(fusion->enc_vel_rad_s) <= fusion->cfg.static_enc_vel_th_rad_s) &&
                        (fabsf(fusion->acc_norm_g - 1.0f) <= fusion->cfg.static_acc_norm_th_g)) ? 1u : 0u;

    if (static_candidate != 0u)
    {
        if (fusion->static_counter < fusion->cfg.static_confirm_samples)
        {
            fusion->static_counter++;
        }
    }
    else
    {
        fusion->static_counter = 0u;
    }

    fusion->is_static = (fusion->static_counter >= fusion->cfg.static_confirm_samples) ? 1u : 0u;

    if (fusion->is_static != 0u)
    {
        fusion->gyro_bias_rad_s += fusion->cfg.bias_alpha_static *
                                   ((fusion->gyro_z_raw_rad_s - fusion->gyro_temp_comp_rad_s) - fusion->gyro_bias_rad_s);
    }

    fusion->gyro_z_corr_rad_s = fusion->gyro_z_raw_rad_s - fusion->gyro_bias_rad_s - fusion->gyro_temp_comp_rad_s;
    fusion->yaw_gyro_rad += fusion->gyro_z_corr_rad_s * fusion->dt_s;

    yaw_pred_rad = fusion->yaw_est_rad + fusion->gyro_z_corr_rad_s * fusion->dt_s;
    if (fusion->enc_ready != 0u)
    {
        fusion->yaw_err_rad = YawFusion_WrapPi(fusion->yaw_enc_rad - yaw_pred_rad);
        fusion->yaw_est_rad = yaw_pred_rad + fusion->cfg.k_enc * fusion->yaw_err_rad;
    }
    else
    {
        fusion->yaw_err_rad = 0.0f;
        fusion->yaw_est_rad = yaw_pred_rad;
    }
}

void YawFusion_GetState(const YawFusion_t *fusion, YawFusionDebug_t *out_state)
{
    if ((fusion == NULL) || (out_state == NULL))
    {
        return;
    }

    out_state->yaw_est_rad = fusion->yaw_est_rad;
    out_state->yaw_enc_rad = fusion->yaw_enc_rad;
    out_state->yaw_gyro_rad = fusion->yaw_gyro_rad;
    out_state->gyro_bias_rad_s = fusion->gyro_bias_rad_s;
    out_state->gyro_temp_comp_rad_s = fusion->gyro_temp_comp_rad_s;
    out_state->enc_vel_rad_s = fusion->enc_vel_rad_s;
    out_state->yaw_err_rad = fusion->yaw_err_rad;
    out_state->gyro_z_corr_rad_s = fusion->gyro_z_corr_rad_s;
    out_state->is_static = fusion->is_static;
    out_state->enc_ready = fusion->enc_ready;
}
