#ifndef __YAW_FUSION_H__
#define __YAW_FUSION_H__

#include <stdint.h>

/**
 * @brief Yaw 融合参数配置。
 * @note  模块内部统一使用 rad / rad/s / s，边界换算由调用方负责。
 */
typedef struct
{
    uint16_t enc_counts_per_rev;       /**< 编码器每圈计数，例如 8192 count/rev */
    int8_t enc_dir;                    /**< 编码器正方向映射，取值通常为 +1 或 -1 */
    int8_t gyro_dir;                   /**< 陀螺 z 轴正方向映射，取值通常为 +1 或 -1 */
    float k_enc;                       /**< 编码器校正增益，控制低频绝对角修正强度 */
    float bias_alpha_static;           /**< 静止时 gyro 零偏慢速更新系数 [0, 1] */
    float temp_ref_c;                  /**< 温漂参考温度，单位：摄氏度 */
    float k_temp_rad_s_per_c;          /**< 线性温漂斜率，单位：rad/s/degC */
    float static_gyro_th_rad_s;        /**< 静止检测陀螺阈值，单位：rad/s */
    float static_enc_vel_th_rad_s;     /**< 静止检测编码器角速度阈值，单位：rad/s */
    float static_acc_norm_th_g;        /**< 静止检测加速度模长偏差阈值，单位：g */
    uint16_t static_confirm_samples;   /**< 静止判稳所需连续样本数 */
    float dt_min_s;                    /**< 合法 dt 下限，单位：s */
    float dt_max_s;                    /**< 合法 dt 上限，单位：s */
} YawFusionConfig_t;

/**
 * @brief Yaw 融合内部状态。
 * @note  该结构体保存全部运行状态，不使用动态内存。
 */
typedef struct
{
    YawFusionConfig_t cfg;             /**< 配置副本 */
    uint16_t enc_raw_last_count;       /**< 上一拍原始编码器计数 */
    int32_t enc_unwrapped_count;       /**< 展开后的连续编码器计数 */
    float yaw_enc_rad;                 /**< 当前编码器连续角，单位：rad */
    float yaw_enc_last_run_rad;        /**< 上一拍 Run 使用过的编码器角，单位：rad */
    float enc_vel_rad_s;               /**< 当前编码器角速度估计，单位：rad/s */
    float gyro_z_raw_rad_s;            /**< 原始陀螺 z 轴输入，单位：rad/s */
    float gyro_z_corr_rad_s;           /**< bias 和温漂补偿后的陀螺，单位：rad/s */
    float gyro_bias_rad_s;             /**< 在线估计的陀螺零偏，单位：rad/s */
    float gyro_temp_comp_rad_s;        /**< 当前线性温漂补偿量，单位：rad/s */
    float temp_c;                      /**< 当前温度输入，单位：摄氏度 */
    float acc_norm_g;                  /**< 当前加速度模长输入，单位：g */
    float dt_s;                        /**< 当前使用的采样周期，单位：s */
    float yaw_est_rad;                 /**< 互补融合输出，单位：rad */
    float yaw_gyro_rad;                /**< 纯陀螺积分输出，单位：rad */
    float yaw_err_rad;                 /**< 编码器与预测角误差，单位：rad */
    uint16_t static_counter;           /**< 静止判稳计数器 */
    uint8_t enc_ready;                 /**< 编码器是否已建立连续角参考 */
    uint8_t is_static;                 /**< 当前是否判定为静止 */
} YawFusion_t;

/**
 * @brief Yaw 融合调试输出。
 * @note  该结构体用于外部观测，不暴露内部所有缓存量。
 */
typedef struct
{
    float yaw_est_rad;                 /**< 互补融合输出角，单位：rad */
    float yaw_enc_rad;                 /**< 编码器连续角，单位：rad */
    float yaw_gyro_rad;                /**< 纯陀螺积分角，单位：rad */
    float gyro_bias_rad_s;             /**< 当前零偏估计，单位：rad/s */
    float gyro_temp_comp_rad_s;        /**< 当前温漂补偿量，单位：rad/s */
    float enc_vel_rad_s;               /**< 当前编码器角速度估计，单位：rad/s */
    float yaw_err_rad;                 /**< 当前编码器校正误差，单位：rad */
    float gyro_z_corr_rad_s;           /**< 当前补偿后陀螺角速度，单位：rad/s */
    uint8_t is_static;                 /**< 当前静止状态标志 */
    uint8_t enc_ready;                 /**< 编码器连续角是否已初始化 */
} YawFusionDebug_t;

/**
 * @brief 初始化 yaw 融合模块。
 * @param fusion 融合状态对象指针，不能为空。
 * @param cfg 配置指针，不能为空。
 * @retval 无
 */
void YawFusion_Init(YawFusion_t *fusion, const YawFusionConfig_t *cfg);

/**
 * @brief 复位 yaw 融合运行态。
 * @param fusion 融合状态对象指针，不能为空。
 * @retval 无
 * @note  仅清空运行态，保留初始化时写入的配置副本。
 */
void YawFusion_Reset(YawFusion_t *fusion);

/**
 * @brief 将融合输出对齐到当前编码器连续角。
 * @param fusion 融合状态对象指针，不能为空。
 * @retval 无
 * @note  只在编码器已准备就绪时生效，不修改零偏估计。
 */
void YawFusion_AlignToEncoder(YawFusion_t *fusion);

/**
 * @brief 输入一拍原始编码器计数并完成包角展开。
 * @param fusion 融合状态对象指针，不能为空。
 * @param enc_raw_count 原始编码器计数，通常范围为 0 ~ counts_per_rev-1。
 * @retval 无
 */
void YawFusion_UpdateEncoderRaw(YawFusion_t *fusion, uint16_t enc_raw_count);

/**
 * @brief 输入一拍 IMU 观测量。
 * @param fusion 融合状态对象指针，不能为空。
 * @param gyro_z_rad_s 原始 gyro z 角速度，单位：rad/s。
 * @param acc_norm_g 当前加速度模长，单位：g。
 * @param dt_s 当前采样周期，单位：s。
 * @retval 无
 */
void YawFusion_UpdateImu(YawFusion_t *fusion, float gyro_z_rad_s, float acc_norm_g, float dt_s);

/**
 * @brief 输入一拍温度观测量。
 * @param fusion 融合状态对象指针，不能为空。
 * @param temp_c 当前温度，单位：摄氏度。
 * @retval 无
 */
void YawFusion_UpdateTemperature(YawFusion_t *fusion, float temp_c);

/**
 * @brief 执行一拍 yaw 融合更新。
 * @param fusion 融合状态对象指针，不能为空。
 * @retval 无
 */
void YawFusion_Run(YawFusion_t *fusion);

/**
 * @brief 读取当前 yaw 融合调试状态。
 * @param fusion 融合状态对象指针，不能为空。
 * @param out_state 输出调试状态指针，不能为空。
 * @retval 无
 */
void YawFusion_GetState(const YawFusion_t *fusion, YawFusionDebug_t *out_state);

#endif /* __YAW_FUSION_H__ */
