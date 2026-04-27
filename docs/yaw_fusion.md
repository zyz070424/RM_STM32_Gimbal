# Yaw Fusion

## 模块目标

这套 `yaw_fusion` 模块是给当前 RoboMaster 云台工程做并行对比用的 yaw 融合层，用来对比现有 `BMI088_Complementary_Filter()` 的纯 IMU yaw 连续积分方案。

当前实现只做观测，不替换现有 `Gimbal_Euler_Angle_to_send.yaw` 控制链。

## 单位约定

- 角度：`rad`
- 角速度：`rad/s`
- 采样周期：`s`
- 加速度模长：`g`
- 温度：`degC`

## 核心流程

1. 输入原始编码器计数，做跨圈展开，得到连续 `yaw_enc_rad`
2. 输入 gyro z、加速度模长、温度
3. 先做 `bias + 温漂` 补偿，得到 `gyro_z_corr_rad_s`
4. 联合 `|gyro_z_corr|`、`|enc_vel|`、`|acc_norm - 1|` 做静止检测
5. 静止确认后，慢速更新 `gyro_bias_rad_s`
6. 用补偿后 gyro 做积分预测
7. 用编码器连续角做低频误差校正

核心公式：

```text
gyro_temp_comp = k_temp * (temp - temp_ref)
gyro_z_corr = gyro_z_raw - gyro_bias - gyro_temp_comp

yaw_pred = yaw_est + gyro_z_corr * dt
yaw_err  = wrap_pi(yaw_enc - yaw_pred)
yaw_est  = yaw_pred + k_enc * yaw_err
```

## 当前工程接线点

- 模块文件：
  - `user_file/1_middlewares/Algorthm/YawFusion/yaw_fusion.h`
  - `user_file/1_middlewares/Algorthm/YawFusion/yaw_fusion.c`
  - `user_file/1_middlewares/Algorthm/YawFusion/yaw_fusion_cfg.h`
- 并行接线：
  - `user_file/3_Module/Gimbal/Gimbal.c`
  - `user_file/3_Module/Gimbal/Gimbal.h`

当前 `Gimbal_Euler()` 每 1 ms 并行喂入：

- `encoder_raw_count = Gimbal_Motor_Yaw.RxData.Last_encoder_angle`
- `gyro_z_rad_s = Gimbal_IMU_Data.gyro.z * pi / 180`
- `acc_norm_g = sqrt(ax^2 + ay^2 + az^2)`
- `temp_c = Gimbal_IMU_Data.temp`
- `dt_s = imu_dt`

## 调试字段

`YawFusionDebug_t` 当前提供以下观测量：

- `yaw_est_rad`：互补融合输出
- `yaw_enc_rad`：编码器连续角
- `yaw_gyro_rad`：纯 gyro 积分角
- `gyro_bias_rad_s`：静止学习得到的零偏
- `gyro_temp_comp_rad_s`：线性温漂补偿量
- `enc_vel_rad_s`：编码器角速度估计
- `yaw_err_rad`：编码器校正误差
- `gyro_z_corr_rad_s`：最终补偿后 gyro
- `is_static`：静止判定
- `enc_ready`：编码器参考是否建立

## 参数调试顺序

1. 先确认 `enc_dir` 和 `gyro_dir` 正方向一致
2. 把 `k_temp_rad_s_per_c` 先设成 `0`
3. 调 `k_enc`
   - 太小：长时间更依赖 gyro，漂移压不住
   - 太大：会把编码器量化噪声直接带进输出
4. 调静止阈值
   - `static_gyro_th_rad_s`
   - `static_enc_vel_th_rad_s`
   - `static_acc_norm_th_g`
5. 调 `static_confirm_samples`
   - 太小容易误判静止
   - 太大静止 bias 学习太慢
6. 最后调 `bias_alpha_static`
   - 太小 bias 收敛慢
   - 太大容易把短时扰动学进去
7. 有温度数据后再拟合 `k_temp_rad_s_per_c`

## 如何接入现有 yaw 控制环

当前方案只做并行对比，不改主控链。

如果后续要试切到新融合输出，建议只改 yaw 外环反馈源，把当前：

```text
Gimbal_Euler_Angle_to_send.yaw
```

替换成：

```text
YawFusionDebug_t.yaw_est_rad -> rad 转 deg
```

并保持 yaw 速度内环、电机驱动、IMU 读取链路不变。第一次切换前，建议先调用一次 `YawFusion_AlignToEncoder()`，避免模式切换时跳角。

## 还需要补充的硬件参数

- 编码器正方向是否与定义的正 yaw 一致
- gyro z 正方向是否与定义的正 yaw 一致
- yaw 电机编码器到实际 yaw 轴是否 1:1
- 线性温漂斜率 `k_temp_rad_s_per_c`
- 机械前向零位与编码器零位之间的固定偏移
