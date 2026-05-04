# IMU 实验代码包

## 1. 目的

这个文件夹用于把你当前的 `YawFusion` 和 `Kalman` 实验代码集中打包，方便后续做这几件事：

- 归档保存
- 单独调参
- 和主链路结果做对比
- 后面继续整理成正式模块

这个包是刻意和当前 `yuntai` 主构建分开的：

- 不替换现有主控制链
- 不自动加入 `yuntai/CMakeLists.txt`
- 可以在仓库里长期保留，作为实验区和文档入口

按 2026-05-03 当前代码状态来看：

- `YawFusion` 已经是一个结构比较完整、可以反复测试的 yaw 实验模块
- `Kalman` 研究代码仍然更像研究稿，目前效果也还不够理想

## 2. 目录结构

```text
imu_experiment_pack/
|-- README.md
|-- source_map.md
|-- yaw_fusion/
|   |-- yaw_fusion.c
|   |-- yaw_fusion.h
|   |-- yaw_fusion_cfg.h
|   |-- yaw_fusion_test.c
|   `-- yaw_fusion_usage_example.c
`-- kalman_research/
    |-- alg_quaternion_paper_research.c
    `-- kalman_research_usage_example.c
```

## 3. 包里都有什么

### 3.1 `YawFusion`

`YawFusion` 是一个只处理 `yaw` 的融合实验模块。它当前使用的输入包括：

- yaw 电机编码器原始计数
- IMU 的 `gyro z`
- 加速度模长
- 温度
- 当前周期 `dt`

它的核心思路是：

1. 把编码器角度展开成连续角
2. 在静止状态下慢慢学习 gyro 零偏
3. 按需要做温漂补偿
4. 用补偿后的 gyro 做积分预测
5. 用编码器连续角把低频漂移拉回来

这一版已经比较适合反复做对比测试。

### 3.2 `Kalman` 研究代码

`alg_quaternion_paper_research.c` 是一份单文件研究版姿态解算代码，里面包含：

- 按轴自适应 Kalman 预处理 gyro
- 类最小二乘方式的 bias 趋势处理
- 四元数积分
- 连续 yaw 展开
- 一层观测封装接口，方便快速对比

但它现在仍然有几个明显特点：

- 不在主工程 `CMakeLists.txt` 里
- 还没有独立的对外头文件
- 更适合当“研究样例”，不建议直接当生产模块上车

## 4. `YawFusion` 如何使用

### 4.1 最小接入流程

1. 包含 `yaw_fusion.h` 和 `yaw_fusion_cfg.h`
2. 创建一个 `YawFusion_t` 状态对象
3. 调用 `YawFusion_Init()`
4. 每个控制周期依次喂入：
   - `YawFusion_UpdateEncoderRaw()`
   - `YawFusion_UpdateImu()`
   - `YawFusion_UpdateTemperature()`
5. 编码器参考建立后，调用一次 `YawFusion_AlignToEncoder()`
6. 调用 `YawFusion_Run()`
7. 用 `YawFusion_GetState()` 读结果

### 4.2 典型调用顺序

```c
YawFusion_t fusion;
YawFusionConfig_t cfg = YAW_FUSION_CFG_DEFAULT_INITIALIZER;
YawFusionDebug_t debug_state;
uint8_t aligned = 0u;

YawFusion_Init(&fusion, &cfg);

YawFusion_UpdateEncoderRaw(&fusion, encoder_raw_count);
YawFusion_UpdateImu(&fusion, gyro_z_rad_s, acc_norm_g, dt_s);
YawFusion_UpdateTemperature(&fusion, temp_c);

if ((aligned == 0u) && (fusion.enc_ready != 0u))
{
    YawFusion_AlignToEncoder(&fusion);
    aligned = 1u;
}

YawFusion_Run(&fusion);
YawFusion_GetState(&fusion, &debug_state);
```

注意：`YawFusion_AlignToEncoder()` 正常情况下只应在下面这些时机调用：

- 编码器第一次建立参考时
- 模块被 reset 之后
- 编码器掉线恢复之后

可以直接参考 `yaw_fusion/yaw_fusion_usage_example.c`。

### 4.3 重点观察哪些输出

优先看这些字段：

- `yaw_est_rad`：融合后的 yaw
- `yaw_enc_rad`：编码器连续角
- `yaw_gyro_rad`：纯 gyro 积分角
- `gyro_bias_rad_s`：当前学习到的 gyro 零偏
- `gyro_temp_comp_rad_s`：当前温漂补偿量
- `is_static`：静止判定结果
- `enc_ready`：编码器参考是否建立

### 4.4 如何运行现有 `YawFusion` 自测

`yaw_fusion_test.c` 已经内置了一组小测试，覆盖：

- 编码器跨圈展开
- 静止 bias 收敛
- 温漂补偿
- 匀速转动跟踪
- 编码器固定时的长时间不漂移

如果想在主机侧快速单独跑，思路是：

```text
编译 yaw_fusion.c + yaw_fusion_test.c
定义 YAW_FUSION_TEST_MAIN
运行生成的可执行文件
检查返回码
```

例如：

```text
gcc yaw_fusion.c yaw_fusion_test.c -DYAW_FUSION_TEST_MAIN -I. -lm -o yaw_fusion_test
```

如果你更想在固件环境里临时测试，也可以直接调用：

```c
int failed = YawFusion_RunAllTests();
```

然后把 `failed` 通过串口、USB 或调试口打出来。

### 4.5 推荐调参顺序

1. 先确认 `enc_dir` 和 `gyro_dir`
2. 先把 `k_temp_rad_s_per_c` 设成 `0`
3. 再调 `k_enc`
4. 再调静止判定阈值
5. 再调 `static_confirm_samples`
6. 最后调 `bias_alpha_static`
7. 温漂斜率放到最后再拟合

## 5. `Kalman` 研究代码如何使用

### 5.1 建议定位

先把这份代码当成“研究对比样例”，不要直接当量产模块来接主链路。

原因很明确：

- 还没有公共头文件
- 仍是单文件结构
- 还没接入主构建
- 你自己也已经观察到效果还不够好

### 5.2 当前可直接调用的入口

这份文件里目前已经有这些对外函数：

- `quat_paper_research_init()`
- `quat_paper_research_reset()`
- `quat_paper_research_update()`
- `quat_paper_research_map_sensor_to_body()`
- `quat_paper_research_yaw_reset()`
- `quat_paper_research_yaw_to_continuous()`
- `quat_paper_research_observe_reset()`
- `quat_paper_research_observe_update()`

如果只是想快速试一版，最直接的入口是：

- `quat_paper_research_observe_reset()`
- `quat_paper_research_observe_update()`

因为这层封装已经帮你处理了：

- 传感器坐标到机体系映射
- 研究版算法更新
- 连续 yaw 展开

### 5.3 快速试用方式

由于它现在没有单独头文件，最省事的临时试法是：

1. 单独写一个实验用 `c` 文件
2. 只在这个实验文件里 `#include "alg_quaternion_paper_research.c"`
3. 创建观测对象
4. 调 `quat_paper_research_observe_reset()`
5. 每周期喂入一帧 IMU 数据
6. 调 `quat_paper_research_observe_update()`
7. 读 `observe.paper_euler`、`observe.paper_debug` 等字段

这个做法只建议用于临时实验，不建议拿去做正式工程结构。

可以参考 `kalman_research/kalman_research_usage_example.c`。

### 5.4 如果后面还想继续做下去

建议先做结构整理，再继续深调参数：

1. 把公共结构体和函数声明拆到新的 `.h`
2. 保留内部工具函数为 `static`
3. 单独加一个测试入口
4. 用同一份日志同时对比 Mahony、YawFusion、Kalman 研究版

否则很难判断当前效果差，到底是：

- 算法本身问题
- 参数问题
- 轴方向问题
- 包角处理问题
- 还是 `dt`、输入预处理的问题

## 6. 推荐对比方法

如果你现在觉得效果不理想，建议按下面顺序排查：

1. 先确认所有链路的轴映射一致
2. 确认所有链路使用的是同一套 `dt`
3. 对比原始 gyro z、补偿后 gyro z、编码器 yaw、最终 yaw
4. 确认连续 yaw 的展开逻辑一致
5. 最后再比较最终输出效果

这样能避免把“方向符号错了”或者“包角处理不一致”误判成融合算法本身的问题。

## 7. 对当前构建的影响

这个代码包默认不会影响当前固件构建。

原因是顶层 [yuntai/CMakeLists.txt](../CMakeLists.txt) 仍然只引用原始源文件路径，没有把 `imu_experiment_pack/` 加进去。

所以你可以把这个目录当成：

- 实验归档区
- 调参沙盒
- 文档入口
- 后面重构前的源码快照
