# yuntai

`yuntai` 是一套基于 `STM32F405xx + STM32CubeMX + FreeRTOS` 的云台控制工程。当前代码已经具备完整的板级初始化、任务调度、姿态解算、双轴电机级联 PID、USB/CAN/SPI 通信抽象，以及链路存活检测与状态切换处理机制，适合作为云台控制与视觉联调主工程继续迭代。

这份 README 以当前仓库里的真实代码为准，不按历史计划或旧版调试文档描述。

## 1. 当前工程状态

当前版本已经不是“纯正弦测试目标版”了，主链路更接近一套“哨兵扫描 + 视觉跟踪 + 丢目标回扫”的完整云台控制框架：

```text
BMI088(SPI1) -> Mahony姿态解算 -> Gimbal_Euler_Angle_to_send
                                   |-> 作为当前控制反馈
                                   |-> USB CDC 回传给视觉/上位机

Yaw 电机编码器 + IMU 陀螺仪 + 温度 -> YawFusion 并行观测
                                      |-> Gimbal_Yaw_Fusion_Observe
                                      |-> Gimbal_Euler_Angle_Final_Observe（仅观测，不接管控制环）

USB CDC 视觉帧 -> Manifold 解包 -> Target_Valid/目标角
                                  -> Sentry 目标生成（SCAN / TRACK_ARMOR / LOST_TARGET_RETURN_SCAN）
                                  -> 双轴角度外环 PID
                                  -> 双轴速度内环 PID
                                  -> CAN2 -> GM6020
```

需要特别说明的是：

- `Gimbal_Motor_Control_ALL_Test()` 这个函数名还保留了早期 `Test` 命名，但当前内部控制逻辑已经接上 `Sentry` 目标生成，不再使用正弦测试目标。
- 视觉 USB 接收目标 `Rx_Data.Taget_Pitch` / `Rx_Data.Taget_Yaw` 已经进入控制流程，但不是“裸接 PID”，而是先经过 `Gimbal_Sentry_Target_Update()` 做目标有效性、滤波、状态切换和回扫处理。
- `YawFusion` 已经在运行，但现在仍属于“并行观测链路”，主要用于和现有 Mahony yaw 做比对，不直接接管当前控制反馈和 USB 回传输出。
- `DR16`、`VOFA`、`drv_usart`、`Motor_Protect` 等模块已经存在，但还没有进入当前运行主线。

如果你现在重新接手这个工程，可以把它理解成：

- 底层外设和通信框架已经搭好。
- 云台双轴闭环和视觉目标接入已经跑通。
- 视觉逻辑已经具备基本可用的状态机，而不是只停留在协议解包层。
- 还存在一些预留模块、观测链路和待清理的历史残留。

## 2. 启动流程

系统启动路径如下：

```text
main()
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO_Init()
  -> MX_DMA_Init()
  -> MX_CAN1_Init()
  -> MX_CAN2_Init()
  -> MX_SPI1_Init()
  -> MX_USART1_UART_Init()
  -> MX_USART6_UART_Init()
  -> MX_TIM1_Init()
  -> MX_USART3_UART_Init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
  -> StartDefaultTask()
       -> MX_USB_DEVICE_Init()
       -> Task_Init()
            -> Gimbal_Init()
       -> Task_loop()
            -> 创建 4 个业务任务
       -> defaultTask 自删除
```

这里的分层比较清晰：

- `Core/Src/main.c` 只负责板级初始化和拉起 RTOS。
- `Core/Src/freertos.c` 里的 `defaultTask` 只负责“启动业务系统”。
- 真正的应用入口在 `user_file/4_Task/MyTask.c`。

## 3. 运行时任务结构

当前业务任务由 `Task_loop()` 创建，共 4 个：

| 任务 | 周期/触发方式 | 作用 |
| --- | --- | --- |
| `Gimbal_Motor_Control_ALL_Test` | `1 ms` 周期 | 读取电机反馈；等待 Yaw 编码器就绪；更新 Sentry 目标；按状态切换控制参数；执行双轴级联 PID；通过 CAN2 下发 GM6020 控制量 |
| `Gimbal_Euler` | `1 ms` 周期 | 读取 BMI088 陀螺仪/加速度计/温度；使用 DWT 估计 dt；执行 Mahony 姿态解算；并行更新 `YawFusion` 观测量 |
| `Gimbal_Manifold_Control` | `10 ms` 周期 | 通过 USB CDC 将当前 `Gimbal_Euler_Angle_to_send` 回传给视觉或上位机 |
| `Gimbal_Task` | `1 ms` 周期 | 每 `100 ms` 执行一次 CAN / SPI / USB 存活判断，并在在线状态切换时执行复位或清空动作 |

几个关键点：

- `defaultTask` 不承担业务逻辑，建完业务任务后会 `vTaskDelete(NULL)`。
- IMU 数据就绪中断接口已经预留，但宏 `GIMBAL_IMU_DRDY_ENABLE` 当前为 `0`，所以现在走的是 `1 kHz` 轮询模式。
- Yaw 电机反馈未 ready 前，控制任务不会进入正常闭环，而是持续下发零输出，避免编码器未初始化时误动作。

## 4. 目录结构

工程目录建议按“CubeMX 生成代码”和“手写业务代码”来理解：

```text
yuntai/
|-- Core/                      CubeMX 生成的主程序、外设初始化、FreeRTOS 入口
|-- USB_DEVICE/                CubeMX 生成的 USB CDC 设备层
|-- Drivers/                   STM32 HAL / CMSIS 官方驱动
|-- Middlewares/               FreeRTOS 与 USB Device 中间件
|-- user_file/                 手写应用代码
|   |-- 1_middlewares/
|   |   |-- Driver/            对 HAL 的二次封装：CAN / SPI / USB / USART
|   |   `-- Algorthm/          PID / Quaternion / DWT / YawFusion / Matrix
|   |-- 2_Device/              设备抽象：BMI088 / Motor / Motor_Protect / Manifold / DR16 / VOFA
|   |-- 3_Module/
|   |   |-- Gimbal/            云台模块主逻辑
|   |   `-- Sentry/            扫描、跟踪、回扫目标生成与模式控制
|   `-- 4_Task/                任务创建与系统级任务入口
|-- cmake/                     工具链与 CubeMX CMake 拼装脚本
|-- CMakeLists.txt             顶层构建入口
|-- CMakePresets.json          Debug / Release 预设
|-- yuntai.ioc                 STM32CubeMX 工程文件
|-- startup_stm32f405xx.s      启动文件
`-- STM32F405XX_FLASH.ld       链接脚本
```

### 4.1 `Core/` 和 `USB_DEVICE/`

这两部分主要是 CubeMX 生成代码：

- `Core/Src/main.c`：主入口。
- `Core/Src/freertos.c`：RTOS 初始化和默认任务。
- `Core/Src/can.c` / `spi.c` / `usart.c` / `gpio.c`：外设初始化。
- `USB_DEVICE/App/usbd_cdc_if.c`：USB CDC 和自定义 `drv_usb` 的衔接点。

这部分原则上应尽量保持“可再生成”，自定义逻辑尽量放在 `USER CODE` 区域或 `user_file/` 中。

### 4.2 `user_file/1_middlewares`

这是工程里最重要的可复用基础设施层。

#### Driver

- `Driver/CAN/`：CAN 双缓冲接收、标准 ID 提取、发送封装、链路存活检测。
- `Driver/SPI/`：SPI1 DMA 收发、BMI088 片选管理、事务同步、链路存活检测。
- `Driver/USB/`：USB CDC 双缓冲接收、发送忙状态管理、最小发送间隔、链路存活检测。
- `Driver/USART/`：基于 `ReceiveToIdle` 的串口双缓冲收发封装，当前主线未接入。

#### Algorthm

- `PID/`：通用 PID，支持输出限幅、积分限幅、目标限幅、死区、输出低通、斜率限制、摩擦补偿等。
- `Quaternion/`：Mahony 姿态更新和四元数转欧拉角。
- `DWT/`：优先用 DWT 周期计数器计算 dt，失败再回退到 HAL Tick。
- `YawFusion/`：融合 Yaw 编码器、IMU 陀螺仪和温度的并行观测链路，当前用于调试比对。
- `Matrix/`：目前基本是占位模块，不在主链路中承担核心功能。

### 4.3 `user_file/2_Device`

这一层是“具体硬件/协议”的设备抽象。

- `BMI088/`：BMI088 初始化、陀螺仪/加速度计/温度读取、姿态融合入口。
- `Motor/`：DJI 电机反馈解析、控制帧映射、级联 PID 调用。
- `Motor_Protect/`：堵转检测 / 回退 / 冷却 / 故障状态机，当前未接入主控制流程。
- `Manifold/`：USB 视觉协议收发，当前协议为 `[Header][Yaw][Pitch][Target_Valid][Tail]`。
- `DR16/`：遥控器数据解析与状态边沿判断，当前未接入云台主流程。
- `VOFA/`：VOFA 数据发送接口，当前未接入主流程。

### 4.4 `user_file/3_Module`

这一层承载真正的业务逻辑。

#### `Gimbal/`

- 负责模块级初始化。
- 维护 IMU 输出、控制目标、电机对象和链路状态切换处理。
- 统一组织 BMI088、Motor、Manifold、PID、YawFusion 等子模块。

#### `Sentry/`

- 维护 `SCAN / TRACK_ARMOR / LOST_TARGET_RETURN_SCAN` 三态状态机。
- 根据视觉目标有效位、超时和滤波结果生成当前 pitch/yaw 目标角。
- 根据状态切换控制整形参数，并在目标突变时重置角度环 PID 动态状态。

### 4.5 `user_file/4_Task`

这里只有一层很薄的任务调度封装：

- `Task_Init()`：调用 `Gimbal_Init()`。
- `Task_loop()`：创建 4 个业务任务。

这层更像“应用启动器”，不承担复杂业务逻辑。

## 5. 关键数据流

### 5.1 姿态链路

`Gimbal_Euler()` 每 `1 ms` 执行：

1. 通过 `BMI088_ReadGyro()` / `BMI088_ReadAccel()` / `BMI088_ReadTemp()` 采样 IMU。
2. 通过 `ALG_DWT_Timebase_GetDtS()` 计算本周期 dt。
3. 通过 `BMI088_Complementary_Filter()` 执行 Mahony 姿态解算，得到 `euler_raw`。
4. 将 `euler_raw` 写入 `Gimbal_Euler_Angle_to_send`，供控制任务和 USB 回传任务使用。
5. 并行读取 Yaw 电机编码器计数，调用 `YawFusion_UpdateEncoderRaw()` / `YawFusion_UpdateImu()` / `YawFusion_UpdateTemperature()`。
6. 编码器首次 ready 或掉线恢复后，执行 `YawFusion_AlignToEncoder()`。
7. 运行 `YawFusion_Run()`，输出到 `Gimbal_Yaw_Fusion_Observe`。
8. 生成 `Gimbal_Euler_Angle_Final_Observe` 作为观测量：`roll/pitch` 取 Mahony 输出，`yaw` 优先取融合结果。

需要注意：

- 当前真正进入控制环和 USB 回传的是 `Gimbal_Euler_Angle_to_send`。
- `Gimbal_Euler_Angle_Final_Observe` 和 `Gimbal_Yaw_Fusion_Observe` 主要用于观测，不直接接管控制环。

### 5.2 控制链路

`Gimbal_Motor_Control_ALL_Test()` 每 `1 ms` 执行：

1. 从 CAN 缓冲提取 Pitch/Yaw 电机反馈。
2. 若 Yaw 编码器尚未初始化，则持续输出 `0`，不进入正常闭环。
3. 调用 `Gimbal_Sentry_Target_Update()` 更新当前目标角。
4. 读取 `Gimbal_Sentry_Target_Get_Pitch()` / `Get_Yaw()` / `Get_State()`。
5. 按当前状态调用 `Gimbal_Sentry_Control_Apply_Mode_Params()` 切换控制整形参数。
6. 若处于目标突变或状态切换场景，则调用 `Gimbal_Sentry_Control_Reset_Angle_PID_Dynamic_State()` 复位角度环动态状态。
7. 使用双轴级联 PID 计算输出：
   - Pitch 角度反馈：`-Gimbal_Euler_Angle_to_send.pitch`
   - Yaw 角度反馈：`Gimbal_Euler_Angle_to_send.yaw`
8. 经过限幅后，通过 `CAN2` 发给两路 `GM6020`。

当前目标的来源不是单一模式，而是分三种状态：

- `SCAN`：无目标时按内置扫描轨迹摆动搜索。
- `TRACK_ARMOR`：视觉目标有效时直接跟踪目标角。
- `LOST_TARGET_RETURN_SCAN`：丢目标后先平滑回到扫描轨迹，再重新接回扫描态。

### 5.3 USB 视觉链路

当前 USB 协议在 `dvc_manifold.c` 中实现：

- 接收帧格式：`[Header][Yaw(float)][Pitch(float)][Target_Valid(uint8_t)][Tail]`
- 发送帧格式：`[Header][Yaw(float)][Pitch(float)][Tail]`

接收端支持：

- 一个回调中拼接多帧
- 单帧跨多个回调分片接收
- 帧头重同步
- Pitch/Yaw 数值合法性检查
- `Target_Valid` 目标有效位
- 调试用的原始字节和完整帧缓存

当前视觉目标的接入关系是：

- `Manifold_USB_Rx_Callback()` 负责把协议帧解析到 `Rx_Data.Taget_Pitch / Taget_Yaw / Target_Valid`
- `Gimbal_Sentry_Target_Update()` 负责消费这些目标，并做滤波、超时与状态切换
- 控制任务最终读取的是 Sentry 模块整理后的目标角，而不是直接读取 `Rx_Data`

## 6. 当前硬件映射

按现有代码，大致可以整理为：

- MCU：`STM32F405xx`
- RTOS：`FreeRTOS`
- IMU：`BMI088`，走 `SPI1 + DMA`
- 云台电机：两路 `GM6020`，挂在 `CAN2`
  - Pitch 电机 ID：`4`
  - Yaw 电机 ID：`2`
- USB：`USB FS CDC`，用于视觉/上位机通信
- USART3：预留给 `DR16`
- USART1 / USART6：通用串口接口，当前主链路未使用
- EXTI：已配置 `ACCEL_INT` / `GYRO_INT` 中断脚，但默认未启用 IMU 中断驱动模式

需要注意：

- `CAN1` 已初始化，但当前应用层主逻辑实际使用的是 `CAN2`。
- `BMI088_Init()` 若失败，会直接从 `Gimbal_Init()` 返回；而 `Task_loop()` 仍会继续创建任务，所以联调前要优先保证 IMU 上电正常。

## 7. 构建与烧录

### 7.1 构建依赖

建议准备以下环境：

- `CMake >= 3.22`
- `Ninja`
- `arm-none-eabi-gcc`
- `STM32CubeMX` 或 `STM32CubeIDE`，用于维护 `.ioc`

工程使用 `CMakePresets.json` 管理构建预设，默认是 `Ninja + gcc-arm-none-eabi`。

注意当前 `CMakePresets.json` 中写死了 Ninja 路径：

```json
"CMAKE_MAKE_PROGRAM": "C:/ST/STM32CubeCLT_1.21.0/Ninja/bin/ninja.exe"
```

如果你的本机安装路径不同，需要先改这里。

### 7.2 命令行构建

在工程根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成产物默认在：

```text
build/Debug/yuntai.elf
build/Debug/yuntai.map
```

### 7.3 烧录

可以使用以下任一方式：

- STM32CubeIDE 直接下载
- STM32CubeProgrammer / `STM32_Programmer_CLI`
- 其他 ST-Link / J-Link 工具链

命令行示例：

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/yuntai.elf -v -rst
```

## 8. 二次开发建议

如果你接下来要继续往下做，最常改的地方基本是下面这些：

### 想调控制参数

优先看：

- `user_file/3_Module/Gimbal/Gimbal.c`
- `user_file/3_Module/Sentry/gimbal_sentry_control.*`
- `user_file/1_middlewares/Algorthm/PID/alg_pid.*`

包括：

- Pitch/Yaw 外环、内环 PID
- 电机输出限幅
- 扫描态和跟踪态的整形参数
- 目标突变触发 PID 重置的阈值
- IMU dt 保护范围

### 想改扫描/跟踪/回扫逻辑

优先看：

- `user_file/3_Module/Sentry/gimbal_sentry.*`
- `user_file/3_Module/Sentry/gimbal_sentry_target.*`
- `user_file/3_Module/Gimbal/Gimbal.c`

适合在这里改：

- 扫描幅值和频率
- 丢目标后的回扫速度
- 视觉目标保持超时
- 视觉目标滤波时间常数
- 跟踪态与扫描态的切换策略

### 想让 YawFusion 真正接管控制或回传

优先看：

- `user_file/1_middlewares/Algorthm/YawFusion/*`
- `user_file/3_Module/Gimbal/Gimbal.c`

当前代码已经把 `YawFusion` 的输入和观测量接好了；如果你要进一步落地，关键就是决定：

1. 是只替换控制用的 yaw 反馈。
2. 还是连 USB 回传一起切到融合结果。
3. 以及切换时如何避免初始对齐跳变。

### 想接回遥控器控制

优先看：

- `user_file/2_Device/DR16/dvc_dr16.*`
- `user_file/1_middlewares/Driver/USART/drv_usart.*`
- `Core/Src/usart.c`

当前 DR16 解析器已经完成，但还没在 `Gimbal_Init()` 和任务层接起来。

### 想继续强化底层稳定性

优先看：

- `user_file/1_middlewares/Driver/CAN/drv_can.*`
- `user_file/1_middlewares/Driver/SPI/drv_spi.*`
- `user_file/1_middlewares/Driver/USB/drv_usb.*`
- `user_file/2_Device/Motor/dvc_motor_protect.*`

这几层很适合继续补：

- 错误码统计
- 超时恢复策略
- 堵转保护接入控制链
- 更细粒度的任务通知/队列机制

## 9. 当前版本的真实边界

为了避免后面继续被旧文档误导，这里把当前边界单独列出来：

- 当前主控制任务已经是“扫描 / 视觉跟踪 / 丢目标回扫”闭环，不再是正弦测试目标版本。
- `YawFusion` 已经运行，但目前仍是观测链路，不直接替换控制反馈和 USB 回传结果。
- `DR16`、`VOFA`、`drv_usart`、`Motor_Protect` 等模块存在，但尚未进入运行主线。
- `Matrix` 当前基本是占位模块，不承担主链路计算。
- IMU 中断触发模式代码已预留，但默认关闭。
- `CAN1` 和部分串口已初始化，但当前业务主线主要使用 `CAN2 + SPI1 + USB FS CDC`。
- `Core/` 和 `USB_DEVICE/` 仍带有 CubeMX 再生成属性，改动时要注意保留用户代码边界。

## 10. 推荐阅读顺序

如果是重新接手这个工程，建议按这个顺序读：

1. `Core/Src/main.c`
2. `Core/Src/freertos.c`
3. `user_file/4_Task/MyTask.c`
4. `user_file/3_Module/Gimbal/Gimbal.c`
5. `user_file/3_Module/Sentry/gimbal_sentry_target.*`
6. `user_file/3_Module/Sentry/gimbal_sentry_control.*`
7. `user_file/2_Device/Manifold/*`
8. `user_file/2_Device/BMI088/*`
9. `user_file/2_Device/Motor/*`
10. `user_file/1_middlewares/Driver/*`
11. `user_file/1_middlewares/Algorthm/PID/*`
12. `user_file/1_middlewares/Algorthm/YawFusion/*`

这样读下来，基本可以从“系统怎么起来”一路看到“视觉目标怎么变成电机输出”，同时也能看清哪些模块已经接入主线，哪些还只是预留能力。
