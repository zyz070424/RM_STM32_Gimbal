# yuntai

`yuntai` 是一套基于 `STM32F405xx + STM32CubeMX + FreeRTOS` 的云台控制工程。当前仓库已经稳定收口到“`Core/USB_DEVICE` 保持 C，`user_file` 业务层全面使用 C++ 类对象”的形态，适合作为长期维护版本继续保存和交接。

这份 README 以当前仓库真实代码为准，不按历史调试记录或旧版文档描述。

## 1. 当前版本结论

当前主链路已经是一套完整可运行的云台控制系统，而不是早期的单纯测试工程：

- IMU：`BMI088 + SPI1 + DMA`
- 姿态解算：`Mahony Quaternion`
- 控制链：`Pitch/Yaw 双轴级联 PID`
- 电机：`CAN2 -> GM6020`
- 视觉链路：`USB FS CDC -> Manifold 协议 -> Sentry 目标生成`
- 目标状态机：`SCAN / TRACK_ARMOR / LOST_TARGET_RETURN_SCAN`
- 保护逻辑：`Pitch 轴堵转/限位保护已接入主控制流程`
- 在线检测：`CAN / SPI / USB` 三路链路在线状态检测与状态切换处理

如果你现在重新接手这个工程，可以把它理解成：

- 板级初始化、任务调度、通信驱动和控制主链都已经搭好。
- `user_file/` 已经完成 C++ 化，模块接口语义统一为 `Class_*`。
- 只有少量仍必须保留给 `Core/USB_DEVICE` 的 C 入口还存在。
- 仓库已经比较适合“长期保存”和“低频维护”，而不是继续做大规模架构迁移。

## 2. 当前架构原则

当前仓库已经固定采用下面这套分层原则：

### 2.1 生成代码与业务代码分层

- `Core/`：CubeMX 生成的主程序、外设初始化、中断入口、FreeRTOS 启动代码，保持 C
- `USB_DEVICE/`：CubeMX 生成的 USB CDC 设备层，保持 C
- `user_file/`：手写驱动、算法、设备、模块、任务调度，统一使用 C++

### 2.2 类型命名原则

`user_file/` 里的管理对象统一直接使用真实类名：

- `Class_Motor`
- `Class_PID`
- `Class_BMI088`
- `Class_Manifold`
- `Class_Gimbal`
- `Class_Gimbal_Sentry_Target`
- `Class_Gimbal_Sentry_Control`

不再使用会引发歧义的类别名，例如：

- `Motor_TypeDef`
- `PID_TypeDef`
- `Gimbal_TypeDef`
- `Struct_Manifold`（作为类对象别名时）

也就是说，现在看到 `Class_*`，就明确表示“这是一个有状态、有方法的 C++ 对象”。

当前仓库也不再使用“`Struct_*_Data + Class_*` 继承”的过渡模式：

- 模块、设备、控制器直接定义为单个 `Class_*`
- 协议帧、配置项、原始传感器数据继续保留 `struct`

### 2.3 仍保留的 C 入口

当前仍然保留的 C 入口只有三类：

1. `Core` 仍然直接调用的入口
   - `Task_Init()`
   - `Task_loop()`
   - `Gimbal_IMU_EXTI_Callback()`

2. `USB_DEVICE/App/usbd_cdc_if.c` 仍然直接调用的 USB 桥接入口
   - `USB_Rx_Callback()`
   - `USB_TxCplt_Callback()`
   - 以及其余 `USB_*` 对外函数

3. HAL 官方规定名字的回调
   - `HAL_CAN_RxFifo0MsgPendingCallback()`
   - `HAL_SPI_TxRxCpltCallback()`
   - `HAL_UARTEx_RxEventCallback()`
   - 等等

除了这些入口，`user_file/` 内部已经尽量统一为直接调用对象方法。

## 3. 启动流程

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
            -> Gimbal_Object.Init(nullptr)
       -> Task_loop()
            -> 创建业务任务
       -> defaultTask 自删除
```

其中：

- `Core/Src/main.c` 只负责板级初始化和拉起 RTOS
- `Core/Src/freertos.c` 负责默认任务启动
- `user_file/4_Task/MyTask.cpp` 是应用层真正的启动入口
- `user_file/3_Module/Gimbal/Gimbal.cpp` 承载云台主逻辑

## 4. 运行时任务结构

当前业务任务由 `Task_loop()` 创建，共 4 个：

| 任务名 | 周期/触发方式 | 实际调用对象方法 | 作用 |
| --- | --- | --- | --- |
| `Task_Gimbal_Motor_Control_Test` | `1 ms` 周期 | `Gimbal_Object.MotorControlTask()` | 读取电机反馈、更新目标、执行双轴级联 PID、做 Pitch 保护处理、通过 CAN 下发控制量 |
| `Task_Gimbal_Euler` | `1 ms` 周期 | `Gimbal_Object.EulerTask()` | 读取 BMI088、计算 dt、执行 Mahony 姿态解算 |
| `Task_Gimbal_Task` | `1 ms` 周期 | `Gimbal_Object.TaskLoop()` | 每 `100 ms` 执行一次 CAN / SPI / USB 在线检测，并在状态变化时执行复位或清空动作 |
| `Task_Gimbal_Manifold_Control` | `10 ms` 周期 | `Gimbal_Object.ManifoldControlTask()` | 通过 USB CDC 向视觉/上位机发送当前姿态 |

当前任务层已经不再通过一堆 `Gimbal_*` 兼容函数转发，而是直接接 `Gimbal_Object`。

## 5. 当前主控制链

### 5.1 姿态链路

`Gimbal_Object.EulerTask()` 每 `1 ms` 执行一次：

1. `BMI088_Manage_Object.ReadGyro()`
2. `BMI088_Manage_Object.ReadAccel()`
3. `BMI088_Manage_Object.ReadTemp()`
4. `Imu_Timebase.GetDtS()` 估计当前 dt
5. `BMI088_Manage_Object.ComplementaryFilter()` 执行 Mahony 姿态解算
6. 输出到 `Euler_Angle_To_Send`

这份欧拉角同时提供给：

- 双轴控制反馈
- USB 姿态回传

### 5.2 视觉目标链路

USB CDC 接收目标之后，流程如下：

```text
USB CDC -> drv_usb -> Class_Manifold::UsbRxCallback()
        -> Rx_Data / 帧序号更新
        -> Class_Gimbal_Sentry_Target::Update()
        -> Class_Gimbal_Sentry::Update()
        -> 输出当前 pitch/yaw 目标
```

当前视觉目标不是“裸数据直接进 PID”，而是先经过：

- 帧重同步
- 数值合法性检查
- 目标有效性判断
- 目标缓存与超时保持
- 一阶低通滤波
- `SCAN / TRACK_ARMOR / LOST_TARGET_RETURN_SCAN` 状态机切换

### 5.3 电机控制链路

`Gimbal_Object.MotorControlTask()` 每 `1 ms` 执行一次：

1. 读取 Pitch / Yaw 电机反馈
2. 若 Yaw 编码器未 ready，则持续输出 0，不进入正常闭环
3. 更新 Sentry 目标
4. 读取当前状态下的 pitch / yaw 目标
5. 根据状态下发控制整形参数
6. 目标突变时重置角度环动态状态
7. 执行双轴级联 PID：
   - 外环：角度 -> 目标速度
   - 内环：速度 -> 电机输出
8. 对 Pitch 轴执行堵转/限位保护逻辑
9. 通过 `CAN2` 发给两路 `GM6020`

## 6. 当前主线里真正接入的模块

### 6.1 已接入主线

- `Class_Gimbal`
- `Class_Gimbal_Sentry`
- `Class_Gimbal_Sentry_Target`
- `Class_Gimbal_Sentry_Control`
- `Class_BMI088`
- `Class_Motor`
- `Class_Motor_Protect_Pitch`
- `Class_Manifold`
- `Class_PID`
- `Class_DWT_Timebase`
- `Class_Mahony_Quaternion`
- `Class_CAN_Manage_Object`
- `Class_SPI_Manage_Object`
- `Class_USB_Manage_Object`

### 6.2 存在但未进入主运行主线

- `Class_DR16`
- `Class_UART_Manage_Object`
- `VOFA`
- 部分预留串口能力
- `CAN1` 的业务层使用

也就是说：

- `DR16` 模块已经完成 C++ 化，但当前没有接入云台任务流程
- `USART1 / USART3 / USART6` 已有驱动封装，但当前业务主线不依赖它们

## 7. 当前硬件映射

按现有代码，大致可以整理为：

- MCU：`STM32F405xx`
- RTOS：`FreeRTOS`
- IMU：`BMI088`，走 `SPI1 + DMA`
- 云台电机：两路 `GM6020`，挂在 `CAN2`
  - Pitch 电机 ID：`4`
  - Yaw 电机 ID：`2`
- USB：`USB FS CDC`，用于视觉/上位机通信
- USART3：预留给 `DR16`
- USART1 / USART6：预留通用串口
- EXTI：`ACCEL_INT` / `GYRO_INT` 中断脚已配置，但默认未启用 IMU 中断驱动模式

需要注意：

- `CAN1` 已初始化，但当前主业务逻辑实际使用的是 `CAN2`
- IMU 中断唤醒逻辑已预留，但默认宏配置仍然关闭

## 8. 目录结构

```text
yuntai/
|-- Core/                      CubeMX 生成的主程序、外设初始化、FreeRTOS 启动代码（C）
|-- USB_DEVICE/                CubeMX 生成的 USB CDC 设备层（C）
|-- Drivers/                   STM32 HAL / CMSIS 官方驱动
|-- Middlewares/               FreeRTOS 与 USB Device 中间件
|-- imu_experiment_pack/       已归档的 IMU 实验代码与说明
|-- user_file/                 手写应用代码（C++）
|   |-- 1_middlewares/
|   |   |-- Driver/            CAN / SPI / USB / USART 二次封装
|   |   `-- Algorthm/          PID / DWT / Quaternion / Matrix
|   |-- 2_Device/              BMI088 / Motor / Motor_Protect / DR16 / Manifold / VOFA
|   |-- 3_Module/
|   |   |-- Gimbal/            云台主逻辑
|   |   `-- Sentry/            扫描、跟踪、回扫目标生成与控制参数切换
|   `-- 4_Task/                应用任务入口与任务创建
|-- cmake/                     工具链和 CMake 拼装脚本
|-- CMakeLists.txt             顶层构建入口
|-- CMakePresets.json          Debug / Release 预设
|-- STM32F405XX_FLASH.ld       链接脚本
`-- yuntai.ioc                 CubeMX 工程文件
```

## 9. 当前代码风格约定

这一条很重要，因为仓库已经进入稳定期：

### 9.1 对象命名

- 管理对象统一命名为 `Class_*`
- 全局单例对象统一命名为 `*_Object` 或 `*_Manage_Object`
- 不再用 `xxx_TypeDef` 去指代 C++ 类对象

### 9.2 C/C++ 边界

- `Core/` 和 `USB_DEVICE/` 保持 C
- `user_file/` 统一使用 C++
- 非必要的 C 兼容桥接已经移除
- 保留下来的桥接只服务于 `Core`、`USB_DEVICE` 和 HAL 官方回调

### 9.3 注释规则

- 文件头、类、函数使用完整 Doxygen 块注释
- 函数注释放在 `.cpp` 定义处
- 成员变量使用右侧 `/**< ... */`
- 自定义项目回调使用 `@novel`
- 官方 HAL / USB / FreeRTOS 规定名字的回调不加 `@novel`

## 10. 构建与烧录

### 10.1 构建依赖

建议准备以下环境：

- `CMake >= 3.22`
- `Ninja`
- `arm-none-eabi-gcc`
- `STM32CubeMX` 或 `STM32CubeIDE`

工程使用 `CMakePresets.json` 管理构建预设，默认是 `Ninja + gcc-arm-none-eabi`。

### 10.2 命令行构建

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

### 10.3 烧录

可以使用以下任一方式：

- STM32CubeIDE 直接下载
- STM32CubeProgrammer / `STM32_Programmer_CLI`
- 其他 ST-Link / J-Link 工具链

示例：

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/yuntai.elf -v -rst
```

## 11. 当前版本真实边界

为了避免后面继续被旧文档误导，这里单独列出来：

- 当前主控制已经是“扫描 / 视觉跟踪 / 丢目标回扫”完整链路
- `Pitch` 堵转/限位保护已经接入主控制流程
- `YawFusion / Kalman` 实验代码已归档到 `imu_experiment_pack/`
- `DR16` 和 `USART` 驱动已经完成 C++ 化，但未接入主线
- `Matrix` 目前不承担主链路关键计算
- `Core/` 与 `USB_DEVICE/` 依旧具有 CubeMX 再生成属性

## 12. 推荐阅读顺序

如果以后重新接手这个工程，建议按下面顺序阅读：

1. `Core/Src/main.c`
2. `Core/Src/freertos.c`
3. `user_file/4_Task/MyTask.cpp`
4. `user_file/3_Module/Gimbal/Gimbal.cpp`
5. `user_file/3_Module/Sentry/gimbal_sentry_target.*`
6. `user_file/3_Module/Sentry/gimbal_sentry_control.*`
7. `user_file/2_Device/Manifold/*`
8. `user_file/2_Device/BMI088/*`
9. `user_file/2_Device/Motor/*`
10. `user_file/1_middlewares/Driver/*`
11. `user_file/1_middlewares/Algorthm/PID/*`

这样读下来，可以从“系统怎么起来”一路看到“视觉目标怎么变成电机输出”，同时也能清楚哪些模块已经进入主线，哪些模块只是保留能力。

## 13. 维护建议

如果这个工程之后基本不再做大改，我建议把下面这几点当成默认维护原则：

1. 除非必须，不再重新引入 `TypeDef` 风格的类别名
2. `Core/` 和 `USB_DEVICE/` 里的改动尽量限制在用户代码区
3. 新增功能优先直接写进现有 `Class_*` 对象，不再回退到“数据结构 + 行为分离”的过渡写法
4. 如果将来重新接入 `DR16`、`VOFA` 或其他实验链路，优先在 `user_file/` 内闭环，不要先改生成代码层

当前这份 README 的目标不是介绍“计划中的系统”，而是作为这版仓库的实际交接文档。以后就算你很久不再动这个项目，重新回来时也能靠它快速找回全局。 
