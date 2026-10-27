# 四轴飞行器飞控固件（STM32F401RE + µC/OS-II）

> 嵌入式系统综合性训练项目：基于 STM32F4 与实时操作系统 µC/OS-II 的四轴飞行器姿态感知与控制系统固件。

## 项目概览

本工程实现了一个多任务架构的无人机飞控底层固件：由 GY-86 九轴传感器模块采集姿态数据，通过 **Madgwick AHRS** 算法进行四元数姿态解算，同时支持 **PPM 遥控信号输入** 与 **PWM 电调输出**，并通过 USART1 以蓝牙串口向外输出遥测数据流（CSV 格式，可直接导入 Excel / Python 分析）。

| 项目 | 说明 |
|---|---|
| MCU | STM32F401RE（Cortex-M4，84 MHz，HSE 25 MHz） |
| RTOS | µC/OS-II V2.93（SysTick 节拍 1000 Hz） |
| IDE | Keil MDK uVision（工程文件 `ucOS_II.uvprojx`） |
| 外设库 | ST 标准外设库（StdPeriph，`Library/`） |
| 传感器 | GY-86 模块：MPU6050（六轴）+ HMC5883L（磁力计）+ MS5611（气压计） |
| 调试 | SEGGER RTT + SystemView（`SEGGER_*` 目录） |

## 系统架构

### 任务划分（µC/OS-II）

| 任务 | 优先级 | 周期 / 触发 | 职责 |
|---|---|---|---|
| `AppTaskStart` | 3 | 一次性 | 时钟、外设、传感器自检与校准，创建其余任务后自我删除 |
| `AppTaskGY86` | 5 | 5 ms（200 Hz） | 读取 MPU6050 → 单位换算 → Madgwick 姿态解算；每 100 ms 读一次磁力计 |
| `AppTaskMotor` | 4 | PPM 帧完成信号量驱动 | 解析通道油门量 → 限幅 → 输出四路 PWM |
| `AppTaskBlueTooth` | 6 | ~110 ms | 通过 USART1 输出姿态 + 原始数据 + 诊断遥测（CSV） |

任务间共享数据用信号量 / 互斥信号量保护（`Sem_PPM_Frame`、`Sem_SensorData`）。

### 硬件资源分配

| 功能 | 外设 / 引脚 | 配置 |
|---|---|---|
| 电调 PWM × 4 | TIM3 CH1–CH4：PA6 / PA7 / PB0 / PB1 | 50 Hz，脉宽 1000–2000 µs |
| PPM 遥控输入 | TIM2 CH2 输入捕获：PA1 | 1 µs 分辨率（1 MHz 计数） |
| GY-86 传感器 | 软件 I2C：PC0 (SCL) / PC1 (SDA) | MPU6050@0x68、HMC5883L@0x1E、MS5611@0x77 |
| 蓝牙 / 调试串口 | USART1：PA9 (TX) / PA10 (RX) | 9600 bps |
| 微秒级时基 | DWT CYCCNT | 计算姿态解算真实 dt |

## 核心算法：Madgwick AHRS 姿态解算

实现位于 [`Hardware/attitude.c`](Hardware/attitude.c) / [`Hardware/attitude.h`](Hardware/attitude.h)，参考 SOH Madgwick 论文 *"An efficient orientation filter for inertial and inertial/magnetic sensor arrays"*（2010）及官方 `MadgwickAHRS.c`（2012-02-19 版），算法本身未做改动，仅：

1. 用实测 `dt`（基于 DWT 周期计数器）替代硬编码采样频率；
2. 用结构体 `AttitudeState` 替代全局变量；
3. 增加零加速度 / 零梯度的除零保护，避免产生 NaN。

工程上的关键处理：

- **开机流程**：先用加速度计反解初始俯仰 / 滚转（偏航置 0），避免从单位四元数起步需要约 3 s 收敛；再静止采样 600 点做**陀螺零偏校准**，并用标准差门槛（`ATT_GYRO_CALIB_STD_MAX`）拒绝"校准时被人碰了"的不可信结果。
- **clone 芯片量程标定**：MPU6050 仿制片的 `GYRO_CONFIG` 写得进、读得出，但实际量程仍是 ±2000°/s。固件内置"陀螺纯积分 vs 加速度计反解角"的 `ratio` 自检（≈100 正常，≈400 偏小 4 倍），最终按实测采用 `16.4 LSB/(°/s)`。
- **轴系映射**：GY-86 竖直插装在转接板上，传感器系到机体系的映射矩阵在 `attitude.h` 中通过三组实测姿态反解标定（保证行列式 = +1 的右手系）。

## 目录结构

```
.
├── User/            # main.c / app.c：任务创建、调度与业务逻辑
├── Hardware/        # 自写驱动：attitude（Madgwick）、gy86、i2c、pwm、ppm、usart1
├── BSP/             # 板级初始化封装
├── Core/ Port/      # µC/OS-II 内核与 Cortex-M4 移植层
├── Cfg/             # os_cfg.h / app_cfg.h 等配置
├── Start/           # 启动文件、system_stm32f4xx.c、CMSIS
├── Library/         # ST STM32F4xx 标准外设库
├── SEGGER_RTT/      # SEGGER RTT 实时数据通道
├── SEGGER_SystemView/  # SystemView Trace 查看
├── SEGGER_ucos2/    # µC/OS-II 的 SystemView 适配
├── Tests/           # PC 端离线单元测试（不依赖硬件）
└── ucOS_II.uvprojx  # Keil 工程文件
```

## 编译与烧录

1. 安装 Keil MDK 及 STM32F4xx Device Family Pack；
2. 打开 `ucOS_II.uvprojx`，选择 Target 后 `Rebuild`；
3. 通过 J-Link / ST-Link 下载 `Objects/ucOS_II.hex`（或 axf）。

> `Objects/`、`Listings/` 等编译产物已由 `.gitignore` 排除，不入库；clone 后直接重新编译生成即可。

### PC 端单元测试

`Tests/test_attitude.c` 在不带硬件的电脑上验证姿态解算正确性（配合 `Tests/stubs/` 的最小桩头文件）：

```bash
gcc -std=c99 -Wall -Wextra -O2 test_attitude.c ../Hardware/attitude.c -lm -o test_attitude
./test_attitude
```

## 遥测数据格式

上电后经 USART1（蓝牙模块透传）输出 CSV：

```
roll,pitch,yaw,ax,ay,az,gx,gy,gz,gix,giy,giz
```

- 角度单位 0.01°，原始值为传感器 LSB，`gi*` 为陀螺纯积分角（0.1°，用于量程自检）；
- 周期性附带 `[DIAG]` 传感器自检、`[STAT]` 采样率 / dt / 量程 ratio、`[MAG]` 磁力计原始值等诊断行。

## 开发历程（当前阶段）

- [x] µC/OS-II 移植与多任务框架
- [x] 软件 I2C 时序调通，MPU6050 / HMC5883L 驱动 + 上电自检
- [x] Madgwick 姿态解算（含零偏校准、轴映射标定、clone 芯片量程实测修正）
- [x] PPM 接收 + PWM 电机控制（油门直通）
- [x] PC 端离线单元测试 + SystemView 任务级调优
- [ ] 姿态角 PID 闭环 → 四电机差动混控（增稳模式）
- [ ] 九轴融合（引入磁力计 / 气压计）与偏航保持

---

*本项目为课程综合性训练成果，欢迎交流指正。*
