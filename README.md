# SGP-Control-System 
### Stewart Platform General Purpose Control System

本项目致力于为 **Stewart 平台（六自由度并联机器人）** 提供高精度、高动态响应的底层控制方案。系统基于 **STM32F407** 核心板开发，利用 **15AS 高电流直流电机驱动器** 实现对 6 根线性推杆的闭环同步控制。

---

## 🛠 硬件架构 (Hardware Stack)

* **主控单元 (MCU)**: STM32F407VGT6。
* **电机驱动 (Motor Driver)**: 15AS 15A 直流电机驱动模块，支持高达 15A 负载电流及 PWM 速度控制。
* **执行器 (Actuators)**: 集成双相霍尔传感器的直流线性推杆，支持高精度位置反馈。
* **应用背景**: 设计用于精密实验平台或大型设备挂装。

---

## 🚀 核心特性 (Key Features)

* **四倍频编码器解码**: 采用外部中断实时处理 A/B 相信号，实现推杆行程的毫米级精确感知。
* **动力学标定表 (LUT)**: 整合了 1300-4150 占空比区间的速度测试数据，通过线性插值算法克服执行器的非线性特性。
* **多环闭环控制**: 拟采用位置环与速度环嵌套的 PID 算法，确保多杆协同运动时的强同步性。
* **现代开发流**: 基于 VS Code + Git + Keil Assistant 的协作开发模式。

---

## 📁 目录结构 (Project Structure)

```text
.
├── Core/               # CubeMX 自动生成的硬件初始化代码
├── Drivers/            # HAL 库及 CMSIS 驱动
├── User/               # 【核心】用户自定义驱动与算法
│   ├── Actuator/       # 推杆控制逻辑、PWM 映射表
│   ├── PID/            # 通用位置/速度 PID 闭环算法
│   └── Kinematics/     # Stewart 平台正逆解运动学算法
├── MDK-ARM/            # Keil uVision 5 工程文件
└── README.md           # 项目说明文档
