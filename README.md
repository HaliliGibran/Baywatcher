# Baywatcher

> 基于龙芯 2K0300 嵌入式平台的自主循迹小车视觉与控制系统

[![Platform](https://img.shields.io/badge/Platform-Loongson%202K0300-blue)](https://www.loongson.cn/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B11-orange)](https://en.cppreference.com/w/cpp/11)
[![Build](https://img.shields.io/badge/Build-CMake%203.16%2B-green)](https://cmake.org/)

---

## 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [硬件平台](#硬件平台)
- [系统架构](#系统架构)
- [目录结构](#目录结构)
- [快速开始](#快速开始)
  - [环境依赖](#环境依赖)
  - [编译与部署](#编译与部署)
  - [运行](#运行)
- [视觉处理流程](#视觉处理流程)
- [控制系统](#控制系统)
- [特殊元素处理](#特殊元素处理)
- [目标识别](#目标识别)
- [实时调参（VOFA）](#实时调参vofa)
- [数据流总览](#数据流总览)
- [开发说明](#开发说明)

---

## 项目简介

**Baywatcher** 是一款面向竞赛场景的自主循迹小车系统，运行于**龙芯 2K0300（LoongArch64）**嵌入式平台。系统融合了传统控制理论（多级 PID 级联）与现代机器学习（ONNX 目标识别），能够实时完成赛道巡线、特殊元素（环岛、十字路口、斑马线）处理，以及基于深度学习的目标分类与行为决策。

---

## 功能特性

| 功能模块 | 描述 |
|----------|------|
| 🚗 **赛道循迹** | 基于摄像头的实时赛道中线提取与跟随 |
| 🔵 **环岛处理** | 六状态有限状态机，自动完成环岛进入、绕行与退出 |
| ➕ **十字路口处理** | 检测双侧角点，平稳通过交叉路口 |
| 🦓 **斑马线检测** | 视觉条纹检测，触发减速/停车动作 |
| 🎯 **目标识别** | ONNX 模型推理，识别 `supply`（补给）、`weapon`（障碍）、`vehicle`（车辆） |
| 🏎️ **级联 PID 控制** | 三级 PID（角度→角速度→转速→PWM）精准控制差速转向 |
| 📡 **实时通信** | UDP/TCP/UART 协议，支持 VOFA 上位机实时调参 |
| 📺 **视频推流** | 实时将处理后的图像流传至上位机，方便调试 |
| 📝 **数据日志** | 带时间戳的运行数据本地存储 |

---

## 硬件平台

| 类别 | 型号 / 说明 |
|------|------------|
| **主控** | 龙芯 2K0300（LoongArch64，双核，1GHz） |
| **操作系统** | Linux（带定制 HAL 驱动） |
| **摄像头** | WW_CAMERA 模块（支持 640×480 / 160×120） |
| **IMU** | MPU6050 / ICM42605 / LSM6DSR（I²C） |
| **测距传感器** | VL53L0X ToF（I²C，可选） |
| **编码器** | 正交编码器（GPIO） |
| **电机** | 直流有刷电机 + H 桥 PWM 驱动 |
| **显示屏** | TFT18 / IPS20（SPI） |
| **模型文件** | `cls.onnx`（1.9 MB，3分类 64×64 ONNX） |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                        Baywatcher                           │
│                                                             │
│  ┌──────────┐   ┌──────────────────────────────────────┐   │
│  │  摄像头   │──▶│           视觉处理流水线              │   │
│  └──────────┘   │  - 图像预处理（二值化/形态学）         │   │
│                 │  - 边线搜索 + 透视变换                 │   │
│  ┌──────────┐   │  - 曲率计算 + 元素检测状态机           │   │
│  │  IMU     │──▶│  - ONNX 目标识别链                    │   │
│  └──────────┘   │  - Pure Pursuit 角度计算               │   │
│                 └───────────────┬──────────────────────┘   │
│  ┌──────────┐                   │ pure_angle（横向偏差）    │
│  │ 编码器   │──▶┌───────────────▼──────────────────────┐   │
│  └──────────┘   │         三级级联 PID 控制              │   │
│                 │  Angle PID → YawRate PID → Speed PID  │   │
│                 └───────────────┬──────────────────────┘   │
│                                 │ PWM 输出                 │
│                          ┌──────▼──────┐                   │
│                          │   电机驱动   │                   │
│                          └─────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

**线程模型**

| 线程 | 周期 | 职责 |
|------|------|------|
| `pid_thread` | 5 ms | PID 计算 + 电机 PWM 输出 |
| `sensor_thread` | 20 ms | 读取 IMU / ToF / 按键 |
| `vofa_thread` | 20 ms | VOFA 协议收发（上位机调参同步） |
| 主线程 | — | 视觉流水线（`Vision_System_Run()`） |

---

## 目录结构

```
Baywatcher/
├── CMakeLists.txt              # CMake 构建配置（龙芯交叉编译）
├── build.sh                    # 一键编译 + SCP 部署脚本
├── README.md                   # 本文档
├── Code/
│   ├── main.cpp                # 主入口：线程创建、内核模块加载、参数解析
│   ├── main.hpp                # 全局对象声明与头文件汇总
│   ├── Typedef.hpp             # 项目公共类型定义
│   └── User/
│       ├── inc/                # 功能模块头文件
│       │   ├── PID.h           # 三级级联 PID 结构体与接口
│       │   ├── Motor.h         # 电机 PWM / 方向控制
│       │   ├── Encoder.h       # 编码器测速
│       │   ├── IMU.h           # 陀螺仪 / 加速度计接口
│       │   ├── ESC.h           # 电调接口
│       │   ├── ADC.h           # ADC 采样
│       │   ├── TOF.h           # ToF 测距
│       │   ├── VOFA.h          # VOFA 上位机通信协议
│       │   ├── Logger.h        # 数据日志
│       │   ├── StateMachine.h  # 通用状态机
│       │   ├── TimerThread.h   # 定时线程封装
│       │   ├── Fuzzy.h         # 模糊控制
│       │   ├── Menu.h          # TFT 菜单 UI
│       │   ├── Key.h           # 按键输入
│       │   └── TargetHandler.h # 目标行为决策
│       ├── src/
│       │   ├── Fuzzy.cpp       # 模糊控制实现
│       │   └── Menu.cpp        # 菜单实现
│       └── image/              # 视觉处理核心
│           ├── vision_runtime.h/.cc      # 视觉调度器（模式切换）
│           ├── image_process.h/.cc       # 图像处理主算法
│           ├── image_data.h/.cc          # 视觉全局状态数据
│           ├── recognition_chain.h/.cc   # ONNX 推理链
│           ├── stream_chain.h/.cc        # 视频推流
│           ├── image_handle.cc           # 图像工具函数
│           ├── image_midline_process.cc  # 赛道中线融合
│           ├── image_math.h              # 数学工具
│           ├── transform_table.h         # 透视变换查找表
│           ├── README_IMAGE_PIPELINE.md  # 视觉流水线详细文档
│           └── element/
│               ├── circle.h/.cc          # 环岛检测与状态机
│               ├── crossing.h/.cc        # 十字路口检测与状态机
│               ├── zebra.h/.cc           # 斑马线检测
│               └── element.h/.cc         # 元素状态机顶层调度
├── LQ_Libraries_new/           # 新版模块化硬件库（I2C、显示、陀螺仪）
├── LQ_Libraries_old/           # 旧版库（已弃用 NCNN 链路）
├── cross_lib/
│   ├── opencv/                 # 预编译 OpenCV 4（LoongArch64）
│   └── ncnn/                   # 预编译 NCNN（已禁用）
└── model/
    ├── cls.onnx                # 目标分类模型（3 类，64×64 输入）
    └── class_names.json        # 类别名称：["supply","vehicle","weapon"]
```

---

## 快速开始

### 环境依赖

**编译主机**

| 依赖 | 版本 |
|------|------|
| CMake | ≥ 3.16 |
| 交叉编译器 | `loongarch64-linux-gnu-gcc` v8.3 |
| OpenCV（交叉编译版） | 已随项目打包于 `cross_lib/opencv/` |

**目标板（龙芯 2K0300）**

- Linux 内核含以下驱动模块：
  - `lq_i2c_all_dev`（I²C 总线）
  - `lq_i2c_mpu6050_drv`（IMU）
  - `TFT18_dev` / `TFT18_dri`（TFT 显示屏，可选）
- 网络可达（用于 SCP 部署与 VOFA 通信）

### 编译与部署

```bash
# 克隆仓库
git clone https://github.com/HaliliGibran/Baywatcher.git
cd Baywatcher

# 一键编译并通过 SCP 部署到目标板
./build.sh
# 等价于：
#   cmake -B output
#   cd output && make -j$(nproc)
#   scp output/main root@172.20.10.9:/home/root/workspace/
#   scp model/cls.onnx model/class_names.json root@172.20.10.9:/home/root/workspace/model/
```

> **注意**：目标板 IP 默认为 `172.20.10.9`，如需修改请编辑 `build.sh`。

**仅本地编译（不部署）**

```bash
cmake -B output
cd output
make -j$(nproc)
```

### 运行

在目标板上执行：

```bash
cd /home/root/workspace

# 完整功能（推流 + 目标识别）
./main --stream --recognition

# 仅循迹，不推流
./main --no-stream --no-recognition

# 仅开启视频推流
./main --stream --no-recognition
```

**命令行参数**

| 参数 | 说明 |
|------|------|
| `--stream` / `--no-stream` | 开启 / 关闭视频推流 |
| `--recognition` / `--no-recognition` | 开启 / 关闭 ONNX 目标识别 |

**运行时控制**

| 按键 | 功能 |
|------|------|
| `c` | 手动重置所有状态（元素状态机 / 识别结果 / 跟随模式） |

---

## 视觉处理流程

视觉流水线运行于主线程，整体分为 **NORMAL**（循迹）和 **RECOGNITION**（识别）两种模式，由检测到红色 ROI 触发模式切换。

```
摄像头采集（640×480 BGR）
       │
       ▼
  resize → 灰度 → OTSU 二值化 → 形态学处理
       │
       ▼  (160×120 binary)
  image_processing()
  ├── 1. 斑马线检测（条纹模式识别 → 触发停车请求）
  ├── 2. 左右边线搜索（从图像核心点出发）
  ├── 3. 透视变换（鸟瞰图映射）
  ├── 4. 点列滤波 + 等间距重采样
  ├── 5. 曲率计算（角点检测）
  ├── 6. 单侧中线生成
  ├── 7. 元素检测（双侧角点→十字 / 单侧角点→环岛）
  ├── 8. 元素状态机更新
  ├── 9. 中线融合（MIXED / MIDLEFT / MIDRIGHT）
  └── 10. Pure Pursuit 角度计算（lateral error → pure_angle）
```

详细文档请参阅 [`Code/User/image/README_IMAGE_PIPELINE.md`](Code/User/image/README_IMAGE_PIPELINE.md)。

---

## 控制系统

采用**三级 PID 级联**实现精准差速转向：

```
pure_angle（视觉横向偏差）
       │
       ▼
  [Angle PID]  → 目标角速度 (yaw_rate_target)
       │
       ▼
  [YawRate PID] → 左右轮差速量
       │
       ▼
  [Speed PID L/R] → 左右轮 PWM
       │
       ▼
  电机 H 桥（引脚 21/22，PWM 占空比 0~10000）
```

| PID 环路 | 输入 | 输出 | 运行周期 |
|----------|------|------|----------|
| 角度环 | 视觉偏角 | 目标角速度 | 5 ms |
| 角速度环 | 角速度误差 | 差速量 | 5 ms |
| 速度环（左/右） | 转速误差 | PWM 占空比 | 5 ms |

---

## 特殊元素处理

### 环岛（Roundabout）

```
BEGIN → IN → RUNNING → OUT → END → (回到 NORMAL)
```

- 检测条件：单侧角点 + 对侧直线段
- 进入时切换为单侧跟随（MIDLEFT / MIDRIGHT）
- 出环后自动恢复双侧中线（MIXED）

### 十字路口（Crossing）

```
IN → RUNNING → (退出)
```

- 检测条件：左右两侧同时出现角点
- 穿越过程中保持当前方向，忽略岔路干扰

### 斑马线（Zebra Crossing）

- 检测赛道区域内的横向条纹规律
- 触发减速/停车请求，等待通行信号后继续

---

## 目标识别

识别链由红色 ROI 区域触发，对目标图像进行 ONNX 模型推理：

| 类别 | 行为决策 |
|------|----------|
| `weapon`（障碍物） | `FORCE_LEFT`：强制跟随左边线绕行 |
| `supply`（补给点） | `FORCE_RIGHT`：强制跟随右边线靠近 |
| `vehicle`（车辆） | 保持当前 pure_angle 约 1 秒，维持航向 |

**识别参数**

| 参数 | 值 |
|------|----|
| 模型输入尺寸 | 64×64 RGB |
| 投票帧数阈值 | 3 帧连续一致 |
| 最大识别窗口 | 2500 ms |
| 识别冷却时间 | 3000 ms |

---

## 实时调参（VOFA）

系统通过 VOFA 协议与上位机（PC / 手机）进行双向通信：

- **下行**（板 → 上位机）：目标转速、实际转速、PWM 输出等波形数据
- **上行**（上位机 → 板）：Kp / Ki / Kd / 基准速度等参数实时下发

支持协议：**UDP**（主要）、**TCP**、**UART（串口）**

---

## 数据流总览

```
Camera ──▶ 预处理 ──▶ image_processing() ──▶ pure_angle
                                                  │
                                           [Angle PID]
                                                  │
IMU ────▶ current_yaw_speed ──────────▶ [YawRate PID]
                                                  │
Encoder ─▶ current_speed ─────────────▶ [Speed PID]
                                                  │
                                           Motor PWM
                                                  │
                                         ◀── Encoder 反馈
```

---

## 开发说明

- **交叉编译**：项目默认使用 `loongarch64-linux-gnu-` 工具链，如需在 x86 主机上本地调试，可修改 `CMakeLists.txt` 中的 `CMAKE_C_COMPILER` / `CMAKE_CXX_COMPILER`。
- **NCNN**：`cross_lib/ncnn/` 目录保留旧版 NCNN 支持，当前版本已切换至 OpenCV DNN + ONNX，NCNN 链路在 `CMakeLists.txt` 中已禁用。
- **日志**：运行日志由 `Logger.h` 写入本地文件，格式为带时间戳的 CSV，便于离线分析。
- **视觉调试**：启用 `--stream` 后，可在上位机通过 VOFA 或自定义 TCP 客户端实时查看二值化图像与元素检测叠加结果。

---

<p align="center">
  Made with ❤️ for the Loongson Smart Car Competition
</p>