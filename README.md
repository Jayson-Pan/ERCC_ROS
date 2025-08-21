# ERCC ROS 机器人系统

## 系统组成

本项目由三个主要组件构成：

| 组件 | 描述 | 详细文档 |
|------|------|----------|
| **STM32 底盘控制** | 基于STM32F103VC的全向轮底盘程序，采用FreeRTOS实时系统 | [Omni_Car/README.md](./Omni_Car/README.md) |
| **ROS 系统** | 完整的ROS工作空间，包含导航、SLAM、传感器驱动等功能包 | [car_ws/README.md](./car_ws/ROS教育机器人使用手册/ROS教育机器人使用手册.md) |
| **Arduino 机械臂** | Arduino控制的舵机抓取系统，用于物体抓取操作 | [arm_handler/README.md](./arm_handler/README.md) |

## 快速开始

### 1. 硬件连接
- 连接机器人WiFi热点 
- 通过SSH登录主控制器 (详见[用户手册](./car_ws/ROS教育机器人使用手册/))

### 2. 系统启动
```bash
# 进入ROS工作空间
cd car_ws
source devel/setup.bash

# 启动完整系统
roslaunch robot_bringup bringup.launch
```

### 3. 基本测试
```bash
# 测试机械臂
rostopic pub /grab std_msgs/Int32 "data: 1"

# 测试移动
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## 目录结构

```
ERCC_ROS/
├── Omni_Car/          # STM32全向轮底盘控制固件
├── car_ws/            # ROS工作空间
│   ├── src/           # ROS功能包源码
│   └── ROS教育机器人使用手册/  # 详细使用指南
├── arm_handler/       # Arduino机械臂控制程序
├── CLAUDE.md          # 开发者参考文档
└── README.md          # 本文件
```

## 开发环境

### 所需工具
- **ROS**: Melodic (Ubuntu 18.04) 或 Noetic (Ubuntu 20.04)
- **STM32开发**: Keil uVision 5 + ARM Compiler 5.06
- **Arduino开发**: Arduino IDE 或 PlatformIO
- **SSH工具**: FinalShell, Xshell 或 MobaXterm

### 版本要求
- OpenCV 3.x/4.x
- Python 2.7/3.x
- GCC 7.x+

详细配置请参考各组件的README文件。

## 主要功能

- 🚗 **全向移动**: 三轮全向轮配置，支持任意方向移动
- 🗺️ **SLAM建图**: 基于激光雷达的同时定位与建图
- 🧭 **自主导航**: 路径规划和避障导航
- 📷 **视觉识别**: AprilTag标签识别和跟踪
- 🦾 **物体抓取**: 机械臂自动抓取功能
- 📱 **远程控制**: Web界面和Flutter APP控制
- 🔗 **多机通信**: ROS分布式系统支持

## 使用文档

### 用户文档
- [ROS教育机器人使用手册](./car_ws/ROS教育机器人使用手册/ROS教育机器人使用手册.md) - 完整的用户操作指南

### 开发者文档
- [STM32固件开发](./Omni_Car/README.md) - 底层控制系统开发
- [Arduino程序开发](./arm_handler/README.md) - 机械臂控制开发
- [CLAUDE.md](./CLAUDE.md) - 开发者快速参考

### 技术参考
- 系统架构和通信协议详见各组件README
- 硬件规格和配置参数详见技术文档
- 故障排除和调试方法详见用户手册
