# STM32 全向机器人控制系统

基于STM32F103VC微控制器和FreeRTOS实时操作系统的全向移动机器人底盘控制程序。

## 硬件平台
### 传感器模块
| 传感器 | 型号 | 功能 |
|--------|------|------|
| IMU | MPU6050 | 三轴加速度计和陀螺仪 |
| 编码器 | 增量式编码器 | 电机速度反馈 |
| ADC | 内置ADC | 电池电压监测 |

### 电机配置
- **电机类型**：带编码器的减速电机
- **数量**：3个（120°分布）
- **减速比**：30:1
- **编码器精度**：13线/11线
- **控制方式**：PWM + PID

## 系统架构
### FreeRTOS任务架构

| 任务名称 | 优先级 | 堆栈大小 | 频率 | 功能描述 |
|----------|--------|----------|------|----------|
| `start_task` | 4 | 256 | - | 系统初始化任务 |
| `Control_task` | 4 | 512 | 100Hz | 运动控制和PID调节 |
| `MPU6050_task` | 3 | 256 | 200Hz | IMU数据采集 |
| `data_task` | 4 | 512 | 50Hz | 串口通信 |

### 硬件抽象层

```
应用层 (FreeRTOS任务)
├── Control_task (运动控制)
├── MPU6050_task (传感器)
└── data_task (通信)

硬件抽象层
├── MOTOR (电机驱动)
├── ENCODER (编码器)
├── MPU6050 (IMU)
├── ADC (电压采集)
└── USART (串口)

底层驱动
└── STM32F10x 标准库
```

## 运动学模型
### 全向轮运动学参数
```c
#define Omni_turn_radiaus    0.215    // 旋转半径 (m)
#define Tyre_Diameter        0.065    // 轮胎直径 (m)
#define X_PARAMETER          (sqrt(3)/2.f)  // X方向参数
#define Y_PARAMETER          (0.5f)         // Y方向参数
```

### 运动控制方程
全向轮机器人采用三轮120°分布设计，运动学方程如下：

```
电机A速度 = -Y方向速度 + 旋转速度 * 转弯半径
电机B速度 = X方向速度 * X_PARAMETER + Y方向速度 * Y_PARAMETER + 旋转速度 * 转弯半径
电机C速度 = -X方向速度 * X_PARAMETER + Y方向速度 * Y_PARAMETER + 旋转速度 * 转弯半径
```

## 通信协议
### 串口配置
- **波特率**：115200
- **数据位**：8
- **停止位**：1
- **校验位**：无

### 数据帧格式

#### 发送数据帧（STM32 → ROS）
```
[帧头] [X速度] [Y速度] [Z速度] [电压] [加速度XYZ] [陀螺仪XYZ] [帧尾]
 0x7B   2字节   2字节   2字节   2字节    6字节       6字节      0x7D
总长度：24字节
```

#### 接收数据帧（ROS → STM32）
```
[帧头] [X速度] [Y速度] [Z速度] [帧尾]
 0x7B   4字节   4字节   4字节   0x7D
总长度：17字节
```

## 开发环境
### 工具链
- **IDE**：Keil uVision 5
- **编译器**：ARM Compiler 5.06
- **调试器**：ST-Link V2
- **固件库**：STM32F10x标准外设库

### 项目结构
```
Omni_Car/
├── User/               # 用户应用代码
│   ├── main.c         # 主函数
│   └── stm32f10x_it.c # 中断服务程序
├── CONTROLLER/         # 控制器模块
│   ├── CONFIG/        # 系统配置
│   └── CONTROL/       # 运动控制
├── HARDWARE/          # 硬件驱动
│   ├── MOTOR/         # 电机驱动
│   ├── ENCODER/       # 编码器
│   ├── MPU6050/       # IMU传感器
│   ├── ADC/           # 模数转换
│   └── USART/         # 串口通信
├── FreeRTOS/          # 实时操作系统
├── Library/           # STM32标准库
└── SYSTEM/            # 系统底层
```


## 配置参数
### 关键宏定义
```c
// 控制频率
#define CONTROL_FREQUENCY    100      // 控制频率 100Hz
#define RATE_100_HZ          100      // 数据发送频率

// 机械参数
#define Omni_turn_radiaus    0.215    // 转弯半径 (m)
#define GEAR_RATIO           30       // 减速比
#define Tyre_Diameter        0.065    // 轮胎直径 (m)
#define ENOCDER_LINES1       13       // 编码器A精度
#define ENOCDER_LINES2       11       // 编码器B/C精度

// 通信协议
#define FRAME_HEADER         0x7B     // 帧头
#define FRAME_TAIL           0x7D     // 帧尾
#define SEND_DATA_SIZE       24       // 发送数据大小
```

### PID参数调节
电机速度控制PID参数可在运行时通过串口调节：
```c
typedef struct {
    float Velocity_KP;  // 比例增益
    float Velocity_KI;  // 积分增益
} Motor_parameter;
```

## 功能模块
### 1. 运动控制模块 (CONTROL)
- 接收ROS速度指令
- 运动学解算
- PID速度控制
- PWM输出控制

### 2. 传感器模块 (HARDWARE)
- MPU6050 IMU数据采集
- 三路编码器速度反馈
- 电池电压监测
- 传感器数据滤波

### 3. 通信模块 (USART)
- 串口数据收发
- 数据帧解析
- 错误检测和重传
- 数据格式转换

### 4. 系统管理
- FreeRTOS任务调度
- 硬件初始化
- 错误处理
- 系统监控

