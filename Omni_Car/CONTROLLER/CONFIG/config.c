#include "config.h"


//机器人软件失能标志位
u8 Flag_Stop = 1;

//小车三轴目标运动速度，单位：m/s
//注意Move_Y不起作用，只有对全向移动小车Move_Y才有意义
float Move_X, Move_Y, Move_Z;

//速度控制PID参数
float Velocity_KP = 300, Velocity_KI = 300; // Adjusted for new PWM range (19999 vs 7200)

//电机的参数结构体
Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C;


//电机(车轮)转1圈对应的编码器数值
float Encoder_precision;
//轮子周长，单位：m
float Wheel_perimeter;
//全向小车旋转半径
float Omni_radiaus;



void systemInit(void)
{
    //Interrupt priority group setting
    //中断优先级分组设置
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //Delay function initialization
    //延时函数初始化
    delay_init();

    //串口1、2初始化，通信波特率115200，可用于与ROS端、arduino端通信
    uart1_init(115200);

    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //ADC引脚初始化
    Adc_Init();

    Robot_Init(Omni_turn_radiaus, GEAR_RATIO, ENOCDER_LINES2, Tyre_Diameter); 

    //Encoder A is initialized to read the real time speed of motor A
    //编码器A初始化，用于读取电机A的实时速度
    Encoder_Init_Tim2();
    //Encoder B is initialized to read the real time speed of motor B
    //编码器B初始化，用于读取电机B的实时速度
    Encoder_Init_Tim4();
    //Encoder C is initialized to read the real time speed of motor C
    //编码器C初始化，用于读取电机C的实时速度
    Encoder_Init_Tim1();

    //Initialize the motor direction control pin,
    //which is used to control the motor forward and reverse rotation
    //初始化电机方向控制引脚，用于控制电机正反转
    Motor_Init();

    //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
    //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
    PWM_Init(7199, 0);
	

    //IIC initialization for MPU6050
    //IIC初始化，用于MPU6050
    IIC_Init();

    //MPU6050 is initialized to read the vehicle's three-axis attitude,
    //three-axis angular velocity and three-axis acceleration information
    //MPU6050初始化，用于读取小车三轴姿态、三轴角速度、三轴加速度信息
    Flag_Mpu6050 = MPU6050_Init();

}
