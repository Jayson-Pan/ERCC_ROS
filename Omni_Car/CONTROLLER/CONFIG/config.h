#ifndef __CONFIG_H
#define __CONFIG_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "encoder.h"
#include "ioi2c.h"
#include "robot_init.h"
#include "motor.h"
#include "mpu6050.h"
#include "usartx.h"
#include "control.h"
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

/***1. FreeRTOS相关宏定义***/
//1.1 任务优先级和堆栈大小宏定义
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY	1000
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

//1.2 串口发送宏定义
#define FRAME_HEADER      0X7B //Frame_header //帧头
#define FRAME_TAIL        0X7D //Frame_tail   //帧尾
#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11
//串口发送任务
#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4

//1.3 MPU6050任务宏定义
#define MPU6050_TASK_PRIO		  3 
#define MPU6050_STK_SIZE 		  256

//1.4 Control任务宏定义
#define CONTROL_TASK_PRIO		4     //Task priority //任务优先级
#define CONTROL_STK_SIZE 		512   //Task stack size //任务堆栈大小

//3. 小车参数宏定义
//全向轮小车旋转半径,单位:米
#define   Omni_turn_radiaus		0.215
//电机减速比
#define   GEAR_RATIO		30
//编码器精度
#define	  ENOCDER_LINES1		13
#define	  ENOCDER_LINES2		11
//小车轮胎直径
#define   Tyre_Diameter		0.065
//编码器倍频数
#define		EncoderMultiples		4
//编码器数据读取频率
#define CONTROL_FREQUENCY 100
//圆周率
#define PI 3.1415f  //PI 

//全向轮小车运动学参数
#define X_PARAMETER    (sqrt(3)/2.f)               //X方向参数
#define Y_PARAMETER    (0.5f)                      //Y方向参数


/***2. 结构体定义***/
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****用于存放陀螺仪加速度计三轴数据的结构体*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2个字节
	short Y_data; //2 bytes //2个字节
	short Z_data; //2 bytes //2个字节
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1个字节
		short X_speed;	            //2 bytes //2个字节
		short Y_speed;              //2 bytes //2个字节
		short Z_speed;              //2 bytes //2个字节
		short Power_Voltage;        //2 bytes //2个字节
		Mpu6050_Data Accelerometer; //6 bytes //6个字节
		Mpu6050_Data Gyroscope;     //6 bytes //6个字节	
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Sensor_Str;
}SEND_DATA;
typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		float X_speed;	            //4 bytes //4个字节
		float Y_speed;              //4 bytes //4个字节
		float Z_speed;              //4 bytes //4个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Control_Str;
}RECEIVE_DATA;

//Parameter structure of robot
//机器人参数结构体
typedef struct  
{
  int   GearRatio;         //Motor_gear_ratio //电机减速比
  int   EncoderAccuracy;   //Number_of_encoder_lines //编码器精度(编码器线数)
  float WheelDiameter;     //Diameter of driving wheel //主动轮直径	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //全向轮小车旋转半径
}Robot_Parament_InitTypeDef;


/***全局变量定义***/
extern u8 Flag_Stop;
extern float Move_X,Move_Y,Move_Z;
extern float Velocity_KP,Velocity_KI;	
extern Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C;
extern float Encoder_precision;
extern float Wheel_perimeter; 
extern float Omni_radiaus;
 

void systemInit(void);

#endif
