#include "config.h"

//任务优先级
#define START_TASK_PRIO         4
//任务堆栈大小
#define START_STK_SIZE          256
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

int main(void)
{
    systemInit();

    //创建开始任务

    xTaskCreate((TaskFunction_t)start_task,             //任务函数
                (const char *)"start_task",             //任务名称
                (uint16_t)START_STK_SIZE,               //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,           //任务优先级
                (TaskHandle_t *)&StartTask_Handler);    //任务句柄

    vTaskStartScheduler();          //开启任务调度

}

//开始任务任务函数
void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //进入临界区
	
		//创建任务
    xTaskCreate(Control_task, "Control_task", CONTROL_STK_SIZE, NULL, CONTROL_TASK_PRIO, NULL);   //小车控制任务
	
		xTaskCreate(MPU6050_task, "MPU6050_task", MPU6050_STK_SIZE, NULL, MPU6050_TASK_PRIO, NULL);   //陀螺仪数据读取任务
	
		xTaskCreate(data_task, "DATA_task",       DATA_STK_SIZE, NULL,    DATA_TASK_PRIO, NULL);     //串口1 发送/接收 数据任务
	
    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区

}
