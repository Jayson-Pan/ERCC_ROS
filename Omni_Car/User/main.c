#include "config.h"

//�������ȼ�
#define START_TASK_PRIO         4
//�����ջ��С
#define START_STK_SIZE          256
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

int main(void)
{
    systemInit();

    //������ʼ����

    xTaskCreate((TaskFunction_t)start_task,             //������
                (const char *)"start_task",             //��������
                (uint16_t)START_STK_SIZE,               //�����ջ��С
                (void *)NULL,                           //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,           //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler);    //������

    vTaskStartScheduler();          //�����������

}

//��ʼ����������
void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //�����ٽ���
	
		//��������
    xTaskCreate(Control_task, "Control_task", CONTROL_STK_SIZE, NULL, CONTROL_TASK_PRIO, NULL);   //С����������
	
		xTaskCreate(MPU6050_task, "MPU6050_task", MPU6050_STK_SIZE, NULL, MPU6050_TASK_PRIO, NULL);   //���������ݶ�ȡ����
	
		xTaskCreate(data_task, "DATA_task",       DATA_STK_SIZE, NULL,    DATA_TASK_PRIO, NULL);     //����1 ����/���� ��������
	
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����

    taskEXIT_CRITICAL();            //�˳��ٽ���

}
