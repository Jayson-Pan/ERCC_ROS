#ifndef __USRATX_H
#define __USRATX_H 

#include "config.h"

void data_task(void *pvParameters);
void USART1_SEND(void);
void data_transition(void);
void usart1_send(u8 data);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
void uart1_init(u32 bound);
int USART1_IRQHandler(void);

float XYZ_Target_Speed_transition(u8 High,u8 Low);

#endif

