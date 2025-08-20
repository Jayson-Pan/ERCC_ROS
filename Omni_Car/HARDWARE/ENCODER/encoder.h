#ifndef __ENCODER_H
#define __ENCODER_H

#include "config.h"

void Encoder_Init_Tim2(void);
int Read_Encoder_TIM2(void);
void Encoder_Init_Tim4(void);
int Read_Encoder_TIM4(void);
void Encoder_Init_Tim1(void);
int Read_Encoder_TIM1(void);
// void Encoder_Init_Tim3(void);
// int Read_Encoder_TIM3(void);

#endif

