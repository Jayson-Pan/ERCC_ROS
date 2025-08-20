#ifndef __CONTROL_H
#define __CONTROL_H

#include "config.h"

void Control_task(void *pvParameters);

void Drive_Motor(float Vx, float Vy, float Vz);
void SetPWM(int PWMA,int PWMB,int PWMC);
float target_limit_float(float insert, float low, float high);
u32 myabs(long int a);
float float_abs(float insert);
int Incremental_PI_A(float Encoder, float Target);
int Incremental_PI_B(float Encoder, float Target);
int Incremental_PI_C(float Encoder, float Target);
void Get_Velocity_Form_Encoder(void);

#endif
