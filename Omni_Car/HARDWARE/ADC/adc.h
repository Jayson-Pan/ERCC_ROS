#ifndef __ADC_H
#define __ADC_H	

#include "config.h"


void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void) ;
u16 Get_adc_Average(u8 chn, u8 times);
extern float Voltage; 	

#endif 


