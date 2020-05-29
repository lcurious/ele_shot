#ifndef _MY_ADC
#define _MY_ADC

#include "sys.h"
#define ADC_CENTER_OFFSET 1990
u16 Get_ADC_Value(u8 ch,u8 times);
void ADCx_Init(void);
float get_angle(void);

extern float present_servo_angle;


#endif 
