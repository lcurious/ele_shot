#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"   

int32_t get_moto_all_angle(uint8_t machine_type);
void ENC1_TIM3_Init(void);
void ENC2_TIM5_Init(void);
void ENC3_TIM2_Init(void);
void ENC4_TIM12_Init(void);
void ENC4_TIM8_Init(void);//TIM8

#endif
