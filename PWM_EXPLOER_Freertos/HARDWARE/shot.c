#include "shot.h"
#include "delay.h"

uint8_t charge_flag = 0;

void charge(void)
{
    charge_flag = 0;	
	TIM3->CNT = 1;
	con_shot   = 0;	
	delay_us(500);	
	con_charge = 1;
	TIM_Cmd(TIM3, ENABLE);  //Ê¹ÄÜTIMx
	
}

void wait_for_shot(void)
{
	if (charge_flag == 1)
	{
		con_charge = 0;
		vTaskDelay(30);
		con_shot   = 1;
		charge_flag = 0;
	}
}