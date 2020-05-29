#include "led_task.h"
#include "gpio.h"
#include "shot.h"
#include "usart.h"
#include "machine.h"
TaskHandle_t LED0Task_Handler;

void led0_task(void * pvParameters)
{
	while(1)
	{
		if (pro < 5)
		{
			wait_for_shot();
		}
		else if (pro == 5)
		{
			if (p5_shot_allow == 1)
			{
				p5_shot_allow = 0;
				wait_for_shot();
			}
			
		}
		else if (pro == 6)
		{
			if (p6_shot_allow == 1)
			{
				p6_shot_allow = 0;
				wait_for_shot();
			}
		}
		vTaskDelay(500);		
	}
}