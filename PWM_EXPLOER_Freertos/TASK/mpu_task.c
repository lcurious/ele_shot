#include "mpu_task.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "led.h"

TaskHandle_t MPUTask_Handler;

float mpu_pitch;
float mpu_roll;
float mpu_yaw; 		//Å·À­½Ç

float pit;

void MPU_task(void * pvParameters)
{
	while(1)
	{
		if(mpu_dmp_get_data(&mpu_pitch, &mpu_roll, &mpu_yaw)==0)
		{
			LED0 = 0;
		}
		else
		{
			LED0 = 1;
		};	
		
		vTaskDelay(5);
	}
		
}
