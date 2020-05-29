#include "machine.h"
#include "pwm.h"
#include "gpio.h"
#include "PID.h"
#include "math.h"
#include "mpu_task.h"
#include "led.h"
#include "encoder.h"
#include "stdlib.h"
#include "usart.h"
#include "myADC.h"
#include "pixy.h"
#include "map.h"
#include "shot.h"
#include "task_debug.h"
#include "oled.h"

TaskHandle_t MACHINETask_Handler;
float set_angle_gimbal0 = 0;
float set_angle_gimbal1 = 0;
float present_servo_angle = 0;
float regulator_return_err = 0;
float present_image_angle = 0;
float regulator_turn_angle = 0;
int32_t pro4_time_cnt = 0;
uint8_t pro4_no_use_tra    = 0;
float   pro4_input_angle = 0;
uint8_t p5_shot_allow = 0;
int32_t pro5_time_cnt = 0;
uint8_t pro5_direction = 0;
int32_t pro5_move_cnt  = 0;
float   pro5_init_angle = 14;
float   shot_area_limit = 6; 

uint8_t search_falg_pro6 = 0;
uint8_t p6_shot_allow = 0;
int32_t pro6_time_cnt = 0;
uint8_t pro6_direction = 0;
float   pro6_init_angle = 21;
int32_t stable_cnt     = 0;


void machine_task(void * pvParameters)
{
	set_gimbal_angle(3, GIMBAL1);
	set_gimbal_angle(0, GIMBAL0);
	
	while(1)
	{
		if (pro == 1) //题目一
		{
			set_gimbal_angle(set_angle_gimbal0, GIMBAL0);
			set_gimbal_angle(set_angle_gimbal1, GIMBAL1);
		}
		else if (pro == 2)//题目二
		{
			OLED_Clear();
			set_gimbal_angle(set_angle_gimbal0, GIMBAL0); //手动调节
			set_gimbal_angle(set_angle_gimbal1, GIMBAL1); //手动调节	

			OLED_ShowNum(15,0, (int32_t)(set_angle_gimbal1), 7, 12);
			OLED_ShowNum(15,2, (int32_t)(set_angle_gimbal0), 7, 12);

		}
		else if (pro == 3)//题目三
		{
			
		}
		else if (pro == 4)//发挥一
		{
			OLED_Clear();
			if (searching_flag == 1)
			{
				pro4_time_cnt++;
				present_servo_angle = get_angle();
				present_image_angle = get_image_ave_angle(2);
				regulator_return_err = pid_regulator(&gimbal_pid[GIMBAL0], 0, -present_image_angle);
				set_gimbal_angle(present_servo_angle + regulator_return_err, GIMBAL0);
				
				if (pro4_no_use_tra == 0)
				{
					set_map_angle(trasonic_distance / 100 - 0.25f);
				}
				else 
				{
					set_gimbal_angle(pro4_input_angle, GIMBAL1); 
				}
				
				OLED_ShowNum(15,0, (int32_t)(pro4_input_angle), 7, 12); 
				if (pro4_time_cnt == 200)
				{
					charge();
				}
			}
			else
			{
				pro4_time_cnt = 0;
				set_gimbal_angle(set_angle_gimbal1, GIMBAL1); //手动调节	

			}
			
		}
		else if (pro == 5)//发挥二
		{
			OLED_Clear();
			present_image_angle = get_image_ave_angle(2);
			
			if (pro5_time_cnt == 0)
			{
				if (present_image_angle > 0)
				{
					pro5_direction = 0;
				}
				else 
				{
					pro5_direction = 1;
				}				
			
			}

			
			if (mov_ave_flag == 1)
			{  
				pro5_time_cnt++;
				
				if ((pro5_time_cnt % 2) == 0)
				{
					if (pro5_direction == 0)
					{
						set_angle_gimbal0--;
						if (set_angle_gimbal0 <= -30)
						{
							pro5_direction = 1;
						}
					}
					else 
					{
						set_angle_gimbal0++;
						if (set_angle_gimbal0 >= 30)
						{
							pro5_direction = 0;
						}
					}
				}
				
				if ((fabs(present_image_angle) < 10) && (charge_flag == 1))
				{
					set_gimbal_angle(pro5_init_angle, GIMBAL1);
				}
				
				if ((fabs(present_image_angle) < shot_area_limit) && (charge_flag == 1))
				{
					p5_shot_allow = 1;
				}
				
				set_gimbal_angle(set_angle_gimbal0, GIMBAL0);

			}
			else 
			{
				pro5_time_cnt = 0;
				set_gimbal_angle(set_angle_gimbal1, GIMBAL1); //手动调节	

			}
			
			OLED_ShowNum(15,0, (int32_t)(pro5_init_angle), 7, 12);
			OLED_ShowNum(15,2, (int32_t)(set_angle_gimbal1), 7, 12);
			OLED_ShowNum(15,4, (int32_t)(shot_area_limit), 7, 12);			
			
			if (pro5_time_cnt == 3)
			{
				charge();
			}
			
		}
		else if (pro == 6)//发挥三
		{

			if (search_falg_pro6 == 1)
			{
				pro6_time_cnt++;
				present_servo_angle = get_angle();
				present_image_angle = get_image_ave_angle(2);
				regulator_return_err = pid_regulator(&gimbal_pid[GIMBAL0], 0, -present_image_angle);
				set_gimbal_angle(present_servo_angle + regulator_return_err, GIMBAL0);
				
				//set_map_angle(trasonic_distance / 100 - 0.3f);
				
				set_gimbal_angle(pro6_init_angle, GIMBAL1); //手动调节	
							
				OLED_ShowNum(15,0, (int32_t)(pro6_init_angle), 7, 12);
				if (fabs(present_image_angle) < 4)
				{
					stable_cnt++;
				}
				else 
				{
					stable_cnt = 0;
				}
				
				if ((stable_cnt > 3) && (charge_flag == 1))
				{
					p6_shot_allow = 1;
				}
			}
			else
			{
				pro6_time_cnt = 0;
				set_gimbal_angle(set_angle_gimbal1, GIMBAL1);
			}
			
			if (pro6_time_cnt == 3)
			{
				charge();
			}
		}
		
	

		vTaskDelay(30);
	
	}
}

void set_gimbal_angle(float set_angle, uint8_t gimbal_type)
{
	if (gimbal_type == GIMBAL0)
	{
		if (set_angle < -30)
		{
			set_angle = -30; 
		}
		else if (set_angle > 30)
		{
			set_angle = 30;
		}
	}
	else if (gimbal_type == GIMBAL1)
	{
		if (set_angle < -45)
		{
			set_angle = -45; 
		}
		else if (set_angle > 45)
		{
			set_angle = 45;
		}
	}

	
	set_angle *= 1.11f;
	
	set_angle += 150.0f;
	
	if (gimbal_type == GIMBAL0)
	{
		TIM_SetCompare1(TIM4, set_angle);	
	}
	else if (gimbal_type == GIMBAL1)
	{
		TIM_SetCompare2(TIM4, set_angle);			
	}
}


void clear(void)
{
	pid_clear(&gimbal_pid[GIMBAL0]);
	pid_clear(&gimbal_pid[GIMBAL1]);
	set_gimbal_angle(0, GIMBAL1);
	set_gimbal_angle(0, GIMBAL0);
    set_angle_gimbal0 = 0;
    set_angle_gimbal1 = 3;
    present_servo_angle = 0;
    regulator_return_err = 0;
    present_image_angle = 0;
    regulator_turn_angle = 0;
}

void set_map_angle(float input_angle)
{
	uint8_t i = 0;
	uint8_t map_remember = 0;
	float present_err = 0;
	float min_err = 10000;
	for (; i < MAP_LENGTH; i++)
	{		 
		present_err = fabs(distance_map[i] - input_angle);
		if (min_err > present_err)
		{
			min_err = present_err;
			map_remember = i;
		}
	}
	
	set_angle_gimbal1 = angle_map[map_remember];
	pro4_input_angle  = 	set_angle_gimbal1;
	set_gimbal_angle(set_angle_gimbal1, GIMBAL1);
	
}

