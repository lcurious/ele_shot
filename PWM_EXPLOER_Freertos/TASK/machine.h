#ifndef _MACHINE_H
#define _MACHINE_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t MACHINETask_Handler;

#define MACHINE_TASK_PRIO 4
#define MACHINE_STK_SIZE 500

#define GIMBAL_NUM 2
#define GIMBAL0 0
#define GIMBAL1 1

extern float present_servo_angle;
extern float regulator_return_err;
extern float present_image_angle;
extern float regulator_turn_angle;
extern float set_angle_gimbal0;
extern float set_angle_gimbal1;
extern uint8_t pro4_no_use_tra;
extern float   pro4_input_angle;
extern uint8_t p5_shot_allow;
extern float pro5_init_angle;
extern float   shot_area_limit;
extern uint8_t p6_shot_allow;
extern int32_t pro6_time_cnt;
extern uint8_t pro6_direction;
extern float   pro6_init_angle;
extern uint8_t search_falg_pro6;

void machine_task(void * pvParameters);
void set_gimbal_angle(float set_angle, uint8_t gimbal_type);
void clear(void);
void set_map_angle(float input_angle);

#endif
