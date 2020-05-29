#ifndef _MPU_TASK_
#define _MPU_TASK_
#include "sys.h"

extern TaskHandle_t MPUTask_Handler;
extern float mpu_pitch;
extern float mpu_roll;
extern float mpu_yaw; 

#define MPU_TASK_PRIO 4
#define MPU_STK_SIZE 1000

void MPU_task(void * pvParameters);
#endif
