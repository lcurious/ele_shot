#ifndef _LED_TASK_
#define _LED_TASK_

#include "sys.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t LED0Task_Handler;


#define LED0_TASK_PRIO 2
#define LED0_STK_SIZE 50

void led0_task(void * pvParameters);

void LED_Init(void);

#endif
