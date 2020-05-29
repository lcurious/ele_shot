#ifndef _DEBUG_H
#define _DEBUG_H

#include "sys.h"

extern TaskHandle_t DEGUGTask_Handler;
extern uint8_t 	 buf_send_float[29];
extern float trasonic_distance; 
#define DEBUG_TASK_PRIO 1
#define DEBUG_STK_SIZE 200
#define BUFF_SENT_USART_SIZE 29

void debug_task(void * pvParameters);
void MY_IDLE_Handler(void);
void MY_RXNE_Handler(void);
void MY_IDLE_Show_Handler(void);

#endif
