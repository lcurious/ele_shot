#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"
#include "gpio.h"
#include "oled.h"
#include "led_task.h"
#include "machine.h"
#include "task_debug.h"
#include "PID.h"
#include "encoder.h"
#include "dma_user.h"
#include "trasonic.h"
#include "pixy.h"
#include "myADC.h"
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);//初始化串口波特率为115200
	USART2_Init(115200);
	
//	USART3_Init(115200); MPU
	USAR4_Init(115200);

 	MYDMA_Config(DMA2_Stream7, DMA_Channel_4, (u32)&USART1->DR, (u32)buf_send_float, BUFF_SENT_USART_SIZE);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.	  
	LED_Init();
	TIM4_PWM_Init(2000 -1 , 840 - 1); 
    ADCx_Init();
    TIM9_CH1_Input_Init(0xffff,168);
	TIM3_Int_Init(9999,7199);//10Khz的计数频率，计数到5000为1s  

    control_gpio_init();
	OLED_Init();
	OLED_Clear();  
    pid_parameter_init();
	
	xTaskCreate((TaskFunction_t)led0_task,
	            (const char*   )"led0_task",
				(uint16_t      )LED0_STK_SIZE,
				(void *        )NULL,
				(UBaseType_t   )LED0_TASK_PRIO,
				(TaskHandle_t *)&LED0Task_Handler);
				
				
    xTaskCreate((TaskFunction_t)machine_task,
	            (const char*   )"machine_task",
				(uint16_t      )MACHINE_STK_SIZE,
				(void *        )NULL,
				(UBaseType_t   )MACHINE_TASK_PRIO,
				(TaskHandle_t *)&MACHINETask_Handler);
				
    xTaskCreate((TaskFunction_t)debug_task,
	            (const char*   )"debug_task",
				(uint16_t      )DEBUG_STK_SIZE,
				(void *        )NULL,
				(UBaseType_t   )DEBUG_TASK_PRIO,
				(TaskHandle_t *)&DEGUGTask_Handler);				
				
	vTaskStartScheduler();						
}
