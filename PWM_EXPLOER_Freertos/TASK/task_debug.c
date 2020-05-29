#include "task_debug.h"
#include "mpu_task.h"
#include "oled.h"
#include "usart.h"
#include "PID.h"
#include "math.h"
#include "machine.h"
#include "dma_user.h"
#include "led.h"
#include "pwm.h"
#include "trasonic.h"
#include "pixy.h"
#include "myADC.h"

#define DSITANCE_BUFF_LENGTH 5
float distance_buff[DSITANCE_BUFF_LENGTH] = {0.0};
float distance_ave = 0;
float distance_sum = 0;
float trasonic_distance = 0;
float shot_temp_distance = 0;
static uint8_t i = 0;

TaskHandle_t DEGUGTask_Handler;

typedef union
{
	float num;
	uint8_t str[4];
}float2str;

float2str 	 f2str;
uint8_t 	 buf_send_float[BUFF_SENT_USART_SIZE]	 ={0};
float pid_value_change           = 0.0f;
uint8_t PID_RECEIVE_BUFF[13];
uint8_t * BUFF_P = PID_RECEIVE_BUFF;
uint8_t numb_received = 0;

void usart1_send_char(u8 c)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   

}

void send_to_anotc_by_usart1_float(float data1, float data2, float data3, float data4, float data5, float data6)
{	
	uint8_t i = 0;
    buf_send_float[0] = 0xAA;
    buf_send_float[1] = 0xAA;
	buf_send_float[2] = 0xF1;
	buf_send_float[3] = 0x18;
	
	f2str.num = data1;
	buf_send_float[4] = f2str.str[3];
	buf_send_float[5] = f2str.str[2];
	buf_send_float[6] = f2str.str[1];
	buf_send_float[7] = f2str.str[0];
	
	f2str.num = data2;
	buf_send_float[8]  = f2str.str[3];
	buf_send_float[9]  = f2str.str[2];
	buf_send_float[10] = f2str.str[1];
	buf_send_float[11] = f2str.str[0];
	
	f2str.num = data3;
	buf_send_float[12] = f2str.str[3];
	buf_send_float[13] = f2str.str[2];
	buf_send_float[14] = f2str.str[1];
	buf_send_float[15] = f2str.str[0];	
	f2str.num = data4;
	buf_send_float[16] = f2str.str[3];
	buf_send_float[17] = f2str.str[2];
	buf_send_float[18] = f2str.str[1];
	buf_send_float[19] = f2str.str[0];
	f2str.num = data5;
	buf_send_float[20] = f2str.str[3];
	buf_send_float[21] = f2str.str[2];
	buf_send_float[22] = f2str.str[1];
	buf_send_float[23] = f2str.str[0];
	f2str.num = data6;
	buf_send_float[24] = f2str.str[3];
	buf_send_float[25] = f2str.str[2];
	buf_send_float[26] = f2str.str[1];
	buf_send_float[27] = f2str.str[0];
	
	buf_send_float[28] = 0x00;
	
	for (i = 0; i < BUFF_SENT_USART_SIZE - 1; i++)
	{
	  buf_send_float[BUFF_SENT_USART_SIZE - 1] += buf_send_float[i];
	}
	
//    for(i = 0; i < BUFF_SENT_USART_SIZE; i++)
//	{
//		usart1_send_char(buf_send_float[i]);	//发送数据到串口1 
//	}
	
	MYDMA_Enable(DMA2_Stream7, BUFF_SENT_USART_SIZE);     //开始一次DMA传输！	  
	
}

void debug_task(void * pvParameters)
{
	while(1)
	{		
//		OLED_ShowString(0, 0, (uint8_t *)"P="); 
//		OLED_ShowNum(15,0, chassis_pid[0].p, 7, 12); 			OLED_ShowNum(70,0, chassis_pid[1].p, 7, 12);
//		OLED_ShowString(0,2,(u8 *)"i=");
//		OLED_ShowNum(15,2, chassis_pid[0].i * 1000, 7, 12);		OLED_ShowNum(70,2, chassis_pid[1].i * 1000, 7, 12);	
//		OLED_ShowString(0,4,(u8 *)"d=");  
//		OLED_ShowNum(15,4, chassis_pid[0].d * 1000, 7, 12);		OLED_ShowNum(70,4, chassis_pid[1].d * 1000, 7, 12);
		LED1 = ~LED1;
		send_to_anotc_by_usart1_float(present_image_angle, present_servo_angle, 0, regulator_return_err, 0, 0);
		distance_sum = 0;
		
		for (i = 0; i < DSITANCE_BUFF_LENGTH - 1; i++)
		{
			distance_buff[i] = distance_buff[i + 1];
			distance_sum += distance_buff[i];
		}	
		shot_temp_distance = shot_one_pluse();
		if (shot_temp_distance > 200 &&  shot_temp_distance < 330)
		{
			distance_buff[i] = shot_one_pluse();
		}

		distance_sum += distance_buff[i];
        trasonic_distance = distance_sum / DSITANCE_BUFF_LENGTH;		
		vTaskDelay(80);
	}
}

void MY_IDLE_Handler(void)
{	
	USART_GetFlagStatus(USART1, USART_FLAG_IDLE);
	USART_ReceiveData(USART1);
	
	if ((PID_RECEIVE_BUFF[0] == 'p')&&(PID_RECEIVE_BUFF[1]== '='))
	{
		pid_value_change = atof((const char *)(PID_RECEIVE_BUFF + 2));
//		chassis_pid[0].p = pid_value_change;
		
	}
	else if ((PID_RECEIVE_BUFF[0] == 'i')&&(PID_RECEIVE_BUFF[1]== '='))
	{
		pid_value_change = atof((const char *)(PID_RECEIVE_BUFF + 2));
//		chassis_pid[0].i = pid_value_change;		
	}
	else if ((PID_RECEIVE_BUFF[0] == 'd')&&(PID_RECEIVE_BUFF[1]== '='))
	{
		pid_value_change = atof((const char *)(PID_RECEIVE_BUFF + 2));
//		chassis_pid[0].d = pid_value_change;						
	}		
		
	BUFF_P = PID_RECEIVE_BUFF;	
}


void MY_IDLE_Show_Handler(void)
{	
	USART_GetFlagStatus(USART1, USART_FLAG_IDLE);
	USART_ReceiveData(USART1);
	
	numb_received = (BUFF_P - PID_RECEIVE_BUFF) / sizeof(uint8_t);
	
	if (numb_received < 4)
	{
		return;
	}
	
	if ((PID_RECEIVE_BUFF[0] == 0x70) && (PID_RECEIVE_BUFF[numb_received - 1] == 0xff))
	{
		if (PID_RECEIVE_BUFF[numb_received - 4] == 0x50)  //P
		{
			PID_RECEIVE_BUFF[numb_received - 1] = 0;
			PID_RECEIVE_BUFF[numb_received - 2] = 0;
			PID_RECEIVE_BUFF[numb_received - 3] = 0;
			PID_RECEIVE_BUFF[numb_received - 4] = 0;
			
			pid_value_change = atof((const char *)(PID_RECEIVE_BUFF + 1));
//			chassis_pid[0].p = pid_value_change;
		}
		else if (PID_RECEIVE_BUFF[numb_received - 4] == 0x49) //I
		{
			PID_RECEIVE_BUFF[numb_received - 1] = 0;
			PID_RECEIVE_BUFF[numb_received - 2] = 0;
			PID_RECEIVE_BUFF[numb_received - 3] = 0;
			PID_RECEIVE_BUFF[numb_received - 4] = 0;
			
			pid_value_change = atof((const char *)(PID_RECEIVE_BUFF + 1));
//			chassis_pid[0].i = pid_value_change;
		}
		else if (PID_RECEIVE_BUFF[numb_received - 4] == 0x44) //D
		{
			PID_RECEIVE_BUFF[numb_received - 1] = 0;
			PID_RECEIVE_BUFF[numb_received - 2] = 0;
			PID_RECEIVE_BUFF[numb_received - 3] = 0;
			PID_RECEIVE_BUFF[numb_received - 4] = 0;
			
			pid_value_change = atof((const char *)(PID_RECEIVE_BUFF + 1));
//			chassis_pid[0].d = pid_value_change;
						
		}
	}
		
	BUFF_P = PID_RECEIVE_BUFF;	
}

void MY_RXNE_Handler(void)
{	
	*BUFF_P = USART_ReceiveData(USART1);//(USART1->DR);	
	 BUFF_P++;
}
