#include "pixy.h"
#include "usart.h"
#include "math.h"
#include "led.h"
#include "pid.h"
#include <stdlib.h>
///////////////////////说明///////////////////////////////
/**
APP蓝牙调参系统说明：
app发送的数据格式为 {@problem_id@func_id@PID_id#PID_value}
d
*/
/////////////////////////////////////////////////////////
float set_x = 150;//目标位置的x像素坐标
float set_y = 95;//目标位置的y像素坐标
float x,y;
float x_buff=0,y_buff=0;
u8 j = 0;
unsigned char Pixy_Data[SendBuff_Size];
int USART2_FAIL = 0;	//通信失败标志、帧头错误置高
int USART2_Half_OK=0;
unsigned char Raw_Data[19];
char USART2_Flag=0 ;
u16 counter;
u16 my;
u16 my_;
u16 mx=0;
u16 mx_=0;
void get_pixel_location();
void  data_analysis();
void 	servo_control(float x,float y);
void send_to_anotc_by_usart1_float(
			float data1, float data2, 
			float data3,float data4, 
			float data5, float data6);
/*********************匿名上位机使用*******************/
typedef union
{
	float num;
	uint8_t str[4];
}float2str;

 float2str 	 f2str;
 uint8_t 	 buf_send_float[29]		 ={0};
/*********************************************************/
void usart1_send_char(u8 c)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   
}

//void USART2_IRQHandler(void)                	//串口2中断服务程序，解析mpu6050串口输出数据
//{

//	u8 i=0;
//	u8 j=0;
//	int pro=0;
//	int fun=0;
//	u16 x_data[3];
//	 char num[6];
//	unsigned char y_data[3];
//	unsigned char temp;
//	u8 x_count = 0,y_count=0;
//	for(j=0;j<6;j++){
//		num[j] = 0;
//	}
//	 /* 接收数据 */
//    if((USART_GetITStatus(USART2, USART_IT_RXNE) != RESET))//&&USART2_Flag==0)  
//		{
//			num[5] = '\0';
//			Raw_Data[counter] = USART_ReceiveData(USART2);
//			counter++;
//			if(Raw_Data[counter-1]==0x7D){
//				if(Raw_Data[0]==0x7B){
//					if(Raw_Data[1]==0x40){
//						switch(Raw_Data[2]-0x30){//问题1-8 数据解析
//							case 0x01:
//								pro = 1;
//								break;
//							case 0x02:
//								pro = 2;
//								break;
//							case 0x03 :
//								pro = 3;
//								break;
//							case 0x04:
//								pro = 4;
//								break;
//							case 0x05:
//								pro = 5;
//								break;
//							case 0x06:
//								pro = 6;
//								break;
//							case 0x07 :
//								pro = 7;
//							case 0x08 :
//								pro = 8;
//								break;
//						}
//						printf("problem id = %d ",pro);
//					}
//					if(Raw_Data[3]==0x40){
//							switch(Raw_Data[4]-0x30){
//								case 0x01:
//									fun = 1;
//									printf(" func id = %d ",fun);
//									break;
//								case 0x02:
//									fun = 2;
//									printf(" func id = %d ",fun);
//									break;
//								case 0x03:
//									fun = 3;
//									printf(" func id = %d ",fun);
//									if(Raw_Data[5]==0x40){
//										i=0;
//										switch(Raw_Data[6]-0x30){//P、I、D 类型解析
//											case 0x01:
//												if(Raw_Data[7]==0x23){
//													while(Raw_Data[8+i] != 0x7d){
//															num[i] = Raw_Data[8+i] ;
//															i++;
//													}
//													printf("P1 = %f",atof(num));
//												}
//													
//											break;
//											case 0x02:
//												if(Raw_Data[7]==0x23){
//													while(Raw_Data[8+i] != 0x7d){
//															num[i] = Raw_Data[8+i] ;
//															i++;
//													}
//													printf("I1 = %f",atof(num));
//												}
//											break;
//											case 0x03:
//												if(Raw_Data[7]==0x23){
//													while(Raw_Data[8+i] != 0x7d){
//															num[i] = Raw_Data[8+i] ;
//															i++;
//													}
//													printf("D1 = %f",atof(num));
//												}
//												break;
//											case 0x04:
//												if(Raw_Data[7]==0x23){
//													while(Raw_Data[8+i] != 0x7d){
//															num[i] = Raw_Data[8+i] ;
//															i++;
//													}
//													printf("P2 = %f",atof(num));
//												}
//												break;
//											case 0x05:
//												if(Raw_Data[7]==0x23){
//													while(Raw_Data[8+i] != 0x7d){
//															num[i] = Raw_Data[8+i] ;
//															i++;
//													}
//													printf("I2 = %f",atof(num));
//												}
//												break;
//											case 0x06:
//												if(Raw_Data[7]==0x23){
//													while(Raw_Data[8+i] != 0x7d){
//															num[i] = Raw_Data[8+i] ;
//															i++;
//													}
//													printf("D2 = %f",atof(num));
//												}
//												break;
//										}
//									}
//									break;
//							}
//					
//					}
//				}

////				printf("x=%d,y=%d \n",mx,my);
////				for(j = 0;j<9;j++){
////					printf("%c",Raw_Data[j]);
////				}
//				printf("\n");
//				counter=0;
//			}
////			else{
////			printf("%c",Raw_Data[counter-1]);
////			
////			}
//				
//	}
//	 
//}

////analysis the data from Pixy camera
//void data_analysis(){
//	int i;
//	counter++;
//	if(counter>=14)
//	{
//		if(Raw_Data[0]==0x55 && Raw_Data[1]==0xAA && Raw_Data[2]==0x55 && Raw_Data[3]==0xAA)       USART2_Half_OK=0x01;//正常码模式，帧头为55 aa 55 aa
//		else if(Raw_Data[0]==0x55 && Raw_Data[1]==0xAA && Raw_Data[2]!=0x55 && Raw_Data[2]!=0x56)  USART2_Half_OK=0x02;//正常码模式，帧头为55 aa
//		else if(Raw_Data[0]==0x55 && Raw_Data[1]==0xAA && Raw_Data[2]==0x56 && Raw_Data[3]==0xAA)  USART2_Half_OK=0x03;//cc模式，帧头为55 aa 56 aa
//		else if(Raw_Data[0]==0x56 && Raw_Data[1]==0xAA)  USART2_Half_OK=0x04;//正常码模式+CC模式，帧头56 aa
//		if(USART2_Half_OK!=0)
//		{
//			if((USART2_Half_OK==0x01 && counter==16)||(USART2_Half_OK==0x02 && counter==14)||(USART2_Half_OK==0x03 && counter==18)||(USART2_Half_OK==0x04 && counter==16))  
//			{
//					for(i=0;i<counter;i++)  
//				 {
//					 Pixy_Data[i]=Raw_Data[i];
//				 }
//				get_pixel_location();
////	printf("x = %f \t y = %f \n",x,y);
//				 LED1 = !LED1;
//				USART2_Flag=1;
//				counter=0;
//			}						 
//		}
//	}

//}
////获取坐标
//void get_pixel_location(){
//	switch(USART2_Half_OK)
//	{
//	case 0x03:
//	{
//		x   = Pixy_Data[8]  + Pixy_Data[9]*256;
//		y   = Pixy_Data[10] + Pixy_Data[11]*256;
//	}break;

//	case 0x01:
//	{
//		x   = Pixy_Data[8]  + Pixy_Data[9]*256;
//		y   = Pixy_Data[10] + Pixy_Data[11]*256;
//	}break;

//	case 0x02:
//	{
//		x   = Pixy_Data[6]  + Pixy_Data[7]*256;
//		y   = Pixy_Data[8]  + Pixy_Data[9]*256;
//	}break;

//	case 0x04:
//	{
//		x   = Pixy_Data[6]  + Pixy_Data[7]*256;
//		y   = Pixy_Data[8]  + Pixy_Data[9]*256;	
//	}break;
//	default:break;
//	}
//}
////控制舵机动作
///**
//对于第二问：从1——5 :set_y > y5 servo_1 向下降 PWM值减小 set_y <y5 servo_1 向上抬 PWM增大 
//									 set_x > x5	servo_2 向上抬 PWM 增加	 set_x <x5 servo_2 向下降 PWM 减小

//*/
//int count = 0;
//void 	servo_control(float x,float y){
//	
////	/************第二题****************/
////	base_func2( x, y);
////	/****************************/
////	base_func3(x, y);
////	/************第四题**************/
////	base_func4(x, y);
////	/***********************/
//		advance_func1(x,y);
//	
//}
///**
//基础功能2
//*/
//void base_func2(float x,float y){
////	execute  the PID controlor
////		pid_regulator(&chassis_pid[0], set_x, x);
////   	pid_regulator(&chassis_pid[1], set_y, y);
//		set_x = x_group[4];//目标位置的x像素坐标
//		set_y = y_group[4];//目标位置的y像素坐标
//		servo_pid_calculation(&servo_pid[0],set_x,x);
//		servo_pid_calculation(&servo_pid[1],set_y,y);
//	//////////////////////////////////////
//	count++;
//	if(count>10&&count%2==0){
//	if(fabs(set_x-x)>5&&fabs(set_x-x)<30){//当偏差小于一定值的时候 才开始调节
//		if(servo_pid[0].Out < 0){//set_x>x5 向上抬 PWM 增加
//			//阈值保护
//			if((140-servo_pid[0].Out)>200){
//				servo_pid[0].Out = 140-200;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);//红线
//		
//		}else{		//set_x>x5 向上抬 PWM 增加
//		//阈值保护
//			if((140-servo_pid[0].Out)<100){
//				servo_pid[0].Out =140-100;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);	//黄线
//		}
//		printf("x=%f \t dx=%f  \t%f \t %f \n",x,140-servo_pid[0].Out,servo_pid[0].Out,set_x-x);
//	}else{//当偏差大于一定值的时候 加前馈  
//		TIM_SetCompare2(TIM4,140);
//	}
//	//servo_1 configuration 
//	if(fabs(set_y-y)>5&&fabs(set_y-y)<50){
//		//set_y > y5 servo_1 向下降 PWM值减小 set_y <y5 servo_1 向上抬 PWM增大 
//			if(servo_pid[1].Out < 0){
//				
//			if(140+servo_pid[1].Out<100){
//				servo_pid[1].Out = 100-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}else{
//			
//			if(140+servo_pid[1].Out>200){
//				servo_pid[1].Out = 200-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}
//		 printf("\t y=%f  \t dy = %f  %f  %f\n",y,140-servo_pid[1].Out,servo_pid[1].Out,set_y-y);
//	}else{
//		TIM_SetCompare1(TIM4,140);
//	}
//	}else if(count<10){
//		TIM_SetCompare1(TIM4,120);
//		TIM_SetCompare2(TIM4,155);
//	}else{
//		TIM_SetCompare1(TIM4,140);
//		TIM_SetCompare2(TIM4,140);
//	}
//}
///**
//基础功能3
//*/
//int time=0;
//int count1=0;
//void base_func3(float x,float y){

////	execute  the PID controlor
////		pid_regulator(&chassis_pid[0], set_x, x);
////   	pid_regulator(&chassis_pid[1], set_y, y);
//			if(fabs(223-x)<6&&fabs(98-y)<6){
//				time++;
//			}	
//			if(time>110){
//				base_func3_step2(x,y);
//			}else{
//				printf("x = %f, y=%f ,time=%d \t %f \t%f \n",x,y,time,set_x-x,set_y-y);
//				base_func3_step1(x,y);
//			}
//			if(time==110){
//			servo_pid_init(&servo_pid[0],2);
//			servo_pid_init(&servo_pid[1],2);
//			}
//			
//			
//}
///**
//基础部分第三问第一部分  从1-4
//*/
//void base_func3_step1(float x,float y){

////	base_func2( x, y);
//		set_x = 223;//目标位置的x像素坐标
//		set_y = 98;//目标位置的y像素坐标
//	
//		servo_pid[0].P = 1.25f;
//		servo_pid[0].I = 0.0f;
//		servo_pid[0].D = 15.0f;
//		
//		servo_pid[1].P = 1.25f;
//		servo_pid[1].I = 0.0f;
//		servo_pid[1].D = 15.0f;
//	
//		servo_pid_calculation(&servo_pid[0],set_x,x);
//		servo_pid_calculation(&servo_pid[1],set_y,y);
//	//////////////////////////////////////
//	count++;
//	if(count>12&count%2==0){
//	if(fabs(set_x-x)>6&&fabs(set_x-x)<50){//当偏差小于一定值的时候 才开始调节
//		if(servo_pid[0].Out < 0){//set_x>x5 向上抬 PWM 增加
//			//阈值保护
//			if((140-servo_pid[0].Out)>200){
//				servo_pid[0].Out = 140-200;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);//红线
//		
//		}else{		//set_x>x5 向上抬 PWM 增加
//		//阈值保护
//			if((140-servo_pid[0].Out)<100){
//				servo_pid[0].Out =140-100;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);	//黄线
//		}
////		printf("x=%f \t dx=%f  \t%f \t %f \n",x,140-servo_pid[0].Out,servo_pid[0].Out,set_x-x);
//	}else{//当偏差大于一定值的时候 加前馈  
//		TIM_SetCompare2(TIM4,140);
//	}
//	//servo_1 configuration 
//	if(fabs(set_y-y)>6&&fabs(set_y-y)<50){
//		//set_y > y5 servo_1 向下降 PWM值减小 set_y <y5 servo_1 向上抬 PWM增大 
//			if(servo_pid[1].Out < 0){
//				
//			if(140+servo_pid[1].Out<100){
//				servo_pid[1].Out = 100-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}else{
//			
//			if(140+servo_pid[1].Out>200){
//				servo_pid[1].Out = 200-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}
////		 printf("\t y=%f  \t dy = %f  %f  %f\n",y,140-servo_pid[1].Out,servo_pid[1].Out,set_y-y);
//	}else{
//		TIM_SetCompare1(TIM4,140);
//	}
//	}else if(count<12){
//		/*************从1-4 count 值为14 PID 分别为 1.2 0 17*************************/
//		TIM_SetCompare1(TIM4,120);
//		TIM_SetCompare2(TIM4,140);
//		/***********************/
//	}else {
//		TIM_SetCompare1(TIM4,140);
//		TIM_SetCompare2(TIM4,140);
//	}
//}
///**
//基础部分第三问第一部分  从4-5
//*/
//void base_func3_step2(float x,float y){
//	
//		set_x = 155;//目标位置的x像素坐标
//		set_y = 100;//目标位置的y像素坐标
//	
//		servo_pid[0].P = 1.2f;
//		servo_pid[0].I = 0.0f;
//		servo_pid[0].D = 15.0f;
//		
//		servo_pid[1].P = 1.4f;
//		servo_pid[1].I = 0.0f;
//		servo_pid[1].D = 15.0f;
//	
//		servo_pid_calculation(&servo_pid[0],set_x,x);
//		servo_pid_calculation(&servo_pid[1],set_y,y);
//	//////////////////////////////////////
//	count1++;
//	if(count1>6&count1%2==0){
//	if(fabs(set_x-x)>6&&fabs(set_x-x)<50){//当偏差小于一定值的时候 才开始调节
//		if(servo_pid[0].Out < 0){//set_x>x5 向上抬 PWM 增加
//			//阈值保护
//			if((140-servo_pid[0].Out)>200){
//				servo_pid[0].Out = 140-200;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);//红线
//		
//		}else{		//set_x>x5 向上抬 PWM 增加
//		//阈值保护
//			if((140-servo_pid[0].Out)<100){
//				servo_pid[0].Out =140-100;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);	//黄线
//		}
//		printf("x=%f \t dx=%f  \t%f \t %f \n",x,140-servo_pid[0].Out,servo_pid[0].Out,set_x-x);
//	}else{//当偏差大于一定值的时候 加前馈  
//		TIM_SetCompare2(TIM4,140);
//	}
//	//servo_1 configuration 
//	if(fabs(set_y-y)>6&&fabs(set_y-y)<50){
//		//set_y > y5 servo_1 向下降 PWM值减小 set_y <y5 servo_1 向上抬 PWM增大 
//			if(servo_pid[1].Out < 0){
//				
//			if(140+servo_pid[1].Out<100){
//				servo_pid[1].Out = 100-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}else{
//			
//			if(140+servo_pid[1].Out>200){
//				servo_pid[1].Out = 200-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}
//		 printf("\t y=%f  \t dy = %f  %f  %f\n",y,140-servo_pid[1].Out,servo_pid[1].Out,set_y-y);
//	}else{
//		TIM_SetCompare1(TIM4,140);
//	}
//	}else if(count1<6){
//		/***********从4到5 count 值为12 PID 分别为 1 0.01 18********/
//		TIM_SetCompare1(TIM4,130);
//		TIM_SetCompare2(TIM4,170);
//		/*********************/
//		/*************从1-4 count 值为14 PID 分别为 1.2 0 17*************************/
////		TIM_SetCompare1(TIM4,125);
////		TIM_SetCompare2(TIM4,140);
//		/***********************/
//	}else {
//		TIM_SetCompare1(TIM4,140);
//		TIM_SetCompare2(TIM4,140);
//	}
//}//从1-4 

//void base_func4(float x,float y){
////	execute  the PID controlor
////		pid_regulator(&chassis_pid[0], set_x, x);
////   	pid_regulator(&chassis_pid[1], set_y, y);
//		set_x = 88;
//		set_y = 31;
//		servo_pid_calculation(&servo_pid[0],set_x,x);
//		servo_pid_calculation(&servo_pid[1],set_y,y);
//	//////////////////////////////////////
//	count++;
//	if(count>13&&count%2==0){
//	if(fabs(set_x-x)>5&&fabs(set_x-x)<50){//当偏差小于一定值的时候 才开始调节
//		if(servo_pid[0].Out < 0){//set_x>x5 向上抬 PWM 增加
//			//阈值保护
//			if((140-servo_pid[0].Out)>200){
//				servo_pid[0].Out = 140-200;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);//红线
//		
//		}else{		//set_x>x5 向上抬 PWM 增加
//		//阈值保护
//			if((140-servo_pid[0].Out)<100){
//				servo_pid[0].Out =140-100;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);	//黄线
//		}
//		printf("x=%f \t dx=%f  \t%f \t %f \n",x,140-servo_pid[0].Out,servo_pid[0].Out,set_x-x);
//	}else{//当偏差大于一定值的时候 加前馈  
//		TIM_SetCompare2(TIM4,140);
//	}
//	//servo_1 configuration 
//	if(fabs(set_y-y)>5&&fabs(set_y-y)<50){
//		//set_y > y5 servo_1 向下降 PWM值减小 set_y <y5 servo_1 向上抬 PWM增大 
//			if(servo_pid[1].Out < 0){
//				
//			if(140+servo_pid[1].Out<100){
//				servo_pid[1].Out = 100-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}else{
//			
//			if(140+servo_pid[1].Out>200){
//				servo_pid[1].Out = 200-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}
//		 printf("\t y=%f  \t dy = %f  %f  %f\n",y,140-servo_pid[1].Out,servo_pid[1].Out,set_y-y);
//	}else{
//		TIM_SetCompare1(TIM4,140);
//	}
//	}else if(count<13){
//		TIM_SetCompare1(TIM4,120);
//		TIM_SetCompare2(TIM4,155);
//	}else {
//		TIM_SetCompare1(TIM4,140);
//		TIM_SetCompare2(TIM4,140);
//	}
//}

///*****************发挥部分第一问*******************/
//int time1=0;
//int time2=0;
//int mFb_time;
//float mFb_x;
//float mFb_y;
//void advance_func1(float x,float y){
////	execute  the PID controlor
//				servo_pid[0].P = 1.3f;
//				servo_pid[0].I = 0.0f;
//				servo_pid[0].D = 17.0f;
//				
//				servo_pid[1].P = 	1.3f;
//				servo_pid[1].I = 0.0f;
//				servo_pid[1].D = 17.0f;
////			if(fabs(set_x-x)<10){
////			servo_pid[0].I = -1.0;
////			}else{
////			servo_pid[0].Isum = 0;
////			}
////			if(fabs(set_y-y)<10){
////				servo_pid[1].I = -1.0;
////			}else{
////				servo_pid[1].Isum = 0;
////			}
//			if(fabs(set_x-x)<6&&fabs(set_y-y)<6){
//				time1++;
//			}
//			if(time1<70){
//				set_x = x_group[7];
//				set_y = y_group[7];
//				mFb_time = fb_time[0];
//				mFb_x = fb_x[0];
//				mFb_y = fb_y[0];
//			}
//			if(time1==70){
//					count = 0;
//			}
//			if(time1>70 &&fabs(set_x-x)<6&&fabs(set_y-y)<6 ){
//				set_x = x_group[5];
//				set_y = y_group[5];
//				time2++;
//				
//				mFb_time = fb_time[1];
//				mFb_x = fb_x[1];
//				mFb_y = fb_y[1];
//			}
//			if(time2==70){
//				count=0;
//			}
//			if(time1>70&&time2>70){
//				set_x = x_group[8];
//				set_y = y_group[8];
//				mFb_time = fb_time[2];
//				mFb_x = fb_x[3];
//				mFb_y = fb_y[3];
//			}

//		servo_pid_calculation(&servo_pid[0],set_x,x);
//		servo_pid_calculation(&servo_pid[1],set_y,y);

//	//////////////////////////////////////
//	count++;
//	if(count>mFb_time&&count%2==0){
//	if(fabs(set_x-x)>8&&fabs(set_x-x)<100){//当偏差小于一定值的时候 才开始调节
//		if(servo_pid[0].Out < 0){//set_x>x5 向上抬 PWM 增加
//			//阈值保护
//			if((140-servo_pid[0].Out)>200){
//				servo_pid[0].Out = 140-200;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);//红线
//		
//		}else{		//set_x>x5 向上抬 PWM 增加
//		//阈值保护
//			if((140-servo_pid[0].Out)<100){
//				servo_pid[0].Out =140-100;
//			}
//			//set PWM for servo_2
//			TIM_SetCompare2(TIM4,140-servo_pid[0].Out);	//黄线
//		}
//		printf("x=%f \t dx=%f  \t%f \t %f \n",x,140-servo_pid[0].Out,servo_pid[0].Out,set_x-x);
//	}else{//当偏差大于一定值的时候 加前馈  
//		TIM_SetCompare2(TIM4,140);
//	}
//	//servo_1 configuration 
//	if(fabs(set_y-y)>8&&fabs(set_y-y)<100){
//		//set_y > y5 servo_1 向下降 PWM值减小 set_y <y5 servo_1 向上抬 PWM增大 
//			if(servo_pid[1].Out < 0){
//				
//			if(140+servo_pid[1].Out<100){
//				servo_pid[1].Out = 100-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}else{
//			
//			if(140+servo_pid[1].Out>200){
//				servo_pid[1].Out = 200-140;
//			}
//			TIM_SetCompare1(TIM4,140+servo_pid[1].Out);	//黄线
//		}
//		 printf("\t y=%f  \t dy = %f  %f  %f\n",y,140-servo_pid[1].Out,servo_pid[1].Out,set_y-y);
//	}else{
//		TIM_SetCompare1(TIM4,140);
//	}
//	}else if(count<mFb_time){
//		TIM_SetCompare1(TIM4,mFb_x);
//		TIM_SetCompare2(TIM4,mFb_y);
//	}else {
//		TIM_SetCompare1(TIM4,140);
//		TIM_SetCompare2(TIM4,140);
//	}
//}
