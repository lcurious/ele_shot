#include "sys.h"
#include "usart.h"	
#include "task_debug.h"
#include "string.h"
#include "stdlib.h"
#include "shot.h"
#include "machine.h"

//用于陀螺仪
static unsigned char counter=0;
unsigned char sign,t,he;
static unsigned char Temp[11];
float a[3],w[3],angle[3];
static unsigned char Raw_Data[19];
float input_distance = 0;
float input_err_angle = 0;
uint8_t mov_ave_flag = 0;
int pro=0;
uint8_t searching_flag = 0;
void mpu_data_decoder(void);

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#endif 
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//IDLE

	//Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}



void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		MY_RXNE_Handler();	
	} 
	
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		MY_IDLE_Handler();
//		MY_IDLE_Show_Handler();
	}
} 



#endif	

/**
PA2 U_TX
PA3 U_RX
**/

 void USART2_Init(u32 bound)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3 ; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
		
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}

/*******************************************************************************
* 函 数 名         : USART2_IRQHandler
* 函数功能		   : USART2中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/ 
void USART2_IRQHandler(void)                	//串口1中断服务程序
{
		u8 i=0;
	u8 j=0;
	int fun=0;
	u16 x_data[3];
	 char num[6];
	unsigned char y_data[3];
	unsigned char temp;
	u8 x_count = 0,y_count=0;
	for(j=0;j<6;j++){
		num[j] = 0;
	}
	 /* 接收数据 */
    if((USART_GetITStatus(USART2, USART_IT_RXNE) != RESET))//&&USART2_Flag==0)  
		{
			num[5] = '\0';
			Raw_Data[counter] = USART_ReceiveData(USART2);
			counter++;
			if(Raw_Data[counter-1]==0x7D){
				if(Raw_Data[0]==0x7B){
					if(Raw_Data[1]==0x40){
						switch(Raw_Data[2]-0x30){//问题1-8 数据解析
							case 0x01:
								pro = 1;
								break;
							case 0x02:
								pro = 2;
								break;
							case 0x03 :
								pro = 3;
								break;
							case 0x04:
								pro = 4;
								break;
							case 0x05:
								pro = 5;
								break;
							case 0x06:
								pro = 6;
								break;
							case 0x07 :
								pro = 7;
							case 0x08 :
								pro = 8;
								break;
						}

					}
					if(Raw_Data[3]==0x40){
							switch(Raw_Data[4]-0x30){
								case 0x01:
									fun = 1; //舵机开始
									if (pro == 4)
									{
										searching_flag = 1;
									}
								
									else if (pro == 5)
									{
										mov_ave_flag = 1;
									}
									else if (pro == 6)
									{
										search_falg_pro6 = 1;
									}
									break;
									
								case 0x02:   //复位
									fun = 2;
								
									if (pro == 4)
									{
										searching_flag  = 0;
										pro4_no_use_tra = 0;
																			
									}
									else if (pro == 5)
									{
										mov_ave_flag = 0;
									}
									else if (pro == 6)
									{
										search_falg_pro6 = 0;
									}
										clear();

									break;
									
								case 0x04:  //开始演示
									fun = 4;
									if (pro <= 3)
									{
										charge();
									}
									break;
									
								case 0x03:  
									fun = 3;

								
									if(Raw_Data[5]==0x40){
										i=0;
										switch(Raw_Data[6]-0x30){//P、I、D 类型解析
											case 0x01:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
													
													if(pro==2){ //输入目标距离
														input_distance= atof(num);
														set_map_angle(input_distance);
														
													}
												
												}
													
											break;
											case 0x02:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
													
													if(pro==1 || pro==2){
														
														set_angle_gimbal0 += atof(num);	
													}
													else if (pro == 5)
													{
														shot_area_limit += atof(num);
													}
												}
											break;
											case 0x03:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
													
													if(pro==1||pro==2){
														
														set_angle_gimbal1 += atof(num);	
													}
													else if (pro == 4)
													{
														pro4_no_use_tra = 1;
														pro4_input_angle += atof(num);
														
													}
													else if (pro == 5)
													{
														pro5_init_angle +=atof(num);
													}
													else if (pro == 6)
													{
														pro6_init_angle += atof(num);
													}
												}
												break;
											case 0x04:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
//													printf("P2 = %f",atof(num));
												}
												break;
											case 0x05:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
//													printf("I2 = %f",atof(num));
												}
												break;
											case 0x06:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
//													printf("D2 = %f",atof(num));
												}
												break;
										}
									}
									break;
							}
					
					}
				}

//				printf("x=%d,y=%d \n",mx,my);
//				for(j = 0;j<9;j++){
//					printf("%c",Raw_Data[j]);
//				}
//				printf("\n");
				counter=0;
			}
//			else{
//			printf("%c",Raw_Data[counter-1]);
//			
//			}
				
	}
	 

} 	

/*
PB10 U3_Tx
PB11 U3_Rx
*/

 void USART3_Init(u32 bound)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	
	//USART3端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11 ; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11
	

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_Cmd(USART3, ENABLE);  //使能串口3 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}

/*******************************************************************************
* 函 数 名         : USART3_IRQHandler
* 函数功能		   : USART3中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/ 
void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		Temp[counter] = USART_ReceiveData(USART3);   //接收数据
		if(counter == 0 && Temp[0] != 0x55) return;      //第 0 号数据不是帧头，跳过
		counter++; 
		if(counter==11) //接收到 11 个数据
		{ 
			mpu_data_decoder();
			counter = 0;
		}    
	} 

} 	

void mpu_data_decoder(void)
{
	if(Temp[0]==0x55)       //检查帧头
         {  
            switch(Temp[1])
            {
               case 0x51: //标识这个包是加速度包
                  a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X轴加速度
                  a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y轴加速度
                  a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z轴加速度
                  break;
               case 0x52: //标识这个包是角速度包
                  w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X轴角速度
                  w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y轴角速度
                  w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z轴角速度
                  break;
               case 0x53: //标识这个包是角度包
                  angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X轴滚转角（x 轴）
                  angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y轴俯仰角（y 轴）
                  angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z轴偏航角（z 轴）
                  break;
               default:  break;
            }
         }
}

