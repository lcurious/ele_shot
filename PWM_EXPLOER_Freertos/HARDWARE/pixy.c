#include "pixy.h"

/*
PA0 U4_Tx
PA1 U4_Rx
*/
float image_x = 0;
float image_y = 0;
float set_x = 150;//目标位置的x像素坐标
float set_y = 95;//目标位置的y像素坐标
float x,y;
float x_buff=0,y_buff=0;
u8 j = 0;

unsigned char Raw_Data[29];
;
u16 counter;
int mCount;
u16 my;
u16 my_buff;
u16 my_;
u16 mx=0;
u16 mx_buff;
u16 mx_=0;
u8 i=0;
char x_data[4];
char y_data[4];
unsigned char temp;
u8 x_count = 0,y_count=0;

 void USAR4_Init(u32 bound)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART4时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0复用为USART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1复用为UART4
	
	//USART4端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 ; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	

   //USART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口4
	
	USART_Cmd(UART4, ENABLE);  //使能串口4
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
		
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel =UART4_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}

/*******************************************************************************
* 函 数 名         : USART4_IRQHandler
* 函数功能		   : USART4中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/ 
void UART4_IRQHandler(void)                	//串口4中断服务程序
{
			x_data[3] = '\0';
		y_data[3] = '\0';
	x_count = 0;
	y_count = 0;
	 /* 接收数据 */
    if((USART_GetITStatus(UART4, USART_IT_RXNE) != RESET))//&&USART2_Flag==0)  
		{
			Raw_Data[mCount] = USART_ReceiveData(UART4);
			mCount++;
			if(Raw_Data[mCount-1]==0x7D){
				if(Raw_Data[0]==0x7B){
					for(i=1;x_count<=4&&Raw_Data[i] != 0x2c;i++){
						mx = Raw_Data[i];
						
						x_data[i-1] = Raw_Data[i]-0x30;
						mx_ = x_data[i-1];
						x_count++;
					}
					i+=1;
					for(j=0;y_count<=4&&Raw_Data[i] != 0x7D;i++){
						y_data[j] = Raw_Data[i]-0x30;
						j++;
						y_count++;
					}

				}
				my_buff = my;
				mx_buff = mx;
				switch(x_count){
					case 3:
						mx = x_data[0]*100+x_data[1]*10+x_data[2];
					break;
					case 2 :
						mx = x_data[0]*10+x_data[1];
						break;
					case 1:
						mx = x_data[0];
						break;
					default:
						mx = 0;
						break;
				}
				switch(y_count){
					case 3:
						my = y_data[0]*100+y_data[1]*10+y_data[2];
					break;
					case 2 :
						my = y_data[0]*10+y_data[1];
						break;
					case 1:
						my = y_data[0];
						break;
					default:
						my = 0;
						break;
				}

				if(mx==0||my==0){
					mx = mx_buff;
					my = my_buff;
				
				}else{
			
				}
				for(j = 0;j<29;j++){
					Raw_Data[j] = 0;
				}
				mCount=0;
			
			}
			
            image_x = mx - image_center_offset;
			image_x = -image_x * 0.5f;
			USART_ClearITPendingBit(UART4, USART_IT_RXNE);	
	}
	 
} 	

float get_image_ave_angle(uint16_t time_delay)
{
	float sum = 0;
	uint16_t i = 0;
	for ( i = 0; i < 5; i++)
	{
		sum += image_x;
		vTaskDelay(time_delay);
	}
	return sum / 5.0f;
}