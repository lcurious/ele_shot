#include "pixy.h"

/*
PA0 U4_Tx
PA1 U4_Rx
*/
float image_x = 0;
float image_y = 0;
float set_x = 150;//Ŀ��λ�õ�x��������
float set_y = 95;//Ŀ��λ�õ�y��������
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
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART4ʱ��
 
	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0����ΪUSART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1����ΪUART4
	
	//USART4�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 ; //GPIOB10��GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	

   //USART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure); //��ʼ������4
	
	USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
		
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel =UART4_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����	
}

/*******************************************************************************
* �� �� ��         : USART4_IRQHandler
* ��������		   : USART4�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/ 
void UART4_IRQHandler(void)                	//����4�жϷ������
{
			x_data[3] = '\0';
		y_data[3] = '\0';
	x_count = 0;
	y_count = 0;
	 /* �������� */
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