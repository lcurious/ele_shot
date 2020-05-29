#include "sys.h"
#include "usart.h"	
#include "task_debug.h"
#include "string.h"
#include "stdlib.h"
#include "shot.h"
#include "machine.h"

//����������
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
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//IDLE

	//Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}



void USART1_IRQHandler(void)                	//����1�жϷ������
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
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
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART1ʱ��
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART2�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3 ; //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3
	

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
		
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����	
}

/*******************************************************************************
* �� �� ��         : USART2_IRQHandler
* ��������		   : USART2�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/ 
void USART2_IRQHandler(void)                	//����1�жϷ������
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
	 /* �������� */
    if((USART_GetITStatus(USART2, USART_IT_RXNE) != RESET))//&&USART2_Flag==0)  
		{
			num[5] = '\0';
			Raw_Data[counter] = USART_ReceiveData(USART2);
			counter++;
			if(Raw_Data[counter-1]==0x7D){
				if(Raw_Data[0]==0x7B){
					if(Raw_Data[1]==0x40){
						switch(Raw_Data[2]-0x30){//����1-8 ���ݽ���
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
									fun = 1; //�����ʼ
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
									
								case 0x02:   //��λ
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
									
								case 0x04:  //��ʼ��ʾ
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
										switch(Raw_Data[6]-0x30){//P��I��D ���ͽ���
											case 0x01:
												if(Raw_Data[7]==0x23){
													while(Raw_Data[8+i] != 0x7d){
															num[i] = Raw_Data[8+i] ;
															i++;
													}
													
													if(pro==2){ //����Ŀ�����
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
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	
	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11 ; //GPIOB10��GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PB10��PB11
	

   //USART3 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����	
}

/*******************************************************************************
* �� �� ��         : USART3_IRQHandler
* ��������		   : USART3�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/ 
void USART3_IRQHandler(void)                	//����3�жϷ������
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Temp[counter] = USART_ReceiveData(USART3);   //��������
		if(counter == 0 && Temp[0] != 0x55) return;      //�� 0 �����ݲ���֡ͷ������
		counter++; 
		if(counter==11) //���յ� 11 ������
		{ 
			mpu_data_decoder();
			counter = 0;
		}    
	} 

} 	

void mpu_data_decoder(void)
{
	if(Temp[0]==0x55)       //���֡ͷ
         {  
            switch(Temp[1])
            {
               case 0x51: //��ʶ������Ǽ��ٶȰ�
                  a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X����ٶ�
                  a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y����ٶ�
                  a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z����ٶ�
                  break;
               case 0x52: //��ʶ������ǽ��ٶȰ�
                  w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X����ٶ�
                  w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y����ٶ�
                  w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z����ٶ�
                  break;
               case 0x53: //��ʶ������ǽǶȰ�
                  angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X���ת�ǣ�x �ᣩ
                  angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y�ḩ���ǣ�y �ᣩ
                  angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z��ƫ���ǣ�z �ᣩ
                  break;
               default:  break;
            }
         }
}

