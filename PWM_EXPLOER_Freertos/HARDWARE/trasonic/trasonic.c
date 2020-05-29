#include "trasonic.h"
#include "delay.h"

u8 TIM9_CH1_CAPTURE_STA=0; //���벶���״̬
u16 TIM9_CH1_CAPTURE_VAL=0;//���벶��ֵ(TIM9_CNT 16λ)

/**
PE5   -->TIM9_CH1 trasonic_Echo
PE6   -->normal_use trasonic_Trig
**/

void TIM9_CH1_Input_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);//�ܽŸ���
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;//�ܽ�����
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; //�������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//�ٶ�Ϊ100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ���ṹ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;//�ܽ�����
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //�������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//�ٶ�Ϊ100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ���ṹ��
	
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);	
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1; //ͨ��1
	TIM_ICInitStructure.TIM_ICFilter=0x00;  //�˲�
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//������
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; //��Ƶϵ��
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//ֱ��ӳ�䵽TI1
	TIM_ICInit(TIM9,&TIM_ICInitStructure);
	TIM_ITConfig(TIM9,TIM_IT_Update|TIM_IT_CC1,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;//�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);
		
	TIM_Cmd(TIM9,ENABLE); //ʹ�ܶ�ʱ��

}

/*******************************************************************************
* �� �� ��         : TIM9_IRQHandler
* ��������		   : TIM9�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	if((TIM9_CH1_CAPTURE_STA&0x80)==0) //��δ�ɹ�����
	{
		if(TIM_GetITStatus(TIM9,TIM_IT_Update)) //���������ж�
		{
			if(TIM9_CH1_CAPTURE_STA&0X40)//�����˸ߵ�ƽ
			{
				if((TIM9_CH1_CAPTURE_STA&0x3f)==0x3f) //�ߵ�ƽʱ��̫��
				{
					TIM9_CH1_CAPTURE_STA|=0x80; //��־һ�β���ɹ�
					TIM9_CH1_CAPTURE_VAL=0xffff;
				}
				else
				{
					TIM9_CH1_CAPTURE_STA++;
				}
			}
		}
		if(TIM_GetITStatus(TIM9,TIM_IT_CC1)) //���������ж�
		{
			if(TIM9_CH1_CAPTURE_STA&0X40)//�����˵͵�ƽ
			{
				TIM9_CH1_CAPTURE_STA|=0x80; //�ɹ�����һ�θߵ�ƽ
				TIM9_CH1_CAPTURE_VAL=TIM_GetCapture1(TIM9);
				TIM_OC1PolarityConfig(TIM9,TIM_ICPolarity_Rising); //���������ز���
			}
			else
			{
				TIM9_CH1_CAPTURE_STA=0;
				TIM9_CH1_CAPTURE_VAL=0;
				TIM9_CH1_CAPTURE_STA|=0x40; //���񵽸ߵ�ƽ ��־
				TIM_Cmd(TIM9,DISABLE);
				TIM_SetCounter(TIM9,0); //��ʱ����ֵΪ0
				TIM_OC1PolarityConfig(TIM9,TIM_ICPolarity_Falling); //�����½��ز���
				TIM_Cmd(TIM9,ENABLE);
			}
		}
	}
	TIM_ClearITPendingBit(TIM9,TIM_IT_CC1|TIM_IT_Update);
}

float shot_one_pluse(void)
{	
	static float trasonic_distance = 0;
	int32_t normal_cnt = 0;
	
	if ((TIM9_CH1_CAPTURE_STA&0x80) != 0)
	{		
		normal_cnt = TIM9_CH1_CAPTURE_VAL + 0xffff * (TIM9_CH1_CAPTURE_STA & 0x3f);
		trasonic_distance = 0.017f * (float)normal_cnt;
		TIM9_CH1_CAPTURE_STA = 0;
	}
	else 
	{
		PEout(6) = 1;
		delay_us(45);
		PEout(6) = 0;		
	}
	
	return trasonic_distance;
}
