/******************************************
定时器3的编码器模式配置，可以直接对两个通道口的输入进行计数，得到编码值
为TIM3->CNT
编码器数值：磁环为26级充磁，电机减速比为1:30
            一圈数值为2*26*30=1560
*******************************************/

/***
TIM3_CH1 PA6   MACHIN1
TIM3_CH2 PA7

TIM5_CH1 PA0   MACHIN2
TIM5_CH2 PA1

TIM2_CH1 PA0   MACHIN3 ----> (remap) TIM2_REMAP[1:0] = 01 PA15
TIM2_CH2 PA1											  PB3

TIM12_CH1 PB14 MACHIN4
TIM12_CH2 PB15

**/

#include "encoder.h"
#include "machine.h"

const int32_t encoder_period = 65535;

int32_t ang_round_cnt[4] = {0};

/**
TIM3_CH1 PA6   MACHIN1
TIM3_CH2 PA7
**/
void ENC1_TIM3_Init(void)//TIM3
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* Encoder unit connected to TIM3, 4X mode */    
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock source enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* Enable GPIOA, clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


	GPIO_StructInit(&GPIO_InitStructure);
	/* Configure PA.06,07 as encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	/* Enable the TIM3 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = encoder_period; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;   
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, 
							   TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	// Clear all pending interrupts
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM3->CNT = 1;

	TIM_Cmd(TIM3, ENABLE); 
}

/**
TIM5_CH1 PA0 MACHIN2
TIM5_CH2 PA1
**/

void ENC2_TIM5_Init(void)//TIM5
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* Encoder unit connected to TIM5, 4X mode */    
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM5 clock source enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	/* Enable GPIOA, clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


	GPIO_StructInit(&GPIO_InitStructure);
	/* Configure PA.0,1 as encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	/* Enable the TIM3 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = encoder_period; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;   
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, 
							   TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	// Clear all pending interrupts
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM5->CNT = 1;

	TIM_Cmd(TIM5, ENABLE); 
}

/**
TIM2_CH1 PA0   MACHIN3 ----> (remap) TIM2_REMAP[1:0] = 01 PA15
TIM2_CH2 PA1											  PB3
TIM2_CH3 PA2 
TIM2_CH4 PA3
**/

void ENC3_TIM2_Init(void)//TIM2
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* Encoder unit connected to TIM2, 4X mode */    
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM2 clock source enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Enable GPIOA, clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	GPIO_StructInit(&GPIO_InitStructure);
	/* Configure PA.15,PB.3 as encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*remap PA15 PB3*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	/* Enable the TIM2 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = encoder_period; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;   
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, 
							   TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	// Clear all pending interrupts
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM2->CNT = 1;

	TIM_Cmd(TIM2, ENABLE); 
}

/**
TIM12_CH1 PB14 MACHIN4
TIM12_CH2 PB15
**/

//void ENC4_TIM12_Init(void)//TIM12
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;

//	/* Encoder unit connected to TIM12, 4X mode */    
//	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	/* TIM12 clock source enable */
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
//	/* Enable GPIOA, clock */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


//	GPIO_StructInit(&GPIO_InitStructure);
//	/* Configure PB14,15 as encoder input */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//		
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);

//	/* Enable the TIM12 Update Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	/* Timer configuration in Encoder mode */
//	TIM_DeInit(TIM12);
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

//	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
//	TIM_TimeBaseStructure.TIM_Period = encoder_period; 
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;   
//	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

//	TIM_EncoderInterfaceConfig(TIM12, TIM_EncoderMode_TI12, 
//							   TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 6;
//	TIM_ICInit(TIM12, &TIM_ICInitStructure);

//	// Clear all pending interrupts
//	TIM_ClearFlag(TIM12, TIM_FLAG_Update);
//	TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
//	//Reset counter
//	TIM12->CNT = 1;

//	TIM_Cmd(TIM12, ENABLE); 
//}

/**
TIM8_CH1 PC6
TIM8_CH2 PC7
**/

void ENC4_TIM8_Init(void)//TIM8
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* Encoder unit connected to TIM8, 4X mode */    
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM8 clock source enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	/* Enable GPIOC, clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	/* Configure PC6 PC7 as encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
			
	/*remap*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);

	/* Enable the TIM8 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM8);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = encoder_period; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, 
							   TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	// Clear all pending interrupts
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM8->CNT = 1;

	TIM_Cmd(TIM8, ENABLE); 
}

/*******************************************************************************
* 函 数 名         : TIM3_IRQHandler
* 函数功能		   : TIM3中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void TIM3_IRQHandler(void)  //MACHIN1
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		if((TIM3->CR1 & 0x10) == 0x10)
		{
			ang_round_cnt[MACHINE1]--;
		}
		else if ((TIM3->CR1 & 0x10) == 0x00)
		{
			ang_round_cnt[MACHINE1]++;
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	
}

void TIM5_IRQHandler(void) //MACHIN2
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update))
	{
		if((TIM5->CR1 & 0x10) == 0x10)
		{
			ang_round_cnt[MACHINE2]--;
		}
		else if ((TIM5->CR1 & 0x10) == 0x00)
		{
			ang_round_cnt[MACHINE2]++;
		}
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);	
}

void TIM2_IRQHandler(void) //MACHIN3
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		if((TIM2->CR1 & 0x10) == 0x10)
		{
			ang_round_cnt[MACHINE3]--;
		}
		else if ((TIM2->CR1 & 0x10) == 0x00)
		{
			ang_round_cnt[MACHINE3]++;
		}
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	
}

//void TIM8_BRK_TIM12_IRQHandler(void) //MACHIN4
//{
//	if(TIM_GetITStatus(TIM12,TIM_IT_Update))
//	{
//		if((TIM12->CR1 & 0x10) == 0x10)
//		{
//			ang_round_cnt[MACHINE4]--;
//		}
//		else if ((TIM12->CR1 & 0x10) == 0x00)
//		{
//			ang_round_cnt[MACHINE4]++;
//		}
//	}
//	TIM_ClearITPendingBit(TIM12,TIM_IT_Update);	
//}

void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8,TIM_IT_Update))
	{
		if((TIM8->CR1 & 0x10) == 0x10)
		{
			ang_round_cnt[MACHINE4]--;
		}
		else if ((TIM8->CR1 & 0x10) == 0x00)
		{
			ang_round_cnt[MACHINE4]++;
		}
	}
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);	
}

int32_t get_moto_all_angle(uint8_t machine_type)
{
	if (machine_type == MACHINE1) //ENC1_TIM3_Init
	{
		return TIM3->CNT + ang_round_cnt[machine_type] * encoder_period;
	}
	else if (machine_type == MACHINE2) //ENC2_TIM5_Init
	{
		return TIM5->CNT + ang_round_cnt[machine_type] * encoder_period;
	}
	else if (machine_type == MACHINE3) //ENC3_TIM2_Init
	{
		return TIM2->CNT + ang_round_cnt[machine_type] * encoder_period;
	}
	else if (machine_type == MACHINE4) //ENC4_TIM8_Init
	{
		return TIM8->CNT + ang_round_cnt[machine_type] * encoder_period;
	}
	else
	{		
		return 0;
	}
}
