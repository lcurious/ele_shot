#include "gpio.h"
/**
PD2   -->control0
PD3   -->control1
*/

void control_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; //����ṹ�����	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //ģ������ģʽ
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;//�ܽ�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ���ṹ��	
	
	con_charge = 0;
	con_shot   = 0;
	
}
