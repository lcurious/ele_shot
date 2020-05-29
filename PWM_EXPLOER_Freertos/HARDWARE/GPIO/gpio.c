#include "gpio.h"
/**
PD2   -->control0
PD3   -->control1
*/

void control_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; //定义结构体变量	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //模拟输入模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;//管脚设置
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//浮空
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化结构体	
	
	con_charge = 0;
	con_shot   = 0;
	
}
