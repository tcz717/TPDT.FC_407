#include "LED.h"
#include <rtthread.h>

void LED_init(void)
{
	GPIO_InitTypeDef  initdef;
	
//	PWR_BackupAccessCmd(ENABLE);
//	RCC_LSICmd(DISABLE);
//	BKP_TamperPinCmd(DISABLE);
	
	initdef.GPIO_Mode=GPIO_Mode_OUT;
	initdef.GPIO_Speed=GPIO_Speed_50MHz;
	initdef.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	GPIO_Init(GPIOE,&initdef);
	
	GPIO_SetBits(GPIOE,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}
void LED_set1(uint16_t state)
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_12,state?Bit_RESET:Bit_SET);
}
void LED_set2(uint16_t state)
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_13,state?Bit_RESET:Bit_SET);
}
void LED_set3(uint16_t state)
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_14,state?Bit_RESET:Bit_SET);
}
void LED_set4(uint16_t state)
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_15,state?Bit_RESET:Bit_SET);
}
