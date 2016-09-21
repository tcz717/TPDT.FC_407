#include "led.h"
#include <rtthread.h>

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[256];
static struct rt_thread led_thread;

u8 led_period[4];
void led_thread_entry(void* parameter)
{
	u8 time[4];
	while (1)
	{
		if (led_period[0])
			time[0] = (time[0] + 1) % led_period[0];
		else
			time[0] = 1;
		LED_set1(!time[0]);

		if (led_period[1])
			time[1] = (time[1] + 1) % led_period[1];
		else
			time[1] = 1;
		LED_set2(!time[1]);

		if (led_period[2])
			time[2] = (time[2] + 1) % led_period[2];
		else
			time[2] = 1;
		LED_set3(!time[2]);

		if (led_period[3])
			time[3] = (time[3] + 1) % led_period[3];
		else
			time[3] = 1;
		LED_set4(!time[3]);
		rt_thread_delay(50);
	}
}

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
	
	rt_thread_init(&led_thread,
		"led",
		led_thread_entry,
		RT_NULL,
		led_stack,
		256, 16, 1);
	rt_thread_startup(&led_thread);
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
