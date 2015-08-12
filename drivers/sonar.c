#include "sonar.h"
#include <board.h>
#include <rtthread.h>
#include <components.h>
#include "ahrs.h"
#include "PWM.h"

#define SAMPLE_COUNT 8

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t sonar_stack[ 512 ];
static struct rt_thread sonar_thread;

s16 sonar_avr[SAMPLE_COUNT]={0};
u8 sonar_state=0;

float sonar_h;
static u16 h;
FINSH_VAR_EXPORT(h,finsh_type_uint,sonar height);

void sonar_thread_entry(void* parameter)
{
	rt_kprintf("start sonar\n");
	while(1)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
		
		rt_thread_delay(RT_TICK_PER_SECOND*10/1000);
		
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		
		rt_thread_delay(RT_TICK_PER_SECOND*50/1000);
		
		sonar_h=MoveAve_WMA(PWM8_Time,sonar_avr,SAMPLE_COUNT)/58.0f;
		ahrs.height=sonar_h;
		h=(u16)sonar_h;
		sonar_state=sonar_h>3.0f&&sonar_h<150.0f;
		ahrs_state.sonar=!sonar_state;
		
		rt_event_send(&ahrs_event,AHRS_EVENT_SONAR);
	}
}

void sonar_init()
{
	GPIO_InitTypeDef gpio_init;
	GPIO_StructInit(&gpio_init);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Pin=GPIO_Pin_7;
	GPIO_Init(GPIOE,&gpio_init);
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	
	rt_thread_init(&sonar_thread,
					"sonar",
					sonar_thread_entry,
					RT_NULL,
                    sonar_stack,
					512, 7, 2);
    rt_thread_startup(&sonar_thread);
}
