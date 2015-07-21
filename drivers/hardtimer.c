#include "hardtimer.h"

void Timer4_init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 0xffff; 
	TIM_TimeBaseStructure.TIM_Prescaler =83; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 
	TIM_Cmd(TIM2, ENABLE); 
}
double Timer4_GetSec()
{
	double t=TIM_GetCounter(TIM2)/1000000.0;
	TIM_SetCounter(TIM2,0);
	return t;
}
