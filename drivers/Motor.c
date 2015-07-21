#include "Motor.h"
#define MAXOUT 1000
#define MINOUT 1000
u16 Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
void Motor_Init(void)
{
	PWMOUT1_Init();
	PWMIN1_Init();
	PWMIN2_Init();
	
//	dc_motor_set(0,0);
	Motor_Set(0,0,0,0);
}
void Motor_Set(u16 m1,u16 m2,u16 m3,u16 m4)
{
	Motor_Set1(m1);
	Motor_Set2(m2);
	Motor_Set3(m3);
	Motor_Set4(m4);
}
void Motor_Set1(u16 m)
{
	Motor3=MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000;
	TIM_SetCompare1(TIM4,Motor3);
}
void Motor_Set2(u16 m)
{
	Motor4=MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000;
	TIM_SetCompare2(TIM4,Motor4);
}
void Motor_Set3(u16 m)
{
	Motor5=MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000;
	TIM_SetCompare3(TIM4,Motor5);
}
void Motor_Set4(u16 m)
{
	Motor6=MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000;
	TIM_SetCompare4(TIM4,Motor6);
}

void dc_motor_set1(s16 m)
{
	if(m>0)
	{
		TIM_SetCompare1(TIM4,RangeValue(m,0,1000));
		TIM_SetCompare3(TIM4,0);
	}
	else
	{
		TIM_SetCompare1(TIM4,0);
		TIM_SetCompare3(TIM4,RangeValue(-m,0,500));
	}
}
void dc_motor_set2(s16 m)
{
	if(m>0)
	{
		TIM_SetCompare2(TIM4,RangeValue(m,0,1000));
		TIM_SetCompare4(TIM4,0);
	}
	else
	{
		TIM_SetCompare2(TIM4,0);
		TIM_SetCompare4(TIM4,RangeValue(-m,0,500));
	}
}
void dc_motor_set(s16 m1,s16 m2)
{
	dc_motor_set1(m1);
	dc_motor_set2(m2);
}

