
#ifndef _MOTOR
#define _MOTOR
#include "PID.h"
#include "PWM.h"
void Motor_Init(void);
void Motor_Set(u16,u16,u16,u16);
void Motor_Set1(u16);
void Motor_Set2(u16);
void Motor_Set3(u16);
void Motor_Set4(u16);

extern u16 Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;

//void dc_motor_set1(s16 m);
//void dc_motor_set2(s16 m);
//void dc_motor_set(s16,s16);
#endif
