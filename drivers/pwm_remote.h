#ifndef __PWM_REMOTE_H__
#define __PWM_REMOTE_H__
#include <rtthread.h>
#include "stm32f4xx.h"
typedef struct
{
	float throttle;
	float pitch;
	float roll;
	float yaw;
	
	int8_t switch1;
	int8_t switch2;
	
	float custom;
}pwm_signal_t;

void receive_pwm(pwm_signal_t *);

#endif
