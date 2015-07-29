#ifndef __HMC5883_H__
#define __HMC5883_H__
#include <stm32f4xx.h>
#include <board.h>
#include <rtthread.h>
#include <components.h>

rt_err_t HMC5983_Init(void); 
extern rt_bool_t has_hmc5883;
extern float mag_angle;
extern struct rt_semaphore hmc5883_sem;

#endif 



