#ifndef __SAFE_H__
#define __SAFE_H__

#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "ahrs.h"

#define SAFE_MPU6050	AHRS_EVENT_MPU6050
#define SAFE_HMC5883	AHRS_EVENT_HMC5883
#define SAFE_ADNS3080	AHRS_EVENT_ADNS3080
#define SAFE_SONAR		AHRS_EVENT_SONAR
#define SAFE_CARMERA	AHRS_EVENT_CARMERA

#define SAFE_PWM		(AHRS_EVENT_CARMERA << 1)
#define SAFE_TFCR		(AHRS_EVENT_CARMERA << 2)

rt_err_t check_safe(rt_uint32_t checklist);

#endif
