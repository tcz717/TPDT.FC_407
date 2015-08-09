#ifndef __SAFE_H__
#define __SAFE_H__

#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "ahrs.h"
uint8_t check_safe(void);

#endif
