#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "stm32f4xx.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "tfcr.h"
rt_err_t remote_init(const char * uart_name);

#endif
