#ifndef __BT_H
#define __BT_H
#include <stm32f10x.h>
#include <rthw.h>
#include <rtthread.h>

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))
	  	
void rt_hw_bluetooth_init(void);
#endif


