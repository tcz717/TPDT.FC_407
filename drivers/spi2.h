#ifndef __SPI2_H__
#define __SPI2_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>


struct spi_cs
{
	void (*spi_select)(rt_uint8_t);
};

void rt_hw_spi2_init(void);
void spi_flash_init(void);

#endif
