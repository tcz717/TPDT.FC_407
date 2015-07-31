#ifndef __STM32_SPI_H__
#define __STM32_SPI_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f4xx.h>

struct spi_cs
{
	void (*spi_select)(rt_uint8_t);
};
struct stm32_spi
{
	struct rt_spi_ops parent;
	SPI_TypeDef * spi;
};

void rt_hw_spi2_init(void);
void rt_hw_spi3_init(void);
void spi_flash_init(void);

#endif
