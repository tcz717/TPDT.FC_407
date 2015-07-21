#ifndef __IIC1_H__
#define __IIC1_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

void rt_hw_i2c1_init(void);
rt_err_t i2c_register_read(struct rt_i2c_bus_device *bus,
						rt_uint16_t daddr,
						rt_uint8_t raddr,
						void *buffer,
						rt_size_t count);

rt_err_t i2c_register_write(struct rt_i2c_bus_device *bus,
						rt_uint16_t daddr,
						rt_uint8_t raddr,
						void *buffer,
						rt_size_t count);

#endif
