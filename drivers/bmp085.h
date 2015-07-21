//bmp085
#ifndef __BMP085_H__
#define __BMP085_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

struct bmp085_data
{
	rt_uint32_t temperature;
	rt_uint32_t  pressure;
	double height;
};

rt_err_t bmp085_init(const char * i2c_bus_device_name);
rt_err_t bmp085_read(struct bmp085_data *data);

#endif
