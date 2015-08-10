#ifndef __CONTROL_H__
#define __CONTROL_H__
#include <rtthread.h>
#include "stm32f4xx.h"

typedef rt_err_t (*fc_func)(uint8_t var);

typedef struct
{
	rt_uint8_t id;
	const char *name;
	fc_func func;
	rt_uint8_t var;
}fc_task;

extern rt_bool_t armed;
void control_init(void);
rt_bool_t excute_task(const char * name);

#endif
