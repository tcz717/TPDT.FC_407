#ifndef __CONTROL_H__
#define __CONTROL_H__
#include <rtthread.h>
#include "stm32f4xx.h"

#define tPre	static int state=0; 
#define tReset state=0; 
#define tBegin switch(state) { case 0:
#define tReturn(x) do { state=__LINE__; return x; \
                         case __LINE__:; } while (0)
#define tFinish }

typedef rt_err_t (*fc_func)(uint8_t var);

typedef struct
{
	rt_uint8_t id;
	const char *name;
	fc_func func;
	rt_uint8_t var;
	uint32_t depend;
	rt_bool_t reset;
}fc_task;

extern rt_bool_t armed;
extern fc_task * current_task;

void control_init(void);
rt_err_t arm(rt_int32_t addtion);
rt_err_t disarm(void);

void stable(float pitch,float roll,float yaw);
void althold(float height);
void loiter(float x,float y,float yaw);

void motor_update(u16 th);
void motor_hupdate(u16 th);

float linear(float input,float start ,float end ,float time);

fc_task * find_task(const char * name);
rt_bool_t excute_task(const char * name);

#endif
