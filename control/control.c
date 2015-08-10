#include "PID.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include "safe.h"
#include <components.h>
#include "LED.h"
#include "settings.h"
#include "pwm_remote.h"
#include "Motor.h"
#include "control.h"

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t control_stack[1024];
static struct rt_thread control_thread;

u8 poscon = 0;
PID p_rate_pid, r_rate_pid, y_rate_pid,
p_angle_pid, r_angle_pid, y_angle_pid;
PID x_v_pid, y_v_pid,
x_d_pid, y_d_pid;
PID	h_pid;
s16 pos_X, pos_y;

static float yaw_exp;

rt_bool_t armed=RT_FALSE;

pwm_signal_t pwm;

FINSH_VAR_EXPORT(armed, finsh_type_uchar, armed state)

u8 get_dmp(void);
rt_err_t stable_mode(u8 var);
rt_err_t althold_mode(u8 var);
rt_err_t loiter_mode(u8 var);
rt_err_t wait_mode(u8 var){Motor_Set(0,0,0,0); return RT_EOK;}

fc_task task[16]=
{
	0,"default",RT_NULL,0,
	1,"mayday",RT_NULL,0,
	2,"stable",stable_mode,0,
	3,"althold",althold_mode,50,
	4,"loiter",loiter_mode,50,
	255,"wait",wait_mode,0,
};

fc_task * current_task;

rt_err_t arm(rt_int32_t addtion)
{
	rt_err_t err;
	if(armed)
		return RT_EOK;
	if((err=check_safe(SAFE_MPU6050|SAFE_HMC5883|addtion))==RT_EOK)
	{
		yaw_exp = ahrs.degree_yaw;
		armed=RT_TRUE;
		rt_kprintf("armed.\n");
		return RT_EOK;
	}
	else
	{
		rt_kprintf("pre armed fail: 0x%02X.\n",err);
		return err;
	}
}

rt_err_t disarm()
{
	Motor_Set(0, 0, 0, 0);
	if(!armed)
		return RT_EOK;
	armed=RT_FALSE;
	rt_kprintf("disarmed.\n");
	excute_task("wait");
	return RT_EOK;
}

rt_bool_t excute_task(const char * name)
{
	for(int j=0;j<sizeof(task);j++)
	{
		if(!rt_strcasecmp(name,task[j].name))
		{
			current_task=&task[j];
			rt_kprintf("start task %s.\n",task[j].name);
			return RT_TRUE;
		}
	}
	return RT_FALSE;
}

void stable(float pitch,float roll,float yaw)
{
	float yaw_err;
	rt_uint32_t dump;
	PID_SetTarget(&p_angle_pid,pitch);
	PID_xUpdate(&p_angle_pid, ahrs.degree_pitch);
	PID_SetTarget(&p_rate_pid, -RangeValue(p_angle_pid.out, -80, 80));
	PID_xUpdate(&p_rate_pid, ahrs.gryo_pitch);

	PID_SetTarget(&r_angle_pid,roll);
	PID_xUpdate(&r_angle_pid, ahrs.degree_roll);
	PID_SetTarget(&r_rate_pid, -RangeValue(r_angle_pid.out, -80, 80));
	PID_xUpdate(&r_rate_pid, ahrs.gryo_roll);
	
	if (rt_event_recv(&ahrs_event, AHRS_EVENT_HMC5883, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
	{
		yaw_err = ahrs.degree_yaw - yaw;
		PID_SetTarget(&y_angle_pid, 0);
		if (yaw_err > 180.0f)yaw_err -= 360.0f;
		if (yaw_err < -180.0f)yaw_err += 360.0f;
		PID_xUpdate(&y_angle_pid, yaw_err);
		PID_SetTarget(&y_rate_pid, -RangeValue(y_angle_pid.out, -100, 100));
	}
	PID_xUpdate(&y_rate_pid, ahrs.gryo_yaw);
}

void althold(float height)
{
	rt_uint32_t dump;
	if(check_safe(SAFE_SONAR)) 
	{ 
		h_pid.out = 0; 
		LED4(0); 
		return ;
	} 
	
	if (rt_event_recv(&ahrs_event, AHRS_EVENT_SONAR, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK) 
	{ 
		PID_SetTarget(&h_pid, height); 
		PID_xUpdate(&h_pid, ahrs.height); 
		h_pid.out = RangeValue(h_pid.out, -200, 200); 
	} 
	LED4(4); 
}

void loiter(float x,float y,float yaw)
{
	rt_uint32_t dump;
	if(check_safe(SAFE_ADNS3080))
	{
		x_v_pid.out=0;
		y_v_pid.out=0;
		return;
	}
	if (rt_event_recv(&ahrs_event, AHRS_EVENT_ADNS3080, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
	{
		PID_SetTarget(&x_d_pid, pos_X);
		PID_SetTarget(&y_d_pid, pos_y);

		PID_xUpdate(&x_d_pid, ahrs.x);
		PID_xUpdate(&y_d_pid, ahrs.y);

		PID_SetTarget(&x_v_pid, -RangeValue(x_d_pid.out, -10, 10));
		PID_SetTarget(&y_v_pid, -RangeValue(y_d_pid.out, -10, 10));

		PID_xUpdate(&x_v_pid, ahrs.dx);
		PID_xUpdate(&y_v_pid, ahrs.dy);
	}
	LED4(2);
	stable(+RangeValue(y_v_pid.out, -10, 10) ,-RangeValue(x_v_pid.out, -10, 10),yaw);
}

rt_err_t stable_mode(u8 var)
{
	if (pwm.throttle  > 0.05f && abs(ahrs.degree_pitch) < 40 && abs(ahrs.degree_roll) < 40)
	{
		yaw_exp+=pwm.yaw*0.5f;
		
		stable(pwm.pitch*30.0f,pwm.roll*30.0f,yaw_exp);
		
		Motor_Set1(pwm.throttle * 1000 - p_rate_pid.out - r_rate_pid.out + y_rate_pid.out);
		Motor_Set2(pwm.throttle * 1000 - p_rate_pid.out + r_rate_pid.out - y_rate_pid.out);
		Motor_Set3(pwm.throttle * 1000 + p_rate_pid.out - r_rate_pid.out - y_rate_pid.out);
		Motor_Set4(pwm.throttle * 1000 + p_rate_pid.out + r_rate_pid.out + y_rate_pid.out);
	}
	else
		Motor_Set(60, 60, 60, 60);
	return RT_EOK;
}

rt_err_t althold_mode(u8 height)
{
	const u16 basic_thought=450;
	if (pwm.throttle  > 0.3f && abs(ahrs.degree_pitch) < 40 && abs(ahrs.degree_roll) < 40)
	{
		yaw_exp+=pwm.yaw*0.5f;
		
		stable(pwm.pitch*30.0f,pwm.roll*30.0f,yaw_exp);
		
		althold(height);

		Motor_Set1(basic_thought - p_rate_pid.out - r_rate_pid.out + y_rate_pid.out - h_pid.out);
		Motor_Set2(basic_thought - p_rate_pid.out + r_rate_pid.out - y_rate_pid.out - h_pid.out);
		Motor_Set3(basic_thought + p_rate_pid.out - r_rate_pid.out - y_rate_pid.out - h_pid.out);
		Motor_Set4(basic_thought + p_rate_pid.out + r_rate_pid.out + y_rate_pid.out - h_pid.out);
	}
	else
		Motor_Set(60, 60, 60, 60);
	return RT_EOK;
}

rt_err_t loiter_mode(u8 height)
{
	const u16 basic_thought=450;
	if(check_safe(SAFE_ADNS3080))
		return RT_EIO;
	if (pwm.throttle  > 0.3f && abs(ahrs.degree_pitch) < 40 && abs(ahrs.degree_roll) < 40)
	{
		loiter(pos_X,pos_y,yaw_exp);
		
		althold(height);

		Motor_Set1(basic_thought - p_rate_pid.out - r_rate_pid.out + y_rate_pid.out - h_pid.out);
		Motor_Set2(basic_thought - p_rate_pid.out + r_rate_pid.out - y_rate_pid.out - h_pid.out);
		Motor_Set3(basic_thought + p_rate_pid.out - r_rate_pid.out - y_rate_pid.out - h_pid.out);
		Motor_Set4(basic_thought + p_rate_pid.out + r_rate_pid.out + y_rate_pid.out - h_pid.out);
	}
	else
		Motor_Set(60, 60, 60, 60);
	return RT_EOK;
}

void control_thread_entry(void* parameter)
{
	u8 i;

	LED2(5);
	for (i = 0;i < 15;i++)
	{
		u8 j;
		for (j = 0;j < 200;j++)
		{
			get_dmp();
			rt_thread_delay(2);
		}
	}

	rt_kprintf("start control\n");
	excute_task("wait");

	while (1)
	{
		LED2(armed * 3);
		
		receive_pwm(&pwm);
		
		if(pwm.switch1<2 && pwm.switch1!=-1 && pwm.throttle<0.05f && pwm.throttle > 0.0f)
		{
			arm(SAFE_PWM);
		}
		if(pwm.switch1==2||pwm.switch1==-1||check_safe(SAFE_MPU6050))
		{
			disarm();
		}

		if (get_dmp() && armed)
		{
			if(check_safe(SAFE_PWM)==RT_EOK)
			{
				if(current_task->id != 2 && pwm.switch1 == 1)
				{
					excute_task("stable");
				}
				if(current_task->id != 3 && pwm.switch1 == 0 && pwm.switch2 == 0)
				{
					excute_task("althold");
				}
				if(current_task->id != 4 && pwm.switch1 == 0 && pwm.switch2 == 1)
				{
					pos_X=ahrs.x;
					pos_y=ahrs.y;
					excute_task("loiter");
				}
			}
			
			current_task->func(current_task->var);
		}

		extern u16 dmp_retry;

		rt_thread_delay(2);
	}
}

void control_init()
{
	//default settings
	PID_Init(&p_rate_pid, 0, 0, 0);
	PID_Init(&r_rate_pid, 0, 0, 0);
	PID_Init(&y_rate_pid, 0, 0, 0);
	PID_Init(&p_angle_pid, 0, 0, 0);
	PID_Init(&r_angle_pid, 0, 0, 0);
	PID_Init(&y_angle_pid, 0, 0, 0);
	PID_Init(&x_v_pid, 0, 0, 0);
	PID_Init(&y_v_pid, 0, 0, 0);
	PID_Init(&x_d_pid, 0, 0, 0);
	PID_Init(&y_d_pid, 0, 0, 0);
	PID_Init(&h_pid, 0, 0, 0);

	load_settings(&settings, "/setting", &p_angle_pid, &p_rate_pid
		, &r_angle_pid, &r_rate_pid
		, &y_angle_pid, &y_rate_pid
		, &x_d_pid, &x_v_pid
		, &y_d_pid, &y_v_pid
		, &h_pid);
	
	settings.roll_min = settings.pitch_min = settings.yaw_min = 1017;
	settings.th_min = 1017;
	settings.roll_max = settings.pitch_max = settings.yaw_max = 2021;
	settings.th_max = 2021;
	settings.roll_mid = settings.pitch_mid = settings.yaw_mid = 1519;
	
	get_pid();
	PID_Set_Filt_Alpha(&p_rate_pid, 1.0 / 166.0, 20.0);
	PID_Set_Filt_Alpha(&r_rate_pid, 1.0 / 166.0, 20.0);
	PID_Set_Filt_Alpha(&y_rate_pid, 1.0 / 166.0, 20.0);
	PID_Set_Filt_Alpha(&p_angle_pid, 1.0 / 166.0, 20.0);
	PID_Set_Filt_Alpha(&r_angle_pid, 1.0 / 166.0, 20.0);
	PID_Set_Filt_Alpha(&y_angle_pid, 1.0 / 75.0, 20.0);
	PID_Set_Filt_Alpha(&x_v_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&y_v_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&x_d_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&y_d_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&h_pid, 1.0 / 60.0, 20.0);
	
	rt_thread_init(&control_thread,
		"control",
		control_thread_entry,
		RT_NULL,
		control_stack,
		1024, 3, 5);
	rt_thread_startup(&control_thread);
}
