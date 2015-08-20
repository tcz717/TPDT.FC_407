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
static rt_uint8_t control_stack[2048];
static struct rt_thread control_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t watchdog_stack[512];
static struct rt_thread watchdog_thread;

struct rt_semaphore watchdog;

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
rt_err_t mayday(u8 var){disarm(); return RT_EOK;}

fc_task task[16]=
{
	0,"default",RT_NULL,0,SAFE_MPU6050|SAFE_SONAR|SAFE_TFCR|SAFE_CARMERA,RT_TRUE,
	1,"mayday",mayday,0,0,RT_TRUE,
	2,"stable",stable_mode,0,SAFE_MPU6050|SAFE_PWM,RT_TRUE,
	3,"althold",althold_mode,50,SAFE_MPU6050|SAFE_SONAR|SAFE_PWM,RT_TRUE,
	4,"loiter",loiter_mode,50,SAFE_ADNS3080|SAFE_MPU6050|SAFE_SONAR|SAFE_PWM,RT_TRUE,
	5,"cruise",RT_NULL,4,SAFE_MPU6050|SAFE_SONAR|SAFE_TFCR|SAFE_CARMERA,RT_TRUE,
	5,"throw",RT_NULL,3,SAFE_MPU6050|SAFE_SONAR|SAFE_TFCR|SAFE_CARMERA,RT_TRUE,
	
	254,"test",RT_NULL,0,SAFE_MPU6050|SAFE_SONAR|SAFE_TFCR|SAFE_CARMERA|SAFE_PWM,RT_TRUE,
	255,"wait",wait_mode,0,0,RT_TRUE,
};

fc_task * current_task;

rt_err_t arm(rt_int32_t addtion)
{
	rt_err_t err;
	if(armed)
		return RT_EOK;
	PID_Reset(&p_angle_pid);
	PID_Reset(&p_rate_pid);
	PID_Reset(&r_angle_pid);
	PID_Reset(&r_rate_pid);
	PID_Reset(&y_angle_pid);
	PID_Reset(&y_rate_pid);
	PID_Reset(&h_pid);
	PID_Reset(&x_d_pid);
	PID_Reset(&x_v_pid);
	PID_Reset(&y_d_pid);
	PID_Reset(&y_v_pid);
	if((err=check_safe(SAFE_MPU6050|addtion))==RT_EOK)
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
FINSH_FUNCTION_EXPORT(disarm, disarm motor);

fc_task * find_task(const char * name)
{	
	for(int j=0;j<sizeof(task)/sizeof(fc_task);j++)
	{
		if(!rt_strcasecmp(name,task[j].name))
		{
			return &task[j];
		}
	}
	return RT_NULL;
}

rt_bool_t excute_task(const char * name)
{
	fc_task * t=find_task(name);
	if(t==RT_NULL)
		return RT_FALSE;
	
	if(check_safe(t->depend))
	{
		rt_kprintf("start task %s fail %d.\n",t->name,check_safe(t->depend));
		return RT_FALSE;
	}
	current_task=t;
	current_task->reset=RT_TRUE;
	rt_kprintf("start task %s.\n",t->name);
	return RT_TRUE;
}

void stable(float pitch,float roll,float yaw)
{
	float yaw_err;
	PID_SetTarget(&p_angle_pid,pitch);
	PID_xUpdate(&p_angle_pid, ahrs.degree_pitch);
	PID_SetTarget(&p_rate_pid, -RangeValue(p_angle_pid.out, -80, 80));
	PID_xUpdate(&p_rate_pid, ahrs.gryo_pitch);

	PID_SetTarget(&r_angle_pid,roll);
	PID_xUpdate(&r_angle_pid, ahrs.degree_roll);
	PID_SetTarget(&r_rate_pid, -RangeValue(r_angle_pid.out, -80, 80));
	PID_xUpdate(&r_rate_pid, ahrs.gryo_roll);
	
	yaw_err = ahrs.degree_yaw - yaw;
	PID_SetTarget(&y_angle_pid, 0);
	if (yaw_err > 180.0f)yaw_err -= 360.0f;
	if (yaw_err < -180.0f)yaw_err += 360.0f;
	PID_xUpdate(&y_angle_pid, yaw_err);
	PID_SetTarget(&y_rate_pid, -RangeValue(y_angle_pid.out, -100, 100));
	
	PID_xUpdate(&y_rate_pid, ahrs.gryo_yaw);
}

void althold(float height)
{
	rt_uint32_t dump;
	if(check_safe(SAFE_SONAR)) 
	{ 
		h_pid.out = 0; 
		return ;
	} 
	
	if (rt_event_recv(&ahrs_event, AHRS_EVENT_SONAR, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK) 
	{ 
		PID_SetTarget(&h_pid, height); 
		PID_xUpdate(&h_pid, ahrs.height); 
		h_pid.out = RangeValue(h_pid.out, -200, 200); 
	} 
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
	stable(+RangeValue(y_v_pid.out, -10, 10) ,-RangeValue(x_v_pid.out, -10, 10),yaw);
}

void motor_update(u16 th)
{
	Motor_Set1(th - p_rate_pid.out - r_rate_pid.out + y_rate_pid.out);
	Motor_Set2(th - p_rate_pid.out + r_rate_pid.out - y_rate_pid.out);
	Motor_Set3(th + p_rate_pid.out - r_rate_pid.out - y_rate_pid.out);
	Motor_Set4(th + p_rate_pid.out + r_rate_pid.out + y_rate_pid.out);
}
void motor_hupdate(u16 th)
{
	float weight;
	weight=th/400.0f;
	weight=RangeValue(weight,0,1);
	
	Motor_Set1(th - p_rate_pid.out - r_rate_pid.out + y_rate_pid.out - weight*h_pid.out);
	Motor_Set2(th - p_rate_pid.out + r_rate_pid.out - y_rate_pid.out - weight*h_pid.out);
	Motor_Set3(th + p_rate_pid.out - r_rate_pid.out - y_rate_pid.out - weight*h_pid.out);
	Motor_Set4(th + p_rate_pid.out + r_rate_pid.out + y_rate_pid.out - weight*h_pid.out);
}

rt_err_t stable_mode(u8 var)
{
	if (pwm.throttle  > 0.05f && abs(ahrs.degree_pitch) < 40 && abs(ahrs.degree_roll) < 40)
	{
		yaw_exp+=pwm.yaw*0.5f;
		if (yaw_exp > 360.0f)yaw_exp -= 360.0f;
		if (yaw_exp < 0.0f)yaw_exp += 360.0f;
		
		stable(pwm.pitch*30.0f,pwm.roll*30.0f,yaw_exp);
		
		motor_update(pwm.throttle * 1000);
	}
	else
		Motor_Set(60, 60, 60, 60);
	return RT_EOK;
}

rt_err_t althold_mode(u8 height)
{
	const u16 basic_thought=530;
	if (pwm.throttle  > 0.3f && abs(ahrs.degree_pitch) < 40 && abs(ahrs.degree_roll) < 40)
	{
		yaw_exp+=pwm.yaw*0.5f;
		if (yaw_exp > 360.0f)yaw_exp -= 360.0f;
		if (yaw_exp < 0.0f)yaw_exp += 360.0f;
		
		stable(pwm.pitch*30.0f,pwm.roll*30.0f,yaw_exp);
		
		althold(height);

		motor_hupdate(basic_thought);
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

		motor_hupdate(basic_thought);
	}
	else
		Motor_Set(60, 60, 60, 60);
	return RT_EOK;
}

void wait_dmp()
{
	u8 i;
	for (i = 0;i < 15;i++)
	{
		u8 j;
		for (j = 0;j < 200;j++)
		{
			get_dmp();
			rt_sem_release(&watchdog);
			rt_thread_delay(2);
		}
	}
}

float linear(float input,float start ,float end ,float time)
{
	float out;
	out=input+(end-start)/time;
	if(start>end)
		return RangeValue(out,end,start);
	else
		return RangeValue(out,start,end);
}

void control_thread_entry(void* parameter)
{
	LED2(5);

	wait_dmp();
	rt_kprintf("start control\n");
	excute_task("wait");

	while (1)
	{
		LED2(armed * 3);
		
		receive_pwm(&pwm);
		
		if(pwm.switch1<2 && pwm.switch1!=-1 && pwm.throttle<0.05f && pwm.throttle >= 0.0f)
		{
			arm(SAFE_PWM);
		}
		if((((pwm.switch1==2||pwm.switch1==-1) 
			&&(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==Bit_RESET||GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)==Bit_RESET)
			)||check_safe(current_task->depend))
			)
		{
			disarm();
		}
		
		if (get_dmp() && armed)
		{
			if(check_safe(SAFE_PWM)==RT_EOK&&check_safe(SAFE_TFCR)!=RT_EOK)
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
			
			if(current_task->func(current_task->var)!=RT_EOK)
				disarm();
		}
		if(!armed)
			Motor_Set(0,0,0,0);
		rt_sem_release(&watchdog);
		rt_thread_delay(2);
	}
}


void watchdog_entry(void* parameter)
{
	while(1)
	{
		if(rt_sem_take(&watchdog,RT_TICK_PER_SECOND)!=RT_EOK)
		{
			rt_kprintf("watchdog timeout!!!!!.\n");
			while(1)
			{
				Motor_Set(0,0,0,0);
				disarm();
				rt_thread_suspend(&control_thread);
			}
		}
		if (armed && (abs(ahrs.degree_pitch) > 45.0f || abs(ahrs.degree_roll) > 45.0f))
		{
			Motor_Set(0,0,0,0);
			disarm();
			rt_kprintf("degree out of range.\n");
		}
	}
}

rt_err_t hardfalt_protect(void * stack)
{
	Motor_Set(0,0,0,0);
	disarm();
	return RT_EOK;
}
void assert_protect(const char * c1,const char * c2,rt_size_t size)
{
	Motor_Set(0,0,0,0);
	disarm();
}

void control_init()
{
	GPIO_InitTypeDef gpio_init;
	
	GPIO_StructInit(&gpio_init);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	gpio_init.GPIO_Mode=GPIO_Mode_IN;
	gpio_init.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_4;
	gpio_init.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE,&gpio_init);
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
	PID_Set_Filt_Alpha(&y_angle_pid, 1.0 / 166.0, 20.0);
	PID_Set_Filt_Alpha(&x_v_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&y_v_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&x_d_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&y_d_pid, 1.0 / 100.0, 20.0);
	PID_Set_Filt_Alpha(&h_pid, 1.0 / 60.0, 20.0);
	
	h_pid.maxi=300;
	
	rt_hw_exception_install(hardfalt_protect);
	rt_assert_set_hook(assert_protect);
	
	rt_sem_init(&watchdog,"watchdog",0,RT_IPC_FLAG_FIFO);
	rt_thread_init(&control_thread,
		"control",
		control_thread_entry,
		RT_NULL,
		control_stack,
		2048, 3, 5);
	rt_thread_startup(&control_thread);
//	
	rt_thread_init(&watchdog_thread,
		"watchdog",
		watchdog_entry,
		RT_NULL,
		watchdog_stack,
		512, 1, 1);
	rt_thread_startup(&watchdog_thread);
	
	extern void line_register(void);
	line_register();
}
