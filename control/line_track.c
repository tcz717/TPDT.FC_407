#include "control.h"
#include "camera.h"
#include "PID.h"
#include <dfs_posix.h>
#include <finsh.h>
#include "pwm_remote.h"
extern pwm_signal_t pwm;

#define PID_SS 0xABCD
#define PID_ES 0xDCBA

#define PID_PATH "/lt.pid"

fc_task * line_task;

struct line_pid
{
	uint16_t SS;
	PID dist;
	PID angle;
	uint16_t ES;
}pid;

rt_err_t line_track(u8 var)
{
	static float h;
	static float yaw;
	static uint8_t waitl;
	
	tPre;
	if(line_task->reset)
	{
		tReset;
		line_task->reset=RT_FALSE;
	}
	tBegin;
	yaw=ahrs.degree_yaw;
	h=0;
	waitl=1;
	while(h<49.0f)//take off
	{
		h=linear(h,5,50,RT_TICK_PER_SECOND/2);
		stable(0,0,yaw);
		
		althold(50);
		
		motor_hupdate(400+(u16)h);
		
//		rt_kprintf("%d/%d\n",(u8)ahrs.height,(u8)h);
		
		tReturn(RT_EOK);
	}
	rt_kprintf("start line track.\n");
	rt_uint32_t dump;
	do
	{
		if (rt_event_recv(&ahrs_event, AHRS_EVENT_CARMERA, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
		{
			if(recv.pack.linestate==LINE_STRAIGHT)
				waitl=0;
			else
				waitl++;
			if(waitl>60)
				goto land;
			
		}
		stable(0,0,yaw);
		
		althold(50);
	}while(waitl>0);
	rt_kprintf("find line.\n");
	while(1) //line track
	{
		if (rt_event_recv(&ahrs_event, AHRS_EVENT_CARMERA, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
		{
			if(recv.pack.linestate==LINE_STRAIGHT)
			{
				PID_SetTarget(&pid.dist,0);
				PID_xUpdate(&pid.dist,ahrs.line_err);
			}
			
			rt_kprintf("e:%d\to:%d\th:%d\ta:%d\ty:%d\n",(s16)ahrs.line_err,(s16)pid.dist.out,(s16)ahrs.height,(s16)recv.pack.angle_error,(s16)ahrs.degree_yaw);
			
			if(recv.pack.linestate==LINE_LOST_ERROR)
				goto land;
		}
//		float y=yaw+ahrs.angle_err;
//		if (y > 360.0f)y -= 360.0f;
//		if (y < 0.0f)y += 360.0f;
		stable(pwm.pitch*10.0f,RangeValue(pid.dist.out,-5,5),yaw);
		althold(50);
		motor_hupdate(450);
		
		tReturn(RT_EOK);
	}
land:
	rt_kprintf("land.\n");
	h=450;
	while(h>10.0f)//take off
	{
		h=linear(h,450,0,RT_TICK_PER_SECOND);
		stable(0,0,yaw);
		
		motor_update((u16)h);
		
		tReturn(RT_EOK);
	}
	tReturn(1);
	tFinish
	return RT_EOK;
}

static void init_pid()
{
	int fd;
	fd = open(PID_PATH, O_RDWR | O_CREAT, 0);

	if (fd >= 0)
	{
		if (read(fd, &pid, sizeof(pid)) != sizeof(pid) ||
			pid.SS != PID_SS || pid.ES != PID_ES)
		{
			rt_kprintf("init line track pid.\n");
			pid.SS = PID_SS;
			pid.ES = PID_ES;
			
			PID_Init(&pid.angle,0,0,0);
			PID_Init(&pid.dist,0,0,0);
			
			PID_Set_Filt_Alpha(&pid.angle,1.0f/60.0f,20.0);
			PID_Set_Filt_Alpha(&pid.dist,1.0f/60.0f,20.0);

			write(fd, &pid, sizeof(pid));
		}
		else
		{
			PID_Reset(&pid.angle);
			PID_Reset(&pid.dist);
			PID_Set_Filt_Alpha(&pid.angle,1.0f/60.0f,20.0);
			PID_Set_Filt_Alpha(&pid.dist,1.0f/60.0f,20.0);
			rt_kprintf("line track pid load succeed.\n");
		}
		close(fd);
	}
	else
	{
		rt_kprintf("line track open wrong.\n");
	}
	rt_kprintf("line angle:		%d.%d	%d.%02d	%d.%03d.\n", (s32)pid.angle.p, (s32)(pid.angle.p*10.0f) % 10,
	(s32)pid.angle.i, (s32)(pid.angle.i*100.0f) % 100,
	(s32)pid.angle.d, (s32)(pid.angle.d*1000.0f) % 1000);
	rt_kprintf("line dist :		%d.%02d	%d.%02d	%d.%03d.\n", (s32)pid.dist.p, (s32)(pid.dist.p*100.0f) % 100,
	(s32)pid.dist.i, (s32)(pid.dist.i*100.0f) % 100,
	(s32)pid.dist.d, (s32)(pid.dist.d*1000.0f) % 1000);
}

static void save_pid()
{
	int fd;

	fd = open(PID_PATH, O_WRONLY | O_TRUNC, 0);

	if (fd >= 0)
	{
		write(fd, &pid, sizeof(pid));
		close(fd);
	}
	rt_kprintf("line angle:		%d.%d	%d.%02d	%d.%03d.\n", (s32)pid.angle.p, (s32)(pid.angle.p*10.0f) % 10,
	(s32)pid.angle.i, (s32)(pid.angle.i*100.0f) % 100,
	(s32)pid.angle.d, (s32)(pid.angle.d*1000.0f) % 1000);
	rt_kprintf("line dist :		%d.%02d	%d.%02d	%d.%03d.\n", (s32)pid.dist.p, (s32)(pid.dist.p*100.0f) % 100,
	(s32)pid.dist.i, (s32)(pid.dist.i*100.0f) % 100,
	(s32)pid.dist.d, (s32)(pid.dist.d*1000.0f) % 1000);
}

void set_la(s16 p, s16 i, s16 d)
{
	PID_Init(&pid.angle, p / 10.0f, i / 100.0f, d / 1000.0f);
	save_pid();
}
FINSH_FUNCTION_EXPORT(set_la, set the value of pid in line track angle)

void set_ld(s16 p, s16 i, s16 d)
{
	PID_Init(&pid.dist, p / 100.0f, i / 100.0f, d / 1000.0f);
	save_pid();
}
FINSH_FUNCTION_EXPORT(set_ld, set the value of pid in line track dist)

void line_register()
{
	line_task=find_task("default");
	line_task->func=line_track;
	
	init_pid();
}
