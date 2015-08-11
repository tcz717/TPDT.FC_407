#include "control.h"
#include "camera.h"

fc_task * line_task;

struct line_pid
{
	
}pid;

rt_err_t line_track(u8 var)
{
	static float h;
	static float yaw;
	crBegin
	yaw=ahrs.degree_yaw;
	h=0;
	while(1)
	{
		h=linear(h,5,50,RT_TICK_PER_SECOND);
		stable(0,0,yaw);
		
		althold(h);
		
		motor_hupdate(400+h);
		
		crReturn(RT_EOK);
	}
	crFinish
	return RT_EOK;
}

void line_register()
{
	line_task=find_task("default");
	line_task->func=line_track;
}
