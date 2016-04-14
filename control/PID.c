#include "PID.h"
#define PI 3.1415926

void PID_SetTarget(PID* pid,float value)
{
	pid->expect = value;
}

void PID_Reset(PID* pid)
{
	pid->out = 0;
	pid->iv = 0;
	pid->dv=0;
	pid->outp=0;
	pid->outi=0;
	pid->outd=0;
}
float PID_Update(PID* pid,float value, float dv)
{
	float p, i, d;
	pid->iv += (value - pid->expect);
	pid->iv=RangeValue(pid->iv,-360,+360);
	
	p = (value - pid->expect) * pid->p;
	i = pid->iv * pid->i;
	d = dv * pid->d;
	pid->outp=p;
	pid->outi=RangeValue(i,-100,+100);
	pid->outd=RangeValue(d,-500,+500);
	pid->out =RangeValue(p + i + d,-800,+800);
	pid->input=value;
	if (pid->out<1&&pid->out>-1)
		pid->out=0;
	return pid->out;
}
extern struct ahrs_t ahrs;
float PID_xUpdate(PID* pid,float value)
{
	float dv= (value - pid->input) / pid->dt;
	pid->dv= pid->dv + pid->filt_alpha * (dv-pid->dv);
	
	pid->outp = (value - pid->expect) * pid->p;
	pid->outi += (value - pid->expect)*pid->i*pid->dt;;
	pid->outd = pid->dv * pid->d;

	pid->outp=pid->outp;
	pid->outi=RangeValue(pid->outi,-50,+50);
	pid->outd=RangeValue(pid->outd,-500,+500);
	pid->out =RangeValue(pid->outp + pid->outi + pid->outd,-500,+500);
	pid->input=value;
	return pid->out;
}
float RangeValue(float value,float min,float max)
{
	if (value >= max)
		return max;
	if (value <= min)
		return min;
	return value;
}

void PID_Init(PID* pid,float p,float i,float d)
{
	pid->p=p;
	pid->i=i;
	pid->d=d;
	
	PID_SetTarget(pid,0);
	
	pid->out = 0;
	pid->iv = 0;
	pid->dv=0;
	pid->filt_alpha=1;
}

void PID_Set_Filt_Alpha(PID* pid,float dt,float filt_hz)
{
	float rc = 1/(2*PI*filt_hz);
    pid->filt_alpha = dt / (dt + rc);
	pid->dt=dt;
}
	
