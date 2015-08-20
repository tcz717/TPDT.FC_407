#ifndef _PID
#define _PID

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

typedef struct
{
	float p;
	float i;
	float d;
	float input;
	float expect;
	float out;
	float outp;
	float outi;
	float outd;
	float iv;
	float dv;
	float filt_alpha;
	float dt;
	float maxi;
}PID;

void PID_Reset(PID* pid);
void PID_SetTarget(PID*,float value);
float PID_Update(PID*,float value, float dv);
float PID_xUpdate(PID* pid,float value);
float RangeValue(float value,float min,float max);
void PID_Init(PID*,float p,float i,float d);
void PID_Set_Filt_Alpha(PID* pid,float dt,float filt_hz);

#endif
