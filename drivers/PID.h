#ifndef _PID
#define _PID

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

typedef struct
{
	double p;
	double i;
	double d;
	double input;
	double expect;
	double out;
	double outp;
	double outi;
	double outd;
	double iv;
	double dv;
	double filt_alpha;
	double dt;
}PID;

void PID_SetTarget(PID*,double value);
double PID_Update(PID*,double value, double dv);
double PID_xUpdate(PID* pid,double value);
double RangeValue(double value,double min,double max);
void PID_Init(PID*,double p,double i,double d);
void PID_Set_Filt_Alpha(PID* pid,double dt,double filt_hz);

#endif
