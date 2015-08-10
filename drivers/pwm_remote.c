#include "pwm_remote.h"
#include "PWM.h"
#include "settings.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include "ahrs.h"

static s16 pitch_ctl[16], roll_ctl[16], yaw_ctl[16];
static s16 pitch, roll, yaw;

u8 en_out_pwm = 0;

void receive_pwm(pwm_signal_t * signal)
{
	if (PWM3_Time <= settings.th_max&&PWM3_Time >= settings.th_min)
		signal->throttle = (PWM3_Time - settings.th_min) / (float)(settings.th_max - settings.th_min);
	else
		signal->throttle = 0;

	if (PWM1_Time <= settings.roll_max&&PWM1_Time >= settings.roll_min)
	{
		roll = MoveAve_WMA(PWM1_Time, roll_ctl, 16) - settings.roll_mid;
		if (roll > 5)
			signal->roll = -roll / (float)(settings.roll_max - settings.roll_mid);
		else if (roll < -5)
			signal->roll = -roll / (float)(settings.roll_mid - settings.roll_min);
		else
			signal->roll = 0;
	}
	if (PWM2_Time <= settings.pitch_max&&PWM2_Time >= settings.pitch_min)
	{
		pitch = MoveAve_WMA(PWM2_Time, pitch_ctl, 16) - settings.pitch_mid;
		if (pitch > 5)
			signal->pitch = -pitch / (float)(settings.pitch_max - settings.pitch_mid);
		else if (pitch < -5)
			signal->pitch = -pitch / (float)(settings.pitch_mid - settings.pitch_min);
		else
			signal->pitch = 0;
	}
	if (PWM4_Time <= settings.yaw_max&&PWM4_Time >= settings.yaw_min)
	{
		yaw = MoveAve_WMA(PWM4_Time, yaw_ctl, 16) - settings.yaw_mid;
		if (yaw > 5)
			signal->yaw = -yaw / (float)(settings.yaw_mid - settings.yaw_min);
		else if (yaw < -5)
			signal->yaw = -yaw / (float)(settings.yaw_mid - settings.yaw_min);
		else
			signal->yaw = 0;
	}
	if(PWM5_Time < 2200 && PWM5_Time>1800)
		signal->switch1=2;
	else if(PWM5_Time < 1700 && PWM5_Time>1300)
		signal->switch1=1;
	else if(PWM5_Time < 1200 && PWM5_Time>800)
		signal->switch1=0;
	else
		signal->switch1=-1;
		
	if(PWM7_Time > 1600 && PWM7_Time < 2200)
		signal->switch2=1;
	else if(PWM7_Time < 1400 && PWM7_Time>900)
		signal->switch2=0;
	else
		signal->switch2=-1;	
	
	if (en_out_pwm)
	{
		rt_kprintf("\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
			PWM1_Time, PWM2_Time, PWM3_Time, PWM4_Time,
			PWM5_Time, PWM6_Time, PWM7_Time, PWM8_Time);
		//			rt_kprintf("\t%d\t%d\t%d\t%d\t%d\t%d\n",
		//				Motor1,Motor2,Motor3,
		//				Motor4,Motor5,Motor6);
	}
}
