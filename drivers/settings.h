#ifndef __SETTING_H__
#define __SETTING_H__

#include <board.h>
#include <rtthread.h>
#include <dfs_fs.h>
#include "PID.h"

#define SETTING_MAGIC 0x1234

struct setting_t
{
	u16 magic;
	
	float pitch_angle_p;
	float pitch_rate_p;
	float pitch_rate_i;
	float pitch_rate_d;
	
	float roll_angle_p;
	float roll_rate_p;
	float roll_rate_i;
	float roll_rate_d;
	
	float yaw_angle_p;
	float yaw_rate_p;
	float yaw_rate_i;
	float yaw_rate_d;

	float x_d_p;
	float x_v_p;
	float x_v_i;
	float x_v_d;

	float y_d_p;
	float y_v_p;
	float y_v_i;
	float y_v_d;
	
	float h_p;
	float h_i;
	float h_d;
	
	float angle_diff_pitch;
	float angle_diff_roll;
	float angle_diff_yaw;
	
	u16 roll_min;
	u16 roll_mid;
	u16 roll_max;
	
	u16 pitch_min;
	u16 pitch_mid;
	u16 pitch_max;
	
	u16 th_min;
	u16 th_max;
	
	u16 yaw_min;
	u16 yaw_mid;
	u16 yaw_max;
	
	u8 pwm_init_mode;
};

extern struct setting_t settings;

void get_pid(void);
rt_err_t load_settings(struct setting_t * s, const char * path,
	PID * p_angle, PID * p_rate,
	PID * r_angle, PID * r_rate,
	PID * y_angle, PID * y_rate,
	PID * x_d, PID * x_v,
	PID * y_d, PID * y_v,
	PID * h);
rt_err_t save_settings(struct setting_t * s,const char * path);

#endif
