/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup.
 */

#include <board.h>
#include <rtthread.h>
#include <components.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif

 //#include "i2c1.h"
#include "adns3080.h"
#include "ahrs.h"
#include "bmp085.h"
#include "hardtimer.h"
#include "hmc5883.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "LED.h"
#include "math.h"
#include "Motor.h"
#include "MPU6050.h"
#include "PID.h"
#include "settings.h"
#include "sonar.h"
#include "stm32_iic.h"
#include "stm32_spi.h"
//#include "car_config.h"
//#include "IIC_OLED.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include <dfs_posix.h>
#endif

//#define FC_DEBUG
#ifdef FC_DEBUG
#define debug(fmt, ...)   rt_kprintf(fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

#define LED1(TIME) led_period[0]=(TIME)
#define LED2(TIME) led_period[1]=(TIME)
#define LED3(TIME) led_period[2]=(TIME)
#define LED4(TIME) led_period[3]=(TIME)

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[256];
static struct rt_thread led_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t control_stack[1024];
static struct rt_thread control_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t correct_stack[1024];
static struct rt_thread correct_thread;

u8 led_period[4];
void led_thread_entry(void* parameter)
{
	u8 time[4];
	while (1)
	{
		if (led_period[0])
			time[0] = (time[0] + 1) % led_period[0];
		else
			time[0] = 1;
		LED_set1(!time[0]);

		if (led_period[1])
			time[1] = (time[1] + 1) % led_period[1];
		else
			time[1] = 1;
		LED_set2(!time[1]);

		if (led_period[2])
			time[2] = (time[2] + 1) % led_period[2];
		else
			time[2] = 1;
		LED_set3(!time[2]);

		if (led_period[3])
			time[3] = (time[3] + 1) % led_period[3];
		else
			time[3] = 1;
		LED_set4(!time[3]);
		rt_thread_delay(50);
	}
}

#pragma region MPU6050_DMP
u8 en_out_ahrs = 0;
short gyro[3], accel[3], sensors;
static volatile Quaternion curq = { 1.0,0.0,0.0,0.0 };
#define q0 curq.q0
#define q1 curq.q1
#define q2 curq.q2
#define q3 curq.q3
#define DEFAULT_MPU_HZ  (200)
#define q30  1073741824.0f
const float gyroscale = 2000;
unsigned long sensor_timestamp;
unsigned char more;
long quat[4];
#define PITCH_D -1.25
#define ROLL_D 5
static u16 dmp_retry = 0;
extern volatile int16_t MPU6050_GYR_FIFO[3][256];
static signed char gyro_orientation[9] = { -1, 0, 0,
										   0,-1, 0,
										   0, 0, 1 };
static  unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;
	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}
static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}
static void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7)
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
	}
}
void dmp_init()
{
	Timer4_init();

	while (1 == mpu_init());
	//mpu_set_sensor
	while (1 == mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));

	//mpu_configure_fifo
	while (1 == mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));

	//mpu_set_sample_rate
	while (1 == mpu_set_sample_rate(DEFAULT_MPU_HZ));

	//dmp_load_motion_driver_firmvare
	while (1 == dmp_load_motion_driver_firmware());

	//dmp_set_orientation
	while (1 == dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)));

	//dmp_enable_feature
	while (1 == dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL));

	//dmp_set_fifo_rate
	while (1 == dmp_set_fifo_rate(DEFAULT_MPU_HZ));

	run_self_test();

	while (1 == mpu_set_dmp_state(1));

	rt_kprintf("start mpu6050\n");

	rt_kprintf("start ahrs\n");
}
u8 get_dmp()
{
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT)
	{
		q0 = (float)quat[0] / q30;
		q1 = (float)quat[1] / q30;
		q2 = (float)quat[2] / q30;
		q3 = (float)quat[3] / q30;

		mpu_gryo_pitch = MoveAve_WMA(gyro[0], MPU6050_GYR_FIFO[0], 8);
		mpu_gryo_roll = MoveAve_WMA(gyro[1], MPU6050_GYR_FIFO[1], 8);
		mpu_gryo_yaw = MoveAve_WMA(gyro[2], MPU6050_GYR_FIFO[2], 8);

		ahrs.gryo_pitch = -mpu_gryo_pitch 	* gyroscale / 32767.0f;
		ahrs.gryo_roll = -mpu_gryo_roll 	* gyroscale / 32767.0f;
		ahrs.gryo_yaw = -mpu_gryo_yaw 	* gyroscale / 32767.0f;

		ahrs.degree_roll = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f + settings.angle_diff_roll;   //+ Pitch_error; // pitch
		ahrs.degree_pitch = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f + settings.angle_diff_pitch;  //+ Roll_error; // roll
		if (!has_hmc5883)
			ahrs.degree_yaw = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;  //+ Yaw_error;

		ahrs.time_span = Timer4_GetSec();

		if (en_out_ahrs)
			rt_kprintf("%d,%d,%d		%d\n",
				(s32)(ahrs.degree_pitch),
				(s32)(ahrs.degree_roll),
				(s32)(ahrs.degree_yaw),
				(u32)(1.0f / ahrs.time_span));
		rt_event_send(&ahrs_event, AHRS_EVENT_Update);

		dmp_retry = 0;
		return 1;
	}
lost:
	dmp_retry++;
	return 0;
}
#pragma endregion

u8 balence = 0;
u8 pwmcon = 0;
u8 poscon = 0;
PID p_rate_pid, r_rate_pid, y_rate_pid,
p_angle_pid, r_angle_pid, y_angle_pid;
PID x_v_pid, y_v_pid,
x_d_pid, y_d_pid;
PID	h_pid;
s16 pitch_ctl[16], roll_ctl[16], yaw_ctl[16];
s16 pos_X, pos_y;

rt_bool_t lost_ahrs = RT_FALSE;

u8 en_out_pwm = 0;
FINSH_VAR_EXPORT(pwmcon, finsh_type_uchar, lock state)

void correct_gryo()
{
	rt_uint32_t e;
	rt_uint16_t i;
	rt_int16_t * mpu1, *mpu2, *mpu3;
	rt_int16_t m1, m2, m3;
	rt_kprintf("start sensors correct\n");

	mpu1 = (rt_int16_t *)rt_calloc(255, sizeof(rt_int16_t));
	mpu2 = (rt_int16_t *)rt_calloc(255, sizeof(rt_int16_t));
	mpu3 = (rt_int16_t *)rt_calloc(255, sizeof(rt_int16_t));

	for (i = 0;i < 255;i++)
	{
		if (rt_event_recv(&ahrs_event, AHRS_EVENT_Update,
			RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
			RT_WAITING_FOREVER, &e) == RT_EOK)
		{
			m1 = MoveAve_SMA(mpu_gryo_pitch, mpu1, 255);
			m2 = MoveAve_SMA(mpu_gryo_roll, mpu2, 255);
			m3 = MoveAve_SMA(mpu_gryo_yaw, mpu3, 255);
		}
	}

	MPU6050_Diff[0] -= m1;
	MPU6050_Diff[1] -= m2;
	MPU6050_Diff[2] -= m3;

	rt_free(mpu1);
	rt_free(mpu2);
	rt_free(mpu3);

	rt_kprintf("sensor correct finish.\n");
}
u16 throttle = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;
void control_thread_entry(void* parameter)
{
	float yaw_inc = 0;
	float yaw_exp = 0;
	u8 i;
	u8 take_off = 0;

	p_rate_pid.expect = 0;
	r_rate_pid.expect = 0;
	y_rate_pid.expect = 0;
	p_angle_pid.expect = 0;
	r_angle_pid.expect = 0;

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

	while (1)
	{
		LED2(pwmcon * 3);
		if (pwmcon)
		{
			if (PWM3_Time <= settings.th_max&&PWM3_Time >= settings.th_min)
				throttle = (PWM3_Time - settings.th_min) * 1000 / (settings.th_max - settings.th_min);
			else
				throttle = 0;

			if (PWM1_Time <= settings.roll_max&&PWM1_Time >= settings.roll_min)
			{
				roll = MoveAve_WMA(PWM1_Time, roll_ctl, 16) - settings.roll_mid;
				if (roll > 5)
					PID_SetTarget(&r_angle_pid, -roll / (float)(settings.roll_max - settings.roll_mid)*45.0f);
				else if (roll < -5)
					PID_SetTarget(&r_angle_pid, -roll
						/ (float)(settings.roll_mid - settings.roll_min)*45.0f);
				else
					PID_SetTarget(&r_angle_pid, 0);
			}
			if (PWM2_Time <= settings.pitch_max&&PWM2_Time >= settings.pitch_min)
			{
				pitch = MoveAve_WMA(PWM2_Time, pitch_ctl, 16) - settings.pitch_mid;
				if (pitch > 5)
					PID_SetTarget(&p_angle_pid, -pitch
						/ (float)(settings.pitch_max - settings.pitch_mid)*30.0f);
				else if (pitch < -5)
					PID_SetTarget(&p_angle_pid, -pitch
						/ (float)(settings.pitch_mid - settings.pitch_min)*30.0f);
				else
					PID_SetTarget(&p_angle_pid, 0);
			}
			if (PWM4_Time <= settings.yaw_max&&PWM4_Time >= settings.yaw_min)
			{
				yaw_inc = MoveAve_WMA(PWM4_Time, yaw_ctl, 16) - settings.yaw_mid;
				if (has_hmc5883)
				{
					if (yaw_inc > 5)
						yaw_exp -= yaw_inc / (float)(settings.yaw_max - settings.yaw_mid)*0.5f;
					else if (yaw_inc < -5)
						yaw_exp -= yaw_inc / (float)(settings.yaw_mid - settings.yaw_min)*0.5f;
					if (yaw_exp > 360.0f)yaw_exp -= 360.0f;
					else if (yaw_exp < 0.0f)yaw_exp += 360.0f;
					PID_SetTarget(&y_angle_pid, 0);
				}
				else
				{
					if (yaw > 5)
						PID_SetTarget(&y_rate_pid, -yaw_inc
							/ (float)(settings.yaw_max - settings.yaw_mid)*100.0f);
					else if (yaw < -5)
						PID_SetTarget(&y_rate_pid, -yaw_inc
							/ (float)(settings.yaw_mid - settings.yaw_min)*100.0f);
					else
						PID_SetTarget(&y_rate_pid, 0);
				}
			}
			if (!balence)
				Motor_Set(throttle, throttle, throttle, throttle);
		}
		else if (PWM3_Time > settings.th_min&&PWM3_Time < settings.th_min + 40 &&
			PWM5_Time < 1700 && PWM5_Time>500)
		{
			//set pwm middle
			if (!pwmcon)
			{
				settings.roll_mid = PWM1_Time;
				settings.pitch_mid = PWM2_Time;
				settings.yaw_mid = PWM4_Time;
			}
			pwmcon = 1;
			balence = 1;
			p_rate_pid.iv = 0;
			r_rate_pid.iv = 0;
			y_rate_pid.iv = 0;
			take_off = 1;
			yaw_exp = ahrs.degree_yaw;
		}

		if (get_dmp() && balence)
		{
			rt_uint32_t dump;
			if (throttle > 60 && abs(ahrs.degree_pitch) < 40 && abs(ahrs.degree_roll) < 40)
			{
				if (PWM5_Time < 1200 && PWM5_Time>800 && sonar_state)
				{
					if (rt_event_recv(&ahrs_event, AHRS_EVENT_SONAR, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
					{
						PID_SetTarget(&h_pid, 60.0);
						PID_xUpdate(&h_pid, sonar_h);
						h_pid.out = RangeValue(h_pid.out, -200, 200);
					}
					LED4(4);
					throttle = 500;

					if (has_adns3080&& PWM7_Time > 1500)
					{
						if (!poscon)
						{
							poscon = 1;
							pos_X = opx;
							pos_y = opy;
						}
						if (rt_event_recv(&ahrs_event, AHRS_EVENT_ADNS3080, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
						{
							PID_SetTarget(&x_d_pid, pos_X);
							PID_SetTarget(&y_d_pid, pos_y);

							PID_xUpdate(&x_d_pid, opx);
							PID_xUpdate(&y_d_pid, opy);

							PID_SetTarget(&x_v_pid, -RangeValue(x_d_pid.out, -10, 10));
							PID_SetTarget(&y_v_pid, -RangeValue(y_d_pid.out, -10, 10));

							PID_xUpdate(&x_v_pid, optc_dx);
							PID_xUpdate(&y_v_pid, optc_dy);
						}
						LED4(2);
						PID_SetTarget(&r_angle_pid, -RangeValue(x_v_pid.out, -10, 10));
						PID_SetTarget(&p_angle_pid, +RangeValue(y_v_pid.out, -10, 10));
					}
					else
						poscon = 0;
				}
				else
				{
					h_pid.out = 0;
					LED4(0);
				}

				PID_xUpdate(&p_angle_pid, ahrs.degree_pitch);
				PID_SetTarget(&p_rate_pid, -RangeValue(p_angle_pid.out, -80, 80));
				PID_xUpdate(&p_rate_pid, ahrs.gryo_pitch);

				PID_xUpdate(&r_angle_pid, ahrs.degree_roll);
				PID_SetTarget(&r_rate_pid, -RangeValue(r_angle_pid.out, -80, 80));
				PID_xUpdate(&r_rate_pid, ahrs.gryo_roll);

				if (has_hmc5883)
				{
					if (rt_event_recv(&ahrs_event, AHRS_EVENT_HMC5883, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
					{
						yaw = ahrs.degree_yaw - yaw_exp;
						if (yaw > 180.0f)yaw -= 360.0f;
						if (yaw < -180.0f)yaw += 360.0f;
						PID_xUpdate(&y_angle_pid, yaw);
						PID_SetTarget(&y_rate_pid, -RangeValue(y_angle_pid.out, -100, 100));
					}
				}
				PID_xUpdate(&y_rate_pid, ahrs.gryo_yaw);

				Motor_Set1(throttle - p_rate_pid.out - r_rate_pid.out + y_rate_pid.out - h_pid.out);
				Motor_Set2(throttle - p_rate_pid.out + r_rate_pid.out - y_rate_pid.out - h_pid.out);
				Motor_Set3(throttle + p_rate_pid.out - r_rate_pid.out - y_rate_pid.out - h_pid.out);
				Motor_Set4(throttle + p_rate_pid.out + r_rate_pid.out + y_rate_pid.out - h_pid.out);
			}
			else
				Motor_Set(60, 60, 60, 60);
		}
		else
			LED4(0);

		if (PWM5_Time > 1700)
		{
			Motor_Set(0, 0, 0, 0);
			p_rate_pid.iv = 0;
			r_rate_pid.iv = 0;
			y_rate_pid.iv = 0;
			balence = 0;
			pwmcon = 0;
		}

		if (dmp_retry > 200)
		{
			Motor_Set(0, 0, 0, 0);
			balence = 0;
			pwmcon = 0;
			LED3(2);
			break;
		}

		if (en_out_pwm)
		{
			rt_kprintf("\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
				PWM1_Time, PWM2_Time, PWM3_Time, PWM4_Time,
				PWM5_Time, PWM6_Time, PWM7_Time, PWM8_Time);
			//			rt_kprintf("\t%d\t%d\t%d\t%d\t%d\t%d\n",
			//				Motor1,Motor2,Motor3,
			//				Motor4,Motor5,Motor6);
		}

		rt_thread_delay(2);
	}
}

void correct_thread_entry(void* parameter)
{
	rt_uint32_t e;
	rt_uint16_t i;
	rt_int16_t * mpu1, *mpu2, *mpu3;
	rt_int16_t m1, m2, m3;
	rt_sem_init(&angle_fix_sem, "angle_fix", 0, RT_IPC_FLAG_FIFO);

	while (1)
	{
		rt_sem_take(&angle_fix_sem, RT_WAITING_FOREVER);
		rt_kprintf("start angle fix\n");

		settings.angle_diff_pitch = 0;
		settings.angle_diff_roll = 0;
		settings.angle_diff_yaw = 0;

		mpu1 = (rt_int16_t *)rt_calloc(255, sizeof(rt_int16_t));
		mpu2 = (rt_int16_t *)rt_calloc(255, sizeof(rt_int16_t));
		mpu3 = (rt_int16_t *)rt_calloc(255, sizeof(rt_int16_t));

		for (i = 0;i < 255;i++)
		{
			if (rt_event_recv(&ahrs_event, AHRS_EVENT_Update,
				RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
				RT_WAITING_FOREVER, &e) == RT_EOK)
			{
				m1 = MoveAve_SMA(ahrs.degree_pitch*1000.0f, mpu1, 255);
				m2 = MoveAve_SMA(ahrs.degree_roll	*1000.0f, mpu2, 255);
				m3 = MoveAve_SMA(ahrs.degree_yaw	*1000.0f, mpu3, 255);
			}
		}

		settings.angle_diff_pitch = -m1 / 1000.0f;
		settings.angle_diff_roll = -m2 / 1000.0f;
		settings.angle_diff_yaw = -m3 / 1000.0f;

		rt_free(mpu1);
		rt_free(mpu2);
		rt_free(mpu3);

		rt_kprintf("pitch:%d	roll:%d	yaw:%d\n",
			(s16)(settings.angle_diff_pitch),
			(s16)(settings.angle_diff_roll),
			(s16)(settings.angle_diff_yaw));

		rt_kprintf("degree fix finish.\n");
		save_settings(&settings, "/setting");
	}
}

static void config_bt()
{
	GPIO_InitTypeDef gpio_init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	gpio_init.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &gpio_init);
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

void rt_init_thread_entry(void* parameter)
{
	rt_components_init();

	LED_init();
	Motor_Init();

	rt_kprintf("start device init\n");

	//rt_hw_i2c1_init();
	i2cInit();
	rt_hw_spi2_init();
	rt_hw_spi3_init();

	rt_event_init(&ahrs_event, "ahrs", RT_IPC_FLAG_FIFO);

	dmp_init();
	sonar_init();
	HMC5983_Init();
	adns3080_Init();

	//config_bt();

	rt_thread_init(&led_thread,
		"led",
		led_thread_entry,
		RT_NULL,
		led_stack,
		256, 16, 1);
	rt_thread_startup(&led_thread);

	spi_flash_init();

	//	bmp085_init("i2c1");

	rt_kprintf("device init succeed\n");

	if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
	{
		rt_kprintf("flash0 mount to /.\n");
	}
	else
	{
		rt_kprintf("flash0 mount to / failed.\n");
	}

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

	settings.roll_min = settings.pitch_min = settings.yaw_min = 1000;
	settings.th_min = 1000;
	settings.roll_max = settings.pitch_max = settings.yaw_max = 2000;
	settings.th_max = 2000;

	//	if(settings.pwm_init_mode)
	//	{
	//		Motor_Set(1000,1000,1000,1000);
	//
	//		rt_thread_delay(RT_TICK_PER_SECOND*5);
	//
	//		Motor_Set(0,0,0,0);
	//
	//		settings.pwm_init_mode=0;
	//		save_settings(&settings,"/setting");
	//
	//		rt_kprintf("pwm init finished!\n");
	//	}

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

	rt_thread_init(&correct_thread,
		"correct",
		correct_thread_entry,
		RT_NULL,
		correct_stack,
		1024, 12, 1);
	rt_thread_startup(&correct_thread);

	LED1(5);
}

int rt_application_init()
{
	rt_thread_t tid;

	tid = rt_thread_create("init",
		rt_init_thread_entry, RT_NULL,
		2048, 1, 20);

	if (tid != RT_NULL)
		rt_thread_startup(tid);

	return 0;
}

/*@}*/
