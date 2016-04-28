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
#include "hardtimer.h"
#include "hmc5883.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "led.h"
#include "math.h"
#include "Motor.h"
#include "PID.h"
#include "settings.h"
#include "sonar.h"
#include "stm32_iic.h"
#include "stm32_spi.h"
#include "remote.h"
#include "control.h"

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

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t correct_stack[1024];
static struct rt_thread correct_thread;

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
const float accscale = 2;
unsigned long sensor_timestamp;
unsigned char more;
long quat[4];
#define PITCH_D -1.25
#define ROLL_D 5
u16 dmp_retry = 0;
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
	rt_kprintf("dmp self check 0x%02X.\n",result);
}
void dmp_init()
{
	Timer4_init();

	while (mpu_init());
	//mpu_set_sensor
	while (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));

	//mpu_configure_fifo
	while (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));

	//mpu_set_sample_rate
	while (mpu_set_sample_rate(DEFAULT_MPU_HZ));

	//dmp_load_motion_driver_firmvare
	while (dmp_load_motion_driver_firmware());

	//dmp_set_orientation
	while (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)));

	//dmp_enable_feature
	while (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL));

	//dmp_set_fifo_rate
	while (dmp_set_fifo_rate(DEFAULT_MPU_HZ));

	run_self_test();

	while (mpu_set_dmp_state(1));

	rt_kprintf("start mpu6050\n");
	
	ahrs.height_acc_fix=0;
	ahrs_state.mpu6050=RT_EOK;
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
		
		ahrs.acc_x=accel[0]*accscale / 32767.0f;
		ahrs.acc_y=accel[1]*accscale / 32767.0f;
		ahrs.acc_z=accel[2]*accscale / 32767.0f;

		ahrs.gryo_pitch = -mpu_gryo_pitch 	* gyroscale / 32767.0f;
		ahrs.gryo_roll = -mpu_gryo_roll 	* gyroscale / 32767.0f;
		ahrs.gryo_yaw = -mpu_gryo_yaw 	* gyroscale / 32767.0f;
		
		ahrs.g_x = -2*(q1*q3 - q0*q2);
		ahrs.g_y = -2*(q0*q1 + q2*q3);
		ahrs.g_z = 1 - 2*(q1*q1 + q2*q2);
		
		ahrs.height_acc=low_pass(ahrs.height_acc,
		(ahrs.acc_x*ahrs.g_x+ahrs.acc_y*ahrs.g_y+ahrs.acc_z*ahrs.g_z-1.0f)*9.8f*100.0f-ahrs.height_acc_fix,20,1/166.0f);

		ahrs.degree_roll = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f + settings.angle_diff_roll;   //+ Pitch_error; // pitch
		ahrs.degree_pitch = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f + settings.angle_diff_pitch;  //+ Roll_error; // roll
		
		ahrs.vx+=cosf(toRad(ahrs.degree_pitch))*ahrs.acc_y*9.8f*ahrs.time_span;	
//		if (!has_hmc5883)
//			ahrs.degree_yaw = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;  //+ Yaw_error;
		
		ahrs.degree_yaw+=ahrs.time_span*ahrs.gryo_yaw;
		if (ahrs.degree_yaw > 360.0f)ahrs.degree_yaw -= 360.0f;
		if (ahrs.degree_yaw < 0.0f)ahrs.degree_yaw += 360.0f;
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
	dmp_retry++;
	if (dmp_retry > 200)
	{
		Motor_Set(0, 0, 0, 0);
		ahrs_state.mpu6050=RT_EIO;
		LED3(2);
	}
	return 0;
}
#pragma endregion

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

char log_buf[256]={0};
int tlog(const char * fmt ,...)
{
    va_list args;
	int i=0;
	
    va_start(args, fmt);
	i=vsnprintf(log_buf,sizeof(log_buf),fmt,args);
	va_end(args);
	
	return i;
}
static void print_log(void)
{
	if(log_buf[0])
		rt_kprintf("%s\n",log_buf);
	log_buf[0]='\0';
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
	
	extern rt_err_t camera_init(const char * uart_name);
	
	camera_init("uart1");
	remote_init("uart6");

	//config_bt();


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

	rt_thread_init(&correct_thread,
		"correct",
		correct_thread_entry,
		RT_NULL,
		correct_stack,
		1024, 12, 1);
	rt_thread_startup(&correct_thread);
	
	rt_thread_idle_sethook(print_log);

	control_init();
	
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
