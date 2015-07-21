#ifndef __AHRS_H__
#define __AHRS_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f4xx.h"   

#define MPU6050_GYRO_SCALE	1000
#define MPU6050_ACC_SCALE	8
#define HALF_SIGNED16		32767

void ahrs_put_mpu6050(s16 * data);
void ahrs_update(void);
s16 MoveAve_SMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum);
s16 MoveAve_WMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum);
extern struct ahrs_t
{
	double acc_x;
	double acc_y;
	double acc_z;
	double gryo_pitch;
	double gryo_roll;
	double gryo_yaw;
	double degree_pitch;
	double degree_roll;
	double degree_yaw;
	double time_span;
}ahrs;

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
} Quaternion;

extern int16_t mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern int16_t mpu_gryo_pitch,mpu_gryo_roll,mpu_gryo_yaw;
extern double MPU6050_Diff[];
extern struct rt_semaphore angle_fix_sem;

#endif
