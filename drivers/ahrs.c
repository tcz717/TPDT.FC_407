#include "ahrs.h"
#include "stm32f4xx.h"                  // Device header
#include "hardtimer.h"
#include "math.h"
#include <rtthread.h>
#include <finsh.h>
struct ahrs_t ahrs;
int16_t mpu_acc_x,mpu_acc_y,mpu_acc_z;
int16_t mpu_gryo_pitch,mpu_gryo_roll,mpu_gryo_yaw;
struct rt_semaphore angle_fix_sem;
struct rt_event ahrs_event;

s16 MoveAve_SMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum)
{
	u8 i = 0;
	s16 AveData = 0;
	s32 MoveAve_Sum = 0;

	for (i = 0; i < SampleNum - 1; i++)      
		MoveAve_FIFO[i] = MoveAve_FIFO[i + 1];

	MoveAve_FIFO[SampleNum - 1] = NewData;   

	for (i = 0; i < SampleNum; i++)        
		MoveAve_Sum += MoveAve_FIFO[i];

	AveData = (s16)(MoveAve_Sum / SampleNum);

	return AveData;
}

s16 MoveAve_WMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum)
{
	u8 i = 0;
	s16 AveData = 0;
	u16 SampleSum = 0;
	s32 MoveAve_Sum = 0;

	for (i = 0; i < SampleNum - 1; i++)        
		MoveAve_FIFO[i] = MoveAve_FIFO[i + 1];

	MoveAve_FIFO[SampleNum - 1] = NewData;   

	for (i = 0; i < SampleNum; i++)            
		MoveAve_Sum += MoveAve_FIFO[i] * (i + 1);

	SampleSum = (SampleNum * (SampleNum + 1)) / 2; 
	AveData = (s16)(MoveAve_Sum / SampleSum);  

	return AveData;
}
static float dt ;
static void get_fps()
{
	rt_kprintf("fps:\t\t%d",(int)(1.0f/dt));
}
FINSH_FUNCTION_EXPORT(get_fps, get ahrs fps)

void Quaternion_ToNumQ(Quaternion *pNumQ, float pitch,float roll,float yaw)
{
	float halfP = pitch / 2.0f;
	float halfR = roll / 2.0f;
	float halfY = yaw / 2.0f;

	float sinP = sin(halfP);
	float cosP = cos(halfP);
	float sinR = sin(halfR);
	float cosR = cos(halfR);
	float sinY = sin(halfY);
	float cosY = cos(halfY);

	pNumQ->q0 = cosY * cosR * cosP + sinY * sinR * sinP;
	pNumQ->q1 = cosY * cosR * sinP - sinY * sinR * cosP;
	pNumQ->q2 = cosY * sinR * cosP + sinY * cosR * sinP;
	pNumQ->q3 = sinY * cosR * cosP - cosY * sinR * sinP;
}

void Quaternion_ToAngE(Quaternion *pNumQ)
{
	float NumQ_T11 = pNumQ->q0 * pNumQ->q0 + pNumQ->q1 * pNumQ->q1 - pNumQ->q2 * pNumQ->q2 - pNumQ->q3 * pNumQ->q3;
	float NumQ_T12 = 2.0f * (pNumQ->q0 * pNumQ->q3 + pNumQ->q1 * pNumQ->q2);
	float NumQ_T13 = 2.0f * (pNumQ->q1 * pNumQ->q3 - pNumQ->q0 * pNumQ->q2);
	float NumQ_T23 = 2.0f * (pNumQ->q0 * pNumQ->q1 + pNumQ->q2 * pNumQ->q3);
	float NumQ_T33 = pNumQ->q0 * pNumQ->q0 - pNumQ->q1 * pNumQ->q1 - pNumQ->q2 * pNumQ->q2 + pNumQ->q3 * pNumQ->q3;

	ahrs.degree_roll	=	-asinf(NumQ_T13)* 180.0f / 3.14f;
	ahrs.degree_pitch	=	atan2f(NumQ_T23, NumQ_T33)* 180.0f / 3.14f;
	ahrs.degree_yaw		=	atan2f(NumQ_T12, NumQ_T11)* 180.0f / 3.14f;
//	pAngE->Yaw   = atan2f(NumQ_T12, NumQ_T11);
}

void angle_fix()
{
	rt_sem_release(&angle_fix_sem);
}
FINSH_FUNCTION_EXPORT(angle_fix, fix mpu6050 acc )

rt_inline float toRad(float degree)
{
	return degree *3.14f / 180.0f;
}

//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Definitions
Quaternion curq={1.0,0.0,0.0,0.0};

//#define USE_QUATERNION

#define q0 curq.q0
#define q1 curq.q1
#define q2 curq.q2
#define q3 curq.q3

#define Kp 15.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.02f                // integral gain governs rate of convergence of gyroscope biases
#define halfT (dt / 2.0f)                // half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions

float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

//====================================================================================================
// Function
//====================================================================================================

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az/*, float mx, float my, float mz*/) {
        float norm;
 //       float hx, hy, hz, bx, bz;
        float vx, vy, vz/*, wx, wy, wz*/;
        float ex, ey, ez;

        // auxiliary variables to reduce number of repeated operations
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
//        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
//        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;   
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;          
        
        // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);       
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
//        norm = sqrt(mx*mx + my*my + mz*mz);          
//        mx = mx / norm;
//        my = my / norm;
//        mz = mz / norm;         
        
        // compute reference direction of flux
//        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
//        bx = sqrt((hx*hx) + (hy*hy));
//        bz = hz;        
        
        // estimated direction of gravity and flux (v and w)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
//        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
        
        // error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay*vz - az*vy)/* + (my*wz - mz*wy)*/;
        ey = (az*vx - ax*vz)/* + (mz*wx - mx*wz)*/;
        ez = (ax*vy - ay*vx)/* + (mx*wy - my*wx)*/;
        
        // integral error scaled integral gain
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        
        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        
        // integrate quaternion rate and normalise
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
}

/// todo : reorganize ahrs code ----------------------
float p,r;
int cnt=0;
void ahrs_update()
{
	float ax,ay;
	const float a = 0.96f;
	const float gyroscale = MPU6050_GYRO_SCALE;
	float tmp;
	
	dt = Timer4_GetSec();
	
	ahrs.gryo_pitch		= mpu_gryo_pitch 	* gyroscale / 32767.0f;
	ahrs.gryo_roll		= mpu_gryo_roll 	* gyroscale / 32767.0f;
	ahrs.gryo_yaw		= mpu_gryo_yaw 		* gyroscale / 32767.0f;
#ifndef USE_QUATERNION
#ifdef MEASURE_ROTER
	ax					= atan2(
							mpu_acc_y,
							sqrt(mpu_acc_x * mpu_acc_x +mpu_acc_z * mpu_acc_z)
								)* 180.0f / 3.14f;
	ay					= -atan2(
							mpu_acc_x,
							sqrt(mpu_acc_y * mpu_acc_y + mpu_acc_z * mpu_acc_z)
								) * 180.0f / 3.14f;
	ahrs.degree_pitch	= a * (ahrs.degree_pitch  + ahrs.gryo_pitch * dt) + (1 - a) * ax;
	ahrs.degree_roll	= a * (ahrs.degree_roll + ahrs.gryo_roll * dt) + (1 - a) * ay;
	ahrs.degree_yaw		= ahrs.degree_yaw + ahrs.gryo_yaw * dt;
#else
	ax					= atan2(
							mpu_acc_y,
							sqrt(mpu_acc_x * mpu_acc_x +mpu_acc_z * mpu_acc_z)
								)* 180.0f / 3.14f;
	ay					= -atan2(
							mpu_acc_x,
							sqrt(mpu_acc_y * mpu_acc_y + mpu_acc_z * mpu_acc_z)
								) * 180.0f / 3.14f;
	p	= a * (ahrs.degree_pitch  + ahrs.gryo_pitch * dt) + (1 - a) * ax;
	r	= a * (ahrs.degree_roll + ahrs.gryo_roll * dt) + (1 - a) * ay;
	ahrs.degree_yaw		= ahrs.degree_yaw + ahrs.gryo_yaw * dt;
{
	    float norm;
        float vx, vy, vz;
        float ex, ey, ez;   
		float az;
        float gx, gy, gz;   
	
		ax =mpu_acc_x;
		ay =mpu_acc_y;
		az= mpu_acc_z;
	
		gx =ahrs.gryo_pitch* 3.14f / 180.0f;
		gy =ahrs.gryo_roll* 3.14f / 180.0f;
		gz= ahrs.gryo_yaw* 3.14f / 180.0f;

        // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);      
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;      

        // estimated direction of gravity
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // error is sum of cross product between reference direction of field and direction measured by sensor
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);

        // integral error scaled integral gain
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        // integrate quaternion rate and normalise
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
		
		
	Quaternion_ToAngE(&curq);
}
#endif
#else
	Quaternion_ToNumQ(&curq,ahrs.degree_pitch,ahrs.degree_roll,ahrs.degree_yaw);
	
	AHRSupdate(toRad(ahrs.gryo_pitch)	,toRad(ahrs.gryo_roll)	,toRad(ahrs.gryo_yaw)	,mpu_acc_x	,mpu_acc_y	,mpu_acc_z);
	
	Quaternion_ToAngE(&curq);
#endif
	ahrs.time_span		= dt;
}


volatile int16_t MPU6050_ACC_FIFO[3][256] = {{0}};
volatile int16_t MPU6050_GYR_FIFO[3][256] = {{0}};
float MPU6050_Diff[6]={15,-35,40,0};
void ahrs_put_mpu6050(s16 * data)
{
//	mpu_gryo_pitch=(MoveAve_WMA(data[3], MPU6050_GYR_FIFO[0], 5)+MPU6050_Diff[0]);
//	mpu_gryo_roll=(MoveAve_WMA(data[4], MPU6050_GYR_FIFO[1], 5)+MPU6050_Diff[1]);
//	mpu_gryo_yaw=(MoveAve_WMA(data[5], MPU6050_GYR_FIFO[2], 5)+MPU6050_Diff[2]);
//	mpu_acc_x=MoveAve_WMA(data[0], MPU6050_ACC_FIFO[0], 2)+MPU6050_Diff[3];
//	mpu_acc_y=MoveAve_WMA(data[1], MPU6050_ACC_FIFO[1], 2)+MPU6050_Diff[4];
//	mpu_acc_z=MoveAve_WMA(data[2], MPU6050_ACC_FIFO[2], 2)+MPU6050_Diff[5];
	
	mpu_gryo_pitch=data[3]+MPU6050_Diff[0];
	mpu_gryo_roll=data[4]+MPU6050_Diff[1];
	mpu_gryo_yaw=data[5]+MPU6050_Diff[2];
	mpu_acc_x=data[0]+MPU6050_Diff[3];
	mpu_acc_y=data[1]+MPU6050_Diff[4];
	mpu_acc_z=data[2]+MPU6050_Diff[5];
}

//float kalman_input_baro_altitude;  //卡尔曼滤波输入数据
//float x_mid=0, x_last=0, p_mid=1, p_last=0, x_now =0 ,p_now = 1;
//float Q=1,R=10;
//float z_measure=0;
//float kg=0;

//void ahrs_put_bmp(float height)
//{
////	ahrs.height=ahrs.height*0.9+height*0.1;
//	    //气压计高度  -- 分米单位
//    kalman_input_baro_altitude = height;
//    
//    x_mid = x_last;    //x_last=x(k-1|k-1),x_mid=x(k|k-1)
//    p_mid = p_last + Q;  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
//    kg = p_mid /(p_mid + R); //kg为kalman filter，R为噪声
//    z_measure = kalman_input_baro_altitude    ;//测量值，输入是分米级别的
//    x_now = x_mid + kg * (z_measure - x_mid);//估计出的最优值
//    p_now = (1 - kg) * p_mid;//最优值对应的covariance
//    p_last = p_now;  //更新covariance值
//    x_last = x_now;  //更新系统状态值
//    ahrs.height = x_now;
//}
