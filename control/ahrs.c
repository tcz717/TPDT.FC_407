#include "ahrs.h"
#include "stm32f4xx.h" // Device header
#include "hardtimer.h"
#include "math.h"
#include <rtthread.h>
#include <finsh.h>
#define PI 3.1415926
struct ahrs_t ahrs;
int16_t mpu_acc_x, mpu_acc_y, mpu_acc_z;
int16_t mpu_gryo_pitch, mpu_gryo_roll, mpu_gryo_yaw;
struct rt_semaphore angle_fix_sem;
struct rt_event ahrs_event;
ahrs_state_t ahrs_state;

static float dt;

//均值滤波
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

//加权均值滤波
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

//中值滤波
float Moving_Median(volatile float NewData, volatile float *MoveMid_FIFO, u8 SampleNum)
{
    static float MoveMid_TMP[128];
    u8 i, j;
    float t;

    for (i = 0; i < SampleNum - 1; i++)
        MoveMid_FIFO[i] = MoveMid_FIFO[i + 1];

    MoveMid_FIFO[SampleNum - 1] = NewData;

    for (i = 0; i < SampleNum; i++)
        MoveMid_TMP[i] = MoveMid_FIFO[i];

    for (i = 0; i < SampleNum - 1; i++)
    {
        for (j = 0; j < (SampleNum - 1 - i); j++)
        {
            if (MoveMid_TMP[j] > MoveMid_TMP[j + 1])
            {
                t = MoveMid_TMP[j];
                MoveMid_TMP[j] = MoveMid_TMP[j + 1];
                MoveMid_TMP[j + 1] = t;
            }
        }
    }

    return (MoveMid_TMP[SampleNum / 2]);
}

//获取ahrs计算频率
static void get_fps()
{
    rt_kprintf("fps:\t\t%d", (int)(1.0f / dt));
}
FINSH_FUNCTION_EXPORT(get_fps, get ahrs fps)

//殴拉角转四元数
void Quaternion_ToNumQ(Quaternion *pNumQ, float pitch, float roll, float yaw)
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

//四元数转殴拉角
void Quaternion_ToAngE(Quaternion *pNumQ)
{
    float NumQ_T11 = pNumQ->q0 * pNumQ->q0 + pNumQ->q1 * pNumQ->q1 - pNumQ->q2 * pNumQ->q2 - pNumQ->q3 * pNumQ->q3;
    float NumQ_T12 = 2.0f * (pNumQ->q0 * pNumQ->q3 + pNumQ->q1 * pNumQ->q2);
    float NumQ_T13 = 2.0f * (pNumQ->q1 * pNumQ->q3 - pNumQ->q0 * pNumQ->q2);
    float NumQ_T23 = 2.0f * (pNumQ->q0 * pNumQ->q1 + pNumQ->q2 * pNumQ->q3);
    float NumQ_T33 = pNumQ->q0 * pNumQ->q0 - pNumQ->q1 * pNumQ->q1 - pNumQ->q2 * pNumQ->q2 + pNumQ->q3 * pNumQ->q3;

    ahrs.degree_roll = -asinf(NumQ_T13) * 180.0f / PI;
    ahrs.degree_pitch = atan2f(NumQ_T23, NumQ_T33) * 180.0f / PI;
    ahrs.degree_yaw = atan2f(NumQ_T12, NumQ_T11) * 180.0f / PI;
}

//启动水平面角度修正
void angle_fix()
{
    rt_sem_release(&angle_fix_sem);
}
FINSH_FUNCTION_EXPORT(angle_fix, fix mpu6050 acc)

//角度转弧度
float toRad(float degree)
{
    return degree * PI / 180.0f;
}

//约束角度以360度内正数表示
float rangeYaw(float yaw)
{
    if (yaw > 360.0f)
        yaw -= 360.0f;
    if (yaw < 0.0f)
        yaw += 360.0f;
    return yaw;
}

//计算两个偏航角的差值
float diffYaw(float yaw1, float yaw2)
{
    float yaw_err = yaw1 - yaw2;
    if (yaw_err > 180.0f)
        yaw_err -= 360.0f;
    if (yaw_err < -180.0f)
        yaw_err += 360.0f;
    return yaw_err;
}

//低通滤波函数
float low_pass(float ov, float nv, float hz, float dt)
{
    float rc = 1 / (2 * PI * hz);
    float filt_alpha = dt / (dt + rc);
    return ov + filt_alpha * (nv - ov);
}

Quaternion curq = {1.0, 0.0, 0.0, 0.0};

#define q0 curq.q0
#define q1 curq.q1
#define q2 curq.q2
#define q3 curq.q3

#define Kp 15.0f          // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.02f          // integral gain governs rate of convergence of gyroscope biases
#define halfT (dt / 2.0f) // half the sample period

float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error

// todo : reorganize ahrs code 
//目前ahrs代码位于applications/application.c
//此函数暂时无用
void ahrs_update()
{
    float ax, ay;
    const float gyroscale = MPU6050_GYRO_SCALE;

    dt = Timer4_GetSec();//获取与上次计算间隔

    ahrs.gryo_pitch = mpu_gryo_pitch * gyroscale / 32767.0f;
    ahrs.gryo_roll = mpu_gryo_roll * gyroscale / 32767.0f;
    ahrs.gryo_yaw = mpu_gryo_yaw * gyroscale / 32767.0f;
    
    ax = atan2(
             mpu_acc_y,
             sqrt(mpu_acc_x * mpu_acc_x + mpu_acc_z * mpu_acc_z)) *
         180.0f / PI;
    ay = -atan2(
             mpu_acc_x,
             sqrt(mpu_acc_y * mpu_acc_y + mpu_acc_z * mpu_acc_z)) *
         180.0f / PI;

    {
#ifdef MEASURE_ROTER
        //互补滤波求解
        const float a = 0.96f;
        ahrs.degree_pitch = a * (ahrs.degree_pitch + ahrs.gryo_pitch * dt) + (1 - a) * ax;
        ahrs.degree_roll = a * (ahrs.degree_roll + ahrs.gryo_roll * dt) + (1 - a) * ay;
        ahrs.degree_yaw = ahrs.degree_yaw + ahrs.gryo_yaw * dt;
#else
        //四元数求解
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;
        float az;
        float gx, gy, gz;

        ax = mpu_acc_x;
        ay = mpu_acc_y;
        az = mpu_acc_z;

        gx = ahrs.gryo_pitch * PI / 180.0f;
        gy = ahrs.gryo_roll * PI / 180.0f;
        gz = ahrs.gryo_yaw * PI / 180.0f;

        // normalise the measurements
        norm = sqrt(ax * ax + ay * ay + az * az);
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;

        // estimated direction of gravity
        vx = 2 * (q1 * q3 - q0 * q2);
        vy = 2 * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // error is sum of cross product between reference direction of field and direction measured by sensor
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // integral error scaled integral gain
        exInt = exInt + ex * Ki;
        eyInt = eyInt + ey * Ki;
        ezInt = ezInt + ez * Ki;

        // adjusted gyroscope measurements
        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;

        // integrate quaternion rate and normalise
        q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

        // normalise quaternion
        norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;

        Quaternion_ToAngE(&curq);
#endif
    }

    ahrs.time_span = dt;
}

//ahrs系统mpu6050数据更新接口
volatile int16_t MPU6050_ACC_FIFO[3][256] = {{0}};
volatile int16_t MPU6050_GYR_FIFO[3][256] = {{0}};
float MPU6050_Diff[6] = {15, -35, 40, 0};//传感器静差
void ahrs_put_mpu6050(s16 *data)
{
    mpu_gryo_pitch = data[3] + MPU6050_Diff[0];
    mpu_gryo_roll = data[4] + MPU6050_Diff[1];
    mpu_gryo_yaw = data[5] + MPU6050_Diff[2];
    mpu_acc_x = data[0] + MPU6050_Diff[3];
    mpu_acc_y = data[1] + MPU6050_Diff[4];
    mpu_acc_z = data[2] + MPU6050_Diff[5];
}

void ahrs_init()
{
    memset(&ahrs_state, -1, sizeof(ahrs_state));
}
