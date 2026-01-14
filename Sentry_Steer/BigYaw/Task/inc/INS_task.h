/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"
#include "bsp_can.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1
#define RAD_TO_ANGLE_COEF 57.295779513f
#define ANGLE_TO_RAD_COEF 0.0174532925f
#define PI_2 6.2831853072f

typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    float Gyro[3];          // ���ٶ�
    float Accel[3];         // ���ٶ�
    float MotionAccel_b[3]; // ����������ٶ�
    float MotionAccel_n[3]; // ����ϵ���ٶ�

    float AccelLPF; // ���ٶȵ�ͨ�˲�ϵ��

    // ���ٶ��ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // λ��
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    float LastYaw;
    float LastPitch;
    
		//�ٶ�
		float PitchSpeed;
		float YawSpeed;
		float RollSpeed;
		float PitchLastSpeed;
		float YawLastSpeed;
		float LastRoll;
		float PitchSpeedLPF; //���ٶȵ�ͨ�˲�ϵ��
		float YawSpeedLPF;
		float last_Yaw_Angle;
    
} INS_t;
/**
 * @brief ����������װ���Ĳ���,demo�п�����
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;
extern INS_t INS;
void INS_Init(float pitch,float roll);
void INS_Task_EKF(void);
void INS_Task_Mahony(void);
void INS_task(void const *pvParameters);

void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
