/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
//#include "controller.h"
#include "pid.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "main.h"
#include "debug.h"
#include "iwdgTask.h"
INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

float INS_Gyro_y = 0;
float INS_Gyro_x = 0;
float INS_Gyro_z = 0;
float Last_INS_Gyro_y = 0;
float Last_INS_Gyro_x = 0;
float Last_INS_Gyro_z = 0;
float INS_Gyro_xz = 0;

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 37.25f;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(float pitch,float roll)
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(12, 0.0012, 20000000, 1, 0,pitch,roll);
    // imu heat init
    PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    INS.AccelLPF = 0.0085;
}
float yaw_bais = 0;
float Gyro_Threhold = 0.000;
uint16_t psc = 10;
uint16_t pwm = 10000;
void INS_Task_EKF(void)
{
    static uint32_t count = 0;
	static uint32_t delay_time = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;
	if(delay_time < 500)
	{
		buzzer_on(psc, pwm);
		delay_time++;
		return;
	}
	else if (delay_time == 500)
	{
		/* IMU 预热结束,只执行一次 buzzer_off 并恢复 PSC=9 (buzzer_on 改成了 10)
		 * 之后不再操作 TIM4,避免覆盖 BuzzerAlarmTask 设置的 CCR3 */
		buzzer_off();
		__HAL_TIM_SET_PRESCALER(&htim4, 9);
		delay_time++;
	}
    // ins update
    if ((count % 1) == 0)
    {
//		BMI088.Gyro[X] += BMI088.GyroOffset[X];
//		BMI088.Gyro[Y] += BMI088.GyroOffset[Y];
//		BMI088.Gyro[Z] += BMI088.GyroOffset[Z];
			//已经加过一次
        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];
		INS_Gyro_x = INS.Gyro[X] * -57.30f;
		INS_Gyro_y = INS.Gyro[Y] * -57.30f;
		INS_Gyro_z = INS.Gyro[Z] * -57.30f;

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        //IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        // 获取最终数据
		if(BMI088.Gyro[Z]<Gyro_Threhold && BMI088.Gyro[Z]> -Gyro_Threhold && 0)
		{
			INS.Yaw = INS.last_Yaw_Angle;
			yaw_bais = QEKF_INS.Yaw-INS.Yaw;
			INS.YawTotalAngle = INS.last_Yaw_Angle;
		}
		else 
		{	
			INS.Yaw = QEKF_INS.Yaw - yaw_bais;
			INS.YawTotalAngle = QEKF_INS.YawTotalAngle - yaw_bais;
		}
        INS.Roll = QEKF_INS.Pitch;
        INS.Pitch = - QEKF_INS.Roll;//C板倒放
        float k = 0.2;
//		INS.PitchSpeed = -(INS_Gyro_y * k +(1-k)*Last_INS_Gyro_y);
//        INS.YawSpeed = (INS_Gyro_z * k +(1-k)*Last_INS_Gyro_z);
		INS.PitchSpeed = (INS.Pitch - INS.LastPitch)/dt;;
        INS.YawSpeed = (INS.YawTotalAngle-INS.LastYaw)/dt;
		Last_INS_Gyro_y = INS_Gyro_y;
		Last_INS_Gyro_z = INS_Gyro_z;
        
		INS.RollSpeed = (INS.Roll - INS.LastRoll)/dt;
		INS.LastYaw = INS.YawTotalAngle;
		INS.LastPitch = INS.Pitch;
		INS.LastRoll = INS.Roll;
		INS.last_Yaw_Angle = INS.Yaw;
		
		
    }
    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }
    count++;
}
void INS_Task_Mahony(void)
{			
	
}
float p = 0;
float r = 0;
void INS_task(void const *pvParameters)
{
	portTickType xLastWakeTime;
   const portTickType xFrequency = 1; // 1000HZ
	uint16_t imu_online_check_count = 0;
	BMI088_Read(&BMI088);
	float ax=BMI088.Accel[X];
	float ay=BMI088.Accel[Y];
	float az=BMI088.Accel[Z];
	float pitch = -atan(ax/az);
	float roll = atan(ay/az);
	p = pitch/3.1415926*180;
	r = roll;
	INS_Init(pitch,roll);
    /* Infinite loop */
    for (;;)
    {
		xLastWakeTime = xTaskGetTickCount();
		BMI088_Read(&BMI088);

		/*
		 * 50Hz读取两颗芯片ID作为真实硬件心跳。只有WHO_AM_I正确才刷新
		 * 对应计数；SPI断线或单芯片失效会在Offline_task中超时。
		 */
		if (++imu_online_check_count >= 20u)
		{
			const uint8_t imu_online = BMI088_CheckOnline();
			imu_online_check_count = 0;
			if (imu_online & BMI088_ACCEL_ONLINE)
			{
				LossUpdate(&global_debugger.imu_debugger[0], 0.03f);
			}
			if (imu_online & BMI088_GYRO_ONLINE)
			{
				LossUpdate(&global_debugger.imu_debugger[1], 0.03f);
			}
		}
		INS_Task_EKF();
		Iwdg_ReportAlive(IWDG_HEARTBEAT_INS);
		/*  延时  */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 互补滤波部分，出现跳变去掉
//float pure_acc_pitch = 0.0f;
//float pure_acc_roll = 0.0f;	
//float pitch_w_cal;
//float roll_w_cal;
//float T = 0.001;
//void INS_Task_Mahony(void)
//{			
//	pure_acc_pitch = -atan2((double)BMI088.Accel[0],(double)BMI088.Accel[2])/ 3.1415926f * 180.0f;
//	pure_acc_roll  = atan2((double)BMI088.Accel[1],(double)BMI088.Accel[2])/ 3.1415926f * 180.0f;
//	
//	float acc_norm = BMI088.Accel[0]*BMI088.Accel[0]+BMI088.Accel[1]*BMI088.Accel[1]+BMI088.Accel[2]*BMI088.Accel[2];
//	//各轴角速度解算
//	pitch_w_cal =  (BMI088.Gyro[1]);//基于底盘坐标系的pitch轴
//	roll_w_cal  = BMI088.Gyro[0]*cos(INS.Roll*3.1415926f/180.0f)-BMI088.Gyro[2]*sin(INS.Roll*3.1415926f/180.0f)*cos(INS.Pitch*3.1415926f/180.0f)+\
//		BMI088.Gyro[1]*sin(INS.Roll*3.1415926f/180.0f)*sin(INS.Pitch*3.1415926f/180.0f);
//	
//	float k_filter_pitch_roll = -0.002*fabsf(acc_norm-gNORM*gNORM ) +0.015f ;//+ k_com_w*fabsf(pitch_w_acc);//自适应参数
//		
//	k_filter_pitch_roll = LIMIT_MAX_MIN(0.015f,0.0f,k_filter_pitch_roll);
//	
//	
//	float Pitch = (INS.Roll + pitch_w_cal*T/3.1415926*180.0f)*(1 - k_filter_pitch_roll) + pure_acc_pitch*k_filter_pitch_roll;
//	float Roll  = (INS.Pitch  +roll_w_cal*T/3.1415926f * 180.0f)*(1 - k_filter_pitch_roll) + pure_acc_roll *k_filter_pitch_roll;
//	
//	INS.Roll = Pitch;
//	INS.Pitch = Roll;
//	
//	float sinp = sinf(INS.Pitch/ 3.1415926f * 180.0f);
//	float siny = sinf(INS.Yaw/ 3.1415926f * 180.0f);
//	float sinr = sinf(INS.Roll/ 3.1415926f * 180.0f);
//	
//	float cosp = cosf(INS.Pitch/ 3.1415926f * 180.0f);
//	float cosy = cosf(INS.Yaw/ 3.1415926f * 180.0f);
//	float cosr = cosf(INS.Roll/ 3.1415926f * 180.0f);
//	
//	//更新四元数
//	QEKF_INS.q[0] = cosr*cosp*cosy + sinr*sinp*siny;
//    QEKF_INS.q[1] = sinr*cosp*cosy - cosr*sinp*siny;
//    QEKF_INS.q[2] = cosr*sinp*cosy + sinr*cosp*siny;
//    QEKF_INS.q[3] = cosr*cosp*siny - sinr*sinp*cosy;

//    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
//}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
/**
 * @brief 温度控制
 * 
 */
void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
