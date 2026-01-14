#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "math.h"
#include "pid.h"
#include "GM6020.h"
#include "ins_task.h"
#include "DM_Motor.h"
#include "M2006.h"

#include "my_filter.h"
#include "TD.h"
#include "bsp_dwt.h"

#include "gimbal_config.h"
#include "ZeroCheck.h"
#include "my_filter.h"

typedef struct GimbalController
{
  //// 弹舱盖
  // PID_t bay_pos_pid;
  // PID_t bay_speed_pid;
  // M2006_Recv bay_recv;
  // M2006_Info bay_info;
  // Pitch 轴
  PID_t pitch_current_pid;           // 电流环
  PID_t pitch_speed_pid;             // 速度环
  PID_t pitch_angle_pid;             // 角度环
  Feedforward_t pitch_speed_forward; // 速度环前馈
  Feedforward_t pitch_angle_forward; // 角度环前馈

  GM6020_Recv pitch_recv;
  GM6020_Info pitch_info;
  GM6020_Recv big_yaw_recv;
  GM6020_Info big_yaw_info;
  GM6020_Recv small_yaw_recv;
  GM6020_Info small_yaw_info;

  float set_pitch_speed;
  float set_pitch_current;
  float set_pitch_angle;
  float set_pitch_vol;
  float comp_pitch_current; // 重力补偿

  // 陀螺仪信息及其解算
  float gyro_pitch_speed;
  float gyro_pitch_angle;
  float gyro_last_pitch_angle;
  uint32_t last_cnt;
  float delta_t; // 两帧计算之间的时间差

  float target_pitch_angle; // 设定的角度值

  // Yaw在底盘控制
  // // BIG_YAW
  PID_t big_yaw_current_pid;           // 电流环
  PID_t big_yaw_speed_pid;             // 速度环
  PID_t big_yaw_angle_pid;             // 角度环
  Feedforward_t big_yaw_speed_forward; // 速度环前馈
  Feedforward_t big_yaw_angle_forward; // 角度环前馈
  // // SMALL_YAW
  PID_t small_yaw_current_pid;           // 电流环
  PID_t small_yaw_speed_pid;             // 速度环
  PID_t small_yaw_angle_pid;             // 角度环
  Feedforward_t small_yaw_speed_forward; // 速度环前馈
  Feedforward_t small_yaw_angle_forward; // 角度环前馈


  // GM6020_Recv yaw_recv;
  // GM6020_Info yaw_info; // 电机信息
  DM_MIT DM_Small_Yaw_Motor;
	DM_MIT DM_Big_Yaw_Motor;
  DM_MIT DM_Pitch_Motor;

  float set_big_yaw_speed;
  float set_big_yaw_current;
  float set_big_yaw_angle;
  float set_big_yaw_vol;
  float set_small_yaw_speed;
  float set_small_yaw_current;
  float set_small_yaw_angle;
  float set_small_yaw_vol;

  // 陀螺仪信息及其解算
  float gyro_yaw_speed;
  float gyro_yaw_angle;
  float gyro_last_yaw_angle;

  float target_big_yaw_angle;
  float target_small_yaw_angle;

  TD_t pos_small_yaw_td; // 位置跟踪微分器
  TD_t speed_small_yaw_td;
  TD_t pos_big_yaw_td; // 位置跟踪微分器
  TD_t speed_big_yaw_td;

  // pitch 限位计算
  float pitch_max_gyro_angle;
  float pitch_min_gyro_angle;

} GimbalController;

#define GIMBAL_INIT_WAIT_TIME  150
typedef struct BigYawController
{
  float dynamic_angle_big;//根据小yaw的角度差值动态设置大yaw的设定角度，包括电机角和陀螺仪角
  float big_yaw_gyro_raw;
  float big_yaw_gyro_speed;
  ZeroCheck_Typedef big_yaw_gyro_zerocheck;
  float big_yaw_gyro_after_zerocheck;
  float big_yaw_speed_raw;
  float big_yaw_gyro_bias;//现在用来修正小yaw上电歪头
  float dealed_big_yaw_gyro;
  uint8_t gimbal_last_mode;//下电为0，其他为1
	uint8_t big_yaw_mode;//辅瞄是否锁到目标
  uint8_t gimbal_enable_flag;
  uint16_t gimbal_enable_cnt;

} BigYawController;

extern GimbalController gimbal_controller;
extern BigYawController big_yaw_controller;

void GimbalPidInit(void);
void GimbalClear(void);
void updateGyro(void);

// pitch
void limitPitchAngle(void);
float GimbalPitchComp(void);
float Gimbal_Pitch_Calculate(float set_point);

// Yaw
float Gimbal_Big_Yaw_Calculate(float set_point);
void Big_Yaw_Bias_Cal(void);
float Gimbal_Small_Yaw_Calculate(float set_point);
float Gimbal_Speed_Calculate(float set_point);
float GimbalFrictionModel(void);
void BigYawZeroCheck(void);

#endif // !_GIMBAL_H
