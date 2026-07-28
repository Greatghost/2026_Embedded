#ifndef _CHASIS_CONTROLLER_H
#define _CHASIS_CONTROLLER_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_atan2_f32.h"

#include "tools.h"
#include "counter.h"
#include "ins.h"
#include "user_lib.h"
#include "remote_control.h"
#include "pid.h"

#include "wheel_ins.h"

#include "Offline_Task.h"

#include "can_config.h"
#include "TD.h"

#include "SuperPower.h"
#include "PowerLimit.h"

#include "M3508.h"
#include "GM6020.h"
#include "can_send.h"

#include "robot_config.h"
#include "mecanum.h"
#include "omni.h"
#include "steer.h"

#include "PowerControlTask.h"
#include "NingCap.h"
#include "RefereeTask.h"

typedef struct Vector // 向量
{
  float module; // 模值
  float angle;  // 角度
} Vector;

typedef struct Sensors
{
  GM6020_Recv steer_recv[4];
  GM6020_Info steer_decode[4]; // 解码过后的舵信息,速度未经过滤波
  M3508_Recv wheels_recv[4];
  M3508_Info wheels_decode[4];     // 速度已经经过滤波
  M3508_Info wheels_decode_raw[4]; // 速度未经过滤波
} Sensors;

typedef struct ExcuteTorque
{
  int8_t steers_send_data[8];
  int8_t wheels_send_data[8];

  float steers_set_current[4];
  float wheels_set_current[4];
} ExcuteTorque;

// @TODO :增加四向跟随模式

typedef enum
{
  TWO_SIDES_FOLLOW,     // 双向跟随
  FOUR_SIDES_FOLLOW,    // 四向跟随
  FOUR_SIDES_FOLLOW_45, // 45度四向跟随
  TWO_SIDES_LEFT_RIGHT, // 双向左右跟随
} CHASSIS_FOLLOW_TYPE;

/* 底盘跟随方向  */
typedef enum
{
  CHASSIS_FRONT,
  CHASSIS_BACK,
  CHASSIS_LEFT,
  CHASSIS_RIGHT,
} chassis_direction_e;

typedef struct Infantry
{
  /*  传感器信息 */
  Sensors sensors_info;

  /*  执行机构 */
  ExcuteTorque excute_info;

  /* 移动控制  */
  float target_yaw_v; // rad/s  仅底盘运动模式下才生效，其余模式均由底盘板自动决策
  float target_x_v;   // target_x_v_percent * speed_x_max
  float target_y_v;

  // float target_x_v_percent; // 0.0 -1.0f
  // float target_y_v_percent;
  // float target_yaw_v_percent;
  float receive_x_v;
  float receive_y_v;
  float receive_yaw_v;
  

  float speed_x_max;
  float speed_y_max;
  float speed_yaw_max;

  // 实际控制量
  float set_yaw_v; // rad/s
  float set_x_v;
  float set_y_v;

  // 跟踪微分器
  TD_t x_v_td;
  TD_t y_v_td;
  TD_t yaw_v_td;
	
	TD_t steer_angle_td[4];

  // 实际速度
  float x_v;
  float y_v;
  float yaw_v;

  // 轮电机设定速度
  float wheels_set_v[4];

  uint32_t last_cnt;
  float delta_t; // 前后两帧时间差

  CHASSIS_TYPE chassis_type;
  chassis_direction_e chassis_direction;
  CHASSIS_FOLLOW_TYPE chassis_follow_type;
  YAW_MOTOR_TYPE yaw_motor_type;

  // 跟随参数
  float error_angle;

  // 异常检测
  int16_t abnormal_count; // 异常次数计数
  int8_t is_abnormal;     // 是否正在异常状态

  // 转向PID
  PID_t turn_pid;
  float target_pid_yaw_v;

  // 麦轮底盘跟随前馈
  Feedforward_t Mecanum_Follow_FF;
  float Mecanum_Follow_FF_Coefficient[3];

  // 轮控PID
  PID_t wheels_pid[4];
  float wheels_set_current;

  // 舵控PID
  PID_t steers_speed_pid[4];
  PID_t steers_angle_pid[4];
  float Steer_Speed_Setpoint[4];
  float steers_set_current;

  // 舵轮组件
  int8_t steer_wheel_install_direction[4]; // 舵轮轮电机安装方向修正
  uint16_t steer_init_encoder[4];          // 初始编码器角度
  Vector steer_vector[4];                  // 舵向量
  Vector robot_vector;                     // 底盘运动向量
  // 舵轮6020前馈
  Feedforward_t Steer_6020_FF[4];
  float Steer_6020_FF_Coefficient[3];

  float cos_dir; // 云台与底盘的夹角
  float sin_dir;

#ifdef STEER_TORQUE_FEEDFORWARD
  /* === 整车扭矩前馈（整车速度外环PID + 动力学扭矩分配） === */
  PID_t chassis_translate_x_pid;  /* 平动x外环PID（云台坐标系，输入m/s，输出N）        */
  PID_t chassis_translate_y_pid;  /* 平动y外环PID（云台坐标系，输入m/s，输出N）        */
  PID_t chassis_rotate_pid;       /* 旋转外环PID（输入rad/s，输出N·m）                  */
  float wheels_ff_current[4];     /* 各轮前馈电流输出（C620电流值，调试/Ozone观察用）   */
#endif

  // 功率控制
  PowerLimiter power_limiter;
  float set_power; // 机器人功率设置
  PowerLimitMethod power_limit_method;

  // 测试数据
  float wheels_send_current[4];
	
	// 复活后电机重新上电等待时间
	uint32_t Timer;
	float Motor_Init_Time;
	int All_Motor_On;

} Infantry;

extern Infantry infantry;

void InfantryInit(Infantry *infantry);
void main_control(Infantry *infantry);
void wheels_power_limit(Infantry *infantry);
void execute_control(ExcuteTorque *torque);
void get_sensors_info(Sensors *sensors_info);

// 设置机器人功率以及控制其速度
void set_robot_speed(Infantry *infantry);

// 加速策略
void wheels_accel(Infantry *infantry);

#endif // !_CHASIS_CONTROLLER_H
