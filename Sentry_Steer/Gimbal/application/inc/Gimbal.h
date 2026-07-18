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
#include "GimbalSystemIDConfig.h"
#include "ZeroCheck.h"
#include "my_filter.h"
#include "SignalGenerator.h"
#include "GimbalSystemID.h"

/*==============================================================================
 *                          云台测试结构体定义
 *============================================================================*/
typedef struct CostFunction
{
    float ise;               // ISE: 误差平方积分 ∫e²dt
    float control_cost;      // 控制量惩罚 ∫λ*u²dt
    float total_cost;        // 总目标函数 J = ISE + control_cost
    float cycle_time;        // 当前周期累计时间 (s)
    float max_error;         // 当前周期最大误差
    float final_error;       // 当前周期结束时的误差
    uint8_t cycle_complete;  // 周期完成标志：周期切换时置1，下一周期开始时清零
} CostFunction_t;

/* 云台测试结构体 */
typedef struct GimbalTest
{
    SquareWave pitch_square;      // Pitch方波信号发生器
    // [SMALL_YAW_REMOVED] 小Yaw方波测试相关字段
    // SquareWave small_yaw_square;

    // 目标函数计算（当前周期）
    CostFunction_t pitch_cost;    // Pitch目标函数
    // [SMALL_YAW_REMOVED]
    // CostFunction_t yaw_cost;

    // 周期追踪
    uint16_t last_pitch_cycle;
    // [SMALL_YAW_REMOVED]
    // uint16_t last_yaw_cycle;

    // 历史记录（上一个完整周期的代价）
    float last_pitch_ise;
    float last_pitch_control;
    float last_pitch_max_error;
    // [SMALL_YAW_REMOVED]
    // float last_yaw_ise;
    // float last_yaw_control;
    // float last_yaw_max_error;

} GimbalTest_t;

/*==============================================================================
 *                          云台控制器结构体定义
 *============================================================================*/
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
  TD_t pos_pitch_td;                 // 位置TD：x/θ, dx/ω, ddx/α

  GM6020_Recv pitch_recv;
  GM6020_Info pitch_info;
  GM6020_Recv big_yaw_recv;
  GM6020_Info big_yaw_info;
  // [SMALL_YAW_REMOVED] 小Yaw电机数据接收已删除
  // GM6020_Recv small_yaw_recv;
  // GM6020_Info small_yaw_info;

  float set_pitch_speed;
  float set_pitch_current;
  float set_pitch_angle;
  float set_pitch_vol;
  float comp_pitch_current; // 独立重力补偿输出

  // 陀螺仪信息及其解算
  float gyro_pitch_speed;
  float gyro_pitch_acceleration;
  float gyro_pitch_angle;
  float gyro_last_pitch_angle;
  uint32_t last_cnt;
  float delta_t; // 两帧计算之间的时间差

  float target_pitch_angle; // 设定的角度值
  float target_pitch_speed; // MPC目标角速度, deg/s
  float target_pitch_acceleration; // MPC目标角加速度, deg/s^2

  // Yaw在底盘控制
  // // BIG_YAW
  PID_t big_yaw_current_pid;           // 电流环
  PID_t big_yaw_speed_pid;             // 速度环
  PID_t big_yaw_angle_pid;             // 角度环
  Feedforward_t big_yaw_speed_forward; // 速度环前馈
  Feedforward_t big_yaw_angle_forward; // 角度环前馈
  // [SMALL_YAW_REMOVED] 小Yaw PID/前馈已删除
  // PID_t small_yaw_current_pid;
  // PID_t small_yaw_speed_pid;
  // PID_t small_yaw_angle_pid;
  // Feedforward_t small_yaw_speed_forward;
  // Feedforward_t small_yaw_angle_forward;


  // GM6020_Recv yaw_recv;
  // GM6020_Info yaw_info; // 电机信息
  // [SMALL_YAW_REMOVED] 小Yaw DM电机已删除
  // DM_MIT DM_Small_Yaw_Motor;
  DM_MIT DM_Big_Yaw_Motor;
  DM_MIT DM_Pitch_Motor;

  float set_big_yaw_speed;
  float set_big_yaw_current;
  float set_big_yaw_angle;
  float set_big_yaw_vol;

  /*
   * 大Yaw模型前馈调试量。ref使用TD生成的目标轨迹，三个分量和output
   * 均为乘电机方向符号之前的模型坐标值，便于Ozone采样和实车标定。
   */
  float big_yaw_ff_ref_speed_dps;
  float big_yaw_ff_ref_accel_dps2;
  float big_yaw_ff_inertia;
  float big_yaw_ff_viscous;
  float big_yaw_ff_coulomb;
  float big_yaw_ff_output;
  // [SMALL_YAW_REMOVED] 小Yaw控制量已删除
  // float set_small_yaw_speed;
  // float set_small_yaw_current;
  // float set_small_yaw_angle;
  // float set_small_yaw_vol;

  // 陀螺仪信息及其解算
  float gyro_yaw_speed;
  float gyro_yaw_acceleration;
  float gyro_yaw_angle;
  float gyro_last_yaw_angle;

  float target_big_yaw_angle;
  float target_big_yaw_speed; // MPC目标角速度, deg/s
  float target_big_yaw_acceleration; // MPC目标角加速度, deg/s^2
  // [SMALL_YAW_REMOVED] 小Yaw目标角度和TD已删除
  // float target_small_yaw_angle;
  // TD_t pos_small_yaw_td;
  // TD_t speed_small_yaw_td;
  TD_t pos_big_yaw_td; // 位置跟踪微分器

  // pitch 限位计算
  float pitch_max_gyro_angle;
  float pitch_min_gyro_angle;

  // 云台测试模块
  GimbalTest_t gimbal_test;

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
float Gimbal_Pitch_CalculateFeedforward(float set_point, float set_speed, float set_acceleration);

// 云台测试模块
void GimbalTestInit(GimbalTest_t *test);
void GimbalTestResetCost(GimbalTest_t *test);
void GimbalTestRunPitchCost(GimbalTest_t *test, float error, float control, float delta_t);

// Yaw
float Gimbal_Big_Yaw_Calculate(float set_point);
float Gimbal_Big_Yaw_CalculateFeedforward(float set_point, float set_speed, float set_acceleration);
void Big_Yaw_Bias_Cal(void);
// [SMALL_YAW_REMOVED] 小Yaw计算函数已删除
// float Gimbal_Small_Yaw_Calculate(float set_point);
float Gimbal_Speed_Calculate(float set_point);
float GimbalFrictionModel(void);
void BigYawZeroCheck(void);

#endif // !_GIMBAL_H
