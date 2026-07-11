/**
 ******************************************************************************
 * @file    Judge.c
 * @brief   云台数据接收
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalReceive.h"
#include "ChasisController.h"
#include "Referee.h"  // 用于访问sentry_decision_referee

GimbalReceivePack1 gimbal_receiver_pack1;
GimbalReceivePack2 gimbal_receiver_pack2;
int8_t gimbal_receive_1_update; // 更新标志，说明收到了一帧消息
int8_t gimbal_receive_2_update;
int8_t gimbal_receive_3_update;
int16_t jump_up_cnt = 0;

// 上位机下发哨兵坐标 (CAN 0x151)，单位 cm
int16_t sentry_coord_x_cm = 0;
int16_t sentry_coord_y_cm = 0;

float transition_mode_counter; // 计时器

void Gimbal_msgs_Decode1()
{
  enum ROBOT_STATE robot_state = (enum ROBOT_STATE)gimbal_receiver_pack1.robot_state;
  enum CONTROL_TYPE contro_type = (enum CONTROL_TYPE)gimbal_receiver_pack1.control_type;
  enum CONTROL_MODE_ACTION control_mode_action = (enum CONTROL_MODE_ACTION)gimbal_receiver_pack1.control_mode_action;
  enum GIMBAL_ACTION gimbal_action = (enum GIMBAL_ACTION)gimbal_receiver_pack1.gimbal_mode;
  enum SHOOT_ACTION shoot_action = (enum SHOOT_ACTION)gimbal_receiver_pack1.shoot_mode;
  enum CHASSIS_FORMAT chassis_format = (enum CHASSIS_FORMAT)gimbal_receiver_pack1.chassis_fromat;

  enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack1.super_power;
  enum FlyControlState fly_or_not = (enum FlyControlState)gimbal_receiver_pack1.fly_state;

  setRobotState(robot_state);
  setControlMode(contro_type);
  setControlModeAction(control_mode_action);
  setGimbalAction(gimbal_action);
  setShootAction(shoot_action);
	setChassisFormat(chassis_format);

  setSuperPower(power_state);
  setFlyMode(fly_or_not);

  // 运动百分比
  infantry.receive_x_v = gimbal_receiver_pack1.robot_speed_x / 30.0f;
  infantry.receive_y_v= gimbal_receiver_pack1.robot_speed_y / 30.0f;
  infantry.receive_yaw_v = gimbal_receiver_pack1.robot_speed_w / 7.0f;

  // 更新哨兵姿态到裁判系统数据包 (1=进攻, 2=防御, 3=移动, 0=未知)
  if(gimbal_receiver_pack1.sentry_posture >= 1 && gimbal_receiver_pack1.sentry_posture <= 3)
  {
    sentry_decision_referee.sentry_posture = gimbal_receiver_pack1.sentry_posture;
  }
}

void Gimbal_msgs_Decode2()
{
  // 存储上位机下发的哨兵坐标 (CAN 0x151)
  sentry_coord_x_cm = gimbal_receiver_pack2.sentry_x_cm;
  sentry_coord_y_cm = gimbal_receiver_pack2.sentry_y_cm;

  // 更新接收标志
  gimbal_receive_2_update = 1;
}
