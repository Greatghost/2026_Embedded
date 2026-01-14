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

GimbalReceivePack1 gimbal_receiver_pack1;
//GimbalReceivePack2 gimbal_receiver_pack2;
int8_t gimbal_receive_1_update; // 更新标志，说明收到了一帧消息
int8_t gimbal_receive_2_update;
int8_t gimbal_receive_3_update;
int16_t jump_up_cnt = 0;

float transition_mode_counter; // 计时器

void Gimbal_msgs_Decode1()
{
  enum ROBOT_STATE robot_state = (enum ROBOT_STATE)gimbal_receiver_pack1.robot_state;
  enum CONTROL_TYPE contro_type = (enum CONTROL_TYPE)gimbal_receiver_pack1.control_type;
  enum CONTROL_MODE_ACTION control_mode_action = (enum CONTROL_MODE_ACTION)gimbal_receiver_pack1.control_mode_action;
  enum GIMBAL_ACTION gimbal_action = (enum GIMBAL_ACTION)gimbal_receiver_pack1.gimbal_mode;
  enum SHOOT_ACTION shoot_action = (enum SHOOT_ACTION)gimbal_receiver_pack1.shoot_mode;
  enum CHASSIS_FORMAT chassis_format = (enum CHASSIS_FORMAT)gimbal_receiver_pack1.chassis_fromat;
	enum BOMB_BAY_STATE bomb_bay = (enum BOMB_BAY_STATE)gimbal_receiver_pack1.cover_state;
	
  enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack1.super_power;
  enum FlyControlState fly_or_not = (enum FlyControlState)gimbal_receiver_pack1.fly_state;
	
  setRobotState(robot_state);
  setControlMode(contro_type);
  setControlModeAction(control_mode_action);
  setGimbalAction(gimbal_action);
  setShootAction(shoot_action);
	setChassisFormat(chassis_format);
	setBombBayState(bomb_bay);

  setSuperPower(power_state);
  setFlyMode(fly_or_not);
	
  // 运动百分比
  infantry.receive_x_v = gimbal_receiver_pack1.robot_speed_x / 30.0f;
  infantry.receive_y_v= gimbal_receiver_pack1.robot_speed_y / 30.0f;
  infantry.receive_yaw_v = gimbal_receiver_pack1.robot_speed_w / 30.0f;
}

//void Gimbal_msgs_Decode2()
//{
//  enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack2.super_power;
//  enum FlyControlState fly_or_not = (enum FlyControlState)gimbal_receiver_pack2.fly_state;
//  setSuperPower(power_state);
//  setFlyMode(fly_or_not);
//}
