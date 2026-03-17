#ifndef _GIMBAL_RECEIVE_H
#define _GIMBAL_RECEIVE_H

#include "stdint.h"

#include "remote_control.h"

#pragma pack(push, 1)

//typedef struct GimbalReceivePack1
//{
//  uint16_t robot_state : 1;
//  uint16_t control_type : 2;
//  uint16_t control_mode_action : 4;
//  uint16_t gimbal_mode : 3;
//  uint16_t shoot_mode : 3;
//  uint16_t chassis_fromat : 2;
//  uint16_t is_pc_on : 1;
//  uint8_t autoaim_id; // 自瞄ID

//  uint8_t __;
//  uint8_t cover_state : 1;
//  uint8_t reserved : 7;

//  int8_t robot_speed_x; // * 10 描述 x方向为云台正方向
//  int8_t robot_speed_y; // * 10描述
//  int8_t robot_speed_w;
//} GimbalReceivePack1;

//typedef struct GimbalReceivePack2 // 云台yaw和pitch角度
//{
//  int16_t yaw_motor_angle; // 云台yaw轴电机角度
//  int16_t gimbal_pitch;    // 底盘//不画这个UI了
//  int16_t gimbal_yaw_speed;
//  uint16_t super_power : 1;
//  uint16_t fly_state : 1;
//} GimbalReceivePack2;

typedef struct GimbalReceivePack1
{
  // 第一个uint16_t位域: 16位 = 2字节
  uint16_t robot_state : 1;
  uint16_t control_type : 2;
  uint16_t control_mode_action : 3;
  uint16_t gimbal_mode : 3;
  uint16_t shoot_mode : 3;
  uint16_t chassis_fromat : 1;
  uint16_t is_pc_on : 1;
  uint16_t super_power : 1;
  uint16_t fly_state : 1;        // 共16位

  // 第二个uint8_t位域: 8位 = 1字节
  uint8_t through_hole_flag : 1;
  uint8_t sentry_posture : 2;    // 哨兵姿态: 1=进攻, 2=防御, 3=移动, 0=未知
  uint8_t reserved_bits : 5;     // 共8位

  // yaw_motor_angle: 2字节
  int16_t yaw_motor_angle;       // 云台yaw轴电机角度

  // 速度数据: 3字节
  int8_t robot_speed_x;          // x方向速度
  int8_t robot_speed_y;          // y方向速度
  int8_t robot_speed_w;          // 小陀螺旋转速度

  // 总计: 2 + 1 + 2 + 3 = 8字节
} GimbalReceivePack1;

#pragma pack(pop)

//extern GimbalReceivePack2 gimbal_receiver_pack2;
extern GimbalReceivePack1 gimbal_receiver_pack1;

extern int8_t gimbal_receive_1_update;
extern int8_t gimbal_receive_2_update;

void Gimbal_msgs_Decode1(void);
void Gimbal_msgs_Decode2(void);

#endif // !_GIMBAL_RECEIVE_H
