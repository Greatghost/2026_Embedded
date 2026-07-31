#ifndef _CHASSIS_SEND_H
#define _CHASSIS_SEND_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "remote_control.h"
#include "ins_task.h"
#include "Gimbal.h"
#include "ChassisSolver.h"
#include "pc_serial.h"

#pragma pack(push, 1)

//typedef struct ChassisSendPack1
//{
//  uint16_t robot_state : 1;
//  uint16_t control_type : 2;
//  uint16_t control_mode_action : 3;
//  uint16_t gimbal_mode : 3;
//  uint16_t shoot_mode : 3;
//  uint16_t chassis_format : 1; // 底盘形状
//  uint16_t is_pc_on : 1;
//  uint8_t autoaim_id; // 自瞄ID

//  uint8_t extra_leg_mode;
//  uint8_t cover_state : 1;	// 弹舱盖状态
//  uint8_t reserved : 7;

//  int8_t robot_speed_x; // * 10 描述 x方向为云台正方向
//  int8_t robot_speed_y; // * 10描述
//  int8_t robot_speed_w;
//} ChassisSendPack1;

//typedef struct ChassisSendPack2 // 云台yaw和pitch角度
//{
//  int16_t yaw_motor_angle; // yaw轴电机角度
//  int16_t gimbal_pitch;    // 云台pitch角度
//  int16_t gimbal_yaw_speed;

//	uint16_t super_power : 1;	
//	uint16_t fly_state : 1;	
//} ChassisSendPack2;

typedef struct ChassisSendPack1
{
  // 第一个uint16_t位域: 16位 = 2字节
  uint16_t robot_state : 1;
  uint16_t control_type : 2;
  uint16_t control_mode_action : 3;
  uint16_t gimbal_mode : 3;
  uint16_t shoot_mode : 3;
  uint16_t chassis_format : 1;
  uint16_t is_pc_on : 1;
  uint16_t super_power : 1;
  uint16_t fly_state : 1;        // 共16位

  // 第二个uint8_t位域: 8位 = 1字节
  uint8_t through_hole_flag : 1;
  uint8_t sentry_posture : 3;    // 哨兵姿态: 1=进攻, 2=防御, 3=移动, 4=强化进攻, 5=强化防御, 6=强化移动, 0=未知
  uint8_t pc_control_active : 1; // 1 while chassis motion is controlled by the PC
  uint8_t reserved_bits : 3;

  // yaw_motor_angle: 2字节
  int16_t yaw_motor_angle;       // 云台yaw轴电机角度

  // 速度数据: 3字节
  int8_t robot_speed_x;          // x方向速度
  int8_t robot_speed_y;          // y方向速度
  int8_t robot_speed_w;          // 小陀螺旋转速度

  // 总计: 2 + 1 + 2 + 3 = 8字节
} ChassisSendPack1;
typedef char ChassisSendPack1_SizeMustBe8[
    (sizeof(ChassisSendPack1) == 8U) ? 1 : -1];

// TypeID=0x01 哨兵坐标 → CAN 0x151
typedef struct ChassisSendPack2
{
	int16_t sentry_x_cm;    // 哨兵X坐标 (cm)
	int16_t sentry_y_cm;    // 哨兵Y坐标 (cm)
	uint8_t reserved[4];    // 预留凑满8字节
} ChassisSendPack2;

#pragma pack(pop)

extern ChassisSendPack1 chassis_send_pack1;
extern ChassisSendPack2 chassis_send_pack2;

void Pack_InfantryMode(void);
void Pack_SentryCoord(void);

#endif // !_CHASSIS_SEND_H
