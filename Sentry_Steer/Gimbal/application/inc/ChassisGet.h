#ifndef _CHASSIS_GET_H
#define _CHASSIS_GET_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#pragma pack(push, 1)

typedef struct ChassisGetPack_1
{
  uint8_t is_shootable : 1;     // 热量限制标志位
  uint8_t robot_color : 1;      // 机器人颜色，红色为1，蓝色为0
  uint8_t bullet_level : 2;     // 发射子弹速度上限
  uint8_t buff_state : 4;       // bit 0：机器人血量补血状态 bit 1：枪口热量冷却加速 bit 2：机器人防御加成 bit 3：机器人攻击加成
  uint8_t half_CapVol : 4;      // 电容电压/2
  uint8_t is_balance_robot : 3; // 敌方几号装甲板是平衡步兵,分别对应3，4，5
  uint8_t symbol_flag[4];
  uint16_t bullet_speed;
} ChassisGetPack_1;

// debug
// typedef struct ChassisGetPack_2
// {
//   // uint16_t bullet_speed;
//   uint8_t jump_state; // 跳跃状态
//   uint16_t tof_dis;   // tof距离
//   short speed;        // 速度
// } ChassisGetPack_2;

#pragma pack(pop)

extern ChassisGetPack_1 chassis_pack_get_1;
//extern ChassisGetPack_2 chassis_pack_get_2;

#endif // !_CHASSIS_GET_H
