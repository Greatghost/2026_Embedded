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

// 底盘速度接收数据结构：通过舵电机角度和轮电机速度反解的底盘实际速度
typedef struct ChassisSpeedRecv
{
  int16_t chassis_x_v_100;    // 底盘x方向实际速度 * 100，单位0.01 m/s
  int16_t chassis_y_v_100;    // 底盘y方向实际速度 * 100，单位0.01 m/s
  int16_t chassis_yaw_v_100;  // 底盘角速度 * 100，单位0.01 rad/s
  uint8_t reserve[2];         // 保留字节
} ChassisSpeedRecv_t;

// 射击数据接收结构 (0x0207, CAN ID 0x09B)
typedef struct ShootDataRecv
{
  uint8_t  bullet_type;       // 弹丸类型：1-17mm/2-42mm
  uint8_t  shooter_id;        // 发射机构ID：1-1号17mm/2-2号17mm/3-1号42mm
  uint8_t  bullet_freq;       // 发射频率（发/秒）
  float    bullet_speed;      // 弹丸初速度（m/s）
} ShootDataRecv_t;

// 哨兵信息接收结构 (0x020D, CAN ID 0x09C)
typedef struct SentryInfoRecv
{
  uint32_t sentry_info;       // 0x020D offset 0 (4字节)
  uint16_t sentry_info_2;     // 0x020D offset 4 (2字节)
  uint8_t  reserve[2];        // 填充到8字节
} SentryInfoRecv_t;

// 弹量扩展字段接收结构 (0x0208扩展, CAN ID 0x09D)
typedef struct BulletExtendedRecv
{
  uint16_t projectile_allowance_42mm;    // 42mm弹丸剩余发射数
  uint16_t remaining_gold_coin;          // 剩余金币数量
  uint16_t projectile_allowance_fortress; // 堡垒储备17mm允许发弹量
  uint8_t  rfid_status_2;                // RFID状态扩展8bit (0x0209 offset 4)
  uint8_t  reserve;                      // 填充到8字节
} BulletExtendedRecv_t;

#pragma pack(pop)

extern ChassisGetPack_1 chassis_pack_get_1;
extern ChassisSpeedRecv_t chassis_speed_recv;
extern ShootDataRecv_t shoot_data_recv;       // 射击数据接收
extern SentryInfoRecv_t sentry_info_recv;     // 哨兵信息接收
extern BulletExtendedRecv_t bullet_extended_recv;  // 弹量扩展数据接收

#endif // !_CHASSIS_GET_H