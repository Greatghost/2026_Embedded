#ifndef _GIMBAL_SEND_H
#define _GIMBAL_SEND_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "can1.h"
#include "can2.h"
#include "can_config.h"

#include "HeatControl.h"

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
} GimbalSendPack_1;

typedef struct{
	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Enemy_outpost : 6; //敌方哨兵是否无敌
	uint8_t Robot_Red_Blue : 1; //1 -> red ; 0 -> blue
	uint8_t self_outpost : 6;
	uint8_t sentry_posture : 2;  // 哨兵姿态(来自裁判系统0x020D): 1=进攻, 2=防御, 3=移动, 0=未知
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001
} JudgeData_ForSend1_t;


typedef struct{
	uint16_t x; //裁判系统给的机器人坐标(从float 映射到 uint16_t : float*100 -> uint16_t)
	uint16_t y;
	int16_t yaw_10;
	uint16_t Self_blood;
} JudgeData_ForSend2_t;


typedef struct{
	uint8_t blood_type; //0 -> Friends,1 -> Enemy
	uint16_t ID1 : 6;
	uint16_t ID2 : 6;
	uint16_t ID3 : 6;
	uint16_t ID4 : 6;
	uint16_t ID_reserve : 6;
	uint16_t ID7 : 6;
	uint16_t ID8 : 6;//基地，无法获取时显式指定为0
	uint32_t reserve : 14;
}JudgeBloodData_ForSend1_t;

typedef struct{
	uint8_t data_type; // 0为JudgeData_Buff_t
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
	
}JudgeData_Buff_t; // 裁判系统0x0204

typedef struct{
	uint8_t data_type; // 1为JudgeData_RFID_t
	ext_event_data_t event_data;
	uint32_t rfid_status : 24;
	
	
}JudgeData_RFID_t; // 裁判系统0x0204

typedef struct{
	uint8_t position_type; //00X -> Friends,10X -> Enemy
	int16_t ID_X_100; //乘了100
	int16_t ID_Y_100;
	uint16_t reserve;
	uint8_t reserve_1;
	
}Each_Robot_position_t;

typedef struct{
	Each_Robot_position_t Friend[8];//0~7,但0,5,6不填
	Each_Robot_position_t Enemy[8];//0~7,但0,5,6不填
}JudgeData_position_t;

// 底盘速度数据包：通过舵电机角度和轮电机速度反解的底盘实际速度
typedef struct ChassisSpeedPack
{
	int16_t chassis_x_v_100;    // 底盘x方向实际速度 * 100，单位0.01 m/s
	int16_t chassis_y_v_100;    // 底盘y方向实际速度 * 100，单位0.01 m/s
	int16_t chassis_yaw_v_100;  // 底盘角速度 * 100，单位0.01 rad/s
	uint8_t reserve[2];         // 保留字节
} ChassisSpeedPack_t;

// 射击数据发送包 (0x0207), 用于云台TypeID 7/8
typedef struct ShootData_ForSend
{
	uint8_t  bullet_type;       // 弹丸类型：1-17mm/2-42mm
	uint8_t  shooter_id;        // 发射机构ID：1-1号17mm/2-2号17mm/3-1号42mm
	uint8_t  bullet_freq;       // 发射频率（发/秒）
	float    bullet_speed;      // 弹丸初速度（m/s）
} ShootData_ForSend_t;

// 哨兵信息发送包 (0x020D), 用于云台TypeID 7
typedef struct SentryInfo_ForSend
{
	uint32_t sentry_info;       // 0x020D offset 0 (4字节)
	uint16_t sentry_info_2;     // 0x020D offset 4 (2字节)
	uint8_t  reserve[2];        // 填充到8字节
} SentryInfo_ForSend_t;

// 弹量扩展字段发送包 (0x0208扩展), 用于云台TypeID 8
typedef struct BulletExtended_ForSend
{
	uint16_t projectile_allowance_42mm;    // 42mm弹丸剩余发射数
	uint16_t remaining_gold_coin;          // 剩余金币数量
	uint16_t projectile_allowance_fortress; // 堡垒储备17mm允许发弹量
	uint8_t  rfid_status_2;                // RFID状态扩展8bit (0x0209 offset 4)
	uint8_t  reserve;                      // 填充到8字节
} BulletExtended_ForSend_t;

// 接收云台转发的SentryCmd
typedef struct SentryCmd_FromGimbal
{
	uint32_t sentry_cmd;        // 32-bit命令字，对应裁判0x0120
	uint8_t  reserve[4];        // 填充到8字节
} SentryCmd_FromGimbal_t;


#pragma pack(pop)

extern GimbalSendPack_1 gimbal_pack_send_1;
extern ChassisSpeedPack_t chassis_speed_pack_send;
extern ShootData_ForSend_t shoot_data_send;
extern SentryInfo_ForSend_t sentry_info_send;
extern BulletExtended_ForSend_t bullet_extended_send;
extern SentryCmd_FromGimbal_t sentry_cmd_from_gimbal;  // 接收云台转发的SentryCmd

void GimbalSendPack(void);
void JudgeDataCanSend(void);
void ChassisSpeedPack(void);  // 底盘速度数据打包发送
void ShootDataPack(void);     // 射击数据打包发送 (TypeID 7/8)
void SentryInfoPack(void);    // 哨兵信息打包发送 (TypeID 7)
void BulletExtendedPack(void); // 弹量扩展字段打包发送 (TypeID 8)

#endif // !_GIMBAL_SEND_H
