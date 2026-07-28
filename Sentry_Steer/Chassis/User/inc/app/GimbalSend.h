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
	uint8_t reserve_1bit : 1;  // 保留位, 填充到2字节
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001
} JudgeData_ForSend1_t;  // sizeof == 8 bytes (位域15bits+1bit填充=2字节 + 3×uint16=6字节)


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
	uint8_t  reserve;            // 填充到8字节
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

// 小地图下发指令发送包 (0x0303)
typedef struct RobotCommand_ForSend
{
	int16_t  target_position_x_100;  // 目标X坐标 (float*100 → int16)
	int16_t  target_position_y_100;  // 目标Y坐标 (float*100 → int16)
	uint8_t  cmd_keyboard;           // 键盘按键命令
	uint8_t  target_robot_id;        // 目标机器人ID
	uint16_t cmd_source;             // 指令来源
} RobotCommand_ForSend_t;

// 哨兵姿态时长发送包 (0x020D扩展, 20260713协议更新)
typedef struct SentryDuration_ForSend
{
	uint8_t normal_attack_duration;    // 哨兵进攻姿态弱化前剩余秒数
	uint8_t normal_defend_duration;    // 哨兵防御姿态弱化前剩余秒数
	uint8_t normal_move_duration;      // 哨兵移动姿态弱化前剩余秒数
	uint8_t reserved_duration_1;       // 保留
	uint8_t enhanced_attack_duration;  // 哨兵强化进攻姿态剩余秒数
	uint8_t enhanced_defend_duration;  // 哨兵强化防御姿态剩余秒数
	uint8_t enhanced_move_duration;    // 哨兵强化移动姿态剩余秒数
	uint8_t reserved_duration_2;       // 保留
} SentryDuration_ForSend_t;           // sizeof = 8，恰好一个CAN帧

// 伤害值差发送包 (0x0003, 20260713协议更新)
typedef struct DamageDiff_ForSend
{
	int16_t damage_difference;     // 伤害值差（己方血量总和 - 敌方血量总和）
	uint8_t reserve[6];            // 填充到8字节
} DamageDiff_ForSend_t;

// 电机掉线状态发送包 (CAN 0x0A1, 2026-07-19新增)
// 位图：bit=1 表示电机掉线
//   bit0-3 : 轮电机1-4 (DJI_3508_MOTORS_1..4)
//   bit4-7 : 舵电机1-4 (DJI_6020_MOTORS_1..4)
typedef struct MotorOffline_ForSend
{
	uint8_t motor_offline_bitmap;
	uint8_t reserve[7];            // 填充到8字节
} MotorOffline_ForSend_t;

// UWB角度+舵角发送包 (CAN 0x0A2, 2026-07-21新增)
// 用于云台 TypeID 6 上行帧对齐上位机 ChassisData 结构体
typedef struct UwbSteer_ForSend
{
	uint16_t uwb_angle_yaw;       // UWB偏航角 (来自裁判系统0x0203 Game_Robot_Pos.angle, 取整uint16)
	int16_t  steer_angle_x10;    // 舵角当前角×10 (单位0.1°), 取 steer_decode[0].angle
	uint8_t  reserve[4];          // 填充到8字节
} UwbSteer_ForSend_t;

// 前哨站HP发送包 (CAN 0x0A3, 2026-07-21新增)
// 直接发送裁判系统0x0003原始uint16 HP值, 不做6bit压缩, 保证高精度
typedef struct OutpostHP_ForSend
{
	uint16_t ally_outpost_HP;    // 己方前哨站血量 (原始uint16, 来自 referee_data.Game_Robot_friend_HP.friend_outpost_HP)
	uint16_t enemy_outpost_HP;   // 敌方前哨站血量 (原始uint16, 来自 referee_data.Game_Robot_friend_HP.enemy_outpost_HP)
	uint8_t  reserve[4];         // 填充到8字节
} OutpostHP_ForSend_t;


#pragma pack(pop)

extern GimbalSendPack_1 gimbal_pack_send_1;
extern ChassisSpeedPack_t chassis_speed_pack_send;
extern ShootData_ForSend_t shoot_data_send;
extern SentryInfo_ForSend_t sentry_info_send;
extern BulletExtended_ForSend_t bullet_extended_send;
extern SentryCmd_FromGimbal_t sentry_cmd_from_gimbal;  // 接收云台转发的SentryCmd
extern RobotCommand_ForSend_t robot_command_send;     // 0x0303小地图下发指令
extern SentryDuration_ForSend_t sentry_duration_send; // 哨兵姿态时长发送包
extern DamageDiff_ForSend_t damage_diff_send;         // 伤害值差发送包
extern MotorOffline_ForSend_t motor_offline_send;     // 电机掉线状态发送包
extern UwbSteer_ForSend_t uwb_steer_send;             // UWB角度+舵角发送包 (CAN 0x0A2)
extern OutpostHP_ForSend_t outpost_hp_send;           // 前哨站HP发送包 (CAN 0x0A3)

void GimbalSendPack(void);
void JudgeDataCanSend(void);
void ChassisSpeedPack(void);  // 底盘速度数据打包发送
void ShootDataPack(void);     // 射击数据打包发送 (TypeID 7/8)
void SentryInfoPack(void);    // 哨兵信息打包发送 (TypeID 7)
void SentryDurationPack(void); // 哨兵姿态时长打包发送 (0x020D扩展)
void BulletExtendedPack(void); // 弹量扩展字段打包发送 (TypeID 8)
void RobotCommandPack(void);   // 小地图下发指令打包 (0x0303)
void Can2SendRobotCommand(RobotCommand_ForSend_t *data); // CAN2发送小地图指令
void Can2SendSentryDuration(SentryDuration_ForSend_t *data); // CAN2发送哨兵姿态时长
void DamageDiffPack(void);              // 伤害值差打包
void Can2SendDamageDiff(DamageDiff_ForSend_t *data); // CAN2发送伤害值差
void Can2SendMotorOffline(MotorOffline_ForSend_t *data); // CAN2发送电机掉线状态
void UwbSteerPack(void);                                   // UWB角度+舵角打包 (CAN 0x0A2)
void Can2SendUwbSteer(UwbSteer_ForSend_t *data);           // CAN2发送UWB+舵角
void OutpostHPPack(void);                                 // 前哨站HP打包 (CAN 0x0A3)
void Can2SendOutpostHP(OutpostHP_ForSend_t *data);         // CAN2发送前哨站HP

#endif // !_GIMBAL_SEND_H
