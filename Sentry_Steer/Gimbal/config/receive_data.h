#ifndef _RECEIVE_DATA_H
#define _RECEIVE_DATA_H

#include "struct_typedef.h"
#pragma pack(push, 1)
typedef struct
{
	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Enemy_outpost : 6;	// 敌方哨兵是否无敌
	uint8_t Robot_Red_Blue : 1; // 1 -> red ; 0 -> blue
	uint8_t self_outpost : 6;
	uint8_t reserve_1bit : 1;  // 保留位, 填充到2字节 (原sentry_posture已移至TypeID 7/10独立通道)
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; // 0x208
	uint16_t stage_remain_time;			// 0x0001
} JudgeData_1_t;  // sizeof == 8 bytes

typedef struct
{
	uint16_t x; // 裁判系统给的机器人坐标(从float 映射到 uint16_t : float*100 -> uint16_t)
	uint16_t y;
	int16_t yaw_10;
	uint16_t Self_blood;
} JudgeData_2_t;

typedef struct
{
	uint8_t blood_type; // 0 -> Friends,1 -> Enemy
	uint16_t ID1 : 6;
	uint16_t ID2 : 6;
	uint16_t ID3 : 6;
	uint16_t ID4 : 6;
	uint16_t ID_reserve : 6;
	uint16_t ID7 : 6;
	uint16_t ID8 : 6;
	uint32_t reserve : 14;
} JudgeBloodData_ForSend1_t;

typedef struct
{
	uint8_t data_type; // 0为JudgeData_Buff_t
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;

} JudgeData_Buff_t; // 裁判系统0x0204
typedef char JudgeDataBuffSizeCheck[(sizeof(JudgeData_Buff_t) == 8U) ? 1 : -1];

typedef struct // 0x0101 场地事件数据
{
	uint32_t event_data; // RoboMaster 2026 V2.0.0 bit0..31 原始场地事件
} ext_event_data_t;
typedef char EventDataSizeCheck[(sizeof(ext_event_data_t) == 4U) ? 1 : -1];

typedef struct
{
	uint8_t data_type; // 1为JudgeData_RFID_t
	ext_event_data_t event_data;
	uint32_t rfid_status : 24;

} JudgeData_RFID_t; // 裁判系统0x0204

typedef struct
{
	uint8_t position_type; // 00X -> Friends,10X -> Enemy
	int16_t ID_X_100;	   // 乘了100
	int16_t ID_Y_100;
	uint16_t reserve;
	uint8_t reserve_1;

} Each_Robot_position_t;

typedef struct
{
	Each_Robot_position_t Friend[8]; // 0~7,但0,5,6不填
	Each_Robot_position_t Enemy[8];	 // 0~7,但0,5,6不填
} JudgeData_position_t;

// 0x0207 实时射击信息 (用于TypeID 7/8上行数据同步)
typedef struct
{
	uint8_t bullet_type; // 弹丸类型：1-17mm/2-42mm
	uint8_t shooter_id;	 // 发射机构ID：1-1号17mm/2-2号17mm/3-1号42mm
	uint8_t bullet_freq; // 发射频率（发/秒）
	float bullet_speed;	 // 弹丸初速度（m/s）
} ext_shoot_data_t;

#pragma pack(pop)

#endif
