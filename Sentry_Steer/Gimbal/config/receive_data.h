#ifndef _RECEIVE_DATA_H
#define _RECEIVE_DATA_H

#include "struct_typedef.h"
#pragma pack(push, 1)
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
}JudgeData_1_t;

typedef struct{
	uint16_t x; //裁判系统给的机器人坐标(从float 映射到 uint16_t : float*100 -> uint16_t)
	uint16_t y;
	int16_t yaw_10;
	uint16_t Self_blood;
}JudgeData_2_t;

typedef struct{
	uint8_t blood_type; //0 -> Friends,1 -> Enemy
	uint16_t ID1 : 6;
	uint16_t ID2 : 6;
	uint16_t ID3 : 6;
	uint16_t ID4 : 6;
	uint16_t ID_reserve : 6;
	uint16_t ID7 : 6;
	uint16_t ID8 : 6;
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

typedef struct // 0x0101 场地事件数据
{
	// uint32_t event_type;
	uint8_t self_supply_status : 3;
	uint8_t self_Buff_status : 3;
	uint8_t self_highland_status : 6;
	uint8_t self_BaseShield : 7;
	uint16_t last_dart_time : 9;
	uint8_t dart_target : 2;
	uint8_t gain_point_statuss : 2;
	// uint8_t _ : 3;
} ext_event_data_t;

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
#pragma pack(pop)



#endif
