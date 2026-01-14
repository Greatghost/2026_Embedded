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
	uint8_t Sentry_HomeReturned_flag : 1;
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
	uint16_t ID8 : 6;//基地
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



#pragma pack(pop)

extern GimbalSendPack_1 gimbal_pack_send_1;

void GimbalSendPack(void);
void JudgeDataCanSend(void);

#endif // !_GIMBAL_SEND_H
