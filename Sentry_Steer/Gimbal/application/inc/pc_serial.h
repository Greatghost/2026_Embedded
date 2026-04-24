#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "algorithmOfCRC.h"
#include "ins_task.h"

#include "ChassisGet.h"
#include "robot_config.h"
#include "receive_data.h"
#include "ShootTask.h"

#define COMMUNICATION_OF_IFANTRY 0  //24赛季步兵通信协议、
#define COMMUNICATION_OF_SENTRY 1   //24赛季哨兵通信协议

#define COMMUNICATION_CHOOSE COMMUNICATION_OF_SENTRY

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_IFANTRY

enum AUTOAIM_MODE
{
    AUTO_AIM = 0,
    SMALL_BUFF,
    BIG_BUFF
};

/*   发送数据定义 */
#pragma pack(push, 1)     // 不进行字节对齐
typedef struct PCSendData // 数据顺序不能变,注意32字节对齐
{
    #if ROBOT == QI_TIAN_DA_SHENG
    int8_t start_flag;
    uint8_t type_id;
    float yaw;
    short pitch;
    int8_t crc8;
	#else
    int8_t start_flag; // 一样
    float pitch_now;
    float yaw_now;
    float roll_now; // 0 
    float actual_bullet_speed; //不用
    uint8_t aim_request; // 右键
    uint8_t mode_want; // 模式 辅瞄模式：0 大符模式：1 小符模式：2
    uint8_t number_want; // 不用
    uint8_t enemy_color; //自己：蓝 1 红 0	
    uint16_t crc16;
    #endif
} PCSendData;

typedef struct PCRecvData
{
    #if ROBOT == QI_TIAN_DA_SHENG
    int8_t start_flag;
    uint8_t enemy_id;   // 敌方ID，如果是0的话不击打,云台也不动
    float yaw;
    short pitch;
    int8_t crc8;
    #else
	
    int8_t start_flag; 
    uint8_t enemy_id;//在上位机叫detect_number;
    uint8_t shoot_flag; // 上位机决定打弹与否
    float pitch;//在上位机叫pitch_setpoint;
    float yaw;//在上位机叫yaw_setpoint;
    uint16_t crc16;
    #endif
} PCRecvData;
#pragma pack(pop) // 不进行字节对齐

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);

extern PCRecvData pc_recv_data;
extern PCSendData pc_send_data;

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];
#endif

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY
typedef enum 
{
	USUAL_PC_DATA = 0,
	JUDGE_PC_DATA = 1,
	JUDGE_PC_DATA_BLOOD_1 = 2,
	JUDGE_PC_DATA_BLOOD_2 = 3,
	JUDGE_PC_DATA_RFID_BUFF = 4,
	JUDGE_PC_DATA_POS = 5,
	JUDGE_PC_DATA_EXTENDED = 6
}PC_dataType_enum;
typedef enum{
	OFFLINE_START = 0,
	OFFLINE_CHECKING = 1,
	OFFLINE_PENDING = 2
}pc_offline_check_enum;

typedef struct PC_StateControl
{
	uint8_t CapState;
	uint8_t if_through_hole;
	uint8_t RotateState;
	uint8_t if_target_in_view;
}PC_StateControl;

#pragma pack(push, 1)     //
//所有发送到pc的数据，均为1byte Head,1byte typre ,12byte data,1byte crc8
//上位机下发协议 14 bytes (与lower_downlink_message_contract.md对齐)
typedef struct PCRecvData_1
{
	uint8_t Head;           // '!' = 0x21
	int8_t Aim_v_x;         // Velocity.X
	int8_t Aim_v_y;         // Velocity.Y
	float Aim_Yaw;          // 4 bytes
	float Aim_Pitch;        // 4 bytes
	uint8_t FireCode;       // FireCode位域 (bit0-1:FireStatus, bit2-3:CapState, bit4:HoleMode, bit5:AimMode, bit6-7:Rotate)
	uint8_t Posture;        // 姿态: 1=进攻, 2=防御, 3=移动, 0=保留
	uint8_t tail;           // 0x00
} PCRecvData_1;  // sizeof == 14 bytes
typedef struct PCSendData //
{
    uint8_t start_flag;
	uint8_t data_pack_type;
	float yaw; //4
    float pitch; //4
    uint16_t remain_bullet; //2
	uint8_t Shoot_State; //1
	uint8_t Cap_Vol;
	
    uint8_t crc8;
} PCSendData;
typedef struct PCSendDataJudge
{
	uint8_t start_flag;
	uint8_t data_pack_type;

	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Robot_Red_Blue : 1; //1 -> red ; 0 -> blue
	uint16_t Enemy_outpost : 6; //前哨站血量
	uint16_t self_outpost : 6;
	uint8_t reserve_1bit : 1;  // 保留

	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001

	uint16_t self_blood;
	ext_event_data_t event_data;

	uint8_t crc8;
}PCSendDataJudge;
typedef struct PCSendDataBlood_1
{
	uint8_t start_flag;
	uint8_t data_pack_type;
	
	uint16_t Friend1;
	uint16_t Friend2;
	uint16_t Friend3;
	uint16_t Friend4;
	uint16_t F_base;
	uint16_t self7;
	

	uint8_t crc8;
}PCSendDataBlood_1;//0x098
typedef struct PCSendDataBlood_2
{
	uint8_t start_flag;
	uint8_t data_pack_type;
	
	
	uint16_t Enemy1;
	uint16_t Enemy2;
	uint16_t Enemy3;
	uint16_t Enemy4;
	uint16_t E_base;
	uint16_t Enemy7;

	uint8_t crc8;
}PCSendDataBlood_2;

typedef struct PCSendDataRFIDAndBuff
{
	uint8_t start_flag;
	uint8_t data_pack_type;

	JudgeData_Buff_t PCbuff_send;
	uint32_t rfid_status;

	uint8_t crc8;
}PCSendDataRFIDAndBuff_t;

typedef struct 
{
	uint8_t position_type; //00X -> Friends,10X -> Enemy
	int16_t ID_X_100; //乘了100
	int16_t ID_Y_100;
	
}PCSendEach_Robot_position_t; // 5byte

typedef struct PCSendDataPosition
{
	uint8_t start_flag;
	uint8_t data_pack_type;

	PCSendEach_Robot_position_t Friend;
	PCSendEach_Robot_position_t Enemy;
	uint16_t bullet_speed_100;

	uint8_t crc8;
}PCSendDataPosition_t;

typedef struct PCSendDataExtended
{
	uint8_t start_flag;
	uint8_t data_pack_type;  // = JUDGE_PC_DATA_EXTENDED = 6

	int16_t UWB_yaw_10;
	uint8_t sentry_posture;  // 哨兵姿态: 1=进攻, 2=防御, 3=移动, 0=未知
	uint8_t reserve_8;
	uint32_t gimbal_vel_data1;  // 大YAW陀螺仪角度 + 底盘w速度: byte0+byte1=大YAW角度*10(int16)单位0.1度, byte2+byte3=底盘角速度*100(int16)单位0.01rad/s
	uint32_t gimbal_vel_data2;  // 底盘速度数据: byte0+byte1=底盘x速度(int16), 单位0.01 m/s; byte2+byte3=底盘y速度(int16), 单位0.01 m/s (通过舵电机角度与轮电机速度反解)

	uint8_t crc8;
}PCSendDataExtended_t;


#pragma pack(pop) //


#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData_1)
#define PC_SEND_BLOOD_SIZE sizeof(PCSendDataBlood_1)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

typedef enum{
	ARMOR_NO_AIM,
	ARMOR_AIMED
}ARMOR_STATE_ENUM;

typedef struct{
	float Nav_Speed_x; //m/s
	float Nav_Speed_y; //m/s
	float Nav_Speed_w; //暂时不用
	short Nav_State;//单位改为m/s
}Nav_Cmd_t;


extern ARMOR_STATE_ENUM armor_state;
extern float pc_pitch,pc_yaw;
extern Nav_Cmd_t NAV_cmd;
extern uint8_t current_posture;  // 当前姿态状态: 1=进攻, 2=防御, 3=移动, 0=未知

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(uint8_t data_type);
void NAVReceive(uint8_t Buf[]);
void SendtoNAV(void);

extern PCRecvData_1 pc_recv_data_1;
#endif

#endif // !_PC_SERIAL_H
