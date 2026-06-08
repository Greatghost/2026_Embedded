#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "algorithmOfCRC.h"
#include "ins_task.h"

#include "ChassisGet.h"
#include "robot_config.h"

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
	JUDGE_PC_DATA = 1
}PC_dataType_enum;
#pragma pack(push, 1)     //不进行字节对齐
//上位机下发协议 17 bytes (与2026-05-06_lower_machine_downlink_sentry_cmd_integration.md对齐)
typedef struct PCRecvData_1
{
    uint8_t Head;           // '!' = 0x21
	int8_t Aim_v_x;         // Velocity.X
	int8_t Aim_v_y;         // Velocity.Y
	float Aim_Yaw;          // 4 bytes
	float Aim_Pitch;        // 4 bytes
	uint8_t FireCode;       // FireCode位域 (bit0-1:FireStatus, bit2-3:CapState, bit4:HoleMode, bit5:AimMode, bit6-7:Rotate)
	uint32_t SentryCmd;     // 4 bytes - 裁判系统0x0301/0x0120 sentry_cmd，姿态在bit21-22
	uint8_t Tail;           // 0x00
} PCRecvData_1; // sizeof == 17 bytes
typedef struct PCSendData //数据顺序不能变,注意32字节对齐 //11 bytes
{
    uint8_t start_flag;
	uint8_t data_pack_type;
	
	float yaw; //4
    short pitch; //2
    uint16_t remain_bullet; //2
	int8_t vx; //1
	int8_t vy; //1
	uint8_t Shoot_State; //1
	uint8_t reserve0;
	
    int8_t crc8;
} PCSendData;
typedef struct PCSendDataJudge
{
	uint8_t start_flag;
	uint8_t data_pack_type;

	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Robot_Red_Blue : 1; //1 -> red ; 0 -> blue
	uint16_t Enemy_outpost : 6; //敌方哨兵是否无敌
	uint16_t self_outpost : 6;
	uint8_t sentry_posture : 2;  // 哨兵姿态: 1=进攻, 2=防御, 3=移动, 0=未知

	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001

	uint16_t self_blood;
	int16_t UWB_x; //float*100 -> short
	int16_t UWB_y;

	int8_t crc8;
}PCSendDataJudge;
#pragma pack(pop) //不进行字节对齐

typedef struct PCRecvData
{
    #if ROBOT == QI_TIAN_DA_SHENG
    int8_t start_flag;
    uint8_t enemy_id;   // 敌方ID，如果是0的话不击打,云台也不动
    float yaw;
    short pitch;
    int8_t crc8;
    #elif   ROBOT == GOBLIN
	
    int8_t start_flag; 
    uint8_t enemy_id;//在上位机叫detect_number;
    uint8_t shoot_flag; // 上位机决定打弹与否
    float pitch;//在上位机叫pitch_setpoint;
    float yaw;//在上位机叫yaw_setpoint;
    uint16_t crc16;
    #endif
} PCRecvData;

#pragma pack(push, 1)
typedef struct{
	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Robot_Red_Blue : 1; //1 -> red ; 0 -> blue
	uint8_t Enemy_outpost : 6; //敌方哨兵是否无敌
	uint8_t self_outpost : 6;
	uint8_t sentry_posture : 2;  // 哨兵姿态: 1=进攻, 2=防御, 3=移动, 0=未知
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001
}JudgeData_1_t;

typedef struct{
	uint16_t x; //裁判系统给的机器人坐标(从float 映射到 uint16_t : float*100 -> uint16_t)
	uint16_t y;
	uint8_t commd_keyboard; //云台手指令
	uint8_t Base_Shield;
	uint16_t Self_blood;
}JudgeData_2_t;
#pragma pack(pop)


#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

typedef enum{
	ARMOR_NO_AIM,
	ARMOR_AIMED
}ARMOR_STATE_ENUM;

typedef struct{
	float Nav_Speed_x; //mm/s
	float Nav_Speed_y; //mm/s
	float Nav_Speed_w; //°/s
	short Nav_State;
}Nav_Cmd_t;


extern ARMOR_STATE_ENUM armor_state;
extern float pc_pitch,pc_yaw;
extern Nav_Cmd_t NAV_cmd;
extern uint8_t current_posture;  // 当前姿态状态: 1=进攻, 2=防御, 3=移动, 0=未知
extern uint32_t sentry_cmd_shadow; // 裁判系统哨兵指令影子寄存器

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(uint8_t data_type);
void NAVReceive(uint8_t Buf[]);
void SendtoNAV(void);
void UpdateSentryPosture(uint8_t posture); // 更新哨兵姿态到裁判系统
static inline uint32_t SetSentryPostureBits(uint32_t sentry_cmd, uint8_t posture);

extern PCRecvData pc_recv_data;
#endif

#endif // !_PC_SERIAL_H
