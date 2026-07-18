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

// TypeID 9: RobotCmd - 小地图下发指令 (0x0303 CAN 0x09E)
typedef struct PCSendDataRobotCmd
{
	uint8_t start_flag;
	uint8_t data_pack_type;  // = JUDGE_PC_DATA_ROBOT_COMMAND = 9
	RobotCommand_ForSend_t cmd;  // 8 bytes 小地图指令原样转发
	uint32_t reserved;       // 填充至12字节data
	uint8_t crc8;
} PCSendDataRobotCmd_t;  // sizeof == 15 bytes

#pragma pack(pop) // 不进行字节对齐

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

void PCReceive(const unsigned char *PCbuffer, uint32_t length);
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
	JUDGE_PC_DATA_EXTENDED = 6,
	JUDGE_PC_DATA_SENTRY_DATA = 7,      // TypeID 7: 哨兵信息 (0x020D + 0x0207初速度)
	JUDGE_PC_DATA_BULLET_DATA_AND_RFID2 = 8, // TypeID 8: 弹量数据+RFID扩展
	JUDGE_PC_DATA_ROBOT_COMMAND = 9,    // TypeID 9: 小地图下发指令 (0x0303)
	JUDGE_PC_DATA_SENTRY_DURATION = 10, // TypeID 10: 哨兵姿态时长 (CAN 0x09F)
	JUDGE_PC_DATA_GIMBAL_DYNAMICS = 11 // TypeID 11: 云台角速度/角加速度
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
// 下行帧 DownlinkTypeID 定义（2026-07-11 新协议：上位机→下位机）
// [旧协议已移除] PCRecvData_1(18B,含SentryCmd), SentryCoord_t(0x01坐标), PC_TYPEID_CONTROL/COORD
#define PC_DOWNLINK_CONTROL      0x00  // GimbalControlFrame (13B, legacy compatible)
#define PC_DOWNLINK_SENTRY_CMD   0x01  // SentryCommandFrame (6B)
#define PC_DOWNLINK_MAP_PATH     0x02  // MapPathFrame (107B)
#define PC_DOWNLINK_CUSTOM_INFO  0x03  // CustomInfoFrame (36B)
#define PC_DOWNLINK_COORD        0x04  // SentryCoordinateFrame (17B)
#define PC_DOWNLINK_TRAJECTORY   0x05  // GimbalTrajectoryFrame (26B, MPC)
#define PC_CONTROL_TIMEOUT_MS    200U  // valid 0x00 control-frame timeout

// DownlinkTypeID 0x00: GimbalControlFrame (13B，保持旧协议不变)
typedef struct {
    uint8_t head;        // 0x21
    uint8_t type_id;     // 0x00
    int8_t  vel_x;       // Velocity.X
    int8_t  vel_y;       // Velocity.Y
    float   yaw;         // GimbalAngles.Yaw, float LE
    float   pitch;       // GimbalAngles.Pitch, float LE
    uint8_t fire_code;   // FireCode 位域
} GimbalControlFrame_t;  // sizeof == 13

// DownlinkTypeID 0x05: MPC轨迹原子帧，不承载底盘速度和FireCode。
typedef struct {
    uint8_t head;          // 0x21
    uint8_t type_id;       // 0x05
    float   yaw;           // Yaw目标角度, deg, float LE
    float   pitch;         // Pitch目标角度, deg, float LE
    float   yaw_omega;   // Yaw目标角速度, deg/s, float LE
    float   pitch_omega; // Pitch目标角速度, deg/s, float LE
    float   yaw_alpha;   // Yaw目标角加速度, deg/s^2, float LE
    float   pitch_alpha; // Pitch目标角加速度, deg/s^2, float LE
} GimbalTrajectoryFrame_t; // sizeof == 26

// DownlinkTypeID 0x01: SentryCommandFrame (6B)
typedef struct {
    uint8_t  head;        // 0x21
    uint8_t  type_id;     // 0x01
    uint32_t sentry_cmd;  // RM2026 V2.0 0x0120 命令字, uint32 LE
} SentryCommandFrame_t;   // sizeof == 6

// DownlinkTypeID 0x02: MapPathFrame (107B) — 0x0307 map_data_t 原样转发
typedef struct {
    uint8_t head;               // 0x21
    uint8_t type_id;            // 0x02
    uint8_t map_data[105];      // 0x0307 payload: Intention + StartPos + DeltaX[49] + DeltaY[49] + SenderId
} MapPathFrame_t;               // sizeof == 107

// DownlinkTypeID 0x03: CustomInfoFrame (36B) — 0x0308 custom_info_t 原样转发
typedef struct {
    uint8_t head;               // 0x21
    uint8_t type_id;            // 0x03
    uint8_t custom_data[34];    // 0x0308 payload: SenderId + ReceiverId + UserDataUtf16[30]
} CustomInfoFrame_t;            // sizeof == 36

// DownlinkTypeID 0x04: SentryCoordinateFrame (17B)
typedef struct {
    uint8_t  head;          // 0x21
    uint8_t  type_id;       // 0x04
    int16_t  x_cm;          // X坐标 (cm) 小端
    int16_t  y_cm;          // Y坐标 (cm) 小端
    uint8_t  reserved[10];  // 预留 0x00
    uint8_t  crc8;          // CRC8 (poly=0x31, init=0xFF, 覆盖byte0-15)
} SentryCoordinateFrame_t;  // sizeof == 17

extern int16_t sentry_position_x_cm;
extern int16_t sentry_position_y_cm;

// 上行帧结构体 (下位机→上位机): 1B Head + 1B TypeID + 12B Data + 1B CRC8 = 15B

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
typedef struct PCSendDataBlood_1	//友方血量
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
typedef struct PCSendDataBlood_2	//敌方血量
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

	int16_t damage_difference; // 伤害值差 (己方−敌方HP总和, CAN 0x0A0)
	uint8_t sentry_posture;  // 姿态回读已移至TypeID 7, 此处置0
	uint8_t reserve_8;
	uint32_t gimbal_vel_data1;  // 小YAW偏差角度 + 底盘w速度: byte0+byte1=小YAW偏差角度*10(int16)单位0.1度, byte2+byte3=底盘角速度*100(int16)单位0.01rad/s
	uint32_t gimbal_vel_data2;  // 底盘速度数据: byte0+byte1=底盘x速度(int16), 单位0.01 m/s; byte2+byte3=底盘y速度(int16), 单位0.01 m/s (通过舵电机角度与轮电机速度反解)

	uint8_t crc8;
}PCSendDataExtended_t;

// TypeID 7: SentryData - 哨兵信息 (0x020D + 0x0207初速度)
typedef struct PCSendDataSentry
{
	uint8_t start_flag;
	uint8_t data_pack_type;  // = JUDGE_PC_DATA_SENTRY_DATA = 7
	uint32_t sentry_info;          // 0x020D offset 0
	uint16_t sentry_info_2;        // 0x020D offset 4
	float    bullet_initial_speed; // 0x0207 offset 6 (弹丸初速度)
	uint16_t reserved;             // 填0
	uint8_t crc8;
}PCSendDataSentry_t;  // sizeof == 15 bytes (1+1+12+1)

// TypeID 8: BulletDataAndRfid2 - 弹量数据+RFID扩展 (0x0207+0x0208+rfid_status_2)
typedef struct PCSendDataBulletAndRfid2
{
	uint8_t start_flag;
	uint8_t data_pack_type;  // = JUDGE_PC_DATA_BULLET_DATA_AND_RFID2 = 8
	uint8_t  bullet_type;                      // 0x0207 offset 0
	uint8_t  shooter_number;                   // 0x0207 offset 1
	uint8_t  launching_frequency;              // 0x0207 offset 2
	uint16_t projectile_allowance_17mm;        // 0x0208 offset 0
	uint16_t projectile_allowance_42mm;        // 0x0208 offset 2
	uint16_t remaining_gold_coin;              // 0x0208 offset 4
	uint16_t projectile_allowance_fortress;    // 0x0208 offset 6
	uint8_t  rfid_status_2;                    // 0x0209 offset 4
	uint8_t crc8;
}PCSendDataBulletAndRfid2_t;  // sizeof == 15 bytes (1+1+12+1)



// TypeID 9: RobotCmd - 小地图下发指令 (0x0303 CAN 0x09E)
typedef struct PCSendDataRobotCmd
{
	uint8_t start_flag;
	uint8_t data_pack_type;  // = JUDGE_PC_DATA_ROBOT_COMMAND = 9
	RobotCommand_ForSend_t cmd;  // 8 bytes 小地图指令原样转发
	uint32_t reserved;       // 填充至12字节data
	uint8_t crc8;
} PCSendDataRobotCmd_t;  // sizeof == 15 bytes

// TypeID 10: SentryDuration - 哨兵姿态时长 (0x020D扩展, CAN 0x09F)
typedef struct PCSendDataSentryDuration
{
	uint8_t start_flag;           // '!'
	uint8_t data_pack_type;       // = JUDGE_PC_DATA_SENTRY_DURATION = 10
	uint8_t normal_attack_duration;
	uint8_t normal_defend_duration;
	uint8_t normal_move_duration;
	uint8_t reserved_duration_1;
	uint8_t enhanced_attack_duration;
	uint8_t enhanced_defend_duration;
	uint8_t enhanced_move_duration;
	uint8_t reserved_duration_2;
	uint32_t reserved;            // 填充至12字节payload
	uint8_t crc8;
} PCSendDataSentryDuration_t;  // sizeof == 15 bytes

// TypeID 11: GimbalDynamics - 云台实际角速度/角加速度
// 保持现有上行帧统一15B。速度分辨率0.1 deg/s，加速度分辨率1 deg/s^2。
typedef struct PCSendDataGimbalDynamics
{
	uint8_t start_flag;
	uint8_t data_pack_type;       // = JUDGE_PC_DATA_GIMBAL_DYNAMICS
	int16_t yaw_omega_dps_x10;
	int16_t pitch_omega_dps_x10;
	int16_t yaw_alpha_dps2;
	int16_t pitch_alpha_dps2;
	uint32_t sample_tick_ms;      // HAL_GetTick()采样时间
	uint8_t crc8;
} PCSendDataGimbalDynamics_t;    // sizeof == 15 bytes

/* 协议尺寸是两端的硬约束，结构体布局变化时直接在编译期报错。 */
typedef char GimbalControlFrameSizeCheck[(sizeof(GimbalControlFrame_t) == 13U) ? 1 : -1];
typedef char GimbalTrajectoryFrameSizeCheck[(sizeof(GimbalTrajectoryFrame_t) == 26U) ? 1 : -1];
typedef char GimbalDynamicsFrameSizeCheck[(sizeof(PCSendDataGimbalDynamics_t) == 15U) ? 1 : -1];

#pragma pack(pop) //


#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(MapPathFrame_t)  // 107B, 取最大下行帧
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

/* 一帧PC控制命令的原子快照，控制任务不得逐项读取ISR正在发布的全局量。 */
typedef struct
{
	float yaw;
	float pitch;
	float yaw_omega;
	float pitch_omega;
	float yaw_alpha;
	float pitch_alpha;
	float nav_speed_x;
	float nav_speed_y;
	float nav_speed_w;
	uint8_t shoot_state;
	uint8_t cap_state;
	uint8_t through_hole;
	uint8_t big_yaw_mode;
	uint8_t rotate_state;
} PCControlSnapshot_t;


extern ARMOR_STATE_ENUM armor_state;
extern float pc_pitch,pc_yaw;
extern Nav_Cmd_t NAV_cmd;
extern uint8_t current_posture;  // 当前姿态状态: 1=进攻, 2=防御, 3=移动, 0=未知

// 0x0207 shadow缓存，用于TypeID 7/8同步
extern ext_shoot_data_t last_shoot_data;  // 上一次射击数据缓存

void PCReceive(const unsigned char *PCbuffer, uint32_t length);
uint8_t PCControlIsOnline(void);
uint8_t PCControlGetSnapshot(PCControlSnapshot_t *snapshot);
void SendtoPC(uint8_t data_type);
void NAVReceive(uint8_t Buf[]);
void SendtoNAV(void);

extern PCSendDataRobotCmd_t PCSendRobotCmd;  // TypeID 9 uplink  // TypeID 9
#endif

/* USB CDC是字节流；该入口负责拆包、粘包和跨64B端点重组。 */
void PCStreamReceive(const unsigned char *data, uint32_t length);

#endif // !_PC_SERIAL_H
