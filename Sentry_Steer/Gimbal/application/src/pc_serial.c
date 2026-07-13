/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "Gimbal.h"
#include "arm_atan2_f32.h"
#include "debug.h"
#include "bsp_can.h"  // 新增: 引用Can1SendSentryCmd

#include "SignalGenerator.h"

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_IFANTRY

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
PCSendData pc_send_data;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(unsigned char *PCbuffer)
{
    #if ROBOT == GOBLIN
    if (PCbuffer[0] == '!' && PCbuffer[1] == 0 && PCbuffer[8] == 0) //TODO: 判断方式可能错误
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }
	#elif ROBOT == TIGER
	if (PCbuffer[0] == '!' && PCbuffer[1] == 0 && PCbuffer[8] == 0) //TODO: 判断方式可能错误
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }
    #else
    if (PCbuffer[0] == '!'  && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }
    #endif
}

void SendtoPCPack(unsigned char *buff)
{
	volatile unsigned char aim_press_r = remote_controller.dji_remote.mouse.press_r;
    #if ROBOT == QI_TIAN_DA_SHENG
    pc_send_data.start_flag = '!';
    pc_send_data.type_id = 0;
    pc_send_data.yaw = gimbal_controller.gyro_yaw_angle;
    pc_send_data.pitch = (short)gimbal_controller.gyro_pitch_angle * 100;
    pc_send_data.crc8 = 0;
    #else
	if(remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
		pc_send_data.mode_want = 1;
	else if(remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE)
		pc_send_data.mode_want = 2;
	else
		pc_send_data.mode_want = 0;

	pc_send_data.start_flag = '!';
	pc_send_data.pitch_now = gimbal_controller.gyro_pitch_angle;
	pc_send_data.yaw_now = gimbal_controller.gyro_yaw_angle;
	pc_send_data.roll_now = 0.0f;
	pc_send_data.actual_bullet_speed = 0.0f;
	pc_send_data.aim_request = aim_press_r;
	pc_send_data.number_want = 0;
	pc_send_data.enemy_color = !chassis_pack_get_1.robot_color;
	Append_CRC16_Check_Sum((uint8_t *)(&pc_send_data), PC_SENDBUF_SIZE);
    #endif

	memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);
    CDC_Transmit_FS(SendToPC_Buff, PC_SENDBUF_SIZE);
}

#endif

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

// [旧协议] PCRecvData_1 pc_recv_data_1; // 2026-07-12 协议迁移
PCSendData pc_send_data;
PCSendDataJudge PC_send_data_judge;
PCSendDataBlood_1 pc_send_data_blood_1;
PCSendDataBlood_2 pc_send_data_blood_2;

float pc_pitch,pc_yaw;
uint8_t PC_Shoot_flag;
uint8_t last_shoot_flag;
Nav_Cmd_t NAV_cmd;
uint8_t current_posture = 0;
extern BigYawController big_yaw_controller;

PC_StateControl PC_statecontrol;

extern Shoot_Cmd_t Shoot_Cmd;
JudgeData_1_t JudgeRecieveData;
JudgeData_2_t JudgeRecieveData2;
JudgeBloodData_ForSend1_t JudgeBlood_F,JudgeBlood_E;
JudgeData_Buff_t JudgeData_Buff;
JudgeData_RFID_t JudgeData_RFID;
JudgeData_position_t JudgeData_position;
PCSendDataRFIDAndBuff_t PCSendDataRFIDAndBuff;
PCSendDataPosition_t PCSendPosition;
PCSendDataExtended_t PCSendExtended;
PCSendDataSentry_t PCSendSentry;
PCSendDataBulletAndRfid2_t PCSendBulletAndRfid2;
PCSendDataRobotCmd_t PCSendRobotCmd;

ext_shoot_data_t last_shoot_data;

extern ChassisGetPack_1 chassis_pack_get_1;
char PC_Receive_Flag_2_Armor = 0;

int shootflg_test = 0;

// 哨兵坐标缓存 (DownlinkTypeID=0x04, 2026-07-12协议迁移)
int16_t sentry_position_x_cm = 0;
int16_t sentry_position_y_cm = 0;

// CRC8: poly=0x31, init=0xFF
static uint8_t crc8_calc(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0xFF;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t b = 0; b < 8; b++)
			crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
	}
	return crc;
}

void PCReceive(unsigned char *PCbuffer)
{
	LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
	if(PCbuffer[0] != '!') return;

	switch(PCbuffer[1])
	{
	case PC_DOWNLINK_CONTROL: // 0x00 — 13B GimbalControlFrame
	{
		GimbalControlFrame_t *f = (GimbalControlFrame_t *)PCbuffer;
		pc_yaw = f->yaw;
		pc_pitch = f->pitch;
		NAV_cmd.Nav_Speed_x = f->vel_x / 50.0f;
		NAV_cmd.Nav_Speed_y = f->vel_y / 50.0f;
		Shoot_Cmd.Shoot_State = f->fire_code & 0x03;
		PC_statecontrol.CapState = (f->fire_code >> 2) & 0x03;
		PC_statecontrol.if_through_hole = (f->fire_code >> 4) & 0x01;
		big_yaw_controller.big_yaw_mode = (f->fire_code >> 5) & 0x01;
		PC_statecontrol.RotateState = (f->fire_code >> 6) & 0x03;
		break;
	}
	case PC_DOWNLINK_SENTRY_CMD: // 0x01 — 6B SentryCommandFrame
	{
		SentryCommandFrame_t *f = (SentryCommandFrame_t *)PCbuffer;
		uint32_t cmd = f->sentry_cmd;
		// 姿态提取 V2.0: bit21-23 (1~6)
		uint8_t posture_from_cmd = (cmd >> 21) & 0x07;
		if(posture_from_cmd >= 1 && posture_from_cmd <= 6)
			current_posture = posture_from_cmd;
		Can1SendSentryCmd(cmd);
		break;
	}
	case PC_DOWNLINK_MAP_PATH: // 0x02 — 107B, 转发0x0307
		Can1SendMapPath(PCbuffer + 2);
		break;
	case PC_DOWNLINK_CUSTOM_INFO: // 0x03 — 36B, 转发0x0308
		Can1SendCustomInfo(PCbuffer + 2);
		break;
	case PC_DOWNLINK_COORD: // 0x04 — 17B SentryCoordinateFrame
	{
		SentryCoordinateFrame_t *f = (SentryCoordinateFrame_t *)PCbuffer;
		if(crc8_calc(PCbuffer, 16) == f->crc8)
		{
			sentry_position_x_cm = f->x_cm;
			sentry_position_y_cm = f->y_cm;
		}
		break;
	}
	default:
		break;
	}
}

extern int ShootCount_Number;

void SendtoPCPack(unsigned char *buff)
{
    pc_send_data.start_flag = '!';
	pc_send_data.data_pack_type = USUAL_PC_DATA;
    pc_send_data.Shoot_State = Shoot_Cmd.Shoot_State_send;
	pc_send_data.remain_bullet = JudgeRecieveData.bullet_remaining_num_17mm;
    pc_send_data.pitch = gimbal_controller.gyro_pitch_angle;
    pc_send_data.yaw = gimbal_controller.gyro_yaw_angle;
	pc_send_data.Cap_Vol = chassis_pack_get_1.half_CapVol * 2;
	pc_send_data.crc8 = 0;
    Append_CRC8_Check_Sum((unsigned char *)&pc_send_data, PC_SENDBUF_SIZE);
    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

void Send2PCJudge(unsigned char* buff)
{
	PC_send_data_judge.start_flag = '!';
	PC_send_data_judge.self_blood = JudgeRecieveData2.Self_blood;
	PC_send_data_judge.data_pack_type = JUDGE_PC_DATA;
	PC_send_data_judge.bullet_remaining_num_17mm = JudgeRecieveData.bullet_remaining_num_17mm;
	PC_send_data_judge.Enemy_outpost = JudgeRecieveData.Enemy_outpost;
	PC_send_data_judge.is_game_start = JudgeRecieveData.is_game_start;
	PC_send_data_judge.Robot_Red_Blue = JudgeRecieveData.Robot_Red_Blue;
	PC_send_data_judge.self_outpost = JudgeRecieveData.self_outpost;
	PC_send_data_judge.reserve_1bit = 0;
	PC_send_data_judge.stage_remain_time = JudgeRecieveData.stage_remain_time*2;
	PC_send_data_judge.event_data = JudgeData_RFID.event_data;
	PC_send_data_judge.Heat_update = 0;
	PC_send_data_judge.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PC_send_data_judge, PC_SENDBUF_SIZE);
	memcpy(buff, (void *)&PC_send_data_judge, PC_SENDBUF_SIZE);
}

void SendtoPCBlood_1(unsigned char* buff)
{
	pc_send_data_blood_1.start_flag = '!';
    pc_send_data_blood_1.data_pack_type = JUDGE_PC_DATA_BLOOD_1;
    pc_send_data_blood_1.Friend1 = JudgeBlood_F.ID1 * 10;
    pc_send_data_blood_1.Friend2 = JudgeBlood_F.ID2 * 10;
    pc_send_data_blood_1.Friend3 = JudgeBlood_F.ID3 * 10;
    pc_send_data_blood_1.Friend4 = JudgeBlood_F.ID4 * 10;
    pc_send_data_blood_1.F_base = JudgeBlood_F.ID8 * 100;
    pc_send_data_blood_1.self7 = JudgeBlood_F.ID7 * 10;
	pc_send_data_blood_1.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&pc_send_data_blood_1, PC_SENDBUF_SIZE);
	memcpy(buff, (void *)&pc_send_data_blood_1, PC_SEND_BLOOD_SIZE);
}

void SendtoPCBlood_2(unsigned char* buff)
{
	pc_send_data_blood_2.start_flag = '!';
    pc_send_data_blood_2.data_pack_type = JUDGE_PC_DATA_BLOOD_2;
	pc_send_data_blood_2.Enemy1 = JudgeBlood_E.ID1 * 10;
    pc_send_data_blood_2.Enemy2 = JudgeBlood_E.ID2 * 10;
    pc_send_data_blood_2.Enemy3 = JudgeBlood_E.ID3 * 10;
    pc_send_data_blood_2.Enemy4 = JudgeBlood_E.ID4 * 10;
    pc_send_data_blood_2.E_base = JudgeBlood_E.ID8 * 10;
    pc_send_data_blood_2.Enemy7 = JudgeBlood_E.ID7 * 10;
	pc_send_data_blood_2.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&pc_send_data_blood_2, PC_SENDBUF_SIZE);
	memcpy(buff, (void *)&pc_send_data_blood_2, PC_SEND_BLOOD_SIZE);
}

void SendtoPCRFIDAndBuff(unsigned char* buff)
{
	PCSendDataRFIDAndBuff.start_flag = '!';
	PCSendDataRFIDAndBuff.data_pack_type = JUDGE_PC_DATA_RFID_BUFF;
	PCSendDataRFIDAndBuff.PCbuff_send = JudgeData_Buff;
	PCSendDataRFIDAndBuff.rfid_status = JudgeData_RFID.rfid_status;
	PCSendDataRFIDAndBuff.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendDataRFIDAndBuff, PC_SEND_BLOOD_SIZE);
	memcpy(buff, (void *)&PCSendDataRFIDAndBuff, PC_SEND_BLOOD_SIZE);
}

void SendtoPCPos(unsigned char* buff)
{
	static uint8_t count = 0;
	if(count!=0 && count!=5 && count!=6)
	{
		PCSendPosition.start_flag = '!';
		PCSendPosition.data_pack_type = JUDGE_PC_DATA_POS;
		PCSendPosition.Friend.position_type = JudgeData_position.Friend[count].position_type;
		PCSendPosition.Friend.ID_X_100 = JudgeData_position.Friend[count].ID_X_100;
		PCSendPosition.Friend.ID_Y_100 = JudgeData_position.Friend[count].ID_Y_100;
		PCSendPosition.Enemy.position_type = JudgeData_position.Enemy[count].position_type;
		PCSendPosition.Enemy.ID_X_100 = JudgeData_position.Enemy[count].ID_X_100;
		PCSendPosition.Enemy.ID_Y_100 = JudgeData_position.Enemy[count].ID_Y_100;
		PCSendPosition.bullet_speed_100 = chassis_pack_get_1.bullet_speed;
		PCSendPosition.crc8 = 0;
		Append_CRC8_Check_Sum((unsigned char *)&PCSendPosition, PC_SEND_BLOOD_SIZE);
		memcpy(buff, (void *)&PCSendPosition, PC_SEND_BLOOD_SIZE);
	}
	count++;
	if(count >= 8) count = 0;
}

void SendtoPCExtend(unsigned char* buff)
{
	PCSendExtended.start_flag = '!';
	PCSendExtended.data_pack_type = JUDGE_PC_DATA_EXTENDED;
	PCSendExtended.UWB_yaw_10 = JudgeRecieveData2.yaw_10;
	PCSendExtended.sentry_posture = JudgeRecieveData.sentry_posture;
	PCSendExtended.reserve_8 = 0;
	// [SMALL_YAW_REMOVED] 变量改名: small_yaw_offset → yaw_bias_offset (数据来源不变)
	int16_t yaw_bias_offset_angle_10 = (int16_t)(big_yaw_controller.big_yaw_gyro_bias * 10.0f);
	PCSendExtended.gimbal_vel_data1 = ((uint32_t)(yaw_bias_offset_angle_10 & 0xFFFF)) | ((uint32_t)(chassis_speed_recv.chassis_yaw_v_100 & 0xFFFF) << 16);
	PCSendExtended.gimbal_vel_data2 = ((uint32_t)(chassis_speed_recv.chassis_x_v_100 & 0xFFFF)) | ((uint32_t)(chassis_speed_recv.chassis_y_v_100 & 0xFFFF) << 16);
	PCSendExtended.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendExtended, PC_SEND_BLOOD_SIZE);
	memcpy(buff, (void *)&PCSendExtended, PC_SEND_BLOOD_SIZE);
}

// TypeID 7: 发送哨兵信息 (0x020D + 0x0207初速度)
void SendtoPCSentry(unsigned char* buff)
{
	extern SentryInfoRecv_t sentry_info_recv;  // 来自底盘的哨兵信息
	PCSendSentry.start_flag = '!';
	PCSendSentry.data_pack_type = JUDGE_PC_DATA_SENTRY_DATA;
	PCSendSentry.sentry_info = sentry_info_recv.sentry_info;       // 来自底盘CAN 0x09C
	PCSendSentry.sentry_info_2 = sentry_info_recv.sentry_info_2;   // 来自底盘CAN 0x09C
	PCSendSentry.bullet_initial_speed = last_shoot_data.bullet_speed;  // 来自底盘CAN 0x09B
	PCSendSentry.reserved = 0;
	PCSendSentry.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendSentry, PC_SEND_BLOOD_SIZE);
	memcpy(buff, (void *)&PCSendSentry, PC_SEND_BLOOD_SIZE);
}

// TypeID 8: 发送弹量数据+RFID扩展
void SendtoPCBulletAndRfid2(unsigned char* buff)
{
	extern BulletExtendedRecv_t bullet_extended_recv;  // 来自底盘的弹量扩展数据
	PCSendBulletAndRfid2.start_flag = '!';
	PCSendBulletAndRfid2.data_pack_type = JUDGE_PC_DATA_BULLET_DATA_AND_RFID2;
	PCSendBulletAndRfid2.bullet_type = last_shoot_data.bullet_type;
	PCSendBulletAndRfid2.shooter_number = last_shoot_data.shooter_id;
	PCSendBulletAndRfid2.launching_frequency = last_shoot_data.bullet_freq;
	PCSendBulletAndRfid2.projectile_allowance_17mm = JudgeRecieveData.bullet_remaining_num_17mm;
	PCSendBulletAndRfid2.projectile_allowance_42mm = bullet_extended_recv.projectile_allowance_42mm;     // 来自底盘CAN 0x09D
	PCSendBulletAndRfid2.remaining_gold_coin = bullet_extended_recv.remaining_gold_coin;                 // 来自底盘CAN 0x09D
	PCSendBulletAndRfid2.projectile_allowance_fortress = bullet_extended_recv.projectile_allowance_fortress; // 来自底盘CAN 0x09D
	PCSendBulletAndRfid2.rfid_status_2 = bullet_extended_recv.rfid_status_2;                             // 来自底盘CAN 0x09D
	PCSendBulletAndRfid2.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendBulletAndRfid2, PC_SEND_BLOOD_SIZE);
	memcpy(buff, (void *)&PCSendBulletAndRfid2, PC_SEND_BLOOD_SIZE);
}

// TypeID 9: 发送小地图下发指令
void SendtoPCRobotCmd(unsigned char* buff)
{
	PCSendRobotCmd.start_flag = '!';
	PCSendRobotCmd.data_pack_type = JUDGE_PC_DATA_ROBOT_COMMAND;
	PCSendRobotCmd.cmd = robot_command_recv;  // 8 bytes 原样转发
	PCSendRobotCmd.reserved = 0;
	PCSendRobotCmd.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendRobotCmd, PC_SEND_BLOOD_SIZE);
	memcpy(buff, (void *)&PCSendRobotCmd, PC_SEND_BLOOD_SIZE);
}

void SendtoPC(uint8_t data_type)
{
	if(data_type == USUAL_PC_DATA)
	{
		SendtoPCPack(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA)
	{
		Send2PCJudge(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_BLOOD_1)
	{
		SendtoPCBlood_1(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_BLOOD_2)
	{
		SendtoPCBlood_2(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_RFID_BUFF)
	{
		SendtoPCRFIDAndBuff(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_POS)
	{
		SendtoPCPos(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_EXTENDED)
	{
		SendtoPCExtend(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_SENTRY_DATA)
	{
		SendtoPCSentry(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_BULLET_DATA_AND_RFID2)
	{
		SendtoPCBulletAndRfid2(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_ROBOT_COMMAND)
	{
		SendtoPCRobotCmd(SendToPC_Buff);
	}
	CDC_Transmit_FS(SendToPC_Buff,PC_SENDBUF_SIZE);
}
#endif