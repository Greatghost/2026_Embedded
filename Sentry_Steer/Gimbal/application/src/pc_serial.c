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
#include "usbd_cdc_if.h"   // 用于 TxState 检查, 避免上行帧发送冲突

extern USBD_HandleTypeDef hUsbDeviceFS;

#define PC_RX_RING_SIZE 512U
#define PC_RX_PROCESS_CHUNK_SIZE 64U

static uint8_t pc_rx_ring[PC_RX_RING_SIZE];
static volatile uint16_t pc_rx_ring_head;
static volatile uint16_t pc_rx_ring_tail;
static volatile uint8_t pc_rx_ring_overflow;

/*
 * USB OUT completes in interrupt context. Copy the complete USB packet into
 * an SPSC ring and leave framing, validation and CAN forwarding to a task.
 */
uint8_t PCStreamEnqueueFromISR(const unsigned char *data, uint32_t length)
{
	uint16_t head;
	uint16_t tail;
	uint16_t used;
	uint16_t free_bytes;
	uint32_t i;

	if (data == NULL || length == 0U || length >= PC_RX_RING_SIZE)
	{
		return 0U;
	}

	head = pc_rx_ring_head;
	tail = pc_rx_ring_tail;
	used = (head >= tail)
		? (uint16_t)(head - tail)
		: (uint16_t)(PC_RX_RING_SIZE - tail + head);
	free_bytes = (uint16_t)(PC_RX_RING_SIZE - 1U - used);
	if (length > free_bytes)
	{
		/* Drop the whole USB packet. Never enqueue a partial application
		 * frame; the task resets its framing state after overflow. */
		pc_rx_ring_overflow = 1U;
		return 0U;
	}

	for (i = 0U; i < length; i++)
	{
		pc_rx_ring[head] = data[i];
		head++;
		if (head == PC_RX_RING_SIZE)
		{
			head = 0U;
		}
	}
	__DMB();
	pc_rx_ring_head = head;
	return 1U;
}

void PCStreamProcessPending(void)
{
	unsigned char chunk[PC_RX_PROCESS_CHUNK_SIZE];
	uint16_t tail;
	uint16_t head;
	uint32_t count;

	if (pc_rx_ring_overflow != 0U)
	{
		/* OTG_FS runs at the FreeRTOS syscall ceiling, so this short critical
		 * section gives a coherent reset of the SPSC indices. */
		taskENTER_CRITICAL();
		pc_rx_ring_tail = pc_rx_ring_head;
		pc_rx_ring_overflow = 0U;
		taskEXIT_CRITICAL();
		PCStreamReceive(NULL, 0U);
	}

	for (;;)
	{
		tail = pc_rx_ring_tail;
		head = pc_rx_ring_head;
		count = 0U;
		while (tail != head && count < sizeof(chunk))
		{
			chunk[count++] = pc_rx_ring[tail];
			tail++;
			if (tail == PC_RX_RING_SIZE)
			{
				tail = 0U;
			}
		}

		if (count == 0U)
		{
			return;
		}

		__DMB();
		pc_rx_ring_tail = tail;
		PCStreamReceive(chunk, count);
	}
}

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_IFANTRY

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
PCSendData pc_send_data;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(const unsigned char *PCbuffer, uint32_t length)
{
    if (length != PC_RECVBUF_SIZE)
    {
        return;
    }

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
    if (PCbuffer[0] == '!'  && Verify_CRC16_Check_Sum((uint8_t *)PCbuffer, PC_RECVBUF_SIZE))
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }
    #endif
}

void PCStreamReceive(const unsigned char *data, uint32_t length)
{
	static unsigned char frame_buffer[PC_RECVBUF_SIZE];
	static uint32_t frame_length;
	uint32_t i;

	if (data == NULL)
	{
		frame_length = 0U;
		return;
	}

	for (i = 0U; i < length; i++)
	{
		if (frame_length == 0U && data[i] != '!')
		{
			continue;
		}
		frame_buffer[frame_length++] = data[i];
		if (frame_length == PC_RECVBUF_SIZE)
		{
			PCReceive(frame_buffer, frame_length);
			frame_length = 0U;
		}
	}
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
    uint8_t tx_ret = CDC_Transmit_FS(SendToPC_Buff, PC_SENDBUF_SIZE);
    if (tx_ret == USBD_BUSY)
    {
        usb_cdc_busy_count++;
    }
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

static volatile uint32_t pc_control_last_rx_tick;
static volatile uint8_t pc_control_received;
static volatile uint32_t pc_control_sequence;
static PCControlSnapshot_t pc_control_shadow;
static volatile uint32_t pc_trajectory_last_rx_tick;
static volatile uint8_t pc_trajectory_received;

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
PCSendDataSentryDuration_t PCSendSentryDuration;
PCSendDataGimbalDynamics_t PCSendGimbalDynamics;

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

/* 拒绝NaN/Inf和明显超出机构能力的轨迹量，防止异常串口帧直接进入前馈。 */
static float PCControlDynamicsSanitize(float value, float abs_limit)
{
	if (value != value || value > abs_limit || value < -abs_limit)
	{
		return 0.0f;
	}
	return value;
}

static uint8_t PCControlAnglesValid(float yaw, float pitch)
{
	return (yaw == yaw && pitch == pitch &&
		yaw < 1000000.0f && yaw > -1000000.0f &&
		pitch < 3600.0f && pitch > -3600.0f) ? 1U : 0U;
}

uint8_t PCControlGetSnapshot(PCControlSnapshot_t *snapshot)
{
	uint32_t sequence_start;
	uint32_t sequence_end;
	uint32_t last_rx_tick;
	uint8_t received;

	if (snapshot == NULL)
	{
		return 0U;
	}

	for (;;)
	{
		sequence_start = pc_control_sequence;
		if ((sequence_start & 1U) != 0U)
		{
			continue;
		}

		__DMB();
		*snapshot = pc_control_shadow;
		last_rx_tick = pc_control_last_rx_tick;
		received = pc_control_received;
		__DMB();
		sequence_end = pc_control_sequence;

		if (sequence_start == sequence_end && (sequence_end & 1U) == 0U)
		{
			break;
		}
	}

	if (received == 0U)
	{
		return 0U;
	}

	return ((uint32_t)(HAL_GetTick() - last_rx_tick) <= PC_CONTROL_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t PCControlIsOnline(void)
{
	PCControlSnapshot_t snapshot;
	return PCControlGetSnapshot(&snapshot);
}

void PCReceive(const unsigned char *PCbuffer, uint32_t length)
{
	if (PCbuffer == NULL || length < 2U || PCbuffer[0] != '!')
	{
		return;
	}
	/* 与旧版一致：收到结构完整、帧头正确的PC帧即刷新通信心跳。 */
	LossUpdate(&global_debugger.pc_receive_debugger, 0.02);

	/* 按种类统计收到的下行帧（仅 TypeID 已知且在 0x00~0x05 范围内才计数） */
	if (PCbuffer[1] < PC_DOWNLINK_TYPE_COUNT)
	{
		g_pc_downlink_debug.by_type[PCbuffer[1]]++;
		g_pc_downlink_debug.total++;
	}

	switch(PCbuffer[1])
	{
	case PC_DOWNLINK_CONTROL: // 0x00 — 13B GimbalControlFrame（兼容旧驱动）
	{
		GimbalControlFrame_t frame;
		uint32_t now;

		if (length != sizeof(frame))
		{
			return;
		}
		memcpy(&frame, PCbuffer, sizeof(frame));
		if (PCControlAnglesValid(frame.yaw, frame.pitch) == 0U)
		{
			return;
		}
		now = HAL_GetTick();

		/* 奇偶序列锁保证控制任务只能取得完整的一帧命令。 */
		pc_control_sequence++;
		__DMB();
		if (pc_trajectory_received == 0U ||
			(uint32_t)(now - pc_trajectory_last_rx_tick) > PC_CONTROL_TIMEOUT_MS)
		{
			/* 没有新鲜MPC轨迹时，旧角度接口继续工作且前馈安全清零。 */
			pc_control_shadow.yaw = frame.yaw;
			pc_control_shadow.pitch = frame.pitch;
			pc_control_shadow.yaw_omega = 0.0f;
			pc_control_shadow.pitch_omega = 0.0f;
			pc_control_shadow.yaw_alpha = 0.0f;
			pc_control_shadow.pitch_alpha = 0.0f;
		}
		pc_control_shadow.nav_speed_x = frame.vel_x / 50.0f;
		pc_control_shadow.nav_speed_y = frame.vel_y / 50.0f;
		pc_control_shadow.nav_speed_w = 0.0f;
		pc_control_shadow.shoot_state = frame.fire_code & 0x03U;
		pc_control_shadow.cap_state = (frame.fire_code >> 2) & 0x03U;
		pc_control_shadow.follow_mode  = (frame.fire_code >> 4) & 0x01U;  // [FIX] bit4 = FollowMode (原误读为 through_hole)
		pc_control_shadow.aim_mode    = (frame.fire_code >> 5) & 0x01U;  // [FIX] bit5 = AimMode   (原误读为 big_yaw_mode)
		pc_control_shadow.rotate_state = (frame.fire_code >> 6) & 0x03U;

		/* 保留旧接口镜像；安全关键消费者改用PCControlGetSnapshot。 */
		pc_yaw = frame.yaw;
		pc_pitch = frame.pitch;
		NAV_cmd.Nav_Speed_x = pc_control_shadow.nav_speed_x;
		NAV_cmd.Nav_Speed_y = pc_control_shadow.nav_speed_y;
		NAV_cmd.Nav_Speed_w = 0.0f;
		Shoot_Cmd.Shoot_State = pc_control_shadow.shoot_state;
		PC_statecontrol.CapState = pc_control_shadow.cap_state;
		/* [FIX] 旧字段 through_hole/big_yaw_mode 实为 FireCode.bit4 FollowMode / bit5 AimMode。
		   左侧 legacy 镜像字段名保留（被其他模块读取），但值来自重命名后的 snapshot 字段。 */
		PC_statecontrol.if_through_hole = pc_control_shadow.follow_mode;
		big_yaw_controller.big_yaw_mode = pc_control_shadow.aim_mode;
		PC_statecontrol.RotateState = pc_control_shadow.rotate_state;
		pc_control_last_rx_tick = now;
		pc_control_received = 1U;
		__DMB();
		pc_control_sequence++;
		break;
	}
	case PC_DOWNLINK_TRAJECTORY: // 0x05 — 26B GimbalTrajectoryFrame
	{
		GimbalTrajectoryFrame_t frame;
		uint32_t now;

		if (length != sizeof(frame))
		{
			return;
		}
		memcpy(&frame, PCbuffer, sizeof(frame));
		if (PCControlAnglesValid(frame.yaw, frame.pitch) == 0U)
		{
			return;
		}
		now = HAL_GetTick();

		pc_control_sequence++;
		__DMB();
		pc_control_shadow.yaw = frame.yaw;
		pc_control_shadow.pitch = frame.pitch;
		pc_control_shadow.yaw_omega = PCControlDynamicsSanitize(frame.yaw_omega, 2000.0f);
		pc_control_shadow.pitch_omega = PCControlDynamicsSanitize(frame.pitch_omega, 2000.0f);
		pc_control_shadow.yaw_alpha = PCControlDynamicsSanitize(frame.yaw_alpha, 50000.0f);
		pc_control_shadow.pitch_alpha = PCControlDynamicsSanitize(frame.pitch_alpha, 50000.0f);
		pc_trajectory_last_rx_tick = now;
		pc_trajectory_received = 1U;
		__DMB();
		pc_control_sequence++;
		break;
	}
	case PC_DOWNLINK_SENTRY_CMD: // 0x01 — 6B SentryCommandFrame
	{
		SentryCommandFrame_t frame;
		uint32_t cmd;

		if (length != sizeof(frame))
		{
			return;
		}
		memcpy(&frame, PCbuffer, sizeof(frame));
		cmd = frame.sentry_cmd;
		// 姿态提取 V2.0: bit21-23 (1~6)
		uint8_t posture_from_cmd = (cmd >> 21) & 0x07;
		if(posture_from_cmd >= 1 && posture_from_cmd <= 6)
			current_posture = posture_from_cmd;
		Can1SendSentryCmd(cmd);
		break;
	}
	case PC_DOWNLINK_MAP_PATH: // 0x02 — 64B 分片帧, 重组后转发 0x0307
	{
		static uint8_t map_path_payload[MAP_PATH_PAYLOAD_SIZE];
		if (length != sizeof(MapPathFragment_t)) return;
		if (MapPath_OnFragment(PCbuffer, HAL_GetTick(), map_path_payload) == 1U)
		{
			Can1SendMapPath(map_path_payload);
		}
		break;
	}
	case PC_DOWNLINK_CUSTOM_INFO: // 0x03 — 36B, 转发0x0308
		if (length != sizeof(CustomInfoFrame_t)) return;
		Can1SendCustomInfo(PCbuffer + 2);
		break;
	case PC_DOWNLINK_COORD: // 0x04 — 17B SentryCoordinateFrame
	{
		SentryCoordinateFrame_t frame;

		if (length != sizeof(frame))
		{
			return;
		}
		memcpy(&frame, PCbuffer, sizeof(frame));
		if(crc8_calc(PCbuffer, 16) == frame.crc8)
		{
			sentry_position_x_cm = frame.x_cm;
			sentry_position_y_cm = frame.y_cm;
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
    pc_send_data_blood_2.E_base = JudgeBlood_E.ID8 * 100;
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
	static const uint8_t position_index[5] = {1U, 2U, 3U, 4U, 7U};
	static uint8_t slot = 0U;
	const uint8_t robot_index = position_index[slot];

	PCSendPosition.start_flag = '!';
	PCSendPosition.data_pack_type = JUDGE_PC_DATA_POS;
	PCSendPosition.Friend.position_type = JudgeData_position.Friend[robot_index].position_type;
	PCSendPosition.Friend.ID_X_100 = JudgeData_position.Friend[robot_index].ID_X_100;
	PCSendPosition.Friend.ID_Y_100 = JudgeData_position.Friend[robot_index].ID_Y_100;
	PCSendPosition.Enemy.position_type = JudgeData_position.Enemy[robot_index].position_type;
	PCSendPosition.Enemy.ID_X_100 = JudgeData_position.Enemy[robot_index].ID_X_100;
	PCSendPosition.Enemy.ID_Y_100 = JudgeData_position.Enemy[robot_index].ID_Y_100;
	PCSendPosition.bullet_speed_100 = chassis_pack_get_1.bullet_speed;
	PCSendPosition.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendPosition, PC_SEND_BLOOD_SIZE);
	memcpy(buff, (void *)&PCSendPosition, PC_SEND_BLOOD_SIZE);

	slot++;
	if (slot >= 5U)
	{
		slot = 0U;
	}
}

void SendtoPCExtend(unsigned char* buff)
{
	PCSendExtended.start_flag = '!';
	PCSendExtended.data_pack_type = JUDGE_PC_DATA_EXTENDED;
	// 2026-07-21变更: 对齐上位机 ChassisData 结构体
	// byte 2-3: UWB偏航角 (来自底盘 CAN 0x0A2)
	PCSendExtended.uwb_angle_yaw = uwb_steer_recv.uwb_angle_yaw;
	// byte 4-5: 伤害值差 (CAN 0x0A0)
	PCSendExtended.damage_difference = damage_diff.damage_difference;
	// byte 6-9: ChassisPacked1
	//   low16:  舵角当前角×10 (int16, 单位0.1°), 来自底盘 CAN 0x0A2
	//   high16: 底盘角速度×100 (int16, 单位0.01rad/s), 来自 CAN 0x09A
	int16_t steer_angle_10 = (int16_t)(uwb_steer_recv.steer_angle * 10.0f);
	PCSendExtended.chassis_packed1 = ((uint32_t)(steer_angle_10 & 0xFFFF))
	                               | ((uint32_t)(chassis_speed_recv.chassis_yaw_v_100 & 0xFFFF) << 16);
	// byte 10-13: ChassisPacked2
	//   low16:  底盘x速度×100 (int16, 单位0.01m/s)
	//   high16: 底盘y速度×100 (int16, 单位0.01m/s)
	PCSendExtended.chassis_packed2 = ((uint32_t)(chassis_speed_recv.chassis_x_v_100 & 0xFFFF))
	                               | ((uint32_t)(chassis_speed_recv.chassis_y_v_100 & 0xFFFF) << 16);
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

#define SENTRY_DURATION_FRESH_MS 1500U

static uint8_t SentryDurationDataIsFresh(void)
{
	const uint32_t now = HAL_GetTick();

	if (sentry_duration_last_rx_tick == 0U ||
		outpost_hp_recv.last_rx_tick == 0U)
	{
		return 0U;
	}
	if ((uint32_t)(now - sentry_duration_last_rx_tick) > SENTRY_DURATION_FRESH_MS ||
		(uint32_t)(now - outpost_hp_recv.last_rx_tick) > SENTRY_DURATION_FRESH_MS)
	{
		return 0U;
	}
	return 1U;
}

// TypeID 10: 发送哨兵姿态时长 + 前哨站HP (CAN 0x09F + 0x0003前哨HP)
// 2026-07-21变更: 保持15B布局, 把原 byte 10-13 的 reserved 改成 ally/enemy_outpost_HP
void SendtoPCSentryDuration(unsigned char* buff)
{
	static volatile uint32_t sentry_duration_send_cnt = 0;
	sentry_duration_send_cnt++;

	PCSendSentryDuration.start_flag = '!';
	PCSendSentryDuration.data_pack_type = JUDGE_PC_DATA_SENTRY_DURATION;
	// 哨兵姿态时长 (来自底盘 CAN 0x09F)
	PCSendSentryDuration.normal_attack_duration     = sentry_duration.normal_attack_duration;
	PCSendSentryDuration.normal_defend_duration     = sentry_duration.normal_defend_duration;
	PCSendSentryDuration.normal_move_duration       = sentry_duration.normal_move_duration;
	PCSendSentryDuration.reserved_duration_1       = 0;
	PCSendSentryDuration.enhanced_attack_duration   = sentry_duration.enhanced_attack_duration;
	PCSendSentryDuration.enhanced_defend_duration   = sentry_duration.enhanced_defend_duration;
	PCSendSentryDuration.enhanced_move_duration    = sentry_duration.enhanced_move_duration;
	PCSendSentryDuration.reserved_duration_2       = 0;
	// 前哨站HP: 直接使用底盘CAN 0x0A3转发的原始uint16值, 不走6bit压缩解压
	PCSendSentryDuration.ally_outpost_HP  = outpost_hp_recv.ally_outpost_HP;
	PCSendSentryDuration.enemy_outpost_HP = outpost_hp_recv.enemy_outpost_HP;
	// CRC8 校验 (覆盖 byte 0-13)
	PCSendSentryDuration.crc8 = 0;
	Append_CRC8_Check_Sum((unsigned char *)&PCSendSentryDuration, sizeof(PCSendDataSentryDuration_t));
	memcpy(buff, (void *)&PCSendSentryDuration, sizeof(PCSendDataSentryDuration_t));
}

static int16_t GimbalDynamicsToInt16(float value, float scale)
{
	float scaled;

	if (value != value) // NaN
	{
		return 0;
	}
	scaled = value * scale;
	if (scaled > 32767.0f)
	{
		return 32767;
	}
	if (scaled < -32768.0f)
	{
		return -32768;
	}
	return (int16_t)scaled;
}

// TypeID 11: 发送云台实际角速度/角加速度
void SendtoPCGimbalDynamics(unsigned char* buff)
{
	PCSendGimbalDynamics.start_flag = '!';
	PCSendGimbalDynamics.data_pack_type = JUDGE_PC_DATA_GIMBAL_DYNAMICS;
	PCSendGimbalDynamics.yaw_omega_dps_x10 =
		GimbalDynamicsToInt16(gimbal_controller.gyro_yaw_speed, 10.0f);
	PCSendGimbalDynamics.pitch_omega_dps_x10 =
		GimbalDynamicsToInt16(gimbal_controller.gyro_pitch_speed, 10.0f);
	PCSendGimbalDynamics.yaw_alpha_dps2 =
		GimbalDynamicsToInt16(gimbal_controller.gyro_yaw_acceleration, 1.0f);
	PCSendGimbalDynamics.pitch_alpha_dps2 =
		GimbalDynamicsToInt16(gimbal_controller.gyro_pitch_acceleration, 1.0f);
	PCSendGimbalDynamics.sample_tick_ms = HAL_GetTick();
	PCSendGimbalDynamics.crc8 = 0;
	Append_CRC8_Check_Sum(
		(unsigned char *)&PCSendGimbalDynamics, sizeof(PCSendGimbalDynamics));
	memcpy(buff, (void *)&PCSendGimbalDynamics, sizeof(PCSendGimbalDynamics));
}

void SendtoPC(uint8_t data_type)
{
	if (data_type == JUDGE_PC_DATA_SENTRY_DURATION &&
		SentryDurationDataIsFresh() == 0U)
	{
		return;
	}

	/* TxState 忙时直接丢弃当前帧, 避免覆盖正在 DMA 传输的 SendToPC_Buff。
	 * 这是"避免上行帧发送冲突"的简化方案: 不操作 USB 寄存器,
	 * 只在前一帧未发送完成时跳过本次 Pack+Transmit, 防止单缓冲区被覆盖。 */
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
	if (hcdc != NULL && hcdc->TxState != 0U)
	{
		usb_cdc_busy_count++;
		return;
	}

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
	else if(data_type == JUDGE_PC_DATA_SENTRY_DURATION)
	{
		// TypeID 10: 15B (含CRC8), 与其他上行帧统一长度
		SendtoPCSentryDuration(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA_GIMBAL_DYNAMICS)
	{
		SendtoPCGimbalDynamics(SendToPC_Buff);
	}
	uint8_t tx_ret = CDC_Transmit_FS(SendToPC_Buff,PC_SENDBUF_SIZE);
	if (tx_ret == USBD_BUSY)
	{
		usb_cdc_busy_count++;
	}
	else if (tx_ret == USBD_OK)
	{
		/* 按种类统计成功提交到 USB CDC 的上行帧 */
		if (data_type < PC_UPLINK_TYPE_COUNT)
		{
			g_pc_uplink_debug.by_type[data_type]++;
			g_pc_uplink_debug.total++;
		}
	}
}
#endif

/* ===== 0x02 MapPath 分片重组状态机 (2026-07-18 协议变更) ===== */
/* 详见 map_path_fragment_reassembly.md
 * 上位机将原 107B MapPathFrame 拆成两个固定 64B 物理帧下发:
 *   fragment 0: payload[0..55]  (56B)  -> 重组 buffer [0..55]
 *   fragment 1: payload[0..48]  (49B)  -> 重组 buffer [56..104]
 * 重组后的 105B 即裁判系统 0x0307 map_data_t 原始 payload。
 * CRC16 算法为 reflected 0x1021 (CRC-16/X-25), 复用 Get_CRC16_Check_Sum。
 */
static map_path_pending_t map_path_pending;
MapPathRxDebug_t g_map_path_rx_debug;
PCUplinkDebug_t g_pc_uplink_debug;
PCDownlinkDebug_t g_pc_downlink_debug;

static void map_path_clear_pending(void)
{
	map_path_pending.active = 0U;
	map_path_pending.sequence = 0U;
	map_path_pending.has_fragment[0] = 0U;
	map_path_pending.has_fragment[1] = 0U;
	map_path_pending.started_ms = 0U;
}

/* 校验单个 64B fragment frame 的固定字段和 CRC16。
 * 返回 1 合法, 0 非法。 */
static uint8_t map_path_frame_is_valid(const uint8_t *frame)
{
	const MapPathFragment_t *f = (const MapPathFragment_t *)frame;
	uint16_t crc_expected;

	if (f->head != 0x21U) return 0U;
	if (f->type_id != PC_DOWNLINK_MAP_PATH) return 0U;
	if (f->fragment_count != MAP_PATH_FRAGMENT_COUNT) return 0U;
	if (f->fragment_index > 1U) return 0U;

	/* PayloadLength 必须与 FragmentIndex 严格匹配 */
	if (f->fragment_index == 0U && f->payload_length != MAP_PATH_FRAGMENT0_LEN) return 0U;
	if (f->fragment_index == 1U && f->payload_length != MAP_PATH_FRAGMENT1_LEN) return 0U;

	/* CRC16 校验 byte 0-61, little-endian */
	crc_expected = Get_CRC16_Check_Sum((uint8_t *)frame, 62U, 0xFFFFU);
	if ((uint8_t)(crc_expected & 0xFFU) != frame[62]) return 0U;
	if ((uint8_t)((crc_expected >> 8) & 0xFFU) != frame[63]) return 0U;

	return 1U;
}

/* 校验重组后的 105B payload: intention 必须为 1/2/3 */
static uint8_t map_path_payload_is_valid(const uint8_t *payload_105)
{
	uint8_t intention = payload_105[0];
	return (intention >= 1U && intention <= 3U) ? 1U : 0U;
}

uint8_t MapPath_OnFragment(const uint8_t *frame_64, uint32_t now_ms, uint8_t *out_payload_105)
{
	const MapPathFragment_t *f;
	uint8_t sequence;
	uint8_t index;

	if (frame_64 == NULL || out_payload_105 == NULL)
	{
		return 0U;
	}

	if (map_path_frame_is_valid(frame_64) == 0U)
	{
		g_map_path_rx_debug.invalid_frame_count++;
		map_path_clear_pending();
		return 0U;
	}

	f = (const MapPathFragment_t *)frame_64;
	sequence = f->sequence;
	index = f->fragment_index;
	g_map_path_rx_debug.last_sequence = sequence;
	g_map_path_rx_debug.last_fragment_index = index;
	if (index == 0U)
	{
		g_map_path_rx_debug.fragment0_valid_count++;
	}
	else
	{
		g_map_path_rx_debug.fragment1_valid_count++;
	}

	/* 超时清空 (从首段开始计时 100ms) */
	if (map_path_pending.active != 0U &&
		(uint32_t)(now_ms - map_path_pending.started_ms) > MAP_PATH_TIMEOUT_MS)
	{
		g_map_path_rx_debug.timeout_count++;
		map_path_clear_pending();
	}

	if (index == 0U)
	{
		/* 首段: 清空旧 pending, 以该 sequence 新建 pending */
		map_path_clear_pending();
		map_path_pending.active = 1U;
		map_path_pending.sequence = sequence;
		map_path_pending.started_ms = now_ms;
		memcpy(&map_path_pending.payload[0], f->payload, MAP_PATH_FRAGMENT0_LEN);
		map_path_pending.has_fragment[0] = 1U;
		return 0U;
	}

	/* index == 1: 尾段, 必须有同 sequence 的首段, 且未重复收尾段 */
	if (map_path_pending.active == 0U ||
		map_path_pending.has_fragment[0] == 0U ||
		map_path_pending.sequence != sequence ||
		map_path_pending.has_fragment[1] != 0U)
	{
		g_map_path_rx_debug.order_drop_count++;
		map_path_clear_pending();
		return 0U;
	}

	memcpy(&map_path_pending.payload[MAP_PATH_FRAGMENT0_LEN], f->payload, MAP_PATH_FRAGMENT1_LEN);
	map_path_pending.has_fragment[1] = 1U;

	if (map_path_payload_is_valid(map_path_pending.payload) == 0U)
	{
		g_map_path_rx_debug.invalid_payload_count++;
		map_path_clear_pending();
		return 0U;
	}

	/* 重组完成: 输出 105B payload, 清空 pending */
	memcpy(out_payload_105, map_path_pending.payload, MAP_PATH_PAYLOAD_SIZE);
	g_map_path_rx_debug.reassembly_success_count++;
	map_path_clear_pending();
	return 1U;
}

void PCStreamReceive(const unsigned char *data, uint32_t length)
{
    static unsigned char frame_buffer[PC_RECVBUF_SIZE];
    static uint32_t frame_length;
    static uint32_t expected_length;
	static uint32_t last_byte_tick;
	const uint32_t now = HAL_GetTick();

	if (data == NULL)
	{
		frame_length = 0U;
		expected_length = 0U;
		last_byte_tick = 0U;
		return;
	}
	if (length == 0U)
	{
		return;
	}

	/* 半帧长期未补齐时丢弃，避免下一帧被当成旧帧尾部。 */
	if (frame_length != 0U && (uint32_t)(now - last_byte_tick) > 20U)
	{
		frame_length = 0U;
		expected_length = 0U;
	}
	last_byte_tick = now;

	for (uint32_t i = 0U; i < length; i++)
	{
		const unsigned char byte = data[i];

		if (frame_length == 0U)
		{
			if (byte == '!')
			{
				frame_buffer[0] = byte;
				frame_length = 1U;
			}
			continue;
		}

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY
		if (frame_length == 1U)
		{
			switch (byte)
			{
			case PC_DOWNLINK_CONTROL:
				expected_length = sizeof(GimbalControlFrame_t);
				break;
			case PC_DOWNLINK_SENTRY_CMD:
				expected_length = sizeof(SentryCommandFrame_t);
				break;
			case PC_DOWNLINK_MAP_PATH:
			expected_length = sizeof(MapPathFragment_t);  // 64B 分片帧
			break;
			case PC_DOWNLINK_CUSTOM_INFO:
				expected_length = sizeof(CustomInfoFrame_t);
				break;
			case PC_DOWNLINK_COORD:
				expected_length = sizeof(SentryCoordinateFrame_t);
				break;
			case PC_DOWNLINK_TRAJECTORY:
				expected_length = sizeof(GimbalTrajectoryFrame_t);
				break;
			default:
				frame_length = (byte == '!') ? 1U : 0U;
				expected_length = 0U;
				continue;
			}
		}
#else
		expected_length = PC_RECVBUF_SIZE;
#endif

		if (frame_length >= sizeof(frame_buffer) ||
			expected_length > sizeof(frame_buffer))
		{
			frame_length = 0U;
			expected_length = 0U;
			continue;
		}

		frame_buffer[frame_length++] = byte;
		if (frame_length == expected_length)
		{
			PCReceive(frame_buffer, frame_length);
			frame_length = 0U;
			expected_length = 0U;
		}
	}
}
