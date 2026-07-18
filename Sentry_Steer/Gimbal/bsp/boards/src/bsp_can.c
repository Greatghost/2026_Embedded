#include "bsp_can.h"
#include "can_send_config.h"
#include "ChassisGet.h"  // 新增: 引用底盘数据接收结构体
#include "pc_serial.h"   // 新增: 引用last_shoot_data

/**********************************************************************************************************
 *函 数 名: can_filter_init
 *功能说明: can配置
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
volatile uint8_t JudgeData_update = 0;
volatile uint8_t Blood_update = 0;
// shoot_data_recv, sentry_info_recv, bullet_extended_recv 已在ChassisGet.c中定义

void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	// CAN 1 FIFO0 接收中断
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN1_FIFO0_ID0 << 5;
	can_filter_st.FilterIdLow = CAN1_FIFO0_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN1_FIFO0_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN1_FIFO0_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	// CAN 1 FIFO1 接收中断
	can_filter_st.FilterBank = 1;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN1_FIFO1_ID0 << 5;
	can_filter_st.FilterIdLow = CAN1_FIFO1_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN1_FIFO1_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN1_FIFO1_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	// CAN 1 FIFO1 第二个过滤器（接收底盘速度数据和TypeID 7/8数据）
	can_filter_st.FilterBank = 2;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = GET_CHASSIS_SPEED_CAN_ID << 5;
	can_filter_st.FilterIdLow = GET_SHOOT_DATA_CAN_ID << 5;  // 新增: 0x09B
	can_filter_st.FilterMaskIdHigh = GET_SENTRY_INFO_CAN_ID << 5;  // 新增: 0x09C
	can_filter_st.FilterMaskIdLow = GET_BULLET_EXTENDED_CAN_ID << 5;  // 新增: 0x09D
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	// CAN 1 FIFO1 第三个过滤器（接收哨兵姿态时长数据 0x09F）
	can_filter_st.FilterBank = 3;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = GET_SENTRY_DURATION_CAN_ID << 5;
	can_filter_st.FilterIdLow = GET_SENTRY_DURATION_CAN_ID << 5;
	can_filter_st.FilterMaskIdHigh = GET_SENTRY_DURATION_CAN_ID << 5;
	can_filter_st.FilterMaskIdLow = GET_SENTRY_DURATION_CAN_ID << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	// CAN 1 FIFO1 第四个过滤器（接收伤害值差数据 0x0A0）
	can_filter_st.FilterBank = 4;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = GET_DAMAGE_DIFF_CAN_ID << 5;
	can_filter_st.FilterIdLow = GET_DAMAGE_DIFF_CAN_ID << 5;
	can_filter_st.FilterMaskIdHigh = GET_DAMAGE_DIFF_CAN_ID << 5;
	can_filter_st.FilterMaskIdLow = GET_DAMAGE_DIFF_CAN_ID << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	// CAN 2 FIFO0 接收中断
	can_filter_st.FilterBank = 15;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN2_FIFO0_ID0 << 5;
	can_filter_st.FilterIdLow = CAN2_FIFO0_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN2_FIFO0_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN2_FIFO0_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	// CAN 2 FIFO1 接收中断
	can_filter_st.FilterBank = 16;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN2_FIFO1_ID0 << 5;
	can_filter_st.FilterIdLow = CAN2_FIFO1_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN2_FIFO1_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN2_FIFO1_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	// CAN 1 发送中断
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	// CAN 2 发送中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}



void MotorReceive(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
	uint8_t temp_CAN_msg_type = data[0];
	if (hcan->Instance == PITCH_MOTOR_CAN && rx_header->StdId == PITCH_MOTOR_CAN_ID)
	{
        if(motor_communication[PITCH_MOTOR].motor_type == GM6020)
        {
            // Pitch接收
            gimbal_controller.pitch_recv.angle = (data[0] << 8) | (data[1]);
            gimbal_controller.pitch_recv.speed = (data[2] << 8) | (data[3]);
            gimbal_controller.pitch_recv.torque_current = (data[4] << 8) | (data[5]);
            gimbal_controller.pitch_recv.temp = data[6];
        }
        else if (motor_communication[PITCH_MOTOR].motor_type == DM_MOTOR)
        {
            DM_Motor_Receive(data, &gimbal_controller.DM_Pitch_Motor);
        }
        LossUpdate(&global_debugger.gimbal_debugger[0], 0.0015f);
	}
	else if (hcan->Instance == FRICTION_WHEEL_CAN && rx_header->StdId == LEFT_FRICTION_WHEEL_CAN_ID)
	{
		// 左摩擦轮接收
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].angle = (data[0] << 8) | (data[1]);
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].speed = (data[2] << 8) | (data[3]);
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].torque_current = (data[4] << 8) | (data[5]);
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].temp = (data[6]);

		LossUpdate(&global_debugger.friction_debugger[LEFT_FRICTION_WHEEL], 0.0015f);
	}
	else if (hcan->Instance == FRICTION_WHEEL_CAN && rx_header->StdId == RIGHT_FRICTION_WHEEL_CAN_ID)
	{
		// 右摩擦轮接收
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].angle = (data[0] << 8) | (data[1]);
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].speed = (data[2] << 8) | (data[3]);
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].torque_current = (data[4] << 8) | (data[5]);
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].temp = (data[6]);

		LossUpdate(&global_debugger.friction_debugger[RIGHT_FRICTION_WHEEL], 0.0015f);
	}
	// [SMALL_YAW_REMOVED] 小Yaw电机CAN接收已删除
	/*
	else if (hcan->Instance == SMALL_YAW_MOTOR_CAN && rx_header->StdId == SMALL_YAW_MOTOR_CAN_ID)
	{
		// 6020
		if (motor_communication[SMALL_YAW_MOTOR].motor_type == GM6020)
		{
			gimbal_controller.small_yaw_recv.angle = (data[0] << 8) | (data[1]);
			gimbal_controller.small_yaw_recv.speed = (data[2] << 8) | (data[3]);
			gimbal_controller.small_yaw_recv.torque_current = (data[4] << 8) | (data[5]);
			gimbal_controller.small_yaw_recv.temp = data[6];
		}
		else if (motor_communication[SMALL_YAW_MOTOR].motor_type == DM_MOTOR)
		{
			DM_Motor_Receive(data, &gimbal_controller.DM_Small_Yaw_Motor);
		}

		LossUpdate(&global_debugger.gimbal_debugger[1], 0.0015f); // 1KHZ
	}
	*/
	else if (hcan->Instance == BIG_YAW_MOTOR_CAN && rx_header->StdId == BIG_YAW_MOTOR_CAN_ID)
	{
		// 6020
		if (motor_communication[BIG_YAW_MOTOR].motor_type == GM6020)
		{
			gimbal_controller.big_yaw_recv.angle = (data[0] << 8) | (data[1]);
			gimbal_controller.big_yaw_recv.speed = (data[2] << 8) | (data[3]);
			gimbal_controller.big_yaw_recv.torque_current = (data[4] << 8) | (data[5]);
			gimbal_controller.big_yaw_recv.temp = data[6];
		}
		else if (motor_communication[BIG_YAW_MOTOR].motor_type == DM_MOTOR)
		{
			DM_Motor_Receive(data, &gimbal_controller.DM_Big_Yaw_Motor);
			BigYawZeroCheck();//接收到数据后就做一个过零检测
		}

		LossUpdate(&global_debugger.gimbal_debugger[2], 0.0015f); // 1KHZ
	}
	else if (hcan->Instance == TOGGLE_MOTOR_CAN && rx_header->StdId == TOGGLE_MOTOR_CAN_ID)
	{
		// 拨弹电机数据接收
		toggle_controller.toggle_recv.angle = (data[0] << 8) | (data[1]);
		toggle_controller.toggle_recv.speed = (data[2] << 8) | (data[3]);
		toggle_controller.toggle_recv.torque_current = (data[4] << 8) | (data[5]);

		LossUpdate(&global_debugger.toggle_debugger, 0.0015f);
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_FROM_CHASSIS_CAN_ID_1)
	{
		memcpy(&chassis_pack_get_1, data, 8);

		LossUpdate(&global_debugger.receive_chassis_debugger[0], 0.0055f);
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_FROM_CHASSIS_CAN_ID_2)
	{
//		memcpy(&chassis_pack_get_2, data, 8);
//		LossUpdate(&global_debugger.receive_chassis_debugger[1], 0.0055f);
	}
	else if(hcan->Instance == BIG_YAW_CAN_COMM_CANx && rx_header->StdId == GET_FROM_BIG_YAW_CAN_ID)
	{
		memcpy(&big_yaw_controller.big_yaw_gyro_raw,data,sizeof(float));
		memcpy(&big_yaw_controller.big_yaw_gyro_speed,data+4,sizeof(float));
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == JUDGE_RECEIVE_DATA_CAN_ID_1)
	{
		memcpy(&JudgeRecieveData,data,8);
		JudgeData_update = 1;
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == JUDGE_RECEIVE_DATA_CAN_ID_2)
	{
		memcpy(&JudgeRecieveData2,data,8);
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1)
	{
		if(temp_CAN_msg_type == 0x0)
		{
			memcpy(&JudgeBlood_F,data,8);
		}
		else if(temp_CAN_msg_type == 0x1)
		{
			Blood_update = 1;
			memcpy(&JudgeBlood_E,data,8);
		}
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID)
	{
		if(temp_CAN_msg_type == 0x1)//0 -> buff,1 -> rfid
		{
			memcpy(&JudgeData_RFID,data,8);
		}
		else if(temp_CAN_msg_type == 0x0)
		{
			memcpy(&JudgeData_Buff,data,8);
		}

	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == SEND_TO_GIMBAL_POSITION_DATA_CAN_ID)
	{

		if(0 < temp_CAN_msg_type && temp_CAN_msg_type < 9) memcpy(&JudgeData_position.Friend[temp_CAN_msg_type],data,8);
		else if (100<=temp_CAN_msg_type && temp_CAN_msg_type < 109) memcpy(&JudgeData_position.Enemy[temp_CAN_msg_type-100],data,8);

	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_CHASSIS_SPEED_CAN_ID)
	{
		// 底盘速度数据接收
		memcpy(&chassis_speed_recv, data, sizeof(ChassisSpeedRecv_t));
	}
	// 新增: TypeID 7/8数据接收 (2026-05-06协议)
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_SHOOT_DATA_CAN_ID)
	{
		// 射击数据接收 (0x0207) -> 更新last_shoot_data
		memcpy(&shoot_data_recv, data, sizeof(ShootDataRecv_t));
		// 同步更新last_shoot_data用于TypeID 7/8上行
		last_shoot_data.bullet_type = shoot_data_recv.bullet_type;
		last_shoot_data.shooter_id = shoot_data_recv.shooter_id;
		last_shoot_data.bullet_freq = shoot_data_recv.bullet_freq;
		last_shoot_data.bullet_speed = shoot_data_recv.bullet_speed;
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_SENTRY_INFO_CAN_ID)
	{
		// 哨兵信息接收 (0x020D)
		memcpy(&sentry_info_recv, data, sizeof(SentryInfoRecv_t));
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_BULLET_EXTENDED_CAN_ID)
	{
		// 弹量扩展字段接收 (0x0208扩展)
		memcpy(&bullet_extended_recv, data, sizeof(BulletExtendedRecv_t));
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_SENTRY_DURATION_CAN_ID)
	{
		// 哨兵姿态时长接收 (0x020D扩展, 2026-07-13)
		memcpy(&sentry_duration, data, sizeof(SentryDuration_t));
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_DAMAGE_DIFF_CAN_ID)
	{
		// 伤害值差接收 (0x0003扩展, 2026-07-13)
		memcpy(&damage_diff, data, sizeof(DamageDiff_t));
	}
	else if (hcan->Instance == CAN2 && rx_header->StdId == ROBOT_COMMAND_CAN_ID)
	{
		// 小地图下发指令接收 (0x09E, 来自底盘裁判系统0x0303)
		memcpy(&robot_command_recv, data, sizeof(RobotCommand_ForSend_t));
	}
}

/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 0邮箱中断回调函数
 *形    参:
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	MotorReceive(hcan, &rx_header, rx_data);
}
/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo1MsgPendingCallback
 *功能说明:FIFO 1邮箱中断回调函数
 *形    参:
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // FIFO 1邮箱中断回调函数
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	MotorReceive(hcan, &rx_header, rx_data);
}

int8_t CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id, CAN_TxHeaderTypeDef *Motor_Send, uint32_t *wait_time)
{
	float dwt_start = DWT_GetTimeline_us();
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) // 等待邮箱空闲
	{
		if (DWT_GetTimeline_us() - dwt_start > 200) // 超时200us
		{
			global_debugger.can_send_wait++;
			return FALSE;
		}
	}
	*wait_time = DWT_GetTimeline_us() - dwt_start;

	uint32_t send_mail_box;
	Motor_Send->StdId = std_id;
	Motor_Send->IDE = CAN_ID_STD;
	Motor_Send->RTR = CAN_RTR_DATA;
	Motor_Send->DLC = 0x08;
	if (HAL_CAN_AddTxMessage(hcan, Motor_Send, data, &send_mail_box))
	{
		global_debugger.can_send_wait++;
		return FALSE;
	}

	return TRUE;
}

// 新增: 发送SentryCmd给底盘 (2026-05-06协议)
void Can1SendSentryCmd(uint32_t sentry_cmd)
{
	static CAN_TxHeaderTypeDef tx_header;
	static uint8_t send_data[8] = {0};
	static uint32_t wait_time;

	// 打包数据: 4字节sentry_cmd + 4字节填充
	memcpy(&send_data[0], &sentry_cmd, sizeof(uint32_t));
	send_data[4] = 0;
	send_data[5] = 0;
	send_data[6] = 0;
	send_data[7] = 0;

	CanSend(&hcan1, (int8_t*)send_data, SEND_TO_CHASSIS_SENTRY_CMD_CAN_ID, &tx_header, &wait_time);
}

// 新增: 转发0x0307地图路径给底盘 (2026-07-11协议)
// TODO: 实现多帧CAN分段传输 (105B payload)
void Can1SendMapPath(const uint8_t *map_data_105)
{
	(void)map_data_105;
	// Stub: 待实现多帧传输
}

// 新增: 转发0x0308自定义信息给底盘 (2026-07-11协议)
// TODO: 实现多帧CAN分段传输 (34B payload)
void Can1SendCustomInfo(const uint8_t *custom_data_34)
{
	(void)custom_data_34;
	// Stub: 待实现多帧传输
}
