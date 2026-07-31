#include "bsp_can.h"
#include "can_send_config.h"
#include "ChassisGet.h"  // 新增: 引用底盘数据接收结构体
#include "pc_serial.h"   // 新增: 引用last_shoot_data
#include "FreeRTOS.h"
#include "task.h"

/**********************************************************************************************************
 *函 数 名: can_filter_init
 *功能说明: can配置
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
volatile uint8_t JudgeData_update = 0;
volatile uint8_t Blood_update = 0;
volatile uint32_t can1_bus_off_count = 0U;
volatile uint32_t can2_bus_off_count = 0U;
volatile uint32_t can1_recovery_count = 0U;
volatile uint32_t can2_recovery_count = 0U;
static volatile uint8_t can_bus_off_waiting_mask = 0U;
static volatile uint8_t can_recovery_alert_mask = 0U;
// shoot_data_recv, sentry_info_recv, bullet_extended_recv 已在ChassisGet.c中定义

static uint8_t CanGetRecoveryBit(const CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		return CAN_RECOVERY_ALERT_CAN1;
	}
	if (hcan->Instance == CAN2)
	{
		return CAN_RECOVERY_ALERT_CAN2;
	}
	return 0U;
}

/*
 * Called only after HAL reports a successfully received or transmitted frame.
 * This distinguishes real bus recovery from bxCAN merely leaving BOFF while
 * the cable is still disconnected.
 */
static void CanMarkBusRecovered(CAN_HandleTypeDef *hcan)
{
	const uint8_t bit = CanGetRecoveryBit(hcan);

	if ((bit != 0U) && ((can_bus_off_waiting_mask & bit) != 0U))
	{
		can_bus_off_waiting_mask &= (uint8_t)~bit;
		can_recovery_alert_mask |= bit;
		if (bit == CAN_RECOVERY_ALERT_CAN1)
		{
			can1_recovery_count++;
		}
		else
		{
			can2_recovery_count++;
		}
	}
}

uint8_t CanTakeRecoveryAlerts(void)
{
	uint8_t alerts;

	taskENTER_CRITICAL();
	alerts = can_recovery_alert_mask;
	can_recovery_alert_mask = 0U;
	taskEXIT_CRITICAL();

	return alerts;
}

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
	// CAN 1 FIFO1 第三个过滤器（接收哨兵姿态时长 0x09F + 小地图下发指令 0x09E）
	// [FIX] 0x09E 物理上从底盘 CAN2 进入云台 CAN1, 需要在 CAN1 上配置滤波器
	//       原重复 0x09F 填充 2 个槽位, 现改为重复 0x09E 避免接受杂散 ID
	can_filter_st.FilterBank = 3;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = ROBOT_COMMAND_CAN_ID << 5;            // 0x09E 小地图下发指令(0x0303)
	can_filter_st.FilterIdLow = GET_SENTRY_DURATION_CAN_ID << 5;       // 0x09F 哨兵姿态时长
	can_filter_st.FilterMaskIdHigh = ROBOT_COMMAND_CAN_ID << 5;        // 0x09E (重复, 不接受额外ID)
	can_filter_st.FilterMaskIdLow = GET_SENTRY_DURATION_CAN_ID << 5;   // 0x09F (重复, 不接受额外ID)
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	// CAN 1 FIFO1 第四个过滤器（0x0A0~0x0A3 合并到一个 bank, 释放 3 个滤波器 bank）
	// [FIX] 原 Bank 4~7 各用 4 个槽位接收单一 ID, 浪费 12 个槽位 + 3 个 bank。
	//       16-bit IDLIST 模式每个 bank 可容纳 4 个 ID, 合并后仅需 1 个 bank。
	can_filter_st.FilterBank = 4;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = GET_DAMAGE_DIFF_CAN_ID << 5;       // 0x0A0 伤害值差
	can_filter_st.FilterIdLow = GET_MOTOR_OFFLINE_CAN_ID << 5;      // 0x0A1 电机掉线状态
	can_filter_st.FilterMaskIdHigh = GET_UWB_STEER_CAN_ID << 5;     // 0x0A2 UWB+舵角
	can_filter_st.FilterMaskIdLow = GET_OUTPOST_HP_CAN_ID << 5;     // 0x0A3 前哨站HP
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
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY |
		CAN_IT_BUSOFF | CAN_IT_ERROR);
	// CAN 2 发送中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY |
		CAN_IT_BUSOFF | CAN_IT_ERROR);
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

		if(0U < temp_CAN_msg_type && temp_CAN_msg_type < 8U)
		{
			memcpy(&JudgeData_position.Friend[temp_CAN_msg_type], data, 8U);
		}
		else if (100U <= temp_CAN_msg_type && temp_CAN_msg_type < 108U)
		{
			memcpy(&JudgeData_position.Enemy[temp_CAN_msg_type - 100U], data, 8U);
		}

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
		sentry_duration_last_rx_tick = HAL_GetTick();
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_DAMAGE_DIFF_CAN_ID)
	{
		// 伤害值差接收 (0x0003扩展, 2026-07-13)
		memcpy(&damage_diff, data, sizeof(DamageDiff_t));
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_MOTOR_OFFLINE_CAN_ID)
	{
		// 电机掉线状态接收 (0x0A1, 2026-07-19新增)
		// 只 memcpy 前 8 字节(位图+保留), last_rx_tick 字段单独维护
		motor_offline_recv.motor_offline_bitmap = data[0];
		memcpy(motor_offline_recv.reserve, data + 1, sizeof(motor_offline_recv.reserve));
		motor_offline_recv.last_rx_tick = HAL_GetTick();
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_UWB_STEER_CAN_ID)
	{
		// UWB+舵角接收 (0x0A2, 2026-07-21新增)
		// CAN payload 布局: uint16 uwb_angle_yaw | int16 steer_angle_x10 | reserve[4]
		// 解析后 steer_angle = steer_angle_x10 / 10.0f (单位: 度)
		uwb_steer_recv.uwb_angle_yaw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
		int16_t steer_angle_x10 = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
		uwb_steer_recv.steer_angle = (float)steer_angle_x10 / 10.0f;
		uwb_steer_recv.last_rx_tick = HAL_GetTick();
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_OUTPOST_HP_CAN_ID)
	{
		// 前哨站HP接收 (0x0A3, 2026-07-21新增)
		// CAN payload 布局: uint16 ally_outpost_HP | uint16 enemy_outpost_HP | reserve[4]
		// 直接使用原始uint16值, 不做6bit压缩解压
		outpost_hp_recv.ally_outpost_HP  = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
		outpost_hp_recv.enemy_outpost_HP = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
		outpost_hp_recv.last_rx_tick = HAL_GetTick();
	}
	else if (hcan->Instance == CHASSIS_CAN_COMM_CANx && rx_header->StdId == ROBOT_COMMAND_CAN_ID)
	{
		// 小地图下发指令接收 (0x09E, 来自底盘裁判系统0x0303)
		// [FIX] 物理上 0x09E 从底盘 CAN2 进入云台 CAN1 (CHASSIS_CAN_COMM_CANx = CAN1)
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
	const HAL_StatusTypeDef status =
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	if (status == HAL_OK)
	{
		CanMarkBusRecovered(hcan);
		MotorReceive(hcan, &rx_header, rx_data);
	}
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
	const HAL_StatusTypeDef status =
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	if (status == HAL_OK)
	{
		CanMarkBusRecovered(hcan);
		MotorReceive(hcan, &rx_header, rx_data);
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CanMarkBusRecovered(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CanMarkBusRecovered(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CanMarkBusRecovered(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	const uint32_t error = HAL_CAN_GetError(hcan);

	if ((error & HAL_CAN_ERROR_BOF) != 0U)
	{
		const uint8_t bit = CanGetRecoveryBit(hcan);
		can_bus_off_waiting_mask |= bit;

		if (hcan->Instance == CAN1)
		{
			can1_bus_off_count++;
		}
		else if (hcan->Instance == CAN2)
		{
			can2_bus_off_count++;
		}
	}

	/* Do not let an old BOF bit contaminate a later error callback. */
	(void)HAL_CAN_ResetError(hcan);
}

int8_t CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id, CAN_TxHeaderTypeDef *Motor_Send, uint32_t *wait_time)
{
	float dwt_start = DWT_GetTimeline_us();
	HAL_StatusTypeDef send_status = HAL_BUSY;

	if (hcan == NULL || data == NULL || Motor_Send == NULL || wait_time == NULL)
	{
		return FALSE;
	}

	Motor_Send->StdId = std_id;
	Motor_Send->IDE = CAN_ID_STD;
	Motor_Send->RTR = CAN_RTR_DATA;
	Motor_Send->DLC = 0x08;

	/*
	 * All callers are task-context callers now.  Serialize the mailbox select
	 * and register write so two equal-priority tasks cannot enter the HAL CAN
	 * transmit routine concurrently.  A full mailbox yields instead of busy
	 * spinning; transient contention is retried for up to 3 ms.
	 */
	do
	{
		uint32_t send_mail_box;

		taskENTER_CRITICAL();
		if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0U)
		{
			send_status = HAL_CAN_AddTxMessage(
				hcan, Motor_Send, (uint8_t *)data, &send_mail_box);
		}
		taskEXIT_CRITICAL();

		if (send_status == HAL_OK)
		{
			*wait_time = (uint32_t)(DWT_GetTimeline_us() - dwt_start);
			return TRUE;
		}

		vTaskDelay(pdMS_TO_TICKS(1U));
	} while ((DWT_GetTimeline_us() - dwt_start) <= 3000.0f);

	*wait_time = (uint32_t)(DWT_GetTimeline_us() - dwt_start);
	global_debugger.can_send_wait++;
	return FALSE;
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

// 转发0x0307地图路径给底盘。
// USB CDC回调运行在中断上下文，只在Can1SendMapPath中发布最新105B快照；
// ChassisTask调用Can1ServiceMapPathTx逐帧可靠发送，避免在USB中断内阻塞/抢占CAN HAL邮箱。
// 105B map_data 通过 CAN 0x152 分 15 帧传输
// 帧格式: byte0 = segment_index(0~14), byte1-7 = 7B payload
// segment 0..13 各 7B = 98B, segment 14 = 最后 7B (105-98=7), 总计 105B
// 底盘收齐 15 帧后重组为 105B map_data_t, 封装 0x0307 通过裁判串口发送
// 详见 底盘侧map_data转发实现说明.md
#define MAP_PATH_SEG_COUNT  15U   /* ceil(105/7) = 15 */
#define MAP_PATH_SEG_SIZE   7U    /* 每帧有效负载 7B */
#define MAP_PATH_DATA_SIZE  (MAP_PATH_SEG_COUNT * MAP_PATH_SEG_SIZE)

typedef struct
{
	uint8_t active;
	uint8_t next_segment;
	uint8_t payload[MAP_PATH_DATA_SIZE];
} map_path_can_tx_t;

static uint8_t map_path_staged_payload[MAP_PATH_DATA_SIZE];
static volatile uint32_t map_path_staged_generation;
static uint32_t map_path_loaded_generation;
static map_path_can_tx_t map_path_can_tx;
MapPathTxDebug_t g_map_path_tx_debug;

void Can1SendMapPath(const uint8_t *map_data_105)
{
	uint32_t generation;

	if (map_data_105 == NULL)
	{
		return;
	}

	/* 单写者(USB CDC IRQ)发布快照。奇数generation表示拷贝进行中，
	 * 偶数表示快照稳定；任务侧用前后两次generation校验避免撕裂。 */
	generation = map_path_staged_generation;
	map_path_staged_generation = generation + 1U;
	__DMB();
	memcpy(map_path_staged_payload, map_data_105, MAP_PATH_DATA_SIZE);
	__DMB();
	map_path_staged_generation = generation + 2U;
	g_map_path_tx_debug.snapshot_queued_count++;
}

static uint8_t MapPathLoadLatestSnapshot(void)
{
	uint32_t generation_before;
	uint32_t generation_after;

	if (map_path_can_tx.active != 0U)
	{
		return 0U;
	}

	generation_before = map_path_staged_generation;
	if ((generation_before & 1U) != 0U ||
		generation_before == map_path_loaded_generation)
	{
		return 0U;
	}

	memcpy(map_path_can_tx.payload, map_path_staged_payload, MAP_PATH_DATA_SIZE);
	__DMB();
	generation_after = map_path_staged_generation;
	if (generation_before != generation_after ||
		(generation_after & 1U) != 0U)
	{
		return 0U;
	}

	map_path_loaded_generation = generation_after;
	map_path_can_tx.next_segment = 0U;
	map_path_can_tx.active = 1U;
	g_map_path_tx_debug.batch_started_count++;
	return 1U;
}

void Can1ServiceMapPathTx(void)
{
	static CAN_TxHeaderTypeDef tx_header;
	static uint8_t send_data[8];
	static uint32_t wait_time;
	uint8_t seg;

	if (map_path_can_tx.active == 0U &&
		MapPathLoadLatestSnapshot() == 0U)
	{
		return;
	}

	seg = map_path_can_tx.next_segment;
	send_data[0] = seg;
	memcpy(&send_data[1],
		   &map_path_can_tx.payload[seg * MAP_PATH_SEG_SIZE],
		   MAP_PATH_SEG_SIZE);

	/* CanSend失败时保留当前segment，下一个1ms任务周期重试；只有成功
	 * 放入HAL邮箱后才推进，底盘不会再因中间一帧丢失而丢弃整条路径。 */
	if (CanSend(&hcan1,
				(int8_t *)send_data,
				SEND_TO_CHASSIS_MAP_PATH_CAN_ID,
				&tx_header,
				&wait_time) == TRUE)
	{
		g_map_path_tx_debug.can_frame_sent_count++;
		g_map_path_tx_debug.last_segment_sent = seg;
		map_path_can_tx.next_segment++;
		if (map_path_can_tx.next_segment == MAP_PATH_SEG_COUNT)
		{
			map_path_can_tx.active = 0U;
			g_map_path_tx_debug.batch_completed_count++;
		}
	}
	else
	{
		g_map_path_tx_debug.can_send_retry_count++;
	}
}

// 新增: 转发0x0308自定义信息给底盘 (2026-07-11协议)
// 34B custom_data 通过 CAN 0x153 分 5 帧传输
// 帧格式: byte0 = segment_index(0~4), byte1-7 = 7B payload
// segment 0..3 各 7B = 28B, segment 4 = 最后 6B (34-28=6), byte7 填充 0
// 底盘收齐 5 帧后重组为 34B custom_info_t, 封装 0x0308 通过裁判串口发送
#define CUSTOM_INFO_SEG_COUNT  5U   /* ceil(34/7) = 5 */
#define CUSTOM_INFO_SEG_SIZE  7U   /* 每帧有效负载 7B (最后一帧仅 6B 有效) */
#define CUSTOM_INFO_PAYLOAD   34U   /* 0x0308 custom_info_t payload 字节数 */

typedef struct
{
	uint8_t active;
	uint8_t next_segment;
	uint8_t payload[CUSTOM_INFO_PAYLOAD];
} custom_info_can_tx_t;

static uint8_t custom_info_staged_payload[CUSTOM_INFO_PAYLOAD];
static volatile uint32_t custom_info_staged_generation;
static uint32_t custom_info_loaded_generation;
static custom_info_can_tx_t custom_info_can_tx;

void Can1SendCustomInfo(const uint8_t *custom_data_34)
{
	uint32_t generation;

	if (custom_data_34 == NULL)
	{
		return;
	}

	generation = custom_info_staged_generation;
	custom_info_staged_generation = generation + 1U;
	__DMB();
	memcpy(custom_info_staged_payload, custom_data_34, CUSTOM_INFO_PAYLOAD);
	__DMB();
	custom_info_staged_generation = generation + 2U;
}

static uint8_t CustomInfoLoadLatestSnapshot(void)
{
	uint32_t generation_before;
	uint32_t generation_after;

	if (custom_info_can_tx.active != 0U)
	{
		return 0U;
	}

	generation_before = custom_info_staged_generation;
	if ((generation_before & 1U) != 0U ||
		generation_before == custom_info_loaded_generation)
	{
		return 0U;
	}

	memcpy(custom_info_can_tx.payload,
		   custom_info_staged_payload,
		   CUSTOM_INFO_PAYLOAD);
	__DMB();
	generation_after = custom_info_staged_generation;
	if (generation_before != generation_after ||
		(generation_after & 1U) != 0U)
	{
		return 0U;
	}

	custom_info_loaded_generation = generation_after;
	custom_info_can_tx.next_segment = 0U;
	custom_info_can_tx.active = 1U;
	return 1U;
}

void Can1ServiceCustomInfoTx(void)
{
	static CAN_TxHeaderTypeDef tx_header;
	static uint8_t send_data[8];
	static uint32_t wait_time;
	uint8_t seg;
	uint8_t payload_length;

	if (custom_info_can_tx.active == 0U &&
		CustomInfoLoadLatestSnapshot() == 0U)
	{
		return;
	}

	seg = custom_info_can_tx.next_segment;
	payload_length = (seg == (CUSTOM_INFO_SEG_COUNT - 1U))
		? (uint8_t)(CUSTOM_INFO_PAYLOAD - seg * CUSTOM_INFO_SEG_SIZE)
		: CUSTOM_INFO_SEG_SIZE;
	memset(send_data, 0, sizeof(send_data));
	send_data[0] = seg;
	memcpy(&send_data[1],
		   &custom_info_can_tx.payload[seg * CUSTOM_INFO_SEG_SIZE],
		   payload_length);

	if (CanSend(&hcan1,
				(int8_t *)send_data,
				SEND_TO_CHASSIS_CUSTOM_INFO_CAN_ID,
				&tx_header,
				&wait_time) == TRUE)
	{
		custom_info_can_tx.next_segment++;
		if (custom_info_can_tx.next_segment == CUSTOM_INFO_SEG_COUNT)
		{
			custom_info_can_tx.active = 0U;
		}
	}
}
