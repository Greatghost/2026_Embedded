/**
 ******************************************************************************
 * @file    referee.c
 * @author  Karolance Future
 * @version V1.0.0
 * @date    2022/03/21
 * @brief    RoboMaster裁判系统数据解析、UI绘制与通信实现文件
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
// 包含裁判系统头文件、协议栈、字符串操作、裁判系统硬件驱动
#include "Referee.h"
#include "protocol.h"
#include "string.h"
#include "bsp_referee.h"

/* Private define ------------------------------------------------------------*/
// 定义裁判系统全局核心数据结构体实例
Referee_t referee_data;
// 定义裁判数据更新标志结构体实例
RefereeDataUpdate referee_data_updater;
// 外部声明：雷达消息更新标志
extern uint8_t radar_msg_update_flag;

/* Private variables ---------------------------------------------------------*/
// 裁判系统完整数据接收缓冲区
int8_t fullDataBuffer[MAX_REFEREE_DATA_LEN];

/* Functions -----------------------------------------------------------------*/

// 辅助函数：判断发送者是否是雷达并返回是对应哪一种信号
uint8_t Judge_Radar_id(uint16_t cmd_id){
  switch(cmd_id){
    case 0x0201:      return NEAREST_ENEMY_POS;  
    case 0x0202:      return ALL_ENEMY_POS;      
    case 0x0205:      return ALL_ENEMY_HP;       
    default:      return NOT_RADAR_DATA;     
  }
}

/**
 * @brief  裁判系统全局数据结构体初始化
 * @param  无
 * @retval 无
 * @note   将所有裁判系统数据成员清零，防止初始值干扰
 */
void Referee_StructInit(void)
{
	// 清零协议帧头
	memset(&referee_data.Referee_Receive_Header, 0, sizeof(referee_data.Referee_Receive_Header));

	// 清零 0x000X 比赛基础数据
	memset(&referee_data.Game_Status, 0, sizeof(referee_data.Game_Status));
	memset(&referee_data.Game_Result, 0, sizeof(referee_data.Game_Result));
	memset(&referee_data.Game_Robot_friend_HP, 0, sizeof(referee_data.Game_Robot_friend_HP));

	// 清零 0x010X 场地事件数据
	memset(&referee_data.Event_Data, 0, sizeof(referee_data.Event_Data));
	memset(&referee_data.Supply_Projectile_Action, 0, sizeof(referee_data.Supply_Projectile_Action));
	memset(&referee_data.Supply_Projectile_Booking, 0, sizeof(referee_data.Supply_Projectile_Booking));
	memset(&referee_data.Referee_Warning, 0, sizeof(referee_data.Referee_Warning));
	memset(&referee_data.Dart_Remaining_Time, 0, sizeof(referee_data.Dart_Remaining_Time));

	// 清零 0x020X 机器人状态数据
	memset(&referee_data.Game_Robot_State, 0, sizeof(referee_data.Game_Robot_State));
	memset(&referee_data.Power_Heat_Data, 0, sizeof(referee_data.Power_Heat_Data));
	memset(&referee_data.Game_Robot_Pos, 0, sizeof(referee_data.Game_Robot_Pos));
	memset(&referee_data.Buff_Musk, 0, sizeof(referee_data.Buff_Musk));
	// memset(&referee_data.Aerial_Robot_Energy, 0, sizeof(referee_data.Aerial_Robot_Energy));
	memset(&referee_data.Robot_Hurt, 0, sizeof(referee_data.Robot_Hurt));
	memset(&referee_data.Shoot_Data, 0, sizeof(referee_data.Shoot_Data));
	memset(&referee_data.Bullet_Remaining, 0, sizeof(referee_data.Bullet_Remaining));
	memset(&referee_data.rfid_status, 0, sizeof(referee_data.rfid_status));
	// memset(&referee_data.Dart_Client_Cmd, 0, sizeof(referee_data.Dart_Client_Cmd));
	memset(&referee_data.ground_robot_position, 0, sizeof(referee_data.ground_robot_position));
	// 清零 0x030X 机器人交互数据
	memset(&referee_data.Student_Interactive_Header_Data, 0, sizeof(referee_data.Student_Interactive_Header_Data));
	 memset(&referee_data.Robot_Interactive_Data, 0, sizeof(referee_data.Robot_Interactive_Data));
	memset(&referee_data.Robot_Command, 0, sizeof(referee_data.Robot_Command));
	memset(&referee_data.Client_Map_Command, 0, sizeof(referee_data.Client_Map_Command));
	// 清零雷达站发送的数据
	memset(&referee_data.Radar_Alert_Info, 0, sizeof(referee_data.Radar_Alert_Info));
	memset(&referee_data.Radar_Position_Info, 0, sizeof(referee_data.Radar_Position_Info));
	memset(&referee_data.Radar_Enemy_HP, 0, sizeof(referee_data.Radar_Enemy_HP));
}

/**
 * @brief  从DMA环形缓冲区解包裁判系统FIFO数据
 * @param  无
 * @retval 无
 * @note   采用状态机解析协议帧，包含帧头检测、长度读取、CRC校验
 */
void Referee_UnpackFifoData()
{
	// 计算当前已接收的总数据长度 = DMA剩余长度 + 环形缓冲区圈数*缓冲区大小
	referee_data.decoder.receive_data_len = REFEREE_RECVBUF_SIZE - DMA_GetCurrDataCounter(REFEREE_RECV_DMAx_Streamx) + referee_data.decoder.judgementFullCount * REFEREE_RECVBUF_SIZE;

	// 防止数据长度溢出，限制最大解析长度
	if (referee_data.decoder.receive_data_len - referee_data.decoder.decode_data_len > 2 * REFEREE_RECVBUF_SIZE)
	{
		referee_data.decoder.decode_data_len = referee_data.decoder.receive_data_len - 2 * REFEREE_RECVBUF_SIZE;
	}
	// 计算当前读取的缓冲区索引
	int read_arr = referee_data.decoder.decode_data_len % REFEREE_RECVBUF_SIZE;
	u8 byte;

	// 循环读取未解析的字节数据
	while (referee_data.decoder.receive_data_len > referee_data.decoder.decode_data_len + 1)
	{
		// 从DMA缓冲区读取单字节数据
		byte = Refereebuffer[read_arr];

		// 协议解析状态机
		switch (referee_data.decoder.judgementStep)
		{
		// 步骤1：检测帧头起始符 0xA5
		case STEP_HEADER_SOF:
			if (byte == 0xA5)
			{
				// 存储帧头，切换至长度低字节解析
				fullDataBuffer[referee_data.decoder.index++] = byte;
				referee_data.decoder.judgementStep = STEP_LENGTH_LOW;
			}
			else
			{
				// 帧头不匹配，重置索引
				referee_data.decoder.index = 0;
			}
			break;
		// 步骤2：读取数据长度低字节
		case STEP_LENGTH_LOW:
		{
			referee_data.decoder.data_len = byte;
			fullDataBuffer[referee_data.decoder.index++] = byte;
			referee_data.decoder.judgementStep = STEP_LENGTH_HIGH;
		}
		break;
		// 步骤3：读取数据长度高字节，拼接完整长度
		case STEP_LENGTH_HIGH:
		{
			referee_data.decoder.data_len |= (byte << 8);
			fullDataBuffer[referee_data.decoder.index++] = byte;
			// 长度合法，切换至帧序列号解析
			if (referee_data.decoder.data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
			{
				referee_data.decoder.judgementStep = STEP_FRAME_SEQ;
			}
			else
			{
				// 长度非法，重置状态机
				referee_data.decoder.judgementStep = STEP_HEADER_SOF;
				referee_data.decoder.index = 0;
			}
		}
		break;
		// 步骤4：读取帧序列号
		case STEP_FRAME_SEQ:
		{
			fullDataBuffer[referee_data.decoder.index++] = byte;
			referee_data.decoder.judgementStep = STEP_HEADER_CRC8;
		}
		break;
		// 步骤5：读取帧头CRC8校验位，并校验帧头
		case STEP_HEADER_CRC8:
		{
			fullDataBuffer[referee_data.decoder.index++] = byte;
			if (referee_data.decoder.index == REF_PROTOCOL_HEADER_SIZE)
			{
				// 帧头校验成功，进入数据+CRC16解析
				if (Verify_CRC8_Check_Sum((unsigned char *)fullDataBuffer, REF_PROTOCOL_HEADER_SIZE))
				{
					referee_data.decoder.judgementStep = STEP_DATA_CRC16;
				}
				else
				{
					// 校验失败，重置状态机，错误计数+1
					referee_data.decoder.judgementStep = STEP_HEADER_SOF;
					referee_data.decoder.index = 0;

					global_debugger.referee_debugger.err_msgs_num++;
				}
			}
		}
		break;
		// 步骤6：读取数据域+CRC16校验位，校验完整数据帧
		case STEP_DATA_CRC16:
		{
			// 读取完整数据帧
			if (referee_data.decoder.index < (REF_HEADER_CRC_CMDID_LEN + referee_data.decoder.data_len))
			{
				fullDataBuffer[referee_data.decoder.index++] = byte;
			}
			// 数据读取完成，校验并解析数据
			if (referee_data.decoder.index >= (REF_HEADER_CRC_CMDID_LEN + referee_data.decoder.data_len))
			{
				// 重置状态机
				referee_data.decoder.judgementStep = STEP_HEADER_SOF;
				referee_data.decoder.index = 0;
				// 完整数据帧校验成功，解析数据
				if (Verify_CRC16_Check_Sum((unsigned char *)fullDataBuffer, REF_HEADER_CRC_CMDID_LEN + referee_data.decoder.data_len))
				{
					Referee_SolveFifoData((unsigned char *)fullDataBuffer);
					global_debugger.referee_debugger.recv_msgs_num++;
				}
				else
				{
					// 校验失败，错误计数+1
					global_debugger.referee_debugger.err_msgs_num++;
				}
			}
		}
		break;

		default:
			// 默认状态：重置解析状态机
			referee_data.decoder.judgementStep = STEP_HEADER_SOF;
			referee_data.decoder.index = 0;
			break;
		}

		// 解析长度+1，更新缓冲区读取索引
		referee_data.decoder.decode_data_len++;
		read_arr = read_arr + 1 >= REFEREE_RECVBUF_SIZE ? 0 : read_arr + 1;
		// 重新计算接收总长度
		referee_data.decoder.receive_data_len = REFEREE_RECVBUF_SIZE - DMA_GetCurrDataCounter(REFEREE_RECV_DMAx_Streamx) + referee_data.decoder.judgementFullCount * REFEREE_RECVBUF_SIZE;
	}
	// 环形缓冲区边界处理，防止计数溢出
	if (referee_data.decoder.receive_data_len % REFEREE_RECVBUF_SIZE > (REFEREE_RECVBUF_SIZE / 3) &&
		referee_data.decoder.receive_data_len % REFEREE_RECVBUF_SIZE < (2 * REFEREE_RECVBUF_SIZE / 3))
	{
		referee_data.decoder.decode_data_len -= REFEREE_RECVBUF_SIZE * referee_data.decoder.judgementFullCount;
		referee_data.decoder.judgementFullCount = 0;
	}
}

/**
 * @brief  解析校验通过的裁判系统数据帧
 * @param  frame: 完整数据帧指针
 * @retval 无
 * @note   根据命令ID分发数据，存入对应全局结构体成员
 */
void Referee_SolveFifoData(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t index = 0;

	// 复制协议帧头到全局结构体
	memcpy(&referee_data.Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	// 读取命令ID
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

	// 根据命令ID解析对应数据
	switch (cmd_id)
	{
	// 比赛状态数据 0x0001
	case GAME_STATE_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0001_num++;
		memcpy(&referee_data.Game_Status, frame + index, sizeof(ext_game_status_t));
		break;
		//	case GAME_RESULT_CMD_ID:
		//		memcpy(&referee_data.Game_Result, frame + index, sizeof(ext_game_result_t));
		//		break;
	// 友方机器人血量数据 0x0003
	case GAME_ROBOT_HP_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0003_num++;
		memcpy(&referee_data.Game_Robot_friend_HP, frame + index, sizeof(robot_HP_friend_t));
		break;
		//	case DART_FLYING_STATE_CMD_ID:
		//		break;

	// 场地事件数据 0x0101
	case FIELD_EVENTS_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0101_num++;
		memcpy(&referee_data.Event_Data, frame + index, sizeof(ext_event_data_t));
		break;
		//	case SUPPLY_PROJECTILE_ACTION_CMD_ID:
		//		memcpy(&referee_data.Supply_Projectile_Action, frame + index, sizeof(ext_supply_projectile_action_t));
		//		break;
		//	case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
		//		memcpy(&referee_data.Supply_Projectile_Booking, frame + index, sizeof(ext_supply_projectile_booking_t));
		//		break;
		//	case REFEREE_WARNING_CMD_ID:
		//		memcpy(&referee_data.Referee_Warning, frame + index, sizeof(ext_referee_warning_t));
		//		break;
	// 飞镖剩余时间数据 0x0105
	case DART_REMAINING_TIME_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0105_num++;
		memcpy(&referee_data.Dart_Remaining_Time, frame + index, sizeof(ext_dart_remaining_time_t));
		break;

	// 机器人状态数据 0x0201
	case ROBOT_STATE_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0201_num++;
		referee_data_updater.is_max_power_data_update = TRUE;
		memcpy(&referee_data.Game_Robot_State, frame + index, sizeof(ext_game_robot_state_t));
		break;
	// 功率热量数据 0x0202
	case POWER_HEAT_DATA_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0202_num++;
		referee_data_updater.is_power_data_update = TRUE;
		heat_controller.heat_count++;
		memcpy(&referee_data.Power_Heat_Data, frame + index, sizeof(ext_power_heat_data_t));
		break;
	// 机器人位置数据 0x0203
	case ROBOT_POS_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0203_num++;
		memcpy(&referee_data.Game_Robot_Pos, frame + index, sizeof(ext_game_robot_pos_t));
		break;
	// 机器人BUFF数据 0x0204
	case BUFF_MUSK_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0204_num++;
		memcpy(&referee_data.Buff_Musk, frame + index, sizeof(ext_buff_musk_t));
		break;
		//	case AERIAL_ROBOT_ENERGY_CMD_ID:
		//		memcpy(&referee_data.Aerial_Robot_Energy, frame + index, sizeof(aerial_robot_energy_t));
		//		break;
	// 机器人受伤数据 0x0206
	case ROBOT_HURT_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0206_num++;
		memcpy(&referee_data.Robot_Hurt, frame + index, sizeof(ext_robot_hurt_t));
		break;
	// 射击数据 0x0207
	case SHOOT_DATA_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0207_num++;
		heat_controller.shoot_count++;
		memcpy(&referee_data.Shoot_Data, frame + index, sizeof(ext_shoot_data_t));
		break;
	// 弹丸剩余数量数据 0x0208
	case BULLET_REMAINING_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0208_num++;
		memcpy(&referee_data.Bullet_Remaining, frame + index, sizeof(ext_bullet_remaining_t));
		break;
	// 机器人RFID状态数据 0x0209
	case ROBOT_RFID_STATE_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0209_num++;
		memcpy(&referee_data.rfid_status, frame + index, sizeof(ext_rfid_status_t));
		break;
		//	case DART_CLIENT_CMD_ID:
		//		memcpy(&referee_data.Dart_Client_Cmd, frame + index, sizeof(ext_dart_client_cmd_t));
		//		break;
	// 地面机器人位置数据 0x020B
	case GROUND_ROBOT_POSITION_ID:
		global_debugger.referee_debugger.cmd_0x020B_num++;
		memcpy(&referee_data.ground_robot_position,frame + index, sizeof(ground_robot_position_t));
		break;
	// 哨兵状态数据 0x020D
	case SENTRY_INFO_CMD_ID:
		global_debugger.referee_debugger.cmd_0x020D_num++;
		memcpy(&referee_data.Sentry_info, frame + index, sizeof(sentry_info_t));
		break;
	// 机器人交互数据 0x0301
	case STUDENT_INTERACTIVE_DATA_CMD_ID:
	{
		// 0x0301接收计数
		global_debugger.referee_debugger.cmd_0x0301_num++;

		// 先读取数据段头部(6字节): data_cmd_id + sender_id + receiver_id
		uint16_t data_cmd_id;
		uint16_t sender_id;
		uint16_t receiver_id;
		memcpy(&data_cmd_id, frame + index, sizeof(uint16_t));
		index += sizeof(uint16_t);
		memcpy(&sender_id, frame + index, sizeof(uint16_t));
		index += sizeof(uint16_t);
		memcpy(&receiver_id, frame + index, sizeof(uint16_t));
		index += sizeof(uint16_t);

		// 保存到交互数据头
		referee_data.Robot_Interactive_Data.data_cmd_id = data_cmd_id;
		referee_data.Robot_Interactive_Data.sender_id = sender_id;
		referee_data.Robot_Interactive_Data.receiver_id = receiver_id;

		uint8_t sentry_id;
		// 判断哨兵ID：红方7号/蓝方107号
		if(referee_data.Game_Robot_State.robot_id < 10) sentry_id = 7;
		else sentry_id = 107;

		// 根据data_cmd_id分别解析不同的雷达数据
		if(receiver_id == sentry_id)
		{
			switch(data_cmd_id)
			{
				case 0x0201: // 哨兵预警信息: carID(2) + distance(4) + quadrant(2)
					global_debugger.referee_debugger.radar_0x0201_num++;
					memcpy(&referee_data.Radar_Alert_Info, frame + index, sizeof(radar_sentinel_alert_t));
					radar_msg_update_flag = NEAREST_ENEMY_POS;
					break;
				case 0x0202: // 哨兵赛场坐标: 7个float坐标(哨兵自身+敌方1,2,3,4,6,7)
					global_debugger.referee_debugger.radar_0x0202_num++;
					memcpy(&referee_data.Radar_Position_Info, frame + index, sizeof(radar_sentry_position_t));
					// 同步更新到Robot_Interactive_Data.position以兼容旧代码
					referee_data.Robot_Interactive_Data.position.sentry_x = referee_data.Radar_Position_Info.sentry_x;
					referee_data.Robot_Interactive_Data.position.sentry_y = referee_data.Radar_Position_Info.sentry_y;
					referee_data.Robot_Interactive_Data.position.enemy1_hero_x = referee_data.Radar_Position_Info.enemy1_hero_x;
					referee_data.Robot_Interactive_Data.position.enemy1_hero_y = referee_data.Radar_Position_Info.enemy1_hero_y;
					referee_data.Robot_Interactive_Data.position.enemy2_engineer_x = referee_data.Radar_Position_Info.enemy2_engineer_x;
					referee_data.Robot_Interactive_Data.position.enemy2_engineer_y = referee_data.Radar_Position_Info.enemy2_engineer_y;
					referee_data.Robot_Interactive_Data.position.enemy3_infantry_x = referee_data.Radar_Position_Info.enemy3_infantry_x;
					referee_data.Robot_Interactive_Data.position.enemy3_infantry_y = referee_data.Radar_Position_Info.enemy3_infantry_y;
					referee_data.Robot_Interactive_Data.position.enemy4_infantry_x = referee_data.Radar_Position_Info.enemy4_infantry_x;
					referee_data.Robot_Interactive_Data.position.enemy4_infantry_y = referee_data.Radar_Position_Info.enemy4_infantry_y;
					referee_data.Robot_Interactive_Data.position.enemy6_drone_x = referee_data.Radar_Position_Info.enemy6_drone_x;
					referee_data.Robot_Interactive_Data.position.enemy6_drone_y = referee_data.Radar_Position_Info.enemy6_drone_y;
					referee_data.Robot_Interactive_Data.position.enemy7_sentry_x = referee_data.Radar_Position_Info.enemy7_sentry_x;
					referee_data.Robot_Interactive_Data.position.enemy7_sentry_y = referee_data.Radar_Position_Info.enemy7_sentry_y;
					radar_msg_update_flag = ALL_ENEMY_POS;
					break;
				case 0x0205: // 敌方血量: 5个uint16(敌方1,2,3,4,7号)
					global_debugger.referee_debugger.radar_0x0205_num++;
					memcpy(&referee_data.Radar_Enemy_HP, frame + index, sizeof(radar_enemy_hp_t));
					// 同步更新到Robot_Interactive_Data.enemy_hp以兼容旧代码
					referee_data.Robot_Interactive_Data.enemy_hp.enemy1_hero_hp = referee_data.Radar_Enemy_HP.enemy1_hero_hp;
					referee_data.Robot_Interactive_Data.enemy_hp.enemy2_engineer_hp = referee_data.Radar_Enemy_HP.enemy2_engineer_hp;
					referee_data.Robot_Interactive_Data.enemy_hp.enemy3_infantry_hp = referee_data.Radar_Enemy_HP.enemy3_infantry_hp;
					referee_data.Robot_Interactive_Data.enemy_hp.enemy4_infantry_hp = referee_data.Radar_Enemy_HP.enemy4_infantry_hp;
					referee_data.Robot_Interactive_Data.enemy_hp.enemy7_sentry_hp = referee_data.Radar_Enemy_HP.enemy7_sentry_hp;
					radar_msg_update_flag = ALL_ENEMY_HP;
					break;
				default:
					radar_msg_update_flag = NOT_RADAR_DATA;
					break;
			}
		}
		break;
	}
	// 机器人控制指令 0x0303
	case ROBOT_COMMAND_CMD_ID:
		global_debugger.referee_debugger.cmd_0x0303_num++;
		memcpy(&referee_data.Robot_Command, frame + index, sizeof(ext_robot_command_t));
		break;
		//	case CLIENT_MAP_COMMAND_CMD_ID:
		//		memcpy(&referee_data.Client_Map_Command, frame + index, sizeof(ext_client_map_command_t));
		//		break;

	default:
		break;
	}
}

/*==============================================================================
			  ##### UI图形绘制参数配置函数 #####
  ==============================================================================
*/

/**
 * @brief  配置UI直线绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  Width: 线宽
 * @param  StartX: 起点X坐标
 * @param  StartY: 起点Y坐标
 * @param  EndX: 终点X坐标
 * @param  EndY: 终点Y坐标
 * @retval 无
 */
void UI_Draw_Line(graphic_data_struct_t *Graph,
				  char GraphName[3],
				  uint8_t GraphOperate,
				  uint8_t Layer,
				  uint8_t Color,
				  uint16_t Width,
				  uint16_t StartX,
				  uint16_t StartY,
				  uint16_t EndX,
				  uint16_t EndY)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Line;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

/**
 * @brief  配置UI矩形绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  Width: 线宽
 * @param  StartX: 起点X坐标
 * @param  StartY: 起点Y坐标
 * @param  EndX: 终点X坐标
 * @param  EndY: 终点Y坐标
 * @retval 无
 */
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,
					   char GraphName[3],
					   uint8_t GraphOperate,
					   uint8_t Layer,
					   uint8_t Color,
					   uint16_t Width,
					   uint16_t StartX,
					   uint16_t StartY,
					   uint16_t EndX,
					   uint16_t EndY)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Rectangle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

/**
 * @brief  配置UI圆形绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  Width: 线宽
 * @param  CenterX: 圆心X坐标
 * @param  CenterY: 圆心Y坐标
 * @param  Radius: 半径
 * @retval 无
 */
void UI_Draw_Circle(graphic_data_struct_t *Graph,
					char GraphName[3],
					uint8_t GraphOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t Width,
					uint16_t CenterX,
					uint16_t CenterY,
					uint16_t Radius)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Circle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->radius = Radius;
}

/**
 * @brief  配置UI椭圆绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  Width: 线宽
 * @param  CenterX: 圆心X坐标
 * @param  CenterY: 圆心Y坐标
 * @param  XHalfAxis: X轴半长
 * @param  YHalfAxis: Y轴半长
 * @retval 无
 */
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,
					 char GraphName[3],
					 uint8_t GraphOperate,
					 uint8_t Layer,
					 uint8_t Color,
					 uint16_t Width,
					 uint16_t CenterX,
					 uint16_t CenterY,
					 uint16_t XHalfAxis,
					 uint16_t YHalfAxis)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Ellipse;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

/**
 * @brief  配置UI圆弧绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  StartAngle: 起始角度
 * @param  EndAngle: 结束角度
 * @param  Width: 线宽
 * @param  CenterX: 圆心X坐标
 * @param  CenterY: 圆心Y坐标
 * @param  XHalfAxis: X轴半长
 * @param  YHalfAxis: Y轴半长
 * @retval 无
 */
void UI_Draw_Arc(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t StartAngle,
				 uint16_t EndAngle,
				 uint16_t Width,
				 uint16_t CenterX,
				 uint16_t CenterY,
				 uint16_t XHalfAxis,
				 uint16_t YHalfAxis)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Arc;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = StartAngle;
	Graph->end_angle = EndAngle;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

/**
 * @brief  配置UI浮点数绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  NumberSize: 数字大小
 * @param  Significant: 有效位数
 * @param  Width: 线宽
 * @param  StartX: 起点X坐标
 * @param  StartY: 起点Y坐标
 * @param  FloatData: 浮点数数值
 * @retval 无
 */
void UI_Draw_Float(graphic_data_struct_t *Graph,
				   char GraphName[3],
				   uint8_t GraphOperate,
				   uint8_t Layer,
				   uint8_t Color,
				   uint16_t NumberSize,
				   uint16_t Significant,
				   uint16_t Width,
				   uint16_t StartX,
				   uint16_t StartY,
				   float FloatData)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Float;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->end_angle = Significant;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	// 浮点数放大1000倍转换为整数存储
	int32_t IntData = FloatData * 1000;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = ((IntData & 0x001ffc00) >> 10);
	Graph->end_y = ((IntData & 0xffe00000) >> 21);
}

/**
 * @brief  配置UI整数绘制参数
 * @param  Graph: UI图形结构体指针
 * @param  GraphName: 图形名称(客户端索引)
 * @param  GraphOperate: 图形操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  NumberSize: 数字大小
 * @param  Width: 线宽
 * @param  StartX: 起点X坐标
 * @param  StartY: 起点Y坐标
 * @param  IntData: 整数数值
 * @retval 无
 */
void UI_Draw_Int(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t NumberSize,
				 uint16_t Width,
				 uint16_t StartX,
				 uint16_t StartY,
				 int32_t IntData)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Int;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	// 整数按位拆分存储
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = (IntData & 0x001ffc00) >> 10;
	Graph->end_y = (IntData & 0xffe00000) >> 21;
}

/**
 * @brief  配置UI字符串绘制参数
 * @param  String: UI字符串结构体指针
 * @param  StringName: 字符串名称(客户端索引)
 * @param  StringOperate: 操作类型
 * @param  Layer: 图层(0-9)
 * @param  Color: 颜色
 * @param  CharSize: 字符大小
 * @param  StringLength: 字符串长度
 * @param  Width: 线宽
 * @param  StartX: 起点X坐标
 * @param  StartY: 起点Y坐标
 * @param  StringData: 字符串内容
 * @retval 无
 */
void UI_Draw_String(string_data_struct_t *String,
					char StringName[3],
					uint8_t StringOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t CharSize,
					uint16_t StringLength,
					uint16_t Width,
					uint16_t StartX,
					uint16_t StartY,
					char *StringData)
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye = StringOperate;
	String->graphic_tpye = UI_Graph_String;
	String->layer = Layer;
	String->color = Color;
	String->start_angle = CharSize;
	String->end_angle = StringLength;
	String->width = Width;
	String->start_x = StartX;
	String->start_y = StartY;
	// 清空字符串缓冲区并复制内容
	memset(String->stringdata, 0, 30);
	for (int i = 0; i < StringLength; i++)
		String->stringdata[i] = *StringData++;
}

/*==============================================================================
			  ##### UI图形数据打包上传函数 #####
  ==============================================================================
*/

/**
 * @brief  打包并上传UI图形数据(1/2/5/7个图形)
 * @param  Counter: 图形数量(1/2/5/7)
 * @param  Graphs: 图形数据结构体指针
 * @param  RobotID: 机器人ID
 * @retval 无
 * @note   自动填充帧头、命令ID、CRC校验并发送
 */
void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs;

	/* 配置协议帧头 */
	Graph->Referee_Transmit_Header.SOF = HEADER_SOF;
	if (Counter == 1)
		Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if (Counter == 2)
		Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if (Counter == 5)
		Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if (Counter == 7)
		Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq = Graph->Referee_Transmit_Header.seq + 1;
	// 添加帧头CRC8校验
	Append_CRC8_Check_Sum((uint8_t *)(&Graph->Referee_Transmit_Header), sizeof(frame_header_struct_t));

	/* 配置命令ID */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/* 配置交互数据头 */
	if (Counter == 1)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if (Counter == 2)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if (Counter == 5)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if (Counter == 7)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID = RobotID;		   // 发送者ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; // 接收者ID

	/* 添加帧尾CRC16校验 */
	if (Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph1, sizeof(UI_Graph1_t));
	}
	else if (Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph2, sizeof(UI_Graph2_t));
	}
	else if (Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph5, sizeof(UI_Graph5_t));
	}
	else if (Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph7, sizeof(UI_Graph7_t));
	}

	// 通过裁判系统串口发送数据
	if (Counter == 1)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if (Counter == 2)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if (Counter == 5)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if (Counter == 7)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph7_t));
}

/**
 * @brief  打包并上传UI字符串数据
 * @param  String: 字符串数据结构体指针
 * @param  RobotID: 机器人ID
 * @retval 无
 */
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* 配置协议帧头 */
	String->Referee_Transmit_Header.SOF = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq = String->Referee_Transmit_Header.seq + 1;
	Append_CRC8_Check_Sum((uint8_t *)(&String->Referee_Transmit_Header), sizeof(frame_header_struct_t));

	/* 配置命令ID */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/* 配置交互数据头 */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID = RobotID;			// 发送者ID
	String->Interactive_Header.receiver_ID = RobotID + 256; // 接收者ID

	/* 添加帧尾CRC16校验 */
	Append_CRC16_Check_Sum((uint8_t *)String, sizeof(UI_String_t));

	/* 发送数据 */
	REFEREE_SendBytes((uint8_t *)String, sizeof(UI_String_t));
}

/**
 * @brief  打包并上传UI删除指令
 * @param  Delete: 删除操作结构体指针
 * @param  RobotID: 机器人ID
 * @retval 无
 */
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* 配置协议帧头 */
	Delete->Referee_Transmit_Header.SOF = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq = Delete->Referee_Transmit_Header.seq + 1;
	Append_CRC8_Check_Sum((uint8_t *)(&Delete->Referee_Transmit_Header), sizeof(frame_header_struct_t));

	/* 配置命令ID */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/* 配置交互数据头 */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID = RobotID;			// 发送者ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; // 接收者ID

	/* 添加帧尾CRC16校验 */
	Append_CRC16_Check_Sum((uint8_t *)Delete, sizeof(UI_Delete_t));

	/* 发送数据 */
	REFEREE_SendBytes((uint8_t *)Delete, sizeof(UI_Delete_t));
}
