/**
 ******************************************************************************
 * @file    RefereeTask.c
 * @brief   ??????????????��????????,?????????????
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "RefereeTask.h"

float UI_FRONT_ERR,UI_FRONT_SIN,UI_FRONT_COS;

Sentry_decision_referee_t sentry_decision_referee;
Student_interactive_header_data_t student_interactive_header_data;
uint8_t Decision_Transmit_Pack[SENTRY_DECISION_SIZE]; //哨兵决策数据包

float DT;
uint32_t T;
uint8_t Radar_double_hurt_chance;//供曾世鹏看雷达双倍易伤次数，之后不用就去掉

extern JudgeData_ForSend1_t JudgeData_ForSend1;
extern JudgeData_ForSend2_t JudgeData_ForSend2;
extern Referee_t referee_data;
/**
 * @brief ������UI����
 * @param[in] void
 */
void drawCapBar(graphic_data_struct_t *Graphic, uint8_t GraphOperate)
{
	u8 COLOR;

	if (cap_controller.cap_vol_state == CAP_VOL_HIGH)
		COLOR = UI_Color_Green;
	else if (cap_controller.cap_vol_state == CAP_VOL_MID)
		COLOR = UI_Color_Yellow;
	else
		COLOR = UI_Color_Orange;

	UI_Draw_Line(Graphic, "310", GraphOperate, 4, COLOR, CAP_BAR_WIDTH, CAP_BAR_UI_START_X, CAP_BAR_UI_START_Y, CAP_BAR_UI_START_X + (u16)(cap_controller.cap_energy_pecent * CAP_BAR_LENGTH), CAP_BAR_UI_START_Y);
}

//cmd_id 是帧首ID，哨兵自主决策属于机器人交互指令，故cmd_id为0x0301
void Send_to_Referee(uint16_t cmd_id, uint16_t data_len)
{
	static uint8_t seq = 0;
	static uint8_t Frame_Length;
	Frame_Length = HEADER_LEN + CMD_LEN + CRC_LEN + data_len;

	//帧首部封装
	{
		Decision_Transmit_Pack[0] = 0xA5;
		memcpy(&Decision_Transmit_Pack[1],(uint8_t*)&data_len, sizeof(data_len));//数据段中data的长度
		Decision_Transmit_Pack[3] = seq ++ ;
		Append_CRC8_Check_Sum(Decision_Transmit_Pack,HEADER_LEN);  //帧头校验CRC8
	}

	//命令ID
	memcpy(&Decision_Transmit_Pack[HEADER_LEN],(uint8_t*)&cmd_id, CMD_LEN);
	
	//尾部数据校验CRC16
	Append_CRC16_Check_Sum(Decision_Transmit_Pack,Frame_Length);  
	// DMA发送
	REFEREE_SendBytes(Decision_Transmit_Pack, Frame_Length);
}

void Send_DecisionPack()
{
	uint8_t id_ = referee_data.Game_Robot_State.robot_id;
	
	Student_interactive_header_data_t custom_interactive_header;
	custom_interactive_header.data_cmd_id = 0x0120;//子内容ID
	custom_interactive_header.send_ID = id_;//机器人ID
	custom_interactive_header.receiver_ID = 0x8080;//服务端ID
	
	

	uint8_t header_len = sizeof(Student_interactive_header_data_t);//数据段首部长度 6
	// 帧首部+cmd_id为7个字节
	memcpy((void*)(Decision_Transmit_Pack + 7), &custom_interactive_header, header_len);	//将数据段的数据段进行封装（封装段首）
	memcpy((void*)(Decision_Transmit_Pack + 7 + header_len), &sentry_decision_referee, sizeof(Sentry_decision_referee_t));			//将数据帧的数据段进行封装（封装数据）
	Send_to_Referee(0x0301,sizeof(Sentry_decision_referee_t) + header_len);//打包并发送整帧数据 6 + 4
}

void Sentry_Decision_Init(void)
{
	sentry_decision_referee.sentry_if_revive =0;
	sentry_decision_referee.sentry_immediate_revive =0;
	sentry_decision_referee.sentry_bullet_claim =0;
	sentry_decision_referee.sentry_remote_bullet_claim_times =0;
	sentry_decision_referee.sentry_remote_HP_claim_times =0;
	sentry_decision_referee.sentry_posture =0;  // 初始化姿态为未知
	sentry_decision_referee.reserve = 0;
}
/**
 * @brief ����ϵͳ��������
 * @param[in] void
 */

void Refereetask(void *pvParameters)
{
	portTickType xLastWakeTime;

	uint16_t UI_PushUp_Counter = 261;
	uint8_t is_fric_offline = 0, is_pc_offline = 0, is_cover_off = 0;

	static char chassis_state[5][8] = {"OFFLINE", "NOT_FOLL", "FOLLOW", "ROTATE", "ROTATE"};
	static char gimbal_state[7][8] = {"OFFLINE", "ACT", "AUTOAIM", "TEST", "SI", "SMA_BUFF", "BIG_BUFF"};

	uint16_t UI_PushUp_Counter_500;
	uint16_t UI_PushUp_Counter_60;
	uint16_t UI_PushUp_Counter_20;
	uint16_t UI_PushUp_Counter_10;

	static uint32_t index = 0;

	Sentry_Decision_Init();
	
	while (1)
	{
		Referee_UnpackFifoData();
		if(index % 50  == 0) // 2hz
		{
			if(JudgeData_ForSend1.is_game_start && (JudgeData_ForSend2.Self_blood == 0))
			{
				sentry_decision_referee.sentry_if_revive = 1;
			}
			else
			{
				sentry_decision_referee.sentry_if_revive = 0;
			}

			Send_DecisionPack();
		}

		

		//		DT = GetDeltaT(&T);

// 		/* UI���������� */
// 		UI_PushUp_Counter++;
// 		UI_PushUp_Counter_500 = UI_PushUp_Counter % 500; // 100HZ���� 5��ˢһ��
// 		UI_PushUp_Counter_60 = UI_PushUp_Counter % 60;	 // 600����ˢһ��
// 		UI_PushUp_Counter_20 = UI_PushUp_Counter % 20;	 // 100����ˢһ��
// 		UI_PushUp_Counter_10 = UI_PushUp_Counter % 10;	 // 100����ˢһ��
// 		/*****************************************************��̬���Ʋ���*****************************************************************/
// 		if (UI_PushUp_Counter_500 == 13) // ��������
// 		{
// #if ROBOT == CHEN_JING_YUAN
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "010", UI_Graph_Add, 3, UI_Color_White, 3, 588, 0, 801, 395);
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "011", UI_Graph_Add, 3, UI_Color_White, 3, 1359, 0, 1169, 395);
// #elif ROBOT == NIUNIU
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "010", UI_Graph_Add, 3, UI_Color_White, 3, 210, 0, 605, 600);
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "011", UI_Graph_Add, 3, UI_Color_White, 3, 1660, 0, 1278, 600);
// #elif ROBOT == NIU_MO_SON
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "010", UI_Graph_Add, 3, UI_Color_White, 3, 210, 0, 640, 600);
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "011", UI_Graph_Add, 3, UI_Color_White, 3, 1695, 0, 1197, 600);
// #elif ROBOT == QI_TIAN_DA_SHENG
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "010", UI_Graph_Add, 3, UI_Color_White, 3, 210, 0, 605, 600);
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "011", UI_Graph_Add, 3, UI_Color_White, 3, 1660, 0, 1278, 600);
// #endif		
// 			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if(UI_PushUp_Counter_500 == 53)
// 		{
// 			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "501", UI_Graph_Add, 5, UI_Color_Purple, 5, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y + CHASSIS_POS_WIDTH / 2, CHASSIS_POS_UI_START_X + CHASSIS_POS_LENGTH, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2);
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "502", UI_Graph_Add, 5, UI_Color_Orange, 5, 1700, 750, (uint16_t)(1700 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.sin_dir)), (uint16_t)(750 + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.cos_dir)));
// 			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);	
// 		}
// 		else if (UI_PushUp_Counter_500 == 103)
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "103", UI_Graph_Add, 2, UI_Color_Green, 17, 10, 3, 60, 750, "GIM_STATE:");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 149) // PC����
// 		{
// 			is_pc_offline = !gimbal_receiver_pack1.is_pc_on;
// 			if (!is_pc_offline)
// 				UI_Draw_String(&referee_data.UI_String.String, "002", UI_Graph_Add, 2, UI_Color_Green, 17, 14, 3, 60, 650, "PC      :ON");
// 			else
// 				UI_Draw_String(&referee_data.UI_String.String, "002", UI_Graph_Add, 2, UI_Color_Orange, 17, 14, 3, 60, 650, "PC      :OFF");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// #if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG
// 		else if (UI_PushUp_Counter_500 == 179) // ���ո�����
// 		{
// 			is_cover_off = remote_controller.bomb_bay ? 0 : 1;
// 			if (!is_cover_off)
// 				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Add, 2, UI_Color_Green,  18, 15, 3, 60, 800, "COVER   :OPEN ");
// 			else
// 				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Add, 2, UI_Color_Orange, 18, 15, 3, 60, 800, "COVER   :CLOSE");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// #endif
// 		else if (UI_PushUp_Counter_500 == 211) // ��ǰ���̿���ģʽ
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "100", UI_Graph_Add, 2, UI_Color_Green, 30, 12, 3, 155, 600, chassis_state[remote_controller.control_mode_action]);
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 251) // ��̨����ģʽ
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "101", UI_Graph_Add, 2, UI_Color_Green, 17, 12, 3, 120, 650, gimbal_state[remote_controller.gimbal_action]);
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 283) // ID����
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "102", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 600, 880, "ID:   ");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 317) // ��������
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "200", UI_Graph_Add, 2, UI_Color_Green, 25, 11, 3, 860, 70, "CAP:     V");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// //		else if (UI_PushUp_Counter_500 == 353) // PITCH����
// //		{
// //			UI_Draw_String(&referee_data.UI_String.String, "201", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1280, 800, "PITCH: ");
// //			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// //		}
// 		else if (UI_PushUp_Counter_500 == 353) // 双倍易伤次数
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "209", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1280, 800, "Radar: ");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 389)
// 		{
// 			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "309", UI_Graph_Add, 5, UI_Color_White, 5, CAP_BAR_UI_START_X, CAP_BAR_UI_START_Y + CAP_BAR_WIDTH / 2, CAP_BAR_UI_START_X + CAP_BAR_LENGTH, CAP_BAR_UI_START_Y - CAP_BAR_WIDTH / 2);

// 			drawCapBar(&referee_data.UI_Graph2.Graphic[1], UI_Graph_Add);

// 			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 431)
// 		{
// 			//UI_Draw_Float(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Add, 3, UI_Color_Pink, 15, 2, 4, 1380, 800, gimbal_receiver_pack1.gimbal_pitch / 100.0f);
			
// 			//雷达自定义消息
// 			UI_Draw_Int(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Add, 3, UI_Color_Orange, 15, 4, 1380, 800, Radar_double_hurt_chance);	
			
// 			UI_Draw_Float(&referee_data.UI_Graph5.Graphic[1], "301", UI_Graph_Add, 3, UI_Color_Orange, 25, 2, 4, 960, 70, cap_controller.cap_vol);

// 			UI_Draw_Int(&referee_data.UI_Graph5.Graphic[2], "302", UI_Graph_Add, 3, UI_Color_Cyan, 20, 2, 640, 880, 0);
			
// 			UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_500 == 467)
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "104", UI_Graph_Add, 2, UI_Color_Green, 17, 10, 3, 60, 700, "CHA_STATE:");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}

// 		/*****************************************************��̬���Ʋ���*****************************************************************/
// 		// ������̨����״̬UI
// 		else if (UI_PushUp_Counter_60 == 10)
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "101", UI_Graph_Change, 2, UI_Color_Orange, 17, 12, 3, 220, 750, gimbal_state[remote_controller.gimbal_action]);
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
		
// //		else if(UI_PushUp_Counter_60 == 20)
// //		{
// //			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "501", UI_Graph_Change, 5, UI_Color_Purple, 5, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y + CHASSIS_POS_WIDTH / 2, CHASSIS_POS_UI_START_X + CHASSIS_POS_LENGTH, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2);
// //			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "502", UI_Graph_Change, 5, UI_Color_Orange, 5, 1700, 750, (uint16_t)(1700 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.sin_dir)), (uint16_t)(750 + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.cos_dir)));
// //			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);	
// //		}

// 		// ���Ƶ��̿���״̬UI
// 		else if (UI_PushUp_Counter_60 == 30)
// 		{
// 			UI_Draw_String(&referee_data.UI_String.String, "100", UI_Graph_Change, 2, UI_Color_Orange, 17, 12, 3, 220, 700, chassis_state[remote_controller.control_mode_action]);
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}

// 		// UI����(2Hz)
// 		else if (UI_PushUp_Counter_60 == 40)
// 		{
// 			// pitch���
// //			UI_Draw_Float(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Change, 3, UI_Color_Pink, 15, 2, 4, 1380, 800, gimbal_receiver_pack1.gimbal_pitch / 100.0f);

// 			// Radar双倍易伤次数
// 			UI_Draw_Int(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Change, 3, UI_Color_Orange, 15, 4, 1380, 800, Radar_double_hurt_chance);			
			
// 			UI_Draw_Float(&referee_data.UI_Graph5.Graphic[1], "301", UI_Graph_Change, 3, UI_Color_Orange, 25, 2, 4, 960, 70, cap_controller.cap_vol);
// 			// �������ID
// 			UI_Draw_Int(&referee_data.UI_Graph5.Graphic[2], "302", UI_Graph_Change, 3, UI_Color_Cyan, 20, 2, 640, 880, gimbal_receiver_pack1.autoaim_id);

// 			UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
// 		}
// #if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG
// 		else if (UI_PushUp_Counter_60 == 50)
// 		{
// 			is_cover_off = remote_controller.bomb_bay ? 0 : 1;
// 			if (!is_cover_off)
// 				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Change, 2, UI_Color_Green,  18, 15, 3, 60, 800, "COVER   :OPEN ");
// 			else
// 				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Change, 2, UI_Color_Orange, 18, 15, 3, 60, 800, "COVER   :CLOSE");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// #endif
// 		else if (UI_PushUp_Counter_60 == 59)
// 		{
// 			is_pc_offline = !gimbal_receiver_pack1.is_pc_on;
// 			if (!is_pc_offline)
// 				UI_Draw_String(&referee_data.UI_String.String, "002", UI_Graph_Change, 2, UI_Color_Green, 17, 14, 3, 60, 650, "PC       :ON");
// 			else
// 				UI_Draw_String(&referee_data.UI_String.String, "002", UI_Graph_Change, 2, UI_Color_Orange, 17, 14, 3, 60, 650, "PC       :OFF");
// 			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
// 		}
// 		// ����ʣ���ѹ����
// 		else if (UI_PushUp_Counter_20 == 1)
// 		{
// 			drawCapBar(referee_data.UI_Graph1.Graphic, UI_Graph_Change);

// 			UI_PushUp_Graphs(1, &referee_data.UI_Graph1, referee_data.Game_Robot_State.robot_id);
// 		}
// 		else if (UI_PushUp_Counter_10 == 9)
// 		{
// 			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "501", UI_Graph_Change, 5, UI_Color_Green, 5, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y + CHASSIS_POS_WIDTH / 2, CHASSIS_POS_UI_START_X + CHASSIS_POS_LENGTH, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2);
// 			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "502", UI_Graph_Change, 5, UI_Color_Orange, 5, 1700, 750, (uint16_t)(CHASSIS_POS_UI_START_X + CHASSIS_POS_WIDTH / 2 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * UI_FRONT_SIN)), (uint16_t)(CHASSIS_POS_UI_START_Y + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * UI_FRONT_COS)));
// 			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);	
// 		}

		xEventGroupSetBits(xCreatedEventGroup, REFEREE_TASK_BIT); // ��־λ��һ
		index++;

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
}
