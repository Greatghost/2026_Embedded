#include "DM_Motor.h"

void DM_Motor_Init(DM_MIT *Motor, float P_Max, float T_Max, float V_Max)
{
	Motor->Motor_Enable = 0;
	Motor->P_Max = P_Max;
	Motor->T_Max = T_Max;
	Motor->V_Max = V_Max;
}

/* 给定输入以及模式，输出数据 */
void DM_Motor_Control(DM_MIT *Motor, uint8_t *send_data, DM_MODE mode)
{
	static uint8_t enable_data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	static uint8_t disable_data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	static uint8_t clear_error_data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

	if (Motor->ERR_State == 0xd)
	{
		memcpy(send_data, clear_error_data, 8);
		return;
	}
	if (mode == DM_ENABLE || !Motor->Motor_Enable)
	{
		memcpy(send_data, enable_data, 8);
		Motor->Motor_Enable = 1;
	}
	else if (mode == DM_DISABLE)
	{
		memcpy(send_data, disable_data, 8);
	}
	else
	{
		LIMIT_MAX_MIN(Motor->P_des, +Motor->P_Max, -Motor->P_Max);
		LIMIT_MAX_MIN(Motor->V_des, +Motor->V_Max, -Motor->V_Max);
		LIMIT_MAX_MIN(Motor->t_ff, +Motor->T_Max * 1000 - 1, -Motor->T_Max * 1000 + 1);
		LIMIT_MAX_MIN(Motor->Kd, 5, 0);
		LIMIT_MAX_MIN(Motor->Kp, 500, 0);

		Motor->Send_P_des = (uint16_t)((Motor->P_des) / Motor->P_Max * (DM_P_Data_MAX / 2) + DM_P_Data_MAX / 2);
		Motor->Send_V_des = (uint16_t)((Motor->V_des) / Motor->V_Max * (DM_V_Data_MAX / 2) + DM_V_Data_MAX / 2);
		Motor->Send_t_ff = (uint16_t)((Motor->t_ff / 1000) / Motor->T_Max * (DM_T_Data_MAX / 2) + DM_T_Data_MAX / 2);
		Motor->Send_Kp = (uint16_t)(Motor->Kp);
		Motor->Send_Kd = (uint16_t)(Motor->Kd);

		send_data[0] = ((Motor->Send_P_des) >> 8) & 0xff;
		send_data[1] = ((Motor->Send_P_des)) & 0xff;
		send_data[2] = ((Motor->Send_V_des >> 4)) & 0xff;
		send_data[3] = (((Motor->Send_P_des << 4)) & 0xff) | (((Motor->Send_Kp >> 8)) & 0xff);
		send_data[4] = ((Motor->Send_Kp)) & 0xff;
		send_data[5] = ((Motor->Send_Kd) >> 4) & 0xff;
		send_data[6] = (((Motor->Send_Kd) << 8) & 0xff) | (((Motor->Send_t_ff >> 8)) & 0xff);
		send_data[7] = ((Motor->Send_t_ff)) & 0xff;
	}
}

/**********************************************************************************************************
 *函 数 名: DM_Motor_Receive
 *功能说明: 注意，在使用此驱动的时候，电机的CAN_ID必须为 0x0x（x可以为任意值，但是高15-8位必须为0）
 *形    参:
 *返 回 值:
 **********************************************************************************************************/

void DM_Motor_Receive(uint8_t *Data_Receive, DM_MIT *Motor)
{
	uint16_t Receive_p;
	uint16_t Receive_v;
	uint16_t Receive_t;

	uint8_t T_MOS;
	uint8_t T_Rotor;

	Receive_p = (Data_Receive[1] << 8) | (Data_Receive[2]);
	Receive_v = (Data_Receive[3] << 4) | (Data_Receive[4] >> 4);
	Receive_t = ((Data_Receive[4] & 0x0f) << 8) | (Data_Receive[5]);

	Motor->T_MOS = Data_Receive[6];
	Motor->T_Rotor = Data_Receive[7];
	Motor->V_Receive = (float)(Receive_v - DM_V_Data_MAX / 2) / (DM_V_Data_MAX / 2) * Motor->V_Max;
	Motor->P_Receive = (float)(Receive_p - DM_P_Data_MAX / 2) / (DM_P_Data_MAX / 2) * Motor->P_Max / PI * 180.0 + 180;
	Motor->t_ff_Receive = (float)(Receive_t - DM_T_Data_MAX / 2) / (DM_T_Data_MAX / 2) * Motor->T_Max * 1000;

	uint8_t ERR_Data = Data_Receive[0];
	Motor->ERR_State = ERR_Data >> 4;
	if (!Motor->ERR_State)
	{
		Motor->Motor_Enable = 0;
	}
}
