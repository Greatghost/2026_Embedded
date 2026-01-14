#ifndef _DM_MOTOR_H_
#define _DM_MOTOR_H_

#include "can.h"
#include "struct_typedef.h"
#include "tools.h"

#define DM_T_Data_MAX 4096   //非电机最大力矩，驱动内部参数,不修改
#define DM_P_Data_MAX 65536
#define DM_V_Data_MAX 4096

// TODO: 缩进&编码

typedef enum DM_MODE
{
	DM_ENABLE,
	DM_DISABLE,
	DM_MIT_CONTROL
} DM_MODE;
#pragma pack(push, 1)
typedef struct DM_MIT
{
	float P_des;
	float V_des;
	uint16_t Kp;
	uint16_t Kd;
	float t_ff;

	uint16_t Send_P_des;
	uint16_t Send_V_des;
	uint16_t Send_Kp;
	uint16_t Send_Kd;
	uint16_t Send_t_ff;

	float T_Max;
	float P_Max;
	float V_Max;

	char Motor_Enable;

	float P_Receive;
	float V_Receive;
	float t_ff_Receive;

	uint8_t ERR_State;

	// 温度
	uint8_t T_MOS;
	uint8_t T_Rotor;
} DM_MIT;

#pragma pack(pop)

void DM_Motor_Control(DM_MIT *Motor, uint8_t *send_data, DM_MODE mode);
void DM_Motor_Receive(uint8_t *Data_Receive, DM_MIT *Motor);
void DM_Motor_Init(DM_MIT *Motor, float P_Max, float T_Max, float V_Max);

#endif
