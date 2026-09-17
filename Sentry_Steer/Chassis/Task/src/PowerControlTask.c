#include "PowerControlTask.h"

/* 无裁判系统(离线/未安装)时的电池直连默认功率上限 W。
 * 有裁判时以 0x0201 chassis_power_limit 为准。 */
#define NO_REFEREE_POWER_LIMIT 100.0f

/* 功率策略调试记录 */
typedef struct
{
	uint8_t referee_online; // 1=裁判功率数据(0x0201)在线
	uint8_t cap_online;	// 1=电容板在线
	float referee_power;	// 本周期采用的功率基准
	float set_power;		// 本周期输出的底盘功率上限
} PowerCtrlDebug;

PowerCtrlDebug power_ctrl_debug;

float Interval;
uint32_t timtim;
float CapPowerSet()
{
	if( cap_controller.cap_vol<8.0f) return 0.f;
	else if(remote_controller.super_power_state == POWER_TO_SuperPower)
	{
		if(cap_controller.cap_vol >16.0f) return 100.0f;
		else if(cap_controller.cap_vol >8.0f) return (25.0f*(cap_controller.cap_vol - 12.0f));
		else return 0.f;

	}
	else{
		return LIMIT_MAX_MIN(5.0f*(cap_controller.cap_vol - 24.0f),30.0f,-60.0f);
	}

}

void PowerControlTask(void *pvParameters)
{
	portTickType xLastWakeTime;

	static int i = 0;

	CapControllerInit();

	float referee_power;
	float dynamic_referee_power;
	float dynamic_cap_power;


	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();

		// 从裁判系统读取底盘功率上限，限制范围30-200W；
		// 裁判离线/未安装(电池直连)时使用默认功率
		if (offline_detector.referee_power_state == REFEREE_POWER_ON)
			referee_power = LIMIT_MAX_MIN(referee_data.Game_Robot_State.chassis_power_limit, 200, 30);
		else
			referee_power = NO_REFEREE_POWER_LIMIT;
		//referee_power = 80.0f;
		//referee_data.Power_Heat_Data.buffer_energy = 60;
		uint8_t If_Game_Start = (referee_data.Game_Status.game_progress ==0x04)?1:0;

		// 使用裁判系统功率限制作为基准，根据缓冲能量动态调整
		float buffer_energy = referee_data.Power_Heat_Data.buffer_energy;
		//if (buffer_energy > 40.0f)
		//	dynamic_referee_power = referee_power + (buffer_energy - 40.0f);  // 缓冲能量高可超限使用
		//else
		//	dynamic_referee_power = referee_power - (40.0f - buffer_energy) * 0.5f;  // 缓冲能量低要保守，避免扣血

		dynamic_referee_power = referee_power;

		if (offline_detector.super_cap_state == SUPER_CAP_ON)
		{
			// 电容板在线: 原电容策略逻辑
			dynamic_cap_power = CapPowerSet();

			if (remote_controller.fly_state == IS_FLY)
			{
				NingCapControl(referee_data.Power_Heat_Data.buffer_energy, dynamic_referee_power, 300.0f);//全向轮300W飞坡姿态良好
			}
			else if (remote_controller.super_power_state == POWER_TO_SuperPower)
			{
				NingCapControl(referee_data.Power_Heat_Data.buffer_energy, dynamic_referee_power, dynamic_referee_power + dynamic_cap_power);
			}
			else if(gimbal_receiver_pack1.through_hole_flag)
			{
				NingCapControl(referee_data.Power_Heat_Data.buffer_energy, dynamic_referee_power, 45.0f);
			}
			else
			{
				NingCapControl(referee_data.Power_Heat_Data.buffer_energy, dynamic_referee_power, dynamic_referee_power + dynamic_cap_power);
			}

			if (i % 4 == 0) // 250HZ
			{
				SendCapPack(&cap_send_data, cap_controller.cap_power, referee_data.Power_Heat_Data.buffer_energy);
				Interval = GetDeltaT(&timtim);
				CanSend(SUPER_POWER_CAN, (int8_t *)(&cap_send_data), SEND_TO_SUPER_POWER_CAN_ID, 8);
			}
		}
		else
		{
			// 电容板离线/未安装: 电池直连, 绕过电容电压状态机
			// (不再 -5W 裁剪、不再计算充电功率、不再向 0x050 发帧)
			cap_controller.set_power = gimbal_receiver_pack1.through_hole_flag ? 45.0f : dynamic_referee_power;
			cap_controller.cap_power = 0.0f;
		}

		// 调试记录
		power_ctrl_debug.referee_online = (offline_detector.referee_power_state == REFEREE_POWER_ON);
		power_ctrl_debug.cap_online = (offline_detector.super_cap_state == SUPER_CAP_ON);
		power_ctrl_debug.referee_power = referee_power;
		power_ctrl_debug.set_power = cap_controller.set_power;

		i++;

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}
