#ifndef _OFFLINE_TASK_H
#define _OFFLINE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

enum ROBOT_SENSORS_DETECT
{
	NOT_INIT,  // 未初始化掉线检测
	REMOTE_ON, // 遥控器
	REMOTE_OFF,
	WHEEL_3508_ON,
	WHEEL_3508_OFF, // 掉线有处理
	STEER_6020_ON,
	STEER_6020_OFF,// 掉线有处理
	SUPER_CAP_ON,
	SUPER_CAP_OFF,// 掉了不做处理
	COMM_ON,
	COMM_OFF,	// 掉了做处理
};
#pragma pack(1)
typedef struct
{
	int16_t remote_receive_num;
	int16_t cap_receive_num;
	int16_t comm_receive_num[2];
	int16_t wheel_3508_receive_num[4];
	int16_t steer_6020_receive_num[4];

	enum ROBOT_SENSORS_DETECT remote_state;
	enum ROBOT_SENSORS_DETECT super_cap_state;
	enum ROBOT_SENSORS_DETECT comm_state[2];
	enum ROBOT_SENSORS_DETECT wheel_3508_state[4];
	enum ROBOT_SENSORS_DETECT steer_6020_state[4];
	int16_t wheel_3508_off_time[4];
	int16_t steer_6020_off_time[4];
	int16_t gimbal_comm_off_time;

	int8_t is_sensor_off;  // 是否有关键传感器掉线
	int8_t is_motor_error; // 电机报错检测
} OfflineDetector;
#pragma pack()

void Offline_task(void *pvParameters);

extern OfflineDetector offline_detector;

#endif // !_OFFLINE_TASK_H
