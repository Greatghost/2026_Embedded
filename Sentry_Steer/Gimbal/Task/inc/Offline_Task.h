#ifndef _OFFLINE_TASK_H
#define _OFFLINE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

/* 接收端心跳超时。监测任务周期必须明显小于这里的最小值。 */
#define OFFLINE_MONITOR_PERIOD_MS       10u
#define GIMBAL_FEEDBACK_TIMEOUT_MS     100u
#define IMU_FEEDBACK_TIMEOUT_MS        100u
#define SHOOT_FEEDBACK_TIMEOUT_MS      100u
#define PC_FEEDBACK_TIMEOUT_MS         200u
#define REMOTE_FEEDBACK_TIMEOUT_MS     100u
#define OFFLINE_RECOVERY_UPDATES         3u

enum ROBOT_SENSORS_DETECT
{
    NOT_INIT, //未初始化掉线检测
    IMU_ON,   // IMU
    IMU_OFF,
    REMOTE_ON, //遥控器
    REMOTE_OFF,
    PITCH_MOTOR_ON,
    PITCH_MOTOR_OFF,
    YAW_MOTOR_ON,
    YAW_MOTOR_OFF,
    FRICTION_WHEEL_MOTOR_ON,
    FRICTION_WHEELS_MOTOR_OFF,
    TOGGLE_MOTOR_ON,
    TOGGLE_MOTOR_OFF,
    PC_ON,
    PC_OFF
};
typedef struct
{
    uint16_t imu_receive_num[2];
    uint16_t remote_receive_num;
    uint16_t pitch_motor_receive_num;
    uint16_t yaw_motor_receive_num;
    uint16_t friction_motor_receive_num[2];
    uint16_t toggle_motor_receive_num;
    uint16_t pc_receive_num;

    uint32_t imu_last_update_time[2];
    uint32_t remote_last_update_time;
    uint32_t pitch_motor_last_update_time;
    uint32_t yaw_motor_last_update_time;
    uint32_t friction_motor_last_update_time[2];
    uint32_t toggle_motor_last_update_time;
    uint32_t pc_last_update_time;

    uint8_t imu_received[2];
    uint8_t remote_received;
    uint8_t pitch_motor_received;
    uint8_t yaw_motor_received;
    uint8_t friction_motor_received[2];
    uint8_t toggle_motor_received;
    uint8_t pc_received;

    /* 掉线后必须收到若干次连续的新反馈，单个毛刺帧不能重新使能力矩。 */
    uint8_t imu_recovery_updates[2];
    uint8_t remote_recovery_updates;
    uint8_t pitch_motor_recovery_updates;
    uint8_t yaw_motor_recovery_updates;
    uint8_t friction_motor_recovery_updates[2];
    uint8_t toggle_motor_recovery_updates;
    uint8_t pc_recovery_updates;

    enum ROBOT_SENSORS_DETECT imu_state[2];
    enum ROBOT_SENSORS_DETECT remote_state;
    enum ROBOT_SENSORS_DETECT pitch_motor_state;
    enum ROBOT_SENSORS_DETECT yaw_motor_state;
    enum ROBOT_SENSORS_DETECT friction_motor_state[2];
    enum ROBOT_SENSORS_DETECT toggle_motor_state;
    enum ROBOT_SENSORS_DETECT pc_state;
} OfflineDetector;

void Offline_task(void *pvParameters);

extern OfflineDetector offline_detector;

#endif // !_OFFLINE_TASK_H
