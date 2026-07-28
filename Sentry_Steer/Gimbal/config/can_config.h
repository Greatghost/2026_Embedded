#ifndef _MOTOR_CONFIG_H
#define _MOTOR_CONFIG_H

#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal_conf.h"

#include "can.h"
#include "robot_config.h"

// 电机接收配置(注意不会影响发送)



#if ROBOT == GOBLIN
// pitch 电机
#define PITCH_MOTOR_CAN_ID 0x11 // pitch轴ID
#define PITCH_MOTOR_CAN CAN2

// 摩擦轮电机
#define LEFT_FRICTION_WHEEL_CAN_ID 0x202  // 摩擦轮ID1(左) — 电调实际ID
#define RIGHT_FRICTION_WHEEL_CAN_ID 0x201 // 摩擦轮ID2(右) — 电调实际ID
#define FRICTION_WHEEL_CAN CAN2

// 大Yaw轴电机接收
#define BIG_YAW_MOTOR_CAN_ID 0x10 // big_yaw轴电机，DM8006
#define BIG_YAW_MOTOR_CAN CAN1

// [SMALL_YAW_REMOVED] 小Yaw轴电机已删除
// #define SMALL_YAW_MOTOR_CAN_ID 0x205 // small_yaw轴电机,RM6020
// #define SMALL_YAW_MOTOR_CAN CAN2

// 拨弹电机接受代码
#define TOGGLE_MOTOR_CAN_ID 0x204 // 拨弹电机
#define TOGGLE_MOTOR_CAN CAN1

// 底盘通信
#define SEND_TO_CHASSIS_CAN_ID_1 0x150
#define SEND_TO_CHASSIS_CAN_ID_2 0x151
#define GET_FROM_CHASSIS_CAN_ID_1 0x160
#define GET_FROM_CHASSIS_CAN_ID_2 0x161
#define JUDGE_RECEIVE_DATA_CAN_ID_1 0x094
#define JUDGE_RECEIVE_DATA_CAN_ID_2 0x096
#define SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1 0x097
#define SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID 0x098
#define SEND_TO_GIMBAL_POSITION_DATA_CAN_ID 0x099
#define GET_CHASSIS_SPEED_CAN_ID 0x09A  // 接收底盘速度数据CAN ID
// 新增: TypeID 7/8上行数据所需CAN ID (2026-05-06协议)
#define GET_SHOOT_DATA_CAN_ID 0x09B           // 接收底盘射击数据(0x0207)
#define GET_SENTRY_INFO_CAN_ID 0x09C          // 接收哨兵信息(0x020D)
#define GET_BULLET_EXTENDED_CAN_ID 0x09D      // 接收弹量扩展字段(0x0208扩展)
#define ROBOT_COMMAND_CAN_ID 0x09E              // 接收小地图下发指令(0x0303)
#define GET_SENTRY_DURATION_CAN_ID 0x09F          // 接收哨兵姿态时长数据(0x020D扩展)
#define GET_DAMAGE_DIFF_CAN_ID 0x0A0            // 接收伤害值差数据(0x0003扩展)
#define GET_MOTOR_OFFLINE_CAN_ID 0x0A1          // 接收电机掉线状态(2026-07-19新增)
#define GET_UWB_STEER_CAN_ID 0x0A2                // 接收UWB角度+舵角当前角(2026-07-21新增)
#define GET_OUTPOST_HP_CAN_ID 0x0A3              // 接收前哨站HP原始值(2026-07-21新增)
// 新增: 发送SentryCmd给底盘
#define SEND_TO_CHASSIS_SENTRY_CMD_CAN_ID 0x15A  // 发送SentryCmd给底盘
// 新增: 转发0x0307 map_data 给底盘 (2026-07-18 协议, 多帧分段传输)
#define SEND_TO_CHASSIS_MAP_PATH_CAN_ID 0x152  // 105B 分 15 帧, byte0=segment_index
// 新增: 转发0x0308 custom_info 给底盘 (2026-07-11 协议, 多帧分段传输)
#define SEND_TO_CHASSIS_CUSTOM_INFO_CAN_ID 0x153  // 34B 分 5 帧, byte0=segment_index
#define CHASSIS_CAN_COMM_CAN_Handlerx hcan1
#define CHASSIS_CAN_COMM_CANx CAN1

// 大yaw通信
#define BIG_YAW_CAN_COMM_CANx CAN2
#define GET_FROM_BIG_YAW_CAN_ID 0x166

//CAN1
// FIFO 0 接收ID
#define CAN1_FIFO0_ID0 SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1
#define CAN1_FIFO0_ID1 SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID
#define CAN1_FIFO0_ID2 TOGGLE_MOTOR_CAN_ID
#define CAN1_FIFO0_ID3 BIG_YAW_MOTOR_CAN_ID

// FIFO 1 接收ID
#define CAN1_FIFO1_ID0 JUDGE_RECEIVE_DATA_CAN_ID_1
#define CAN1_FIFO1_ID1 GET_FROM_CHASSIS_CAN_ID_1
#define CAN1_FIFO1_ID2 JUDGE_RECEIVE_DATA_CAN_ID_2
#define CAN1_FIFO1_ID3 SEND_TO_GIMBAL_POSITION_DATA_CAN_ID

//CAN2
// FIFO 0 接收ID
#define CAN2_FIFO0_ID0 GET_FROM_BIG_YAW_CAN_ID
#define CAN2_FIFO0_ID1 PITCH_MOTOR_CAN_ID
// [FIX] 原 0x000 占位会接受总线上的杂散 ID 0x000 帧, 改为重复已 listed ID 避免误收
#define CAN2_FIFO0_ID2 GET_FROM_BIG_YAW_CAN_ID  // 重复 ID0 (原为 SMALL_YAW_MOTOR_CAN_ID, 已删除)
#define CAN2_FIFO0_ID3 PITCH_MOTOR_CAN_ID       // 重复 ID1 (原误配 ROBOT_COMMAND_CAN_ID, 0x09E 物理进入 CAN1)

// FIFO 1 接收ID
#define CAN2_FIFO1_ID0 0x003
#define CAN2_FIFO1_ID1 LEFT_FRICTION_WHEEL_CAN_ID
#define CAN2_FIFO1_ID2 RIGHT_FRICTION_WHEEL_CAN_ID
#define CAN2_FIFO1_ID3 LEFT_FRICTION_WHEEL_CAN_ID  // [FIX] 原 0x000 改为重复 ID1

#elif ROBOT == TIGER

// pitch 电机
#define PITCH_MOTOR_CAN_ID 0x11 // pitch轴ID
#define PITCH_MOTOR_CAN CAN2

// 摩擦轮电机
#define LEFT_FRICTION_WHEEL_CAN_ID 0x202  // 摩擦轮ID1(左) — 电调实际ID
#define RIGHT_FRICTION_WHEEL_CAN_ID 0x201 // 摩擦轮ID2(右) — 电调实际ID
#define FRICTION_WHEEL_CAN CAN2

// 大Yaw轴电机接收
#define BIG_YAW_MOTOR_CAN_ID 0x10 // big_yaw轴电机，DM8006
#define BIG_YAW_MOTOR_CAN CAN1

// [SMALL_YAW_REMOVED] 小Yaw轴电机已删除
// #define SMALL_YAW_MOTOR_CAN_ID 0x205 // small_yaw轴电机,RM6020
// #define SMALL_YAW_MOTOR_CAN CAN2

// 拨弹电机接受代码
#define TOGGLE_MOTOR_CAN_ID 0x204 // 拨弹电机
#define TOGGLE_MOTOR_CAN CAN1

// 底盘通信
#define SEND_TO_CHASSIS_CAN_ID_1 0x150
#define SEND_TO_CHASSIS_CAN_ID_2 0x151
#define GET_FROM_CHASSIS_CAN_ID_1 0x160
#define GET_FROM_CHASSIS_CAN_ID_2 0x161
#define JUDGE_RECEIVE_DATA_CAN_ID_1 0x094
#define JUDGE_RECEIVE_DATA_CAN_ID_2 0x096
#define SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1 0x097
#define SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID 0x098
#define SEND_TO_GIMBAL_POSITION_DATA_CAN_ID 0x099
#define GET_CHASSIS_SPEED_CAN_ID 0x09A  // 接收底盘速度数据CAN ID
// 新增: TypeID 7/8上行数据所需CAN ID (2026-05-06协议)
#define GET_SHOOT_DATA_CAN_ID 0x09B           // 接收底盘射击数据(0x0207)
#define GET_SENTRY_INFO_CAN_ID 0x09C          // 接收哨兵信息(0x020D)
#define GET_BULLET_EXTENDED_CAN_ID 0x09D      // 接收弹量扩展字段(0x0208扩展)
#define ROBOT_COMMAND_CAN_ID 0x09E              // 接收小地图下发指令(0x0303)
#define GET_SENTRY_DURATION_CAN_ID 0x09F          // 接收哨兵姿态时长数据(0x020D扩展)
#define GET_DAMAGE_DIFF_CAN_ID 0x0A0            // 接收伤害值差数据(0x0003扩展)
#define GET_MOTOR_OFFLINE_CAN_ID 0x0A1          // 接收电机掉线状态(2026-07-19新增)
#define GET_UWB_STEER_CAN_ID 0x0A2                // 接收UWB角度+舵角当前角(2026-07-21新增)
#define GET_OUTPOST_HP_CAN_ID 0x0A3              // 接收前哨站HP原始值(2026-07-21新增)
// 新增: 发送SentryCmd给底盘
#define SEND_TO_CHASSIS_SENTRY_CMD_CAN_ID 0x15A  // 发送SentryCmd给底盘
// 新增: 转发0x0307 map_data 给底盘 (2026-07-18 协议, 多帧分段传输)
#define SEND_TO_CHASSIS_MAP_PATH_CAN_ID 0x152  // 105B 分 15 帧, byte0=segment_index
// 新增: 转发0x0308 custom_info 给底盘 (2026-07-11 协议, 多帧分段传输)
#define SEND_TO_CHASSIS_CUSTOM_INFO_CAN_ID 0x153  // 34B 分 5 帧, byte0=segment_index
#define CHASSIS_CAN_COMM_CAN_Handlerx hcan1
#define CHASSIS_CAN_COMM_CANx CAN1

// 大yaw通信
#define BIG_YAW_CAN_COMM_CANx CAN2
#define GET_FROM_BIG_YAW_CAN_ID 0x166

//CAN1
// FIFO 0 接收ID
#define CAN1_FIFO0_ID0 SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1
#define CAN1_FIFO0_ID1 SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID
#define CAN1_FIFO0_ID2 TOGGLE_MOTOR_CAN_ID
#define CAN1_FIFO0_ID3 BIG_YAW_MOTOR_CAN_ID

// FIFO 1 接收ID
#define CAN1_FIFO1_ID0 JUDGE_RECEIVE_DATA_CAN_ID_1
#define CAN1_FIFO1_ID1 GET_FROM_CHASSIS_CAN_ID_1
#define CAN1_FIFO1_ID2 JUDGE_RECEIVE_DATA_CAN_ID_2
#define CAN1_FIFO1_ID3 SEND_TO_GIMBAL_POSITION_DATA_CAN_ID

//CAN2
// FIFO 0 接收ID
#define CAN2_FIFO0_ID0 GET_FROM_BIG_YAW_CAN_ID
#define CAN2_FIFO0_ID1 PITCH_MOTOR_CAN_ID
// [FIX] 原 0x000 占位会接受总线上的杂散 ID 0x000 帧, 改为重复已 listed ID 避免误收
#define CAN2_FIFO0_ID2 GET_FROM_BIG_YAW_CAN_ID  // 重复 ID0 (原为 SMALL_YAW_MOTOR_CAN_ID, 已删除)
#define CAN2_FIFO0_ID3 PITCH_MOTOR_CAN_ID       // 重复 ID1 (原误配 ROBOT_COMMAND_CAN_ID, 0x09E 物理进入 CAN1)

// FIFO 1 接收ID
#define CAN2_FIFO1_ID0 0x003
#define CAN2_FIFO1_ID1 LEFT_FRICTION_WHEEL_CAN_ID
#define CAN2_FIFO1_ID2 RIGHT_FRICTION_WHEEL_CAN_ID
#define CAN2_FIFO1_ID3 LEFT_FRICTION_WHEEL_CAN_ID  // [FIX] 原 0x000 改为重复 ID1

#endif

// // 弹舱盖电机
// #define BAY_MOTOR_CAN_ID 0x204
// #define BAY_MOTOR_CAN CAN2

#endif // !_MOTOR_CONFIG_H