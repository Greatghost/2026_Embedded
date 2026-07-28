/**
 ******************************************************************************
 * @file    protocol.h
 * @author  Karolance Future
 * @version V2.0.0
 * @date    2026/06/26
 * @brief   依据 RoboMaster 2026 通信协议 V2.0.0
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF 0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128U
#define REF_PROTOCOL_HEADER_SIZE sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2

#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

typedef enum
{
  GAME_STATE_CMD_ID = 0x0001,        // 比赛状态数据
  GAME_RESULT_CMD_ID = 0x0002,       // 比赛结果数据
  GAME_ROBOT_HP_CMD_ID = 0x0003,     // 机器人血量数据

  FIELD_EVENTS_CMD_ID = 0x0101,        // 场地事件数据
  REFEREE_WARNING_CMD_ID = 0x0104,     // 裁判警告数据
  DART_REMAINING_TIME_CMD_ID = 0x0105, // 飞镖发射相关数据

  ROBOT_STATE_CMD_ID = 0x0201,           // 机器人性能体系数据
  POWER_HEAT_DATA_CMD_ID = 0x0202,       // 底盘缓冲能量和射击热量
  ROBOT_POS_CMD_ID = 0x0203,             // 机器人位置
  BUFF_MUSK_CMD_ID = 0x0204,             // 机器人增益和底盘能量
  ROBOT_HURT_CMD_ID = 0x0206,            // 伤害状态
  SHOOT_DATA_CMD_ID = 0x0207,            // 实时射击信息
  BULLET_REMAINING_CMD_ID = 0x0208,      // 允许发弹量与剩余金币
  ROBOT_RFID_STATE_CMD_ID = 0x0209,      // 机器人 RFID 状态
  DART_CLIENT_CMD_ID = 0x020A,           // 飞镖选手端指令
  GROUND_ROBOT_POSITION_ID = 0x020B,     // 地面机器人位置
  RADAR_MARK_DATA_CMD_ID = 0x020C,       // 雷达标记进度
  SENTRY_INFO_CMD_ID = 0x020D,           // 哨兵自主决策信息同步
  RADAR_INFO_CMD_ID = 0x020E,            // 雷达自主决策信息同步
  STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301, // 机器人间通信
  ROBOT_COMMAND_CMD_ID = 0x0303,            // 选手端小地图交互
  CLIENT_MAP_COMMAND_CMD_ID = 0x0305,       // 选手端小地图接收雷达数据
  MAP_DATA_CMD_ID = 0x0307,                 // 选手端小地图接收路径
  CUSTOM_INFO_CMD_ID = 0x0308,              // 选手端小地图接收机器人信息
  IDCustomData,
} referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF = 0,
  STEP_LENGTH_LOW = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16 = 5,
} unpack_step_e;

#pragma pack(push, 1)

typedef struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

#pragma pack(pop)

#endif // ROBOMASTER_PROTOCOL_H
