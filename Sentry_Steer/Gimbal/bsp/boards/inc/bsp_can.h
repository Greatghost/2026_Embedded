#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"
#include "can_config.h"

#include "FrictionWheel.h"
#include "Gimbal.h"
#include "debug.h"
#include "ToggleBullet.h"
#include "ChassisGet.h"
#include "receive_data.h"


extern volatile uint8_t JudgeData_update;
extern volatile uint8_t Blood_update;
extern JudgeData_1_t JudgeRecieveData;
extern JudgeData_2_t JudgeRecieveData2;
extern JudgeBloodData_ForSend1_t JudgeBlood_F,JudgeBlood_E;
extern JudgeData_Buff_t JudgeData_Buff;
extern JudgeData_RFID_t JudgeData_RFID;
extern JudgeData_position_t JudgeData_position;

// chassis_pack_get_1 和 chassis_speed_recv 已在ChassisGet.h中声明
// shoot_data_recv, sentry_info_recv, bullet_extended_recv 已在ChassisGet.h中声明

void can_filter_init(void);

int8_t CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id, CAN_TxHeaderTypeDef *Motor_Send, uint32_t *wait_time);

// 新增: 发送SentryCmd给底盘 (2026-05-06协议)
void Can1SendSentryCmd(uint32_t sentry_cmd);

// 新增: 转发0x0307地图路径 (2026-07-11协议 DownlinkTypeID=0x02)
// TODO: 多帧CAN分段传输 (105B > 8B DLC)
void Can1SendMapPath(const uint8_t *map_data_105);

// 新增: 转发0x0308自定义信息 (2026-07-11协议 DownlinkTypeID=0x03)
// TODO: 多帧CAN分段传输 (34B > 8B DLC)
void Can1SendCustomInfo(const uint8_t *custom_data_34);

#endif
