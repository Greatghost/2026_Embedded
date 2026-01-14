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

void can_filter_init(void);

int8_t CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id, CAN_TxHeaderTypeDef *Motor_Send, uint32_t *wait_time);
#endif
