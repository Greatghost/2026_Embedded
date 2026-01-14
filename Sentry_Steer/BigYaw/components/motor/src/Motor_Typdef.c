#include "Motor_Typdef.h"

uint32_t MOTOR_STD_ID_LIST[SEND_ID_NUMS] = {0x1FF, 0x1FE, 0x2FF, 0x2FE, 0x200, 0X05, 0x06, 0x100};
uint32_t motor_wait_time[2][SEND_ID_NUMS];
CAN_TxHeaderTypeDef motor_tx_header[2][SEND_ID_NUMS];
