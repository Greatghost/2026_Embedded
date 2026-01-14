#ifndef _CAN_SEND_CONFIG_H
#define _CAN_SEND_CONFIG_H

#include "Motor_Typdef.h"
#include "stm32f4xx_hal_conf.h"
#include "robot_config.h"

void Motor_Config_Init();



extern Motor_Communication motor_communication[MOTOR_APP_NUMS];

#endif