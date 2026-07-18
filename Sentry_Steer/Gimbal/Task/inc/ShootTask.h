#ifndef _SHOOTTASK_H
#define _SHOOTTASK_H


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ChassisSolver.h"
#include "can_config.h"

#include "ChassisSend.h"

#include "bsp_can.h"
#include "can.h"

typedef struct{
	uint8_t Friction_cmd;
	uint8_t Shoot_State; // only 2 ( at mask 0b11) bits are used
	uint8_t Shoot_State_send; // only 2 (at mask 0b11) bits are used
	uint8_t Shoot_Freq_cmd;
} Shoot_Cmd_t;

extern Shoot_Cmd_t shoot_cmd;
extern GimbalController gimbal_controller;
void ShootTask(void *pvParameters);
void Shoot_Cal(void);
void Shoot_FeedbackSafeStop(void);


#endif
