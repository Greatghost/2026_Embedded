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

extern GimbalController gimbal_controller;
void ShootTask(void *pvParameters);



#endif