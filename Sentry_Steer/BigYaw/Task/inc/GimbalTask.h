#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_control.h"
#include "Gimbal.h"
#include "FrictionWheel.h"
#include "can_config.h"

#include "M3508.h"
#include "ins_task.h"
#include "Offline_Task.h"
#include "BombBay.h"
#include "SystemIdentification.h"
#include "SignalGenerator.h"

#include "pc_serial.h"
#include "ToggleBullet.h"
#include "DM_Motor.h"
#include "can.h"
#include "Motor_Typdef.h"
#include "bsp_can.h"
#include "bsp_dwt.h"

#include "can_send_config.h"

void GimbalTask(void *pvParameters);

#endif // !_GIMBAL_TASK_H
