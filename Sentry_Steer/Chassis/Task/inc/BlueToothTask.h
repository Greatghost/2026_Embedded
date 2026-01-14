#ifndef _BLUE_TOOTH_TASK_H
#define _BLUE_TOOTH_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#include "bluetooth.h"
#include "ChasisControlTask.h"

void BlueToothTask(void *pvParameters);

#endif // !_BLUE_TOOTH_TASK_H
