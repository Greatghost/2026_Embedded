#ifndef _TEST_TASK_H
#define _TEST_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "bsp_dwt.h"

#define TEST_TASK_ON 0

void Test_task(void *pvParameters);

#endif // _TEST_TASK_H
