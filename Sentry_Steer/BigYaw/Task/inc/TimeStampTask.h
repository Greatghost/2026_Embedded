#ifndef __TIMESTAMPTASK_H__
#define __TIMESTAMPTASK_H__

#include "stm32f4xx_hal.h"   // HAL库
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>           // sprintf/printf
#include <string.h>          // strcpy/strlen
#include "usart.h"

void TimeStampTask(void const *pvParameters);

#endif