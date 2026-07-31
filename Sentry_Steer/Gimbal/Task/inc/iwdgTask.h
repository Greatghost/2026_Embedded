#ifndef _IWDG_TASK_H
#define _IWDG_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx_hal.h"

#define IWDG_TASK_ON 1

#define IWDG_HEARTBEAT_INS       (1UL << 0)
#define IWDG_HEARTBEAT_ACTION    (1UL << 1)
#define IWDG_HEARTBEAT_GIMBAL    (1UL << 2)
#define IWDG_HEARTBEAT_OFFLINE   (1UL << 3)
#define IWDG_HEARTBEAT_CHASSIS   (1UL << 4)
#define IWDG_HEARTBEAT_ALL       (IWDG_HEARTBEAT_INS |     \
                                  IWDG_HEARTBEAT_ACTION |  \
                                  IWDG_HEARTBEAT_GIMBAL |  \
                                  IWDG_HEARTBEAT_OFFLINE | \
                                  IWDG_HEARTBEAT_CHASSIS)

extern volatile uint32_t iwdg_feed_count;
extern volatile uint32_t iwdg_last_missing_mask;
extern volatile uint8_t iwdg_was_reset;

void Iwdg_task(void *pvParameters);
void Iwdg_ReportAlive(uint32_t heartbeat);

#endif // !_IWDG_TASK_H
