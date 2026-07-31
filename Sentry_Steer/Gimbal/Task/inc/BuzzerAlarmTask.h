#ifndef _BUZZER_ALARM_TASK_H
#define _BUZZER_ALARM_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/* CAN bus-off recovery audible indicator:
 * - CAN1 first successful frame after bus-off: one loud short beep.
 * - CAN2 first successful frame after bus-off: two loud short beeps.
 */
void BuzzerAlarmTask(void const * argument);

#endif // _BUZZER_ALARM_TASK_H
