/**
  ******************************************************************************
  * @file    BuzzerAlarmTask.c
  * @brief   Audible indication after a CAN bus recovers from bus-off.
  ******************************************************************************
  */

#include "BuzzerAlarmTask.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"

/* The normal INS startup tone is about 364 Hz.  A 4 kHz, 50% duty signal is
 * much brighter and uses the loudest square-wave duty cycle. */
#define CAN_RECOVERY_BEEP_FREQUENCY_HZ  4000U
#define CAN_RECOVERY_BEEP_ON_MS          200U
#define CAN_RECOVERY_BEEP_OFF_MS         140U
#define CAN_RECOVERY_STARTUP_GUARD_MS    700U
#define CAN_RECOVERY_POLL_MS              20U
#define CAN_RECOVERY_BUS_GAP_MS          250U

static void BuzzerPlayCanRecovery(uint8_t beep_count)
{
    uint8_t index;

    for (index = 0U; index < beep_count; index++)
    {
        Set_Buzzer_Frequency(CAN_RECOVERY_BEEP_FREQUENCY_HZ);
        vTaskDelay(pdMS_TO_TICKS(CAN_RECOVERY_BEEP_ON_MS));
        Set_Buzzer_Frequency(0U);

        if ((index + 1U) < beep_count)
        {
            vTaskDelay(pdMS_TO_TICKS(CAN_RECOVERY_BEEP_OFF_MS));
        }
    }
}

void BuzzerAlarmTask(void const *argument)
{
    (void)argument;

    Set_Buzzer_Frequency(0U);

    /* INS_task owns the buzzer during its roughly 500 ms warm-up indication. */
    vTaskDelay(pdMS_TO_TICKS(CAN_RECOVERY_STARTUP_GUARD_MS));

    for (;;)
    {
        const uint8_t alerts = CanTakeRecoveryAlerts();

        if ((alerts & CAN_RECOVERY_ALERT_CAN1) != 0U)
        {
            BuzzerPlayCanRecovery(1U);
        }

        if ((alerts & CAN_RECOVERY_ALERT_CAN2) != 0U)
        {
            if ((alerts & CAN_RECOVERY_ALERT_CAN1) != 0U)
            {
                vTaskDelay(pdMS_TO_TICKS(CAN_RECOVERY_BUS_GAP_MS));
            }
            BuzzerPlayCanRecovery(2U);
        }

        if (alerts == 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(CAN_RECOVERY_POLL_MS));
        }
    }
}
