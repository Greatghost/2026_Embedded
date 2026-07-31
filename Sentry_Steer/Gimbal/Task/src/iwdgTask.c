#include "iwdgTask.h"

#define IWDG_WRITE_ACCESS_KEY  0x5555U
#define IWDG_RELOAD_KEY        0xAAAAU
#define IWDG_START_KEY         0xCCCCU
#define IWDG_PRESCALER_256     0x0006U
#define IWDG_RELOAD_VALUE      999U
#define IWDG_CHECK_PERIOD_MS   100U

/*
 * Nominal timeout is (999 + 1) * 256 / 32 kHz = 8 s.  The actual LSI
 * frequency varies, so the shortest timeout is still above the two-second
 * startup delays in ActionTask and GimbalTask.
 */
static volatile uint32_t iwdg_heartbeat_mask = 0U;
volatile uint32_t iwdg_feed_count = 0U;
volatile uint32_t iwdg_last_missing_mask = IWDG_HEARTBEAT_ALL;
volatile uint8_t iwdg_was_reset = 0U;

void Iwdg_ReportAlive(uint32_t heartbeat)
{
    taskENTER_CRITICAL();
    iwdg_heartbeat_mask |= (heartbeat & IWDG_HEARTBEAT_ALL);
    taskEXIT_CRITICAL();
}

static uint32_t Iwdg_TakeHeartbeatSnapshot(void)
{
    uint32_t snapshot;

    taskENTER_CRITICAL();
    snapshot = iwdg_heartbeat_mask;
    iwdg_heartbeat_mask = 0U;
    taskEXIT_CRITICAL();

    return snapshot;
}

static void Iwdg_Start(void)
{
    /* Keep the watchdog stopped while a debugger has halted the Cortex-M4. */
    __HAL_DBGMCU_FREEZE_IWDG();

    /* Start first so the LSI is running while prescaler/reload update. */
    IWDG->KR = IWDG_START_KEY;
    IWDG->KR = IWDG_WRITE_ACCESS_KEY;
    IWDG->PR = IWDG_PRESCALER_256;
    IWDG->RLR = IWDG_RELOAD_VALUE;
    while (IWDG->SR != 0U)
    {
    }

    IWDG->KR = IWDG_RELOAD_KEY;
}

void Iwdg_task(void *pvParameters)
{
    (void)pvParameters;

    iwdg_was_reset = ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) ? 1U : 0U;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    Iwdg_Start();

    while (1)
    {
        const uint32_t alive = Iwdg_TakeHeartbeatSnapshot();
        const uint32_t missing = IWDG_HEARTBEAT_ALL & ~alive;

        iwdg_last_missing_mask = missing;
        if (missing == 0U)
        {
            IWDG->KR = IWDG_RELOAD_KEY;
            iwdg_feed_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(IWDG_CHECK_PERIOD_MS));
    }
}
