#include "GimbalTask.h"



/**
 * @brief 云台控制任务
 * @param[in] void
 */
void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        HeatUpdate();
        
        JudgeDataCanSend();//发送裁判系统及血量数据

        

        xEventGroupSetBits(xCreatedEventGroup, GIMBAL_TASK_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}
