#include "ActionTask.h"

void Action_task(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(2000);
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();
#ifdef NO_GIMBAL_TEST
        // 仅仅在仅底盘运动状态生效
        get_control_info(&chassis_solver);

        // 转换速度
        infantry.target_x_v = chassis_solver.chassis_speed_x;
        infantry.target_y_v = chassis_solver.chassis_speed_y;
        infantry.target_yaw_v = chassis_solver.chassis_speed_w;
#else
         Gimbal_msgs_Decode1();
//         Gimbal_msgs_Decode2();
#endif

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
