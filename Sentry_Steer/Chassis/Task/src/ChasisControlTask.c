/**
 ******************************************************************************
 * @file    ChasisControlTask.c
 * @brief   底盘控制任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChasisControlTask.h"

uint8_t cnt;

/**
 * @brief 底盘控制任务
 * @param[in] void
 */
void ChasisControl_task(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(500);

    infantry.delta_t = GetDeltaT(&infantry.last_cnt);
    InfantryInit(&infantry);

    // 初始化时间差
    infantry.delta_t = GetDeltaT(&infantry.last_cnt);
    vTaskDelay(1);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        // 获取两帧之间时间差
        infantry.delta_t = GetDeltaT(&infantry.last_cnt);

        get_sensors_info(&infantry.sensors_info);

        // 舵轮正运动学：计算底盘实际速度（通过舵角度和轮速度反解）
        if (infantry.chassis_type == STEER_WHEEL)
        {
            steer_pos_kinematics();
        }

        set_robot_speed(&infantry);

        wheels_accel(&infantry);

        main_control(&infantry);
        // steer_angle_debug();  // 舵轮角度调试：90°方波跳跃调参（调参时取消注释，同时注释掉上面的main_control）

        wheels_power_limit(&infantry);

        /* 执行控制 */
        execute_control(&infantry.excute_info);

        /*  喂狗 */
        xEventGroupSetBits(xCreatedEventGroup, CHASIS_CONTROL_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
