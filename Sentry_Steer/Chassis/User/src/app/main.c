/**
 ******************************************************************************
 * @file    main.c
 * @brief   主函数，程序进口
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "main.h"

/* clock frequency */
RCC_ClocksTypeDef get_rcc_clock;

int main(void)
{
    /* get clock frequency */

 	RCC_GetClocksFreq(&get_rcc_clock);

#ifdef DEBUG_MODE
    SEGGER_RTT_Init();
    LOG_CLEAR();                                                                                    
#ifdef JSCOPE_RTT_MODE
    JscopeRTTInit();
#endif // JSCOPE_RTT_MODE
#endif // DEBUG_MODE

    /* init */
    BSP_Init();
#if !TEST_TASK_ON
    Robot_Init();
		Is_Motor_On_Check();
#endif // DEBUG

    /* create task */
    startTask();

    /* 调度 */
    vTaskStartScheduler();

    while (1)
        ;
}

/**
 * @brief STM32初始化
 * @param[in] void
 */
void BSP_Init(void)
{
#ifdef DEBUG_MODE_FREERTOS
    /*  TIM7初始化，用于FreeRTOS 计算CPU占用情况 */
    OS_TICK_Configuration();
    delay_ms(1);
#endif // DEBUG_MODE_FREERTOS

    /*  计数器初始化，us级计时 */
    COUNTER_Configuration();
    delay_ms(1);

    // SuperPower_Configuration();
    // delay_ms(1);

    /*  CAN1初始化 */
    CAN1_Configuration();
    delay_ms(1);

    /*  CAN2初始化,初始化前CAN1需初始化 */
    CAN2_Configuration();
    delay_ms(1);

    // #ifdef NO_GIMBAL_TEST // 仅无云台测试时获得遥控器信息
    //     DJI_REMOTE_Configuration();
    //     delay_ms(1);
    // #endif

    /*  与PC间的串口通信 */
    //    PC_UART_Configuration();
    //    delay_ms(1);

    REFEREE_Configuration();
    delay_ms(1);

    // WIFI_Configuration();
    // delay_ms(1);

    BLUE_TOOTH_Configuration(); // 可当VOFA使用
    delay_ms(1);

}
/**
 * @brief 机器人初始化
 * @param[in] void
 */
void Robot_Init(void)
{
    ONLY_LED_R_ON;

    delay_ms(10);

    initRemoteControl(DJI_REMOTE_CONTROL);

    ONLY_LED_B_ON;

    setRobotType(); // 设置机器人类型
}

static uint8_t Required_Motors_Ready(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (global_debugger.wheels_comm_debugger[i].recv_msgs_num <= 5)
        {
            return FALSE;
        }

        if (infantry.chassis_type == STEER_WHEEL &&
            global_debugger.steers_comm_debugger[i].recv_msgs_num <= 5)
        {
            return FALSE;
        }
    }

    return TRUE;
}

/**
 * @brief 有界等待必需电机反馈；超时后进入运行期掉线保护
 * @param[in] void
 */
void Is_Motor_On_Check(void)
{
	infantry.All_Motor_On = FALSE;
	infantry.Motor_Init_Time = 0.0f;
	GetDeltaT(&infantry.Timer); // 仅初始化计时基准，避免把此前启动耗时算进超时

	while(infantry.Motor_Init_Time < 1.0f && infantry.All_Motor_On == FALSE)
	{
			if (Required_Motors_Ready())
			{
				infantry.All_Motor_On = TRUE;
			}
				
			infantry.Motor_Init_Time += GetDeltaT(&infantry.Timer);
			delay_ms(1);
	}
	/*
	 * 超时后也必须继续启动调度器。运行期 chassis_control_check() 会保持
	 * 电机零电流，并在所有必需反馈连续恢复 500 ms 后自动重新使能。
	 */
}
