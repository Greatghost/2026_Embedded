#include "ChassisTask.h"

int8_t send_to_chassis_data[2][8];        // 数据
CAN_TxHeaderTypeDef chassis_tx_header[2]; // 传输头
uint32_t chassis_send_wait_time[2];       // 传输等待时间

extern uint8_t motor_send_data[2][SEND_ID_NUMS][8];
extern uint8_t is_has_motor_data[2][SEND_ID_NUMS];




/**
 * @brief 处理速度数据，将底盘期望速度发送给底盘stm32
 * @param[in] void
 */
void ChassisTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1000HZ

    static int i = 0;

    /* Preserve the original five-second chassis-output warm-up without
     * leaving USB RX unserviced (a 512-byte ring would otherwise overflow). */
    {
        const TickType_t warmup_start = xTaskGetTickCount();
        while ((TickType_t)(xTaskGetTickCount() - warmup_start) <
               pdMS_TO_TICKS(5000U))
        {
            PCStreamProcessPending();
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    }

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        /* USB IRQ only enqueues bytes; consume and validate them here. */
        PCStreamProcessPending();

        if (i % 2 == 0) // 500HZ
        {
						Pack_InfantryMode();//两帧合在一起，位置不够，去掉了传给底盘的PITCH角度(画UI用，先不画了)

            memcpy(send_to_chassis_data[0], &chassis_send_pack1, 8); // 模式信息 (结构体大小为8字节)

            CanSend(&CHASSIS_CAN_COMM_CAN_Handlerx, send_to_chassis_data[0], SEND_TO_CHASSIS_CAN_ID_1, &chassis_tx_header[0], &chassis_send_wait_time[0]);
        }
				// if(i % 4 == 0) // 250hz
				// {//Motor_Data_Pack_1();
				// Motor_Data_Send_1();
				// }
        //哨兵坐标 → CAN 0x151 (10Hz)
        if (i % 50 == 0) //10Hz
        {
            Pack_SentryCoord();
            memcpy(send_to_chassis_data[1], &chassis_send_pack2, 8);
            CanSend(&CHASSIS_CAN_COMM_CAN_Handlerx, send_to_chassis_data[1], SEND_TO_CHASSIS_CAN_ID_2, &chassis_tx_header[1], &chassis_send_wait_time[1]);
        }

        /* USB中断只发布路径快照；这里每1ms发送或重试一个CAN 0x152分段。
         * 15段通常在15ms内完成，且不会与USB IRQ并发进入HAL CAN发送。 */
        Can1ServiceMapPathTx();
        Can1ServiceCustomInfoTx();

//        // 1 kHZ
//        Pack_Yaw();

//        memcpy(send_to_chassis_data[1], &chassis_send_pack2, 8); // yaw轴信息

//        CanSend(&CHASSIS_CAN_COMM_CAN_Handlerx, send_to_chassis_data[1], SEND_TO_CHASSIS_CAN_ID_2, &chassis_tx_header[1], &chassis_send_wait_time[1]);

        i++;

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
