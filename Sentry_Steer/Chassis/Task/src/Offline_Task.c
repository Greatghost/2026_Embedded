#include "Offline_Task.h"

OfflineDetector offline_detector;

void Offline_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // 待机器人初始化后开始检测

    while (1)
    {
        // 6020电机
        for (int i = 0; i < 4; i++)
        {
            if (global_debugger.steers_comm_debugger[i].recv_msgs_num != offline_detector.steer_6020_receive_num[i])
            {
                offline_detector.steer_6020_state[i] = STEER_6020_ON;
                offline_detector.steer_6020_receive_num[i] = global_debugger.steers_comm_debugger[i].recv_msgs_num;
            }
            else
            {
                offline_detector.steer_6020_state[i] = STEER_6020_OFF;
            }
        }		
			
        // 3508电机
        for (int i = 0; i < 4; i++)
        {
            if (global_debugger.wheels_comm_debugger[i].recv_msgs_num != offline_detector.wheel_3508_receive_num[i])
            {
                offline_detector.wheel_3508_state[i] = WHEEL_3508_ON;
                offline_detector.wheel_3508_receive_num[i] = global_debugger.wheels_comm_debugger[i].recv_msgs_num;
            }
            else
            {
                offline_detector.wheel_3508_state[i] = WHEEL_3508_OFF;
            }
        }

        // 板间通信
        for (int i = 0; i < 2; i++)
        {
            if (global_debugger.gimbal_comm_debugger[i].recv_msgs_num != offline_detector.comm_receive_num[i])
            {
                offline_detector.comm_state[i] = COMM_ON;
                offline_detector.comm_receive_num[i] = global_debugger.gimbal_comm_debugger[i].recv_msgs_num;
            }
            else
            {
                offline_detector.comm_state[i] = COMM_OFF;
            }
        }				
				// 超电板
				 if (global_debugger.super_power_debugger.recv_msgs_num != offline_detector.cap_receive_num)
         {
            offline_detector.super_cap_state = SUPER_CAP_ON;
            offline_detector.cap_receive_num = global_debugger.super_power_debugger.recv_msgs_num;
         }
         else
         {
            offline_detector.super_cap_state = SUPER_CAP_OFF;
         }
			
        // 遥控器
        if (global_debugger.remote_debugger.recv_msgs_num != offline_detector.remote_receive_num)
        {
            offline_detector.remote_state = REMOTE_ON;
            offline_detector.remote_receive_num = global_debugger.remote_debugger.recv_msgs_num;
        }
        else
        {
            offline_detector.remote_state = REMOTE_OFF;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 所有数据都应该超过5HZ
    }
}
