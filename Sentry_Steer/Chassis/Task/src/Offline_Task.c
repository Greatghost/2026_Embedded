#include "Offline_Task.h"
#include "remote_control.h"
#include "GimbalSend.h"  // 新增: Can2SendMotorOffline + motor_offline_send
#include "Referee.h"

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
			
        // 遥控器离线状态已在RemoteReceive中检测处理
        // 此处仅更新状态标志
        offline_detector.remote_receive_num = global_debugger.remote_debugger.recv_msgs_num;

        // [NEW] 2026-07-19: 打包电机掉线状态并发送给云台 (CAN 0x0A1)
        // 位图 bit=1 表示掉线
        //   bit0-3 : 轮电机1-4 (wheel_3508_state)
        //   bit4-7 : 舵电机1-4 (steer_6020_state)
        {
            uint8_t bitmap = 0;
            for (int i = 0; i < 4; i++)
            {
                if (offline_detector.wheel_3508_state[i] == WHEEL_3508_OFF) bitmap |=  (1U << i);
                if (offline_detector.steer_6020_state[i] == STEER_6020_OFF) bitmap |=  (1U << (i + 4));
            }
            motor_offline_send.motor_offline_bitmap = bitmap;
            memset(motor_offline_send.reserve, 0, sizeof(motor_offline_send.reserve));
            Can2SendMotorOffline(&motor_offline_send);
        }

        // [NEW] 2026-07-21: 发送 UWB角度+舵角当前角 (CAN 0x0A2, 10Hz)
        UwbSteerPack();
        Can2SendUwbSteer(&uwb_steer_send);

        // [NEW] 2026-07-21: 发送前哨站HP原始值 (CAN 0x0A3, 1Hz, 高精度不走6bit压缩)
        if (Referee_IsRobotHPFresh())
        {
            OutpostHPPack();
            Can2SendOutpostHP(&outpost_hp_send);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 所有数据都应该超过5HZ
    }
}
