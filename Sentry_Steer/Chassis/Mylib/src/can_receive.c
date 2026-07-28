#include "can_receive.h"
#include "GimbalSend.h"   // 获取sentry_cmd_from_gimbal结构体
#include "GimbalReceive.h" // 获取gimbal_receiver_pack1/2结构体 和 MapPath_OnCanReceive
#include "counter.h"      // GetTime_ms() 用于 0x152 分段超时判定

void CanReceiveAll(CAN_TypeDef *can, CanRxMsg *rx_message)
{
    if (can == DJI_WHEELS_CAN)
    {
        switch (rx_message->StdId)
        {
        case DJI_3508_MOTORS_1:
            infantry.sensors_info.wheels_recv[0].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.wheels_recv[0].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.wheels_recv[0].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.wheels_recv[0].temp = rx_message->Data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[0], 0.0045f);
            offline_detector.wheel_3508_off_time[0] = 0;
            break;
        case DJI_3508_MOTORS_2:
            infantry.sensors_info.wheels_recv[1].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.wheels_recv[1].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.wheels_recv[1].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.wheels_recv[1].temp = rx_message->Data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[1], 0.0045f);
            offline_detector.wheel_3508_off_time[1] = 0;
            break;
        case DJI_3508_MOTORS_3:
            infantry.sensors_info.wheels_recv[2].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.wheels_recv[2].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.wheels_recv[2].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.wheels_recv[2].temp = rx_message->Data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[2], 0.0045f);
            offline_detector.wheel_3508_off_time[2] = 0;
            break;
        case DJI_3508_MOTORS_4:
            infantry.sensors_info.wheels_recv[3].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.wheels_recv[3].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.wheels_recv[3].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.wheels_recv[3].temp = rx_message->Data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[3], 0.0045f);
            offline_detector.wheel_3508_off_time[3] = 0;
            break;
        default:
            break;
        }
    }
    if (can == DJI_STEERS_CAN)
    {
        switch (rx_message->StdId)
        {
        case DJI_6020_MOTORS_1:
            infantry.sensors_info.steer_recv[0].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.steer_recv[0].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.steer_recv[0].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.steer_recv[0].temp = rx_message->Data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[0], 0.0045f);
            offline_detector.steer_6020_off_time[0]=0;
            break;
        case DJI_6020_MOTORS_2:
            infantry.sensors_info.steer_recv[1].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.steer_recv[1].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.steer_recv[1].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.steer_recv[1].temp = rx_message->Data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[1], 0.0045f);
            offline_detector.steer_6020_off_time[1]=0;
            break;
        case DJI_6020_MOTORS_3:
            infantry.sensors_info.steer_recv[2].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.steer_recv[2].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.steer_recv[2].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.steer_recv[2].temp = rx_message->Data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[2], 0.0045f);
            offline_detector.steer_6020_off_time[2]=0;
            break;
        case DJI_6020_MOTORS_4:
            infantry.sensors_info.steer_recv[3].angle = (rx_message->Data[0] << 8) | (rx_message->Data[1]);
            infantry.sensors_info.steer_recv[3].speed = (rx_message->Data[2] << 8) | (rx_message->Data[3]);
            infantry.sensors_info.steer_recv[3].torque_current = (rx_message->Data[4] << 8) | (rx_message->Data[5]);
            infantry.sensors_info.steer_recv[3].temp = rx_message->Data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[3], 0.0045f);
            offline_detector.steer_6020_off_time[3]=0;
            break;
        default:
            break;
        }
    }

    if (can == GIMBAL_CAN_COMM_CANx)
    {
        switch (rx_message->StdId)
        {
        case GIMBAL_COMM_CAN_ID_1:
            memcpy(&gimbal_receiver_pack1, rx_message->Data, 8);
            Gimbal_msgs_Decode1();
            LossUpdate(&global_debugger.gimbal_comm_debugger[0], 0.0085f);
            offline_detector.gimbal_comm_off_time = 0;
            break;
        case GIMBAL_COMM_CAN_ID_2:
            memcpy(&gimbal_receiver_pack2, rx_message->Data, 8);
            Gimbal_msgs_Decode2();
            LossUpdate(&global_debugger.gimbal_comm_debugger[1], 0.1f);  // 10Hz周期
            break;
        case GET_FROM_GIMBAL_SENTRY_CMD_CAN_ID:
            // 接收云台转发的SentryCmd
            memcpy(&sentry_cmd_from_gimbal, rx_message->Data, sizeof(SentryCmd_FromGimbal_t));
            // 更新sentry_decision_referee (合并shadow，不清除其他位)
            {
                extern Sentry_decision_referee_t sentry_decision_referee;
                uint32_t cmd = sentry_cmd_from_gimbal.sentry_cmd;
                // posture: bit21-23 (RM2026 V2.0, 3 bits, 1-6)
                sentry_decision_referee.sentry_posture = (cmd >> 21) & 0x07;
                // 其他位按协议保留，暂不处理
                // sentry_bullet_claim: bit2-12 (累计值)
                sentry_decision_referee.sentry_bullet_claim = (cmd >> 2) & 0x7FF;
                // sentry_remote_bullet_claim_times: bit13-16
                sentry_decision_referee.sentry_remote_bullet_claim_times = (cmd >> 13) & 0x0F;
                // sentry_remote_HP_claim_times: bit17-20
                sentry_decision_referee.sentry_remote_HP_claim_times = (cmd >> 17) & 0x0F;
                // sentry_if_revive: bit0
                sentry_decision_referee.sentry_if_revive = cmd & 0x01;
                // sentry_immediate_revive: bit1
                sentry_decision_referee.sentry_immediate_revive = (cmd >> 1) & 0x01;
            }
            break;
        case GET_FROM_GIMBAL_MAP_PATH_CAN_ID:
            // 接收云台转发的map_data分段(15帧/次, 105B payload)
            MapPath_OnCanReceive(rx_message->Data, (uint32_t)GetTime_ms());
            break;
        case GET_FROM_GIMBAL_CUSTOM_INFO_CAN_ID:
            // 接收云台转发的custom_info分段(5帧/次, 34B payload)
            CustomInfo_OnCanReceive(rx_message->Data, (uint32_t)GetTime_ms());
            break;
        default:
            break;
        }
    }

    if (can == SUPER_POWER_CAN)
    {
        if (rx_message->StdId == SUPER_POWER_CAN_ID)
        {
            memcpy(&cap_recv_data, rx_message->Data, 8);
            LossUpdate(&global_debugger.super_power_debugger, 0.0015f);
        }
    }
}
