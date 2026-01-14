#include "can_receive.h"

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
//        case GIMBAL_COMM_CAN_ID_2:
//            memcpy(&gimbal_receiver_pack2, rx_message->Data, 8);
//						Gimbal_msgs_Decode2();
//            LossUpdate(&global_debugger.gimbal_comm_debugger[1], 0.0015f);
//            break;
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
