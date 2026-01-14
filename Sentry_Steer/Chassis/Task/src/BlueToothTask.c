#include "BlueToothTask.h"

BlueToothSendData data;

/**
 * @brief 蓝牙发送debug任务
 * @param[in] void
 */

void BlueToothTask(void *pvParameters)
{
  portTickType xLastWakeTime;

  vTaskDelay(2000);

  while (1)
  {
    xLastWakeTime = xTaskGetTickCount();

    // 跟踪微分器测试
    // data.fdata[0] = infantry.target_x_v;
    // data.fdata[1] = infantry.target_x_v_percent * 3.0f;

    // 电容测试
    // data.fdata[0] = cap_recv_data.cap_vol / 100.0f;
    // data.fdata[1] = cap_recv_data.chassis_power / 100.0f;
    // data.fdata[2] = cap_recv_data.referee_power / 100.0f;
    // data.fdata[3] = cap_recv_data.V_buckin / 100.0f;
    // data.fdata[4] = referee_data.Power_Heat_Data.buffer_energy;
    // data.fdata[5] = cap_controller.set_power;
    // data.fdata[4] = infantry.x_v;
    // data.fdata[5] = infantry.set_x_v;

    // data.fdata[4] = infantry.power_limiter.predict_send_power;
    // data.fdata[5] = 1;

    //功率拟合
      data.fdata[0] = cap_recv_data.chassis_power / 100.0f;
//      data.fdata[1] = infantry.power_limiter.wheels_scaler.sum_i_2;
//      data.fdata[2] = infantry.power_limiter.wheels_scaler.sum_w_i;
//      data.fdata[3] = infantry.power_limiter.wheels_scaler.sum_w_2;
//      data.fdata[4] = infantry.power_limiter.Steer_scaler.sum_i_2;
//      data.fdata[5] = infantry.power_limiter.Steer_scaler.sum_w_i;
//      data.fdata[6] = infantry.power_limiter.Steer_scaler.sum_w_2;
			data.fdata[1] = infantry.power_limiter.Steer_scaler.predict_send_power+infantry.power_limiter.wheels_scaler.predict_send_power;
		//	data.fdata[2] = referee_data.Power_Heat_Data.buffer_energy;
//			data.fdata[0] = cap_recv_data.chassis_power / 100.0f ;
//			data.fdata[1] = infantry.power_limiter.Steer_scaler.predict_send_power+infantry.power_limiter.wheels_scaler.predict_send_power;
//			data.fdata[2] = referee_data.Power_Heat_Data.buffer_energy;
//			data.fdata[3] = cap_controller.cap_vol;
//			data.fdata[4] = infantry.error_angle;
//			data.fdata[5] = infantry.robot_vector.angle;
//			data.fdata[6] = infantry.target_yaw_v;
//      data.fdata[1] = infantry.power_limiter.Steer_scaler.sum_i_2;
//      data.fdata[2] = infantry.power_limiter.Steer_scaler.sum_w_i;
//      data.fdata[3] = infantry.power_limiter.Steer_scaler.sum_w_2;

    // 功率测试
//    data.fdata[0] = (infantry.sensors_info.wheels_decode[0].speed + infantry.wheels_send_current[0] / 10.0f) * STEER_DEGEREE_S_TO_MS;
//    data.fdata[1] = (infantry.sensors_info.wheels_decode[1].speed + infantry.wheels_send_current[1] / 10.0f) * STEER_DEGEREE_S_TO_MS;
//    data.fdata[2] = (infantry.sensors_info.wheels_decode[2].speed + infantry.wheels_send_current[2] / 10.0f) * STEER_DEGEREE_S_TO_MS;
//    data.fdata[3] = (infantry.sensors_info.wheels_decode[3].speed + infantry.wheels_send_current[3] / 10.0f) * STEER_DEGEREE_S_TO_MS;
//    // data.fdata[4] = infantry.sensors_info.wheels_decode[0].torque_current;
//    // data.fdata[4] = infantry.sensors_info.wheels_decode[1].torque_current;
//    data.fdata[4] = cap_recv_data.chassis_power / 100.0f;
//    // data.fdata[5] = cap_recv_data.referee_power / 100.0f;

    //    data.fdata[0] = infantry.power_limiter.predict_send_power;
    //    data.fdata[1] = referee_data.Game_Robot_State.remain_HP;
    //    data.fdata[2] = cap_recv_data.referee_power / 100.0f;
    //    data.fdata[3] = cap_recv_data.chassis_power / 100.0f;
    //    data.fdata[4] = infantry.set_power;
    //    data.fdata[5] = referee_data.Power_Heat_Data.buffer_energy;

    // data.fdata[0] = heat_controller.CurHeat;
    // data.fdata[1] = heat_controller.HeatMax;
    // data.fdata[2] = heat_controller.shoot_flag;
    // data.fdata[3] = cap_recv_data.chassis_power / 100.0f;
    // data.fdata[4] = infantry.set_power;
    // data.fdata[5] = referee_data.Power_Heat_Data.buffer_energy;

    // 速度测试

    BLUE_TOOTHSendData(&data);

    /*  延时  */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
  }
}
