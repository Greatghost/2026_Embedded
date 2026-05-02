#include "ChassisSend.h"

ChassisSendPack1 chassis_send_pack1;
//ChassisSendPack2 chassis_send_pack2;

//void Pack_InfantryMode()
//{
//  chassis_send_pack1.robot_state = remote_controller.robot_state;
//  chassis_send_pack1.control_type = remote_controller.control_type;
//  chassis_send_pack1.control_mode_action = remote_controller.control_mode_action;
//  chassis_send_pack1.gimbal_mode = remote_controller.gimbal_action;
//  chassis_send_pack1.shoot_mode = remote_controller.shoot_action;
//  chassis_send_pack1.is_pc_on = offline_detector.pc_state == PC_ON;
//  chassis_send_pack1.autoaim_id = pc_recv_data.enemy_id;
//  chassis_send_pack1.chassis_format = remote_controller.chassis_format;

//  if (bomb_bay_state == BOMB_BAY_COVER_OFF || bomb_bay_state == BOMB_BAY_COVER_NOT_INIT) // 1Bit,????????δ????????0???????1
//    bomb_bay_state = 0;
//  chassis_send_pack1.cover_state = bomb_bay_state;

//  chassis_send_pack1.robot_speed_x = (int8_t)(chassis_solver.chassis_speed_x * 100.0f);
//  chassis_send_pack1.robot_speed_y = (int8_t)(chassis_solver.chassis_speed_y * 100.0f);
//  chassis_send_pack1.robot_speed_w = (int8_t)(chassis_solver.chassis_speed_w * 100.0f);
//}

//void Pack_Yaw()
//{
//  if (motor_communication[YAW_MOTOR].motor_type == DM_MOTOR)
//    chassis_send_pack2.yaw_motor_angle = (int16_t)(gimbal_controller.DM_Yaw_Motor.P_Receive * 90);
//  else if (motor_communication[YAW_MOTOR].motor_type == GM6020)
//    chassis_send_pack2.yaw_motor_angle = gimbal_controller.yaw_recv.angle;
//  chassis_send_pack2.gimbal_pitch = (int16_t)(gimbal_controller.gyro_pitch_angle * 100.0f);
//  chassis_send_pack2.gimbal_yaw_speed = (int16_t)(gimbal_controller.gyro_yaw_speed * 100.0f);
//  chassis_send_pack2.super_power = remote_controller.super_power_state;
//  chassis_send_pack2.fly_state = remote_controller.fly_state;
//}

void Pack_InfantryMode()
{
  chassis_send_pack1.robot_state = remote_controller.robot_state;
  chassis_send_pack1.control_type = remote_controller.control_type;
  chassis_send_pack1.control_mode_action = remote_controller.control_mode_action;
  chassis_send_pack1.gimbal_mode = remote_controller.gimbal_action;
  chassis_send_pack1.shoot_mode = remote_controller.shoot_action;
	chassis_send_pack1.chassis_format = remote_controller.chassis_format;
	chassis_send_pack1.is_pc_on = offline_detector.pc_state == PC_ON;
	chassis_send_pack1.super_power = remote_controller.super_power_state;
	chassis_send_pack1.fly_state = remote_controller.fly_state;
  chassis_send_pack1.sentry_posture = current_posture;  // 哨兵姿态发送给底盘板
  // through_hole_flag 在 ChassisSolver.c 中设置

  if (motor_communication[BIG_YAW_MOTOR].motor_type == DM_MOTOR)
    chassis_send_pack1.yaw_motor_angle = (int16_t)(gimbal_controller.DM_Big_Yaw_Motor.P_Receive * 90);
  else if (motor_communication[BIG_YAW_MOTOR].motor_type == GM6020)
    chassis_send_pack1.yaw_motor_angle = gimbal_controller.big_yaw_recv.angle;

  chassis_send_pack1.robot_speed_x = (int8_t)(chassis_solver.chassis_speed_x * 30.0f);
  chassis_send_pack1.robot_speed_y = (int8_t)(chassis_solver.chassis_speed_y * 30.0f);
  chassis_send_pack1.robot_speed_w = (int8_t)(chassis_solver.chassis_speed_w * 7.0f);
}
