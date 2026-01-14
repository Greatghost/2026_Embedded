#include "omni.h"

#include "ChasisController.h"

void omni_pid_init()
{
    // 转向PID初始化
    PID_Init(&infantry.turn_pid, 70, 0, 0, 20.0f, 0, 2.0f, 0, 0, 0.001, 0.002, 1, DerivativeFilter);

    // 轮子控制PID
    PID_Init(&infantry.wheels_pid[LEFT_UP_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT/4, 0, 12.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[RIGHT_UP_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT/4, 0, 12.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[LEFT_DOWN_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT/4, 0, 12.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[RIGHT_DOWN_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT/4, 0, 12.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
}

// 正运动学
void omni_pos_kinematics()
{
    // 计算正运动学，并转到m/s为单位
    float speed1 = infantry.sensors_info.wheels_decode[LEFT_UP_OMNI_WHEEL].speed;
    float speed2 = infantry.sensors_info.wheels_decode[RIGHT_UP_OMNI_WHEEL].speed;
    float speed3 = infantry.sensors_info.wheels_decode[LEFT_DOWN_OMNI_WHEEL].speed;
    float speed4 = infantry.sensors_info.wheels_decode[RIGHT_DOWN_OMNI_WHEEL].speed;

    // 以向前(y)，向右(x)，逆时针(yaw)
    infantry.y_v = (-speed1 + speed2 - speed3 + speed4) / 4 * OMNI_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    infantry.x_v = (-speed1 - speed2 + speed3 + speed4) / 4 * OMNI_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    infantry.yaw_v = (speed1 + speed2 + speed3 + speed4) / 4 / OMNI_RADIUS * OMNI_DEGEREE_S_TO_MS; // rad/s
}

// 逆运动学 X型
void omni_inv_kinematics()
{
    // 逆运动学解算,统一到度/s的单位
    infantry.wheels_set_v[LEFT_UP_OMNI_WHEEL] = (infantry.set_x_v * SQRT_2 - infantry.set_y_v * SQRT_2 + infantry.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    infantry.wheels_set_v[RIGHT_UP_OMNI_WHEEL] = (-infantry.set_x_v * SQRT_2 + infantry.set_y_v * SQRT_2 + infantry.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    infantry.wheels_set_v[LEFT_DOWN_OMNI_WHEEL] = (infantry.set_x_v * SQRT_2 + infantry.set_y_v * SQRT_2 + infantry.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    infantry.wheels_set_v[RIGHT_DOWN_OMNI_WHEEL] = (-infantry.set_x_v * SQRT_2 - infantry.set_y_v * SQRT_2 + infantry.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
}

// 底盘跟随策略
void omni_follow_control()
{
    // 正运动学解算
    omni_pos_kinematics();

    // 速度分解
    infantry.set_x_v = infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir;
    infantry.set_y_v = infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir;

    // 转向计算，角度环
    infantry.target_pid_yaw_v = GIMBAL_MOTOR_SIGN * PID_Calculate(&infantry.turn_pid, infantry.error_angle, 0);
    infantry.set_yaw_v = infantry.target_pid_yaw_v;
	
    // 逆运动学解算
    omni_inv_kinematics();

    // 额外补偿
    // infantry.wheels_set_v[RIGHT_DOWN_OMNI_WHEEL] += omin_comp();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
    }

    // 功率控制
}

// 仅底盘运动策略，底盘将按照规定正方向运动，通常用于测试或者小陀螺状态
void omni_chassis_control()
{
    // 正运动学解算
    omni_pos_kinematics();

    infantry.set_x_v = infantry.target_x_v;
    infantry.set_y_v = infantry.target_y_v;
    infantry.set_yaw_v = infantry.target_yaw_v;

    // 逆运动学解算
    omni_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
    }

    // 功率控制
}

// 底盘小陀螺模式
void omni_rotate_control()
{
    // 正运动学解算
    omni_pos_kinematics();

    infantry.set_x_v = infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir;
    infantry.set_y_v = infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir;
    infantry.set_yaw_v = infantry.target_yaw_v;

    // 逆运动学解算
    omni_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
    }

    // 功率控制
}

void omni_speed_follow_control()
{
     // 正运动学解算
     omni_pos_kinematics();

     // 速度分解
     infantry.set_x_v = infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir;
     infantry.set_y_v = infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir;
 
     // 转向计算，角度环
     infantry.target_pid_yaw_v = GIMBAL_MOTOR_SIGN * PID_Calculate(&infantry.turn_pid, infantry.error_angle, 0);
     infantry.set_yaw_v = infantry.target_pid_yaw_v;
     
     // 逆运动学解算
     omni_inv_kinematics();
 
     // 额外补偿
     // infantry.wheels_set_v[RIGHT_DOWN_OMNI_WHEEL] += omin_comp();
 
     // PID 计算
     for (int i = 0; i < 4; i++)
     {
         infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
     }
 
}
