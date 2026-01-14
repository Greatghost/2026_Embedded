#ifndef _GIMBAL_CONFIG_H
#define _GIMBAL_CONFIG_H

#include "robot_config.h"



#if ROBOT == GOBLIN
// pitch
#define GIMBAL_PITCH_GYRO_SIGN 1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN 1.0f // 云台PITCH电机方向，向上为正

#define GIMBAL_ANGLE_MIN 50.5f // 电机角软限位
#define GIMBAL_ANGLE_MAX 85.0f

#define GIMBAL_PITCH_COMP 4000.0f        // 暂不使用
#define GIMBAL_PITCH_COMP_COEF 1.0f      // 暂不使用

// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_SMALL_YAW_MOTOR_SIGN 1.0f       // 用来标记电机的方向，逆时针为正，达妙电机不是逆时针为正？？
#define GIMBAL_SMALL_YAW_GYRO_SIGN 1.0f        // 用来标记gyro的方向，逆时针为正
#define GIMBAL_SMALL_YAW_POS_FORWARD_COEF 0.6f // 角度环前馈系数
#define GIMBAL_SMALL_YAW_SPEED_FORWARD_COEF 0.f
#define GIMBAL_SMALL_YAW_LIMIT_LEFG 7764        //小yaw电机角左限位
#define GIMBAL_SMALL_YAW_LIMIT_RIGHT 5879       //小yaw电机角右限位
#define GIMBAL_SMALL_YAW_ZERO_POINT 6821        //小yaw电机零点


#define GIMBAL_BIG_YAW_MOTOR_SIGN 1.0f       // 用来标记电机的方向，逆时针为正，达妙电机不是逆时针为正？？
#define GIMBAL_BIG_YAW_GYRO_SIGN 1.0f        // 用来标记gyro的方向，逆时针为正
#define GIMBAL_BIG_YAW_POS_FORWARD_COEF 0.6f // 角度环前馈系数
#define GIMBAL_BIG_YAW_SPEED_FORWARD_COEF 0.f

// #define GIMBAL_YAW_J 4.15f
// #define GIMBAL_YAW_B 18.54f

// 摩擦力模型调参
// #define BORDER_FRICTION_SPEED 6.0f    // 临界计算摩擦力速度，大于此速度将是全摩擦力补偿
// #define FRICTION_CURRENT_COMP 1500.0f // 辨识所得到的摩擦力电流发送值
// #define FRICTION_FORWARD_COEF 0.0f    // 前馈补偿系数


#endif

#endif