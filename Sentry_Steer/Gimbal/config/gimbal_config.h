#ifndef _GIMBAL_CONFIG_H
#define _GIMBAL_CONFIG_H

#include "robot_config.h"

/*==============================================================================
 *                          云台测试模式控制宏
 *============================================================================*/
/* 测试配置选项 (用于预处理条件判断) */
#define GIMBAL_CONFIG_NORMAL          0       // 正常控制模式
#define GIMBAL_CONFIG_PITCH_SQUARE    1       // Pitch轴方波测试
#define GIMBAL_CONFIG_SMALLYAW_SQUARE 2       // 小Yaw轴方波测试

/* 测试配置选择:
 *   = 0: 正常控制模式 (跟随遥控器/上位机)
 *   = 1: Pitch轴方波测试 (±5°, 5秒周期) - 用于PID参数手动观察
 *   = 2: 小Yaw轴方波测试 (±5°, 5秒周期) - 用于PID参数手动观察
 *
 * 控制断开标志:
 *   = 1: 目标角度不再自动更新，用于debug手动设置（重力补偿标定）
 *   = 0: 正常模式，目标角度由遥控器/上位机更新
 *
 * 方波测试参数:
 *   GIMBAL_SQUARE_LOW_ANGLE:  方波低角度值 (度)
 *   GIMBAL_SQUARE_HIGH_ANGLE: 方波高角度值 (度)
 *   GIMBAL_SQUARE_PERIOD_MS:  方波周期 (ms)
 */

#define GIMBAL_TEST_CONFIG            0       // 0-正常控制, 1-Pitch方波测试, 2-小Yaw方波测试
#define GIMBAL_CONTROL_DISCONNECT     0
// 0-正常更新目标角度, 1-目标角度保持不变,ozone修改gimbal_controller.target_pitch_angle并监控 gimbal_controller.gyro_pitch_angle和gimbal_controller.set_pitch_current

#define DEBUG_ROBOT_CMD_SEND             // 开启: 0.5Hz向PC发送0x0303小地图指令(TypeID 9), 注释即屏蔽

#define GIMBAL_SQUARE_LOW_ANGLE     -5.0f   // 方波低角度 (度)
#define GIMBAL_SQUARE_HIGH_ANGLE    5.0f    // 方波高角度 (度)
#define GIMBAL_SQUARE_PERIOD_MS     5000    // 方波周期 (ms) - 5秒

/* 目标函数计算系数 */
#define GIMBAL_COST_LAMBDA          0.0001f // 控制量惩罚系数 λ

/*==============================================================================
 *                          遥控器协议配置
 *============================================================================*/
/* 遥控器协议选择宏定义: 1-使用WBUS协议, 0-使用DJI遥控器协议 */
#define USE_WBUS_PROTOCOL  1

/* WBUS协议帧长度: 25字节 */
#define WBUS_FRAME_LENGTH    25u
/* DJI遥控器协议帧长度: 18字节 */
#define DJI_FRAME_LENGTH     18u

#if USE_WBUS_PROTOCOL
#define REMOTE_FRAME_LENGTH  WBUS_FRAME_LENGTH
#else
#define REMOTE_FRAME_LENGTH  DJI_FRAME_LENGTH
#endif



#if ROBOT == GOBLIN
// pitch
#define GIMBAL_PITCH_GYRO_SIGN 1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN 1.0f // 云台PITCH电机方向，向上为正

#define GIMBAL_ANGLE_MIN 46.2f // 电机角软限位
#define GIMBAL_ANGLE_MAX 95.0f

#define GIMBAL_PITCH_COMP 4000.0f        // 暂不使用
#define GIMBAL_PITCH_COMP_COEF 1.0f      // 暂不使用

// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
// [SMALL_YAW_REMOVED] 小Yaw轴电机宏定义已删除
// #define GIMBAL_SMALL_YAW_MOTOR_SIGN 1.0f
// #define GIMBAL_SMALL_YAW_GYRO_SIGN 1.0f
// #define GIMBAL_SMALL_YAW_POS_FORWARD_COEF 0.6f
// #define GIMBAL_SMALL_YAW_SPEED_FORWARD_COEF 0.f
// #define GIMBAL_SMALL_YAW_LIMIT_LEFG 210.0f
// #define GIMBAL_SMALL_YAW_LIMIT_RIGHT 150.0f
// #define GIMBAL_SMALL_YAW_ZERO_POINT 180.0f


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

#elif ROBOT == TIGER

// pitch
#define GIMBAL_PITCH_GYRO_SIGN 1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN 1.0f // 云台PITCH电机方向，向上为正

#define GIMBAL_ANGLE_MIN 275.0f // 电机角软限位
#define GIMBAL_ANGLE_MAX 330.0f

// Pitch PID参数
#define PITCH_ANGLE_KP      23.0f
#define PITCH_ANGLE_KI       0.0f
#define PITCH_ANGLE_KD       0.0f
#define PITCH_ANGLE_MAXOUT   100.0f
#define PITCH_ANGLE_ILIMIT   5.0f

#define PITCH_SPEED_KP       30.0f
#define PITCH_SPEED_KI        13.0f
#define PITCH_SPEED_KD        0.0f
#define PITCH_SPEED_MAXOUT    700.0f
#define PITCH_SPEED_ILIMIT    0.0f

// DM电机内环阻尼 (Kp=0: 纯MIT力矩控制)
#define PITCH_DM_KP          0.0f
#define PITCH_DM_KD          5.0f

// 大Yaw零点: 云台朝正前方时DM_Big_Yaw_Motor.P_Receive的值
#define GIMBAL_BIG_YAW_ZERO_POINT  142.0f

#define GIMBAL_PITCH_COMP 4000.0f        // 暂不使用
#define GIMBAL_PITCH_COMP_COEF 1.0f      // 暂不使用

// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
// [SMALL_YAW_REMOVED] 小Yaw轴电机宏定义已删除
// #define GIMBAL_SMALL_YAW_MOTOR_SIGN 1.0f
// #define GIMBAL_SMALL_YAW_GYRO_SIGN 1.0f
// #define GIMBAL_SMALL_YAW_POS_FORWARD_COEF 0.6f
// #define GIMBAL_SMALL_YAW_SPEED_FORWARD_COEF 0.f
// #define GIMBAL_SMALL_YAW_LIMIT_LEFG 165.0f
// #define GIMBAL_SMALL_YAW_LIMIT_RIGHT 75.0f
// #define GIMBAL_SMALL_YAW_ZERO_POINT 120.0f


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