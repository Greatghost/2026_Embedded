#ifndef _ROBOT_CONFIG_H
#define _ROBOT_CONFIG_H

/*  机器人宏定义列举 */
#define NIU_MO_SON 0       // 舵轮
#define CHEN_JING_YUAN 1   // 麦轮
#define NIUNIU 2           // 2024.6.16第一辆全向轮
#define QI_TIAN_DA_SHENG 3 // 继承24国赛新全向轮部分参数
#define TIGER 4 					 // 复活赛舵轮哨兵

#define ROBOT TIGER 

typedef enum CHASSIS_TYPE
{
    STEER_WHEEL,   // 舵轮
    MECANUM_WHEEL, // 麦克纳姆轮
    OMNI_WHEEL,    // 全向轮
} CHASSIS_TYPE;

// 麦轮参数
#define MECANUM_WIDTH 0.15f  // 麦轮宽
#define MECANUM_LENGTH 0.20f // 麦轮长

// yaw轴电机类型
typedef enum YAW_MOTOR_TYPE
{
    YAW_GM6020,
    YAW_DM_MOTOR
} YAW_MOTOR_TYPE;


#if ROBOT == QI_TIAN_DA_SHENG
#define GIMBAL_MOTOR_SIGN 1     // 云台电机方向，以逆时针为正
#define GIMBAL_FOLLOW_ZERO 19184 // 底盘跟随机械零点
#elif ROBOT == TIGER
#define GIMBAL_MOTOR_SIGN 1     // 云台电机方向，以逆时针为正
#define GIMBAL_FOLLOW_ZERO 3455 // 底盘跟随机械零点

#define CHASSIS_DEBUG 0

#endif
void setRobotType(void);

#endif
