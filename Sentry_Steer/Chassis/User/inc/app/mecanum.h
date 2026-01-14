#ifndef _MECANUM_H
#define _MECANUM_H

#include "accel.h"
#include "math.h"
#include "tools.h"

// 麦克纳姆轮组件

/*  设置CAN ID时 请严格按照如图来设置 ，否则的话，需要更改以下序号 (按下图配置则LEFT_UP_OMNI_WHEEL等分别为0，1，2，3)*/
/*
||0x201  ||0x202
                -->  右
||0x203  ||0x204

*/

// 以下序号0表示0x201电机
#define LEFT_UP_MECANUM_WHEEL 0    // 左上电机
#define RIGHT_UP_MECANUM_WHEEL 1   // 右上电机
#define LEFT_DOWN_MECANUM_WHEEL 2  // 左下电机
#define RIGHT_DOWN_MECANUM_WHEEL 3 // 右下电机

#define MECANUM_WHEEL_RADIUS 0.06f
#define MECANUM_SPEED_TO_DEGEREE_S (180.0f / MECANUM_WHEEL_RADIUS / PI) // 从米/s转到度/s转化
#define MECANUM_DEGEREE_S_TO_MS (PI * MECANUM_WHEEL_RADIUS / 180.0f)    // 由度/s转到m/s

// 正运动学
void mecanum_pos_kinematics(void);

// 逆运动学
void mecanum_inv_kinematics(void);
// 底盘跟随策略
void mecanum_follow_control(void);

// 仅底盘运动模式或底盘旋转模式
void mecanum_chassis_control(void);
void mecanum_rotate_control(void);
void mecanum_pid_init(void);

#endif
