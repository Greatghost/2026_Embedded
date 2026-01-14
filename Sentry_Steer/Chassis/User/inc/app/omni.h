#ifndef _OMNI_H
#define _OMNI_H

#include "accel.h"
#include "math.h"
#include "tools.h"

//  全向轮组件

/*  设置CAN ID时 请严格按照如图来设置 ，否则的话，需要更改以下序号(按下图配置则LEFT_UP_OMNI_WHEEL等分别为0，1，2，3) */
/*
||0x201  ||0x202
                -->  右
||0x203  ||0x204

*/

// 以下序号0表示0x201电机
#define LEFT_UP_OMNI_WHEEL 0    // 左上电机
#define RIGHT_UP_OMNI_WHEEL 1   // 右上电机
#define LEFT_DOWN_OMNI_WHEEL 3  // 左下电机
#define RIGHT_DOWN_OMNI_WHEEL 2 // 右下电机

#define OMNI_WHEEL_RADIUS 0.08f
#define OMNI_SPEED_TO_DEGEREE_S (180.0f / OMNI_WHEEL_RADIUS / PI) // 从米/s转到度/s转化
#define OMNI_DEGEREE_S_TO_MS (PI * OMNI_WHEEL_RADIUS / 180.0f)    // 由度/s转到m/s
#define OMNI_RADIUS 0.215f

// 正运动学
void omni_pos_kinematics(void);

// 逆运动学
void omni_inv_kinematics(void);
// 底盘跟随策略
void omni_follow_control(void);

// 仅底盘运动模式
void omni_chassis_control(void);

// 底盘旋转模式
void omni_rotate_control(void);

// 速度矢量跟随模式
void omni_speed_follow_control(void);

void omni_pid_init(void);

#endif
