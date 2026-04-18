#ifndef _STEER_H
#define _STEER_H

#include "math.h"
#include "tools.h"


/*舵编号*/
#define STEER1 0
#define STEER2 1
#define STEER3 2
#define STEER4 3

#define STEER_INFANTRY_RADIUS 0.2178f  // 底盘中心到6020中心半径，用于计算小陀螺线速度
#define SPIN_ANGLE 45.0f // 转动时舵角度相对机体为45°

#define DEG2R_RATIO 0.017453292519f
#define R2DEG_RATIO 57.295779513f

#define STEER_WHEEL_RADIUS 0.057f
#define STEER_SPEED_TO_DEGEREE_S (180.0f / STEER_WHEEL_RADIUS / PI) // 从米/s转到度/s转化
#define STEER_DEGEREE_S_TO_MS (PI * STEER_WHEEL_RADIUS / 180.0f)    // 由度/s转到m/s

/* 舵轮角度调试参数（调参时修改） */
#define STEER_DEBUG_TARGET_STEER  STEER1      // 调试的目标舵轮编号
#define STEER_DEBUG_SQUARE_PERIOD 2000        // 正弦波周期(ms)，完整振荡一次的时间
#define STEER_DEBUG_ANGLE_BASE    0.0f        // 基准角度(度)
#define STEER_DEBUG_ANGLE_AMPLITUDE 45.0f     // 正弦波幅度(度)，±45°振荡

void steer_pid_init(void);
void steer_inv_kinematics(void);
void steer_chassis_control(void);
void steer_pos_kinematics(void);  // 舵轮正运动学：从舵角度和轮速度反解底盘速度
void steer_angle_debug(void);     // 舵轮角度调试函数：正弦波振荡调参

#endif
