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

void steer_pid_init(void);
void steer_inv_kinematics(void);
void steer_chassis_control(void);

#endif
