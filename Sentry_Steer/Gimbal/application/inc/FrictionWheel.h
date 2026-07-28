#ifndef _FRICTION_WHEEL_H
#define _FRICTION_WHEEL_H

#include "pid.h"
#include "M2006.h" 
#include "M3508.h"

#include "robot_config.h"

#define LEFT_FRICTION_WHEEL 0
#define RIGHT_FRICTION_WHEEL 1

#define BULLET_17MM_15MS_SPEED_L 24500
#define BULLET_17MM_18MS_SPEED_L 27300
// //#define BULLET_17MM_23MS_SPEED_L 32500
//#define BULLET_17MM_23MS_SPEED_L 35200
//#define BULLET_17MM_23MS_SPEED_L 35600
#define BULLET_17MM_23MS_SPEED_L 35000
#define BULLET_17MM_30MS_SPEED_L 43000
#define BULLET_17MM_15MS_SPEED_R BULLET_17MM_15MS_SPEED_L
#define BULLET_17MM_18MS_SPEED_R BULLET_17MM_18MS_SPEED_L
#define BULLET_17MM_23MS_SPEED_R BULLET_17MM_23MS_SPEED_L
#define BULLET_17MM_30MS_SPEED_R BULLET_17MM_30MS_SPEED_L

/* UNSTOPPABLE 模式下摩擦轮目标转速补偿量
 * 高弹频连续打弹时, 摩擦轮受子弹持续摩擦导致实际转速下沉,
 * 提升目标设定值以让 PID 维持在实际弹速要求的转速附近, 避免弹速下降。
 * 单位与 set_speed_l/r 一致 (deg/s, 带符号由调用处处理)。 */
#define UNSTOPPABLE_FRICTION_BOOST 2000.0f

/*  摩擦轮结构体  */
typedef struct FrictionWheel_t
{
    PID_t PidFrictionSpeed[2];
    M3508_Recv friction_motor_recv[2];
    M3508_Info friction_motor_msgs[2];
    float send_to_motor_current[2];

    float set_speed_l;
    float set_speed_r;
} FrictionWheel_t;

extern FrictionWheel_t friction_wheels;

void FrictionWheel_Init(void);
void FrictionWheel_Set(float speed1, float speed2); // 度/s
void setFrictionSpeed(int8_t shoot_level);

#endif // !_FRICTION_WHEEL_H
