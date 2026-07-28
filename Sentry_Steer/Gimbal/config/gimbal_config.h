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

/*
 * 大Yaw模型前馈（控制量单位为DM驱动的mN*m编码单位）:
 *   u_ff = gain * (J * alpha_ref + B * omega_ref
 *                   + C * sat(omega_ref / friction_blend_dps))
 * 这里的初值只等价替代旧的经验速度/加速度前馈，C保持关闭；换车后必须实测标定。
 */
#define GIMBAL_BIG_YAW_MODEL_FF_ENABLE             1
#define GIMBAL_BIG_YAW_MODEL_FF_J                  0.4f
#define GIMBAL_BIG_YAW_MODEL_FF_B                  1.0f
#define GIMBAL_BIG_YAW_MODEL_FF_C                  0.0f
#define GIMBAL_BIG_YAW_MODEL_FF_FRICTION_BLEND_DPS 6.0f
#define GIMBAL_BIG_YAW_MODEL_FF_GAIN               1.0f

#elif ROBOT == TIGER

// pitch
#define GIMBAL_PITCH_GYRO_SIGN 1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN 1.0f // 云台PITCH电机方向，向上为正

#define GIMBAL_ANGLE_MIN 266.0f // 电机角硬限位
#define GIMBAL_ANGLE_MIN_SOFT 271.0f // 缓冲限位: 271°~266°渐进收紧防振荡
#define GIMBAL_ANGLE_MAX 328.0f

// 旧串联角度前馈（并联PID下不参与控制，由pos_pitch_td.dx直接提供速度参考）
#define PITCH_ANGLE_FF_VEL     0.0f
#define PITCH_ANGLE_FF_ACC     0.05f
#define PITCH_ANGLE_FF_JERK    0.0f
#define PITCH_ANGLE_FF_MAXOUT  10.0f

// 旧通用速度前馈已由下方辨识得到的Pitch J/B/C物理模型替代。
#define PITCH_SPEED_FF_VEL     0.0f
#define PITCH_SPEED_FF_ACC     0.0f
#define PITCH_SPEED_FF_JERK    0.0f
#define PITCH_SPEED_FF_MAXOUT  0.0f

// DM电机内环阻尼 (Kp=0: 纯MIT力矩控制)
#define PITCH_DM_KP          0.0f
#define PITCH_DM_KD          5.0f

// 使用原车Pitch多项式重力补偿
#define PITCH_GRAVITY_COMP_ENABLE

// 大Yaw零点: 云台朝正前方时DM_Big_Yaw_Motor.P_Receive的值
#define GIMBAL_BIG_YAW_ZERO_POINT  124.0f

// 大Yaw电机编码器方向符号:
// 换新电机后若编码器计数方向与旧电机相反(同一物理转角旧为+新为-),
// 会使发给底盘的 yaw_motor_angle 符号翻转, 底盘跟随模式下速度矢量被转反(左右镜像)。
// 将该宏置 -1.0f 即可只翻转"发给底盘的yaw角", 不影响云台自身的IMU反馈+力矩控制闭环。
// 若改后出现底盘走位反而更错(说明诊断方向有误), 直接置回 1.0f 即可恢复。
#define GIMBAL_BIG_YAW_ENC_SIGN   1.0f

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

/*
 * 大Yaw模型前馈（控制量单位为DM驱动的mN*m编码单位）:
 *   u_ff = gain * (J * alpha_ref + B * omega_ref
 *                   + C * sat(omega_ref / friction_blend_dps))
 * J/B/C取本车三次Yaw自动辨识结果的平均值。
 */
#define GIMBAL_BIG_YAW_MODEL_FF_ENABLE             1
#define GIMBAL_BIG_YAW_MODEL_FF_J                  1.346f
#define GIMBAL_BIG_YAW_MODEL_FF_B                  0.0f
#define GIMBAL_BIG_YAW_MODEL_FF_C                  325.5f
#define GIMBAL_BIG_YAW_MODEL_FF_FRICTION_BLEND_DPS 6.0f
#define GIMBAL_BIG_YAW_MODEL_FF_GAIN               1.0f



#endif

/*
 * crossing_hole-main GimbalSystemID 模块使用的模型初值接口。
 * Yaw 直接复用已接入的大 Yaw J/B/C；Pitch 当前正常控制使用多项式重力补偿，
 * 因此首次必须从 STEP_ALL（GRAVITY -> BC -> J）开始，或先分步得到并回填这些值。
 */
#define GIMBAL_YAW_J       GIMBAL_BIG_YAW_MODEL_FF_J
#define GIMBAL_YAW_B       GIMBAL_BIG_YAW_MODEL_FF_B
#define GIMBAL_YAW_C       GIMBAL_BIG_YAW_MODEL_FF_C

#define GIMBAL_PITCH_SIN   1933.07568f
#define GIMBAL_PITCH_COS    336.884491f
#define GIMBAL_PITCH_B        1.11311018f
#define GIMBAL_PITCH_C      442.845795f
#define GIMBAL_PITCH_J     0.0f
#define GIMBAL_PITCH_FRICTION_BLEND_DPS 6.0f

#if (GIMBAL_BIG_YAW_MODEL_FF_ENABLE != 0) && \
    (GIMBAL_BIG_YAW_MODEL_FF_ENABLE != 1)
#error "GIMBAL_BIG_YAW_MODEL_FF_ENABLE must be 0 or 1"
#endif

#endif
