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

/* =============================================================================
 *  整车扭矩前馈开关（整车速度外环 PID + 动力学扭矩分配）
 * =============================================================================
 *  启用方式：保持下面 #define STEER_TORQUE_FEEDFORWARD 未注释（默认启用）
 *  关闭方式：注释掉 #define STEER_TORQUE_FEEDFORWARD
 *
 *  架构（三层）：
 *    上层（运动学层，已有）：
 *      - 逆解 steer_chassis_control()：target_x_v/y_v/yaw_v → 各舵轮舵向角 + 轮向角速度
 *      - 正解 steer_pos_kinematics()：各舵角 + 轮速 → x_v/y_v/yaw_v（反馈信号）
 *
 *    中层（整车速度闭环，新增）：
 *      - 平动外环PID：target_x_v ↔ x_v，target_y_v ↔ y_v → 输出底盘目标牵引力(N)
 *      - 旋转外环PID：target_yaw_v ↔ yaw_v → 输出底盘目标转矩(N·m)
 *
 *    下层（动力学分配层，新增）：
 *      - 整车牵引力 + 整车转矩 + 当前舵角 → 动力学逆解分配每轮扭矩
 *      - 叠加滚动阻力前馈
 *      - 转换为 C620 电流值，叠加到 wheels_set_current[i]
 *
 *  数据流：
 *    target_v → [PID] → F/T → [坐标变换] → [动力学分配] → [扭矩→电流] → wheels_set_current +=
 *                                        ↑
 *                            x_v/y_v/yaw_v（正运动学反馈）
 *
 *  注意：本前馈在现有轮电机速度PID输出上"叠加"，不替换速度PID。
 *        轮电机速度PID仍负责快速跟踪轮速设定，整车外环PID负责修正滑移/外扰导致的整车速度误差。
 * ========================================================================== */
//+#define STEER_TORQUE_FEEDFORWARD

#ifdef STEER_TORQUE_FEEDFORWARD
/* --- 动力学参数（可调，需根据实际整车标定） --- */
#define STEER_FF_CHASSIS_MASS          25.0f   /* 整车质量 kg（含装甲/弹仓，需实测）        */
#define STEER_FF_CHASSIS_INERTIA       0.8f    /* 整车转动惯量 kg·m²（绕z轴，需实测）       */
#define STEER_FF_ROLL_RESISTANCE_COEF 0.05f   /* 滚动阻力系数 mu（地毯+橡胶轮典型值）      */
#define STEER_FF_GRAVITY               9.8f   /* 重力加速度 m/s²                            */

/* --- 力/扭矩 → C620电流值 标定系数（可调） ---
 * 推导：C620 电流命令范围 ±16384 对应 ±20A
 *       M3508 输出轴扭矩常数 K_t ≈ 0.063 N·m/A（含减速比 19.2）
 *       1 N·m 输出轴扭矩 = 1/0.063 A ≈ 15.9 A ≈ 13000 命令单位
 * 实际取保守值 3000，调试时可增大（前馈太弱则增大，太强则减小） */
#define STEER_FF_TORQUE_TO_C620        3000.0f  /* N·m → C620电流命令单位 */

/* --- 限幅（保守值） --- */
#define STEER_FF_MAX_FORCE              80.0f    /* 最大平动牵引力 N（约3.2 m/s² 加速度）     */
#define STEER_FF_MAX_TORQUE             5.0f     /* 最大旋转转矩 N·m（约6.25 rad/s² 角加速度）*/
#define STEER_FF_MAX_CURRENT_PER_WHEEL  3000.0f  /* 单轮最大前馈电流（约C620_MAX的19%）      */
#endif /* STEER_TORQUE_FEEDFORWARD */

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
