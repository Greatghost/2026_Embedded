#include "Gimbal.h"
#include "robot_config.h"
#include "gimbal_config.h"
#include "Offline_Task.h"

GimbalController gimbal_controller;
BigYawController big_yaw_controller;
// gimbal_controller.DM_Big_Yaw_Motor.P_Receive

static uint8_t pitch_feedback_was_ready = 0;
static uint8_t yaw_feedback_was_ready = 0;

static uint8_t Gimbal_IMU_FeedbackReady(void)
{
    return offline_detector.imu_state[0] == IMU_ON &&
           offline_detector.imu_state[1] == IMU_ON;
}

static uint8_t Gimbal_Pitch_FeedbackReady(void)
{
    return Gimbal_IMU_FeedbackReady() &&
           offline_detector.pitch_motor_state == PITCH_MOTOR_ON;
}

static uint8_t Gimbal_Yaw_FeedbackReady(void)
{
    return Gimbal_IMU_FeedbackReady() &&
           offline_detector.yaw_motor_state == YAW_MOTOR_ON;
}

static void Gimbal_Big_Yaw_ModelFeedforwardReset(void)
{
    gimbal_controller.big_yaw_ff_ref_speed_dps = 0.0f;
    gimbal_controller.big_yaw_ff_ref_accel_dps2 = 0.0f;
    gimbal_controller.big_yaw_ff_inertia = 0.0f;
    gimbal_controller.big_yaw_ff_viscous = 0.0f;
    gimbal_controller.big_yaw_ff_coulomb = 0.0f;
    gimbal_controller.big_yaw_ff_output = 0.0f;
}

/**
 * @brief crossing_hole yaw物理模型前馈
 *
 * 参考速度和加速度来自位置TD，而不是实际陀螺仪速度。这样摩擦项在零速
 * 目标附近不会随测量噪声换向，前馈也不会把反馈噪声重新注入力矩通道。
 * 返回值仍是模型正方向；GIMBAL_BIG_YAW_MOTOR_SIGN在总输出处统一处理。
 */
static float Gimbal_Big_Yaw_ModelFeedforward(float ref_speed_dps,
                                              float ref_accel_dps2)
{
    float friction_ratio = 0.0f;
    float output;

    gimbal_controller.big_yaw_ff_ref_speed_dps = ref_speed_dps;
    gimbal_controller.big_yaw_ff_ref_accel_dps2 = ref_accel_dps2;

#if GIMBAL_BIG_YAW_MODEL_FF_ENABLE
    if (GIMBAL_BIG_YAW_MODEL_FF_FRICTION_BLEND_DPS > 0.0f)
    {
        friction_ratio = LIMIT_MAX_MIN(
            ref_speed_dps / GIMBAL_BIG_YAW_MODEL_FF_FRICTION_BLEND_DPS,
            1.0f, -1.0f);
    }

    gimbal_controller.big_yaw_ff_inertia =
        GIMBAL_BIG_YAW_MODEL_FF_J * ref_accel_dps2;
    gimbal_controller.big_yaw_ff_viscous =
        GIMBAL_BIG_YAW_MODEL_FF_B * ref_speed_dps;
    gimbal_controller.big_yaw_ff_coulomb =
        GIMBAL_BIG_YAW_MODEL_FF_C * friction_ratio;

    output = GIMBAL_BIG_YAW_MODEL_FF_GAIN *
             (gimbal_controller.big_yaw_ff_inertia +
              gimbal_controller.big_yaw_ff_viscous +
              gimbal_controller.big_yaw_ff_coulomb);
#else
    gimbal_controller.big_yaw_ff_inertia = 0.0f;
    gimbal_controller.big_yaw_ff_viscous = 0.0f;
    gimbal_controller.big_yaw_ff_coulomb = 0.0f;
    output = 0.0f;
#endif

    gimbal_controller.big_yaw_ff_output = output;
    return output;
}

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit()
{

#if ROBOT == GOBLIN
    // pitch VOL LOOP
    PID_Init(&gimbal_controller.pitch_angle_pid, 150.0f, 48.0f, 0.0f, 52.0f, 0.5f, 0.05f, 0, 0, 0, 0.02f, 1, DerivativeFilter | Integral_Limit | Trapezoid_Intergral);
#if GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    /* 本车Pitch辨识专用PI速度环；积分补偿角度相关负载，微分保持关闭。 */
    PID_Init(&gimbal_controller.pitch_speed_pid,
             5000.0f, 1000.0f, 0.0f,
             40.0f, 20.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, 1, Integral_Limit);
#else
    PID_Init(&gimbal_controller.pitch_speed_pid, 3000, 1200, 0.1f, 58.0f, 6.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
#endif

    // [SMALL_YAW_REMOVED] 小Yaw PID/前馈初始化已删除
    // // yaw GM6020 CURRENT LOOP
    // PID_Init(&gimbal_controller.small_yaw_angle_pid, 180.0, 0, 0.05, 48.0f, 0, 0.3f, 0, 0, 0.0, 0.0f, 1, DerivativeFilter);
    // PID_Init(&gimbal_controller.small_yaw_speed_pid, GM6020_MAX_CURRENT, 1000, 0.5, 170.0f, 1.5f, 0, 0, 0, 0.f, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // yaw DM MOTOR CURRENT LOOP
    PID_Init(&gimbal_controller.big_yaw_angle_pid, 360.0, 0, 0.05, 32.0f, 0.f, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    /* 本车大惯量 Yaw 辨识速度环：纯 P，限制峰值转矩以避免速度阶跃激发抖动。 */
    PID_Init(&gimbal_controller.big_yaw_speed_pid, 3000.0f, 0.0f, 0.0f, 30.0f, 0.0f, 0, 0, 0, 0.0018f, 0, 1, Integral_Limit | Trapezoid_Intergral);
#else
    PID_Init(&gimbal_controller.big_yaw_speed_pid, 6000, 1200, 0.5, 33.0f, 1.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
#endif

    // Feedforward_Init( Feedforward_t,float max_out, float *c, float lpf_rc, uint16_t ref_dot_ols_order, uint16_t ref_ddot_ols_orde)

    // [SMALL_YAW_REMOVED] 小Yaw前馈参数和初始化已删除
    // float small_yaw_angle_ff_c[3] = {0.f, 0.4f, 0.0f};
    // float small_yaw_speed_ff_c[3] = {0.4f, 0.f, 0.0f};
    // Feedforward_Init(&gimbal_controller.small_yaw_angle_forward, 100.0f, small_yaw_angle_ff_c, 0.01f, 5, 5);
    // Feedforward_Init(&gimbal_controller.small_yaw_speed_forward, 500.0f, small_yaw_speed_ff_c, 0.01f, 5, 5);
    // 云台、机械臂的最小二乘法阶数经验值通常为3~5，LPF_rc在0.01~0.05

    float big_yaw_angle_ff_c[3] = {0.f, 0.15f, 0.0f}; // 大yaw前馈参数向量
    Feedforward_Init(&gimbal_controller.big_yaw_angle_forward, 100.0f, big_yaw_angle_ff_c, 0.01f, 5, 5);
    /* 结构保留用于现有无扰切换；力矩前馈已由J/B/C模型统一计算。 */
    float big_yaw_speed_ff_c[3] = {0.0f, 0.0f, 0.0f};
    Feedforward_Init(&gimbal_controller.big_yaw_speed_forward, 500.0f, big_yaw_speed_ff_c, 0.01f, 5, 5);

    // 滤波器，阶数，截止频率，采样频率,FIR有bug
    // FIRFilter_Init(&big_yaw_fir_filter, 10, 100, 500);

#elif ROBOT == TIGER

    /* TIGER并联PID保守初值：位置P输出和速度P输出直接在力矩端相加。 */
    PID_Init(&gimbal_controller.pitch_angle_pid,
             120.0f, 0.0f, 0.0f,
             60.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, 1, NONE);
#if GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    /* 本车Pitch辨识专用PI速度环；积分补偿角度相关负载，微分保持关闭。 */
    PID_Init(&gimbal_controller.pitch_speed_pid,
             5000.0f, 1000.0f, 0.0f,
             40.0f, 150.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, 1, Integral_Limit);
#else
    PID_Init(&gimbal_controller.pitch_speed_pid,
             16000.0f, 0.0f, 0.0f,
             15.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, 1, NONE);
#endif
    /*
     * 系统辨识必须保持“速度 PID -> t_ff”的单一控制链路。
     * 所选轴关闭 DM MIT 内部位置/速度反馈，避免 Kd 阻尼被辨识进 B/J。
     */
#if GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    gimbal_controller.DM_Pitch_Motor.Kp = 0;
    gimbal_controller.DM_Pitch_Motor.Kd = 0;
    gimbal_controller.DM_Pitch_Motor.V_des = 0.0f;
#else
    gimbal_controller.DM_Pitch_Motor.Kp = PITCH_DM_KP;
    gimbal_controller.DM_Pitch_Motor.Kd = PITCH_DM_KD;
#endif

#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    gimbal_controller.DM_Big_Yaw_Motor.Kp = 0;
    gimbal_controller.DM_Big_Yaw_Motor.Kd = 0;
    gimbal_controller.DM_Big_Yaw_Motor.V_des = 0.0f;
#else
    gimbal_controller.DM_Big_Yaw_Motor.Kp = 0;
    gimbal_controller.DM_Big_Yaw_Motor.Kd = 5;
#endif

    // Pitch角度前馈 + 速度前馈
    float pitch_angle_ff_c[3] = {PITCH_ANGLE_FF_VEL, PITCH_ANGLE_FF_ACC, PITCH_ANGLE_FF_JERK};
    Feedforward_Init(&gimbal_controller.pitch_angle_forward, PITCH_ANGLE_FF_MAXOUT, pitch_angle_ff_c, 0.01f, 5, 5);
    float pitch_speed_ff_c[3] = {PITCH_SPEED_FF_VEL, PITCH_SPEED_FF_ACC, PITCH_SPEED_FF_JERK};
    Feedforward_Init(&gimbal_controller.pitch_speed_forward, PITCH_SPEED_FF_MAXOUT, pitch_speed_ff_c, 0.01f, 5, 5);

    // [SMALL_YAW_REMOVED] 小Yaw PID初始化已删除
    // PID_Init(&gimbal_controller.small_yaw_angle_pid, 100.0, 0, 0.05, 15.0f, 0, 0.0f, 0, 0, 0.0, 0.0f, 1, DerivativeFilter);
    // PID_Init(&gimbal_controller.small_yaw_speed_pid, GM6020_MAX_CURRENT, 5000, 0.5, 120.0f, 80.0f, 0, 0, 0, 0.f, 0, 1, Integral_Limit | Trapezoid_Intergral);

    PID_Init(&gimbal_controller.big_yaw_angle_pid,
             180.0f, 0.0f, 0.0f,
             15.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, 1, NONE);
#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    /* 本车大惯量 Yaw 辨识速度环：纯 P，限制峰值转矩以避免速度阶跃激发抖动。 */
    PID_Init(&gimbal_controller.big_yaw_speed_pid, 3000.0f, 0.0f, 0.0f, 30.0f, 0.0f, 0, 0, 0, 0.0018f, 0, 1, Integral_Limit | Trapezoid_Intergral);
#else
    PID_Init(&gimbal_controller.big_yaw_speed_pid,
             16000.0f, 0.0f, 0.0f,
             35.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 0.0f, 1, NONE);
#endif

    float big_yaw_angle_ff_c[3] = {0.0f, 0.0f, 0.0f}; // PID调参：关闭角度目标速度前馈
    Feedforward_Init(&gimbal_controller.big_yaw_angle_forward, 100.0f, big_yaw_angle_ff_c, 0.01f, 5, 5);
    /* 结构保留用于现有无扰切换；力矩前馈已由J/B/C模型统一计算。 */
    float big_yaw_speed_ff_c[3] = {0.0f, 0.0f, 0.0f};
    Feedforward_Init(&gimbal_controller.big_yaw_speed_forward, 500.0f, big_yaw_speed_ff_c, 0.01f, 5, 5);

#endif

    /* 两轴TD参数与 crossing_hole-main 完全一致。 */
    TD_Init(&gimbal_controller.pos_pitch_td, 1000.0f, 0.005f);
    TD_Init(&gimbal_controller.pos_big_yaw_td, 2000.0f, 0.005f);
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
float Gimbal_Pitch_Calculate(float set_point)
{
    float pitch_friction_ratio;

    if (!Gimbal_Pitch_FeedbackReady())
    {
        pitch_feedback_was_ready = 0;
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
        gimbal_controller.set_pitch_angle = gimbal_controller.gyro_pitch_angle;
        gimbal_controller.set_pitch_speed = 0.0f;
        gimbal_controller.set_pitch_current = 0.0f;
        return 0.0f;
    }

    /* 掉线恢复首周期只同步状态，不输出，避免追赶掉线前的旧目标。 */
    if (!pitch_feedback_was_ready)
    {
        pitch_feedback_was_ready = 1;
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
        gimbal_controller.set_pitch_angle = gimbal_controller.gyro_pitch_angle;
        gimbal_controller.set_pitch_speed = 0.0f;
        gimbal_controller.set_pitch_current = 0.0f;
        PID_Clear(&gimbal_controller.pitch_angle_pid);
        PID_Clear(&gimbal_controller.pitch_speed_pid);
        PID_Clear(&gimbal_controller.pitch_current_pid);
        TD_Clear(&gimbal_controller.pos_pitch_td,
                 gimbal_controller.gyro_pitch_angle);
        Feedforward_Reset(&gimbal_controller.pitch_angle_forward,
                          gimbal_controller.gyro_pitch_angle);
        Feedforward_Reset(&gimbal_controller.pitch_speed_forward, 0.0f);
        gimbal_controller.DM_Pitch_Motor.P_des =
            (gimbal_controller.DM_Pitch_Motor.P_Receive - 180.0f) * PI / 180.0f;
        return 0.0f;
    }

#if GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    /*
     * 源辨识模式只运行速度环。done 互锁放在接入层，修复源工程
     * Yaw BC 初始化先写 Ref、但 done 仍为 1 时可能提前运动的问题。
     */
    if (gimbal_sysid.pitch.sysid_done)
    {
        PID_Clear(&gimbal_controller.pitch_speed_pid);
        gimbal_controller.set_pitch_speed = 0.0f;
        gimbal_controller.set_pitch_current = 0.0f;
        return 0.0f;
    }
    gimbal_controller.set_pitch_speed = gimbal_controller.pitch_speed_pid.Ref;
    gimbal_controller.set_pitch_current = GIMBAL_PITCH_MOTOR_SIGN *
        PID_Calculate(&gimbal_controller.pitch_speed_pid,
                      gimbal_controller.gyro_pitch_speed,
                      gimbal_controller.pitch_speed_pid.Ref);
    return gimbal_controller.set_pitch_current;
#endif

    /*
     * crossing_hole-main 并联PID：位置PID跟踪TD.x，速度PID跟踪TD.dx，
     * 两个PID的输出在力矩端直接相加，不再把位置PID输出作为速度给定。
     */
    gimbal_controller.set_pitch_angle =
        TD_Calculate(&gimbal_controller.pos_pitch_td, set_point);
    gimbal_controller.set_pitch_speed = gimbal_controller.pos_pitch_td.dx;
    PID_Calculate(&gimbal_controller.pitch_angle_pid,
                  gimbal_controller.gyro_pitch_angle,
                  gimbal_controller.set_pitch_angle);
    PID_Calculate(&gimbal_controller.pitch_speed_pid,
                  gimbal_controller.gyro_pitch_speed,
                  gimbal_controller.set_pitch_speed);

    /* crossing_hole-main Pitch物理模型前馈：J当前未辨识，保持为0。 */
    pitch_friction_ratio = LIMIT_MAX_MIN(
        gimbal_controller.set_pitch_speed / GIMBAL_PITCH_FRICTION_BLEND_DPS,
        1.0f, -1.0f);
    gimbal_controller.pitch_speed_forward.Output =
        GIMBAL_PITCH_J * gimbal_controller.pos_pitch_td.ddx +
        GIMBAL_PITCH_B * gimbal_controller.set_pitch_speed +
        GIMBAL_PITCH_C * pitch_friction_ratio;

    gimbal_controller.set_pitch_current = GIMBAL_PITCH_MOTOR_SIGN *
        (gimbal_controller.pitch_angle_pid.Output +
         gimbal_controller.pitch_speed_pid.Output +
         gimbal_controller.pitch_speed_forward.Output);

#ifdef PITCH_GRAVITY_COMP_ENABLE
    /* 重力项在电机方向符号之后独立叠加，保持原车标定方向。 */
    gimbal_controller.set_pitch_current += GimbalPitchComp();
#endif

    return gimbal_controller.set_pitch_current;
}

// [SMALL_YAW_REMOVED] 小Yaw计算函数已删除，陀螺仪yaw数据改由大Yaw使用
/*
// 陀螺仪零漂问题解决，大小yaw可解耦控制
float Gimbal_Small_Yaw_Calculate(float set_point)
{

    gimbal_controller.set_small_yaw_angle = TD_Calculate(&gimbal_controller.pos_small_yaw_td, set_point);
    gimbal_controller.set_small_yaw_speed = PID_Calculate(&gimbal_controller.small_yaw_angle_pid, gimbal_controller.gyro_yaw_angle, gimbal_controller.set_small_yaw_angle) + Feedforward_Calculate(&gimbal_controller.small_yaw_angle_forward, gimbal_controller.set_small_yaw_angle);

    TD_Calculate(&gimbal_controller.speed_small_yaw_td, gimbal_controller.set_small_yaw_speed);
    gimbal_controller.set_small_yaw_current = GIMBAL_SMALL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.small_yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_small_yaw_speed) + Feedforward_Calculate(&gimbal_controller.small_yaw_speed_forward, gimbal_controller.set_small_yaw_speed));
    gimbal_controller.set_small_yaw_current = LIMIT_MAX_MIN(gimbal_controller.set_small_yaw_current, GM6020_MAX_CURRENT, -GM6020_MAX_CURRENT);
    if (gimbal_controller.small_yaw_info.angle > GIMBAL_SMALL_YAW_LIMIT_RIGHT && gimbal_controller.small_yaw_info.angle < GIMBAL_SMALL_YAW_LIMIT_LEFG)
    {
        return gimbal_controller.set_small_yaw_current;
    }
    else
    {
        gimbal_controller.set_small_yaw_current = 0; // 卸力
        return gimbal_controller.set_small_yaw_current;
    }
}
*/

ZeroCheck_Typedef big_yaw_angle_zero_check;
float big_yaw_angle_after_zero_check;

void BigYawZeroCheck(void)
{
    big_yaw_angle_after_zero_check = ZeroCheck(&big_yaw_angle_zero_check, gimbal_controller.DM_Big_Yaw_Motor.P_Receive, 360.0f);
}
// 此处过零检测放到接收函数

// 小yaw每转360度，大yaw的yaw值会比小yaw小2.5度，因此需要把大yaw返回的raw_angle加上圈数*2.5
void Big_Yaw_Bias_Cal(void)
{
    static float fix_motor_angle = 0;
    // [SMALL_YAW_REMOVED] 原条件中small_yaw_angle_pid.Err检查已移除，仅检查大Yaw稳定性
    if (fabsf(gimbal_controller.big_yaw_angle_pid.Err) < 1.0f && big_yaw_controller.big_yaw_mode == 0 && big_yaw_controller.gimbal_last_mode != 0)
    {
        // [SMALL_YAW_REMOVED] fix_motor_angle无需计算(小Yaw已删除)
        // fix_motor_angle = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
    }

    // [SINGLE_YAW] 改用云台自身IMU Yaw作为角度反馈，替代底盘CAN 0x166
    big_yaw_controller.dealed_big_yaw_gyro = gimbal_controller.gyro_yaw_angle;

    // 小yaw上电电机角偏置校准：记录进入云台模式时小yaw相对于中心的偏移量
    // uint8_t calibration_flag;
    // calibration_flag = big_yaw_controller.gimbal_last_mode == 0 ;
    // if(calibration_flag){
    //    big_yaw_controller.big_yaw_gyro_bias = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
    // }
    // [SMALL_YAW_REMOVED] 小Yaw删除后bias置0，原计算: small_yaw_info.angle - SMALL_YAW_ZERO_POINT
    big_yaw_controller.big_yaw_gyro_bias = 0.0f;
    // 不修改反馈值dealed_big_yaw_gyro，偏置补偿在目标端处理
}

float big_yaw_angle_after_iir;
float Gimbal_Big_Yaw_Calculate(float set_point)
{
    float model_feedforward;

    if (!Gimbal_Yaw_FeedbackReady())
    {
        yaw_feedback_was_ready = 0;
        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
        gimbal_controller.set_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
        gimbal_controller.set_big_yaw_speed = 0.0f;
        gimbal_controller.set_big_yaw_current = 0.0f;
        Gimbal_Big_Yaw_ModelFeedforwardReset();
        return 0.0f;
    }

    /* 掉线恢复首周期同步TD/PID/前馈，下一周期才重新闭环。 */
    if (!yaw_feedback_was_ready)
    {
        yaw_feedback_was_ready = 1;
        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
        gimbal_controller.set_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
        gimbal_controller.set_big_yaw_speed = 0.0f;
        gimbal_controller.set_big_yaw_current = 0.0f;
        Gimbal_Big_Yaw_ModelFeedforwardReset();
        PID_Clear(&gimbal_controller.big_yaw_angle_pid);
        PID_Clear(&gimbal_controller.big_yaw_speed_pid);
        Feedforward_Reset(&gimbal_controller.big_yaw_angle_forward,
                          big_yaw_controller.dealed_big_yaw_gyro);
        Feedforward_Reset(&gimbal_controller.big_yaw_speed_forward, 0.0f);
        TD_Clear(&gimbal_controller.pos_big_yaw_td,
                 big_yaw_controller.dealed_big_yaw_gyro);
        gimbal_controller.DM_Big_Yaw_Motor.P_des =
            (gimbal_controller.DM_Big_Yaw_Motor.P_Receive - 180.0f) * PI / 180.0f;
        big_yaw_controller.gimbal_enable_flag = 0;
        big_yaw_controller.gimbal_enable_cnt = 0;
        return 0.0f;
    }

#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    /* crossing_hole-main Yaw 辨识模式：只运行大 Yaw 速度环。 */
    Gimbal_Big_Yaw_ModelFeedforwardReset();
    if (gimbal_sysid.yaw.sysid_done)
    {
        PID_Clear(&gimbal_controller.big_yaw_speed_pid);
        gimbal_controller.set_big_yaw_speed = 0.0f;
        gimbal_controller.set_big_yaw_current = 0.0f;
        return 0.0f;
    }
    gimbal_controller.set_big_yaw_speed = gimbal_controller.big_yaw_speed_pid.Ref;
    gimbal_controller.set_big_yaw_current = GIMBAL_BIG_YAW_MOTOR_SIGN *
        PID_Calculate(&gimbal_controller.big_yaw_speed_pid,
                      gimbal_controller.gyro_yaw_speed,
                      gimbal_controller.big_yaw_speed_pid.Ref);
    return gimbal_controller.set_big_yaw_current;
#endif

    // BigYawSetpointSet();

    gimbal_controller.set_big_yaw_angle = TD_Calculate(&gimbal_controller.pos_big_yaw_td, set_point);
    // iir(&big_yaw_angle_after_iir,gimbal_controller.set_big_yaw_angle,0.8f);
    gimbal_controller.set_big_yaw_speed =
        gimbal_controller.pos_big_yaw_td.dx;
    PID_Calculate(&gimbal_controller.big_yaw_angle_pid,
                  big_yaw_controller.dealed_big_yaw_gyro,
                  gimbal_controller.set_big_yaw_angle);
    PID_Calculate(&gimbal_controller.big_yaw_speed_pid,
                  gimbal_controller.gyro_yaw_speed,
                  gimbal_controller.set_big_yaw_speed);
    model_feedforward = Gimbal_Big_Yaw_ModelFeedforward(
        gimbal_controller.pos_big_yaw_td.dx,
        gimbal_controller.pos_big_yaw_td.ddx);
    gimbal_controller.set_big_yaw_current = GIMBAL_BIG_YAW_MOTOR_SIGN *
        (gimbal_controller.big_yaw_angle_pid.Output +
         gimbal_controller.big_yaw_speed_pid.Output +
         model_feedforward);

    // 大yaw缓启动

    if (fabsf(gimbal_controller.big_yaw_angle_pid.Err) < 5.0f)
    {

        big_yaw_controller.gimbal_enable_cnt++;
        if (big_yaw_controller.gimbal_enable_cnt < GIMBAL_INIT_WAIT_TIME)
        {
            gimbal_controller.big_yaw_angle_pid.MaxOut = 160;
        }
        else
        {
            big_yaw_controller.gimbal_enable_flag = 1;
            /* 并联结构中MaxOut是位置PID的直接力矩修正上限。 */
            gimbal_controller.big_yaw_angle_pid.MaxOut = 180;
        }
    }
    if (big_yaw_controller.gimbal_enable_flag == 0)
    {
        gimbal_controller.set_big_yaw_current = LIMIT_MAX_MIN(gimbal_controller.set_big_yaw_current, 1200.0f, -1200.0f);
    }
    return gimbal_controller.set_big_yaw_current;
}

void GimbalClear(void)
{
    PID_Clear(&gimbal_controller.pitch_angle_pid);
    PID_Clear(&gimbal_controller.pitch_speed_pid);
    PID_Clear(&gimbal_controller.pitch_current_pid);

    TD_Clear(&gimbal_controller.pos_pitch_td,
             gimbal_controller.gyro_pitch_angle);

    gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_speed = 0;
    gimbal_controller.set_pitch_current = 0;
    gimbal_controller.comp_pitch_current = 0;

    Feedforward_Reset(&gimbal_controller.pitch_angle_forward,
                      gimbal_controller.gyro_pitch_angle);
    Feedforward_Reset(&gimbal_controller.pitch_speed_forward, 0.0f);

    // DM电机P_des一次性同步为当前位置，之后Kp提供弹簧保持力防止重力下坠
    // P_Receive为度[0,360]，P_des为弧度，转换: (deg-180)*PI/180
    if (offline_detector.pitch_motor_received &&
        offline_detector.pitch_motor_state == PITCH_MOTOR_ON)
    {
        gimbal_controller.DM_Pitch_Motor.P_des =
            (gimbal_controller.DM_Pitch_Motor.P_Receive - 180.0f) * PI / 180.0f;
    }
    if (offline_detector.yaw_motor_received &&
        offline_detector.yaw_motor_state == YAW_MOTOR_ON)
    {
        gimbal_controller.DM_Big_Yaw_Motor.P_des =
            (gimbal_controller.DM_Big_Yaw_Motor.P_Receive - 180.0f) * PI / 180.0f;
    }

    // yaw
    // [SMALL_YAW_REMOVED] 小Yaw PID Clear已删除
    // PID_Clear(&gimbal_controller.small_yaw_angle_pid);
    // PID_Clear(&gimbal_controller.small_yaw_speed_pid);
    PID_Clear(&gimbal_controller.big_yaw_angle_pid);
    PID_Clear(&gimbal_controller.big_yaw_speed_pid);

    // [SMALL_YAW_REMOVED] 小Yaw Feedforward Clear已删除
    // Feedforward_Clear(&gimbal_controller.small_yaw_speed_forward);
    // Feedforward_Clear(&gimbal_controller.small_yaw_angle_forward);
    Feedforward_Reset(&gimbal_controller.big_yaw_angle_forward,
                      big_yaw_controller.dealed_big_yaw_gyro);
    Feedforward_Reset(&gimbal_controller.big_yaw_speed_forward, 0.0f);

    // [SMALL_YAW_REMOVED] 小Yaw TD Clear已删除
    // TD_Clear(&gimbal_controller.pos_small_yaw_td, gimbal_controller.gyro_yaw_angle);
    // TD_Clear(&gimbal_controller.speed_small_yaw_td, gimbal_controller.gyro_yaw_speed);
    TD_Clear(&gimbal_controller.pos_big_yaw_td, big_yaw_controller.dealed_big_yaw_gyro);

    // [SMALL_YAW_REMOVED] 小Yaw目标角度/控制量清零已删除
    // gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    // gimbal_controller.set_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    // gimbal_controller.set_small_yaw_speed = 0;
    // gimbal_controller.set_small_yaw_current = 0;

    gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
    gimbal_controller.set_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
    gimbal_controller.set_big_yaw_speed = 0;
    gimbal_controller.set_big_yaw_current = 0;
    Gimbal_Big_Yaw_ModelFeedforwardReset();

    big_yaw_controller.gimbal_enable_flag = 0;
    big_yaw_controller.gimbal_enable_cnt = 0;
}

/**
 * @brief 云台控制(速度控制)
 * @param[in] set_point 角度值设定 度/s
 */
float Gimbal_Speed_Calculate(float set_point)
{
    //    gimbal_controller.set_yaw_angle = gimbal_controller.gyro_yaw_angle;
    //    PID_Clear(&gimbal_controller.yaw_angle_pid);
    //    gimbal_controller.set_yaw_speed = set_point;
    //    TD_Calculate(&gimbal_controller.speed_yaw_td, gimbal_controller.set_yaw_speed);
    //    gimbal_controller.set_yaw_current = GIMBAL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_yaw_speed) + GimbalFrictionModel());
    //    gimbal_controller.set_yaw_vol = PID_Calculate(&gimbal_controller.yaw_current_pid, gimbal_controller.yaw_info.torque_current, gimbal_controller.set_yaw_current);
    //    return gimbal_controller.set_yaw_vol;
}

/**
 * @brief 限制设置的pitch角度大小
 */

void limitPitchAngle()
{
    float cur_motor_angle;
    float cur_gyro_angle = gimbal_controller.gyro_pitch_angle;

    /* 未收到有效Pitch/IMU反馈时不能用默认P_Receive=0推导机械限位。 */
    static uint8_t pitch_limit_feedback_was_ready = 0;
    static float motor_angle_filtered = 0.0f;
    if (!Gimbal_Pitch_FeedbackReady())
    {
        pitch_limit_feedback_was_ready = 0;
        gimbal_controller.target_pitch_angle = cur_gyro_angle;
        gimbal_controller.pitch_max_gyro_angle = cur_gyro_angle;
        gimbal_controller.pitch_min_gyro_angle = cur_gyro_angle;
        return;
    }

    if (motor_communication[PITCH_MOTOR].motor_type == GM6020)
    {
        cur_motor_angle = gimbal_controller.pitch_info.angle;
    }
    else if (motor_communication[PITCH_MOTOR].motor_type == DM_MOTOR)
    {
        cur_motor_angle = gimbal_controller.DM_Pitch_Motor.P_Receive;
        //    #if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG // PITCH安装位置刚好过圈，下方代替一个过零检测
        //            if (cur_motor_angle > 180)
        //            {
        //                cur_motor_angle -= 360;
        //            }

        //            cur_motor_angle /= 2; // 有2:1减速比
        //    #endif
    }

    // IIR滤波平滑P_Receive，防止限位附近编码器抖动→clamp边界反复跳变→target不连续→FF微分spike→振荡
    if (!pitch_limit_feedback_was_ready)
    {
        pitch_limit_feedback_was_ready = 1;
        motor_angle_filtered = cur_motor_angle;
        gimbal_controller.target_pitch_angle = cur_gyro_angle;
    }
    else
    {
        float iir_alpha = (cur_motor_angle < GIMBAL_ANGLE_MIN_SOFT) ? 0.98f : 0.9f;
        iir(&motor_angle_filtered, cur_motor_angle, iir_alpha);
    }

    gimbal_controller.pitch_max_gyro_angle = cur_gyro_angle + GIMBAL_PITCH_MOTOR_SIGN * (GIMBAL_ANGLE_MAX - motor_angle_filtered);
    gimbal_controller.pitch_min_gyro_angle = cur_gyro_angle + GIMBAL_PITCH_MOTOR_SIGN * (GIMBAL_ANGLE_MIN - motor_angle_filtered);
    gimbal_controller.target_pitch_angle = LIMIT_MAX_MIN(gimbal_controller.target_pitch_angle, gimbal_controller.pitch_max_gyro_angle, gimbal_controller.pitch_min_gyro_angle);
}

/**
 * @brief 更新pitch角速度，以及角度(注意需要标定零点)
 */
void updateGyro()
{
    /*
     * 保持原车控制使用的 INS.Roll 角度、方向、零点和限位关系不变。
     * INS.Roll = QEKF_INS.Pitch，对应机体系 X 轴角速度；速度直接读取
     * 原始陀螺仪，避免姿态角差分因任务不同步而长期显示异常值。
     */
    gimbal_controller.delta_t = DWT_GetDeltaT(&gimbal_controller.last_cnt);
    gimbal_controller.gyro_pitch_angle =
        GIMBAL_PITCH_GYRO_SIGN * (INS.Roll - GIMBAL_PITCH_BIAS);
    float speed = GIMBAL_PITCH_GYRO_SIGN *
                  INS.Gyro[X] * RAD_TO_ANGLE_COEF;

    /* 与 crossing_hole-main 相同的 Pitch 原始陀螺仪速度滤波系数。 */
    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.2f);
    gimbal_controller.gyro_last_pitch_angle = gimbal_controller.gyro_pitch_angle;

    // yaw
    // [SMALL_YAW_REMOVED] GIMBAL_SMALL_YAW_GYRO_SIGN原为1.0f，直接使用
    gimbal_controller.gyro_yaw_angle = 1.0f * INS.YawTotalAngle;
    speed = (gimbal_controller.gyro_yaw_angle - gimbal_controller.gyro_last_yaw_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.4);
    gimbal_controller.gyro_last_yaw_angle = gimbal_controller.gyro_yaw_angle;
}

/**
 * @brief Pitch重力补偿：使用系统辨识得到的正余弦模型。
 */
float GimbalPitchComp(void)
{
    float theta_rad = gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF;
    gimbal_controller.comp_pitch_current =
        GIMBAL_PITCH_SIN * sinf(theta_rad) +
        GIMBAL_PITCH_COS * cosf(theta_rad);
    return gimbal_controller.comp_pitch_current;
}

void Schmitt_PID_changer()
{
    //    if(gimbal_controller.gyro_pitch_angle < -2.0f)
    //    {
    //        gimbal_controller.pitch_angle_pid.Kp = 42.0f;
    //        gimbal_controller.pitch_angle_pid.Ki = 0.3f;
    //			gimbal_controller.pitch_angle_pid.Kd = 0.0f;
    //        gimbal_controller.pitch_speed_pid.Kp = 48.0f;
    //        gimbal_controller.pitch_speed_pid.Ki = 5.0f;
    //
    //    }
    //    else if (gimbal_controller.gyro_pitch_angle > 0.5f&&gimbal_controller.gyro_pitch_angle<16.0f )
    //    {
    //        gimbal_controller.pitch_angle_pid.Kp = 52.0f;
    //        gimbal_controller.pitch_angle_pid.Ki = 0.8f;
    //        gimbal_controller.pitch_speed_pid.Kp = 56.0f;
    //        gimbal_controller.pitch_speed_pid.Ki = 6.0f;

    //    }
    //		else if (gimbal_controller.gyro_pitch_angle> 19.0f)
    //		{
    //			 gimbal_controller.pitch_angle_pid.Kp = 58.0f;
    //        gimbal_controller.pitch_angle_pid.Ki = 0.8f;
    //        gimbal_controller.pitch_speed_pid.Kp = 65.0f;
    //        gimbal_controller.pitch_speed_pid.Ki = 8.0f;

    //		}
}

/*==============================================================================
 *                          云台测试模块实现
 *============================================================================*/

extern GimbalController gimbal_controller;

/**
 * @brief  云台测试模块初始化
 */
void GimbalTestInit(GimbalTest_t *test)
{
    // 初始化方波信号发生器
    SquareWaveInit(&test->pitch_square, GIMBAL_SQUARE_LOW_ANGLE, GIMBAL_SQUARE_HIGH_ANGLE,
                   0.5f, GIMBAL_SQUARE_PERIOD_MS);
    // [SMALL_YAW_REMOVED] 小Yaw方波测试已删除
    // SquareWaveInit(&test->small_yaw_square, GIMBAL_SQUARE_LOW_ANGLE, GIMBAL_SQUARE_HIGH_ANGLE,
    //                0.5f, GIMBAL_SQUARE_PERIOD_MS);

    // 初始化目标函数
    GimbalTestResetCost(test);

    // 初始化周期追踪
    test->last_pitch_cycle = 0;
    // [SMALL_YAW_REMOVED]
    // test->last_yaw_cycle = 0;

    // 初始化历史记录
    test->last_pitch_ise = 0;
    test->last_pitch_control = 0;
    test->last_pitch_max_error = 0;
    // [SMALL_YAW_REMOVED]
    // test->last_yaw_ise = 0;
    // test->last_yaw_control = 0;
    // test->last_yaw_max_error = 0;
}

/**
 * @brief  重置目标函数累加器（清零当前周期）
 */
void GimbalTestResetCost(GimbalTest_t *test)
{
    test->pitch_cost.ise = 0;
    test->pitch_cost.control_cost = 0;
    test->pitch_cost.total_cost = 0;
    test->pitch_cost.cycle_time = 0;
    test->pitch_cost.max_error = 0;
    test->pitch_cost.final_error = 0;

    // [SMALL_YAW_REMOVED] 小Yaw目标函数清零已删除
    // test->yaw_cost.ise = 0;
    // test->yaw_cost.control_cost = 0;
    // test->yaw_cost.total_cost = 0;
    // test->yaw_cost.cycle_time = 0;
    // test->yaw_cost.max_error = 0;
    // test->yaw_cost.final_error = 0;
}

/**
 * @brief  运行Pitch目标函数计算
 *         每个周期开始时自动清零，周期结束时保存结果到历史记录
 * @param  test: 测试结构体指针
 * @param  error: 误差 (target - measure)
 * @param  control: 控制量输出
 * @param  delta_t: 时间步长
 */
void GimbalTestRunPitchCost(GimbalTest_t *test, float error, float control, float delta_t)
{
    CostFunction_t *cost = &test->pitch_cost;
    uint16_t current_cycle = test->pitch_square.cycle_count;

    // 检测周期切换：新周期开始（周期 = 低→高→低，共5秒）
    if (current_cycle != test->last_pitch_cycle)
    {
        // 保存上一个周期的结果（如果有累积数据）
        if (cost->cycle_time > 0.1f)  // 至少累积了0.1秒的数据才保存
        {
            test->last_pitch_ise = cost->ise;
            test->last_pitch_control = cost->control_cost;
            test->last_pitch_max_error = cost->max_error;
        }

        // 清零当前周期（开始新周期）
        cost->ise = 0;
        cost->control_cost = 0;
        cost->total_cost = 0;
        cost->cycle_time = 0;
        cost->max_error = 0;
        cost->final_error = 0;

        // 标记周期完成（放在清零之后，表示上一个周期已完成）
        cost->cycle_complete = 1;
    }

    // 更新周期追踪
    test->last_pitch_cycle = current_cycle;

    // ISE累加: ∫e²dt
    cost->ise += error * error * delta_t;

    // 控制量惩罚累加: ∫λ*u²dt
    cost->control_cost += GIMBAL_COST_LAMBDA * control * control * delta_t;

    // 更新最大误差
    float abs_error = fabsf(error);
    if (abs_error > cost->max_error)
    {
        cost->max_error = abs_error;
    }

    // 记录当前误差
    cost->final_error = error;

    // 更新时间
    cost->cycle_time += delta_t;

    // 计算总目标函数
    cost->total_cost = cost->ise + cost->control_cost;
}

/**
 * @brief 云台摩擦力模型，只使用库伦摩擦力，因粘性摩擦力在辨识中表现不明显，故忽略
 * @param[in] void
 */
// float GimbalFrictionModel()
//{
//     // 根据转速判断符号
//     if (fabsf(gimbal_controller.gyro_yaw_speed) < BORDER_FRICTION_SPEED)
//     {
//         // 部分补偿
//         return gimbal_controller.gyro_yaw_speed / BORDER_FRICTION_SPEED * FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF;
//     }
//     // 全补偿
//     return FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF * sign(gimbal_controller.gyro_yaw_speed);
// }
