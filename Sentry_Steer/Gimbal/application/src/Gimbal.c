#include "Gimbal.h"
#include "robot_config.h"

GimbalController gimbal_controller;
BigYawController big_yaw_controller;
// gimbal_controller.DM_Big_Yaw_Motor.P_Receive

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit()
{

#if ROBOT == GOBLIN
    // pitch VOL LOOP
    PID_Init(&gimbal_controller.pitch_angle_pid, 150.0f, 48.0f, 0.0f, 52.0f, 0.5f, 0.05f, 0, 0, 0, 0.02f, 1, DerivativeFilter | Integral_Limit | Trapezoid_Intergral);
    PID_Init(&gimbal_controller.pitch_speed_pid, 3000, 1200, 0.1f, 58.0f, 6.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // yaw GM6020 CURRENT LOOP
    PID_Init(&gimbal_controller.small_yaw_angle_pid, 180.0, 0, 0.05, 48.0f, 0, 0.3f, 0, 0, 0.0, 0.0f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.small_yaw_speed_pid, GM6020_MAX_CURRENT, 1000, 0.5, 170.0f, 1.5f, 0, 0, 0, 0.f, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // yaw DM MOTOR CURRENT LOOP
    PID_Init(&gimbal_controller.big_yaw_angle_pid, 360.0, 0, 0.05, 32.0f, 0.f, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.big_yaw_speed_pid, 6000, 1200, 0.5, 33.0f, 1.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // 跟踪微分器
    //    TD_Init(&gimbal_controller.pos_big_yaw_td, 10000, 0.01);
    //    TD_Init(&gimbal_controller.speed_big_yaw_td, 90000, 0.01);
    //    TD_Init(&gimbal_controller.pos_small_yaw_td, 30000, 0.003);
    //    TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);
    TD_Init(&gimbal_controller.pos_big_yaw_td, 20000, 0.01);
    TD_Init(&gimbal_controller.speed_big_yaw_td, 90000, 0.01);
    TD_Init(&gimbal_controller.pos_small_yaw_td, 40000, 0.01);
    TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);

    // Feedforward_Init( Feedforward_t,float max_out, float *c, float lpf_rc, uint16_t ref_dot_ols_order, uint16_t ref_ddot_ols_orde)

    float small_yaw_angle_ff_c[3] = {0.f, 0.4f, 0.0f}; // 小yaw前馈参数向量
    float small_yaw_speed_ff_c[3] = {0.4f, 0.f, 0.0f}; // 小yaw前馈参数向量
    Feedforward_Init(&gimbal_controller.small_yaw_angle_forward, 100.0f, small_yaw_angle_ff_c, 0.01f, 5, 5);
    Feedforward_Init(&gimbal_controller.small_yaw_speed_forward, 500.0f, small_yaw_speed_ff_c, 0.01f, 5, 5);
    // 云台、机械臂的最小二乘法阶数经验值通常为3~5，LPF_rc在0.01~0.05

    float big_yaw_angle_ff_c[3] = {0.f, 0.15f, 0.0f}; // 大yaw前馈参数向量
    Feedforward_Init(&gimbal_controller.big_yaw_angle_forward, 100.0f, big_yaw_angle_ff_c, 0.01f, 5, 5);
    float big_yaw_speed_ff_c[3] = {1.0f, 0.4f, 0.0f};
    Feedforward_Init(&gimbal_controller.big_yaw_speed_forward, 500.0f, big_yaw_speed_ff_c, 0.01f, 5, 5);

    // 滤波器，阶数，截止频率，采样频率,FIR有bug
    // FIRFilter_Init(&big_yaw_fir_filter, 10, 100, 500);

#elif ROBOT == TIGER

    // pitch VOL LOOP
	PID_Init(&gimbal_controller.pitch_angle_pid, 500.0f, 48.0f, 0.0f, 40.0f, 0.0f, 0.0f, 0, 0, 0, 0.02f, 1, DerivativeFilter | Integral_Limit| Trapezoid_Intergral);
	PID_Init(&gimbal_controller.pitch_speed_pid, 15000, 1200, 0.1f, 35.0f, 1.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // yaw GM6020 CURRENT LOOP
    PID_Init(&gimbal_controller.small_yaw_angle_pid, 100.0, 0, 0.05, 15.0f, 0, 0.0f, 0, 0, 0.0, 0.0f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.small_yaw_speed_pid, GM6020_MAX_CURRENT, 5000, 0.5, 120.0f, 80.0f, 0, 0, 0, 0.f, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // yaw DM MOTOR CURRENT LOOP
    PID_Init(&gimbal_controller.big_yaw_angle_pid, 360.0, 0, 0.05, 32.0f, 0.f, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.big_yaw_speed_pid, 6000, 1200, 0.5, 33.0f, 1.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // 跟踪微分器
    //    TD_Init(&gimbal_controller.pos_big_yaw_td, 10000, 0.01);
    //    TD_Init(&gimbal_controller.speed_big_yaw_td, 90000, 0.01);
    //    TD_Init(&gimbal_controller.pos_small_yaw_td, 30000, 0.003);
    //    TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);
    TD_Init(&gimbal_controller.pos_big_yaw_td, 20000, 0.01);
    TD_Init(&gimbal_controller.speed_big_yaw_td, 90000, 0.01);
    TD_Init(&gimbal_controller.pos_small_yaw_td, 40000, 0.01);
    TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);

    // Feedforward_Init( Feedforward_t,float max_out, float *c, float lpf_rc, uint16_t ref_dot_ols_order, uint16_t ref_ddot_ols_orde)

    float small_yaw_angle_ff_c[3] = {0.f, 0.4f, 0.0f}; // 小yaw前馈参数向量
    float small_yaw_speed_ff_c[3] = {0.4f, 0.f, 0.0f}; // 小yaw前馈参数向量
    Feedforward_Init(&gimbal_controller.small_yaw_angle_forward, 100.0f, small_yaw_angle_ff_c, 0.01f, 5, 5);
    Feedforward_Init(&gimbal_controller.small_yaw_speed_forward, 500.0f, small_yaw_speed_ff_c, 0.01f, 5, 5);
    // 云台、机械臂的最小二乘法阶数经验值通常为3~5，LPF_rc在0.01~0.05

    float big_yaw_angle_ff_c[3] = {0.f, 0.15f, 0.0f}; // 大yaw前馈参数向量
    Feedforward_Init(&gimbal_controller.big_yaw_angle_forward, 100.0f, big_yaw_angle_ff_c, 0.01f, 5, 5);
    float big_yaw_speed_ff_c[3] = {1.0f, 0.4f, 0.0f};
    Feedforward_Init(&gimbal_controller.big_yaw_speed_forward, 500.0f, big_yaw_speed_ff_c, 0.01f, 5, 5);

#endif
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
float Gimbal_Pitch_Calculate(float set_point)
{
    // pitch 三环 + 重力补偿前馈
    gimbal_controller.set_pitch_angle = set_point;
    gimbal_controller.set_pitch_speed = PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, gimbal_controller.set_pitch_angle);
    gimbal_controller.set_pitch_current = GIMBAL_PITCH_MOTOR_SIGN * PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.set_pitch_speed);

    // 添加重力补偿前馈,调试模式下可以关闭重力补偿以观察重力对系统的影响
    #if GIMBAL_CONTROL_DISCONNECT == 0
    gimbal_controller.set_pitch_current += GimbalPitchComp();
    #endif

    return gimbal_controller.set_pitch_current;
}

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
    if (fabsf(gimbal_controller.big_yaw_angle_pid.Err) < 1.0f && fabsf(gimbal_controller.small_yaw_angle_pid.Err) < 1.0f && big_yaw_controller.big_yaw_mode == 0 && big_yaw_controller.gimbal_last_mode != 0)
    {
        fix_motor_angle = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
    }

    float big_yaw_angle_fix = big_yaw_controller.big_yaw_gyro_raw / 360.0f * 3.03f; //+ fix_motor_angle ;

    big_yaw_controller.dealed_big_yaw_gyro = big_yaw_controller.big_yaw_gyro_raw + big_yaw_angle_fix;

    // 小yaw上电电机角偏置校准：记录进入云台模式时小yaw相对于中心的偏移量
    // uint8_t calibration_flag;
    // calibration_flag = big_yaw_controller.gimbal_last_mode == 0 ;
    // if(calibration_flag){
    //    big_yaw_controller.big_yaw_gyro_bias = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
    // }
    big_yaw_controller.big_yaw_gyro_bias = gimbal_controller.small_yaw_info.angle - GIMBAL_SMALL_YAW_ZERO_POINT;
    // 不修改反馈值dealed_big_yaw_gyro，偏置补偿在目标端处理
}

float big_yaw_angle_after_iir;
float Gimbal_Big_Yaw_Calculate(float set_point)
{
    // BigYawSetpointSet();

    gimbal_controller.set_big_yaw_angle = TD_Calculate(&gimbal_controller.pos_big_yaw_td, set_point);
    // iir(&big_yaw_angle_after_iir,gimbal_controller.set_big_yaw_angle,0.8f);
    gimbal_controller.set_big_yaw_speed = PID_Calculate(&gimbal_controller.big_yaw_angle_pid, big_yaw_controller.dealed_big_yaw_gyro, gimbal_controller.set_big_yaw_angle) + Feedforward_Calculate(&gimbal_controller.big_yaw_angle_forward, gimbal_controller.set_big_yaw_angle);
    TD_Calculate(&gimbal_controller.speed_big_yaw_td, gimbal_controller.set_big_yaw_speed);
    gimbal_controller.set_big_yaw_current = GIMBAL_BIG_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.big_yaw_speed_pid, big_yaw_controller.big_yaw_gyro_speed, gimbal_controller.set_big_yaw_speed) + Feedforward_Calculate(&gimbal_controller.big_yaw_speed_forward, gimbal_controller.set_big_yaw_speed));

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
            gimbal_controller.big_yaw_angle_pid.MaxOut = 300;
        }
    }
    if (big_yaw_controller.gimbal_enable_flag == 0)
    {
        gimbal_controller.set_big_yaw_current = LIMIT_MAX_MIN(gimbal_controller.set_big_yaw_current, 1200.0f, -1200.0f);
    }
    if (gimbal_controller.small_yaw_recv.angle != 0 && big_yaw_controller.big_yaw_gyro_raw != 0) // 防止小yaw掉线，大yaw疯转
    {
        return gimbal_controller.set_big_yaw_current;
    }
    else
        return 0.f;
}

void GimbalClear(void)
{
    PID_Clear(&gimbal_controller.pitch_angle_pid);
    PID_Clear(&gimbal_controller.pitch_speed_pid);
    PID_Clear(&gimbal_controller.pitch_current_pid);

    Feedforward_Clear(&gimbal_controller.pitch_speed_forward);
    Feedforward_Clear(&gimbal_controller.pitch_angle_forward);

    gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_speed = 0;
    gimbal_controller.set_pitch_current = 0;
    gimbal_controller.comp_pitch_current = 0;

    // yaw
    PID_Clear(&gimbal_controller.small_yaw_angle_pid);
    PID_Clear(&gimbal_controller.small_yaw_speed_pid);
    PID_Clear(&gimbal_controller.big_yaw_angle_pid);
    PID_Clear(&gimbal_controller.big_yaw_speed_pid);

    Feedforward_Clear(&gimbal_controller.small_yaw_speed_forward);
    Feedforward_Clear(&gimbal_controller.small_yaw_angle_forward);
    Feedforward_Clear(&gimbal_controller.big_yaw_speed_forward);
    Feedforward_Clear(&gimbal_controller.big_yaw_angle_forward);

    TD_Clear(&gimbal_controller.pos_small_yaw_td, gimbal_controller.gyro_yaw_angle);
    TD_Clear(&gimbal_controller.speed_small_yaw_td, gimbal_controller.gyro_yaw_speed);
    TD_Clear(&gimbal_controller.pos_big_yaw_td, big_yaw_controller.dealed_big_yaw_gyro);
    TD_Clear(&gimbal_controller.speed_big_yaw_td, big_yaw_controller.big_yaw_gyro_speed);

    gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_small_yaw_speed = 0;
    gimbal_controller.set_small_yaw_current = 0;

    gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
    gimbal_controller.set_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
    gimbal_controller.set_big_yaw_speed = 0;
    gimbal_controller.set_big_yaw_current = 0;

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

    gimbal_controller.pitch_max_gyro_angle = cur_gyro_angle + GIMBAL_PITCH_MOTOR_SIGN * (GIMBAL_ANGLE_MAX - cur_motor_angle);
    gimbal_controller.pitch_min_gyro_angle = cur_gyro_angle + GIMBAL_PITCH_MOTOR_SIGN * (GIMBAL_ANGLE_MIN - cur_motor_angle);
    gimbal_controller.target_pitch_angle = LIMIT_MAX_MIN(gimbal_controller.target_pitch_angle, gimbal_controller.pitch_max_gyro_angle, gimbal_controller.pitch_min_gyro_angle);
}

/**
 * @brief 更新pitch角速度，以及角度(注意需要标定零点)
 */
void updateGyro()
{
    // 注意陀螺仪安装的Pitch和roll轴方向
    gimbal_controller.delta_t = DWT_GetDeltaT(&gimbal_controller.last_cnt);
    gimbal_controller.gyro_pitch_angle = GIMBAL_PITCH_GYRO_SIGN * (INS.Pitch - GIMBAL_PITCH_BIAS);
    float speed = (gimbal_controller.gyro_pitch_angle - gimbal_controller.gyro_last_pitch_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.5);
    gimbal_controller.gyro_last_pitch_angle = gimbal_controller.gyro_pitch_angle;

    // yaw
    gimbal_controller.gyro_yaw_angle = GIMBAL_SMALL_YAW_GYRO_SIGN * INS.YawTotalAngle;
    speed = (gimbal_controller.gyro_yaw_angle - gimbal_controller.gyro_last_yaw_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.4);
    gimbal_controller.gyro_last_yaw_angle = gimbal_controller.gyro_yaw_angle;
}

/**
 * @brief 重力补偿 - 使用多项式模型拟合静止电流
 *        val(x) = p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5
 *        x: pitch角度(度), val: 补偿电流值
 */
float GimbalPitchComp()
{
    // 多项式系数 (Poly4拟合结果 - 2026/04/28更新)
    const static float p1 =  0.0001f;
    const static float p2 =  0.0111f;
    const static float p3 = -1.5359f;
    const static float p4 =  52.829f;
    const static float p5 =  585.75f;

    float x = gimbal_controller.gyro_pitch_angle;

    // 多项式计算: p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float comp_current = p1 * x4 + p2 * x3 + p3 * x2 + p4 * x + p5;

    // IIR滤波平滑输出
    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
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
    SquareWaveInit(&test->small_yaw_square, GIMBAL_SQUARE_LOW_ANGLE, GIMBAL_SQUARE_HIGH_ANGLE,
                   0.5f, GIMBAL_SQUARE_PERIOD_MS);

    // 初始化目标函数
    GimbalTestResetCost(test);

    // 初始化周期追踪
    test->last_pitch_cycle = 0;
    test->last_yaw_cycle = 0;

    // 初始化历史记录
    test->last_pitch_ise = 0;
    test->last_pitch_control = 0;
    test->last_pitch_max_error = 0;
    test->last_yaw_ise = 0;
    test->last_yaw_control = 0;
    test->last_yaw_max_error = 0;
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

    test->yaw_cost.ise = 0;
    test->yaw_cost.control_cost = 0;
    test->yaw_cost.total_cost = 0;
    test->yaw_cost.cycle_time = 0;
    test->yaw_cost.max_error = 0;
    test->yaw_cost.final_error = 0;
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
