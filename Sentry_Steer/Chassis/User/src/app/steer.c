#include "ChasisController.h"
#include "PowerLimit.h"

extern uint8_t speed_follow_enable_flag;

/* === 舵电机角度环 Kp 自适应 (2026-07-21 新增) ===
 * 解决问题: 舵电机运动时超调抖动 vs 减小 Kp 影响反应速度的矛盾
 * 策略: 误差大时提高 Kp 加快响应,误差小时降低 Kp 抑制超调
 * 误差 |err| >= 45° : Kp = 33 (基准 30 × 1.1)
 * 误差 |err| <= 5°  : Kp = 27 (基准 30 × 0.9)
 * 5° < |err| < 45° : 线性插值
 * 切换方法: 启用/注释下面的 #define STEER_ANGLE_KP_LINEAR_GAIN
 */
#define STEER_ANGLE_KP_LINEAR_GAIN
#ifdef STEER_ANGLE_KP_LINEAR_GAIN
#define STEER_ANGLE_KP_BASE          30.0f   /* 基准 Kp (与 PID_Init 中保持一致) */
#define STEER_ANGLE_KP_HIGH_SCALE    1.1f    /* 大误差时 Kp 倍率 */
#define STEER_ANGLE_KP_LOW_SCALE     0.7f    /* 小误差时 Kp 倍率 */
#define STEER_ANGLE_ERR_HIGH_DEG     45.0f   /* 大误差阈值 (度) */
#define STEER_ANGLE_ERR_LOW_DEG      5.0f    /* 小误差阈值 (度) */
#endif
void steer_pid_init()
{
    // 舵向初始编码值设定
    infantry.steer_init_encoder[STEER1] = 3798;
    infantry.steer_init_encoder[STEER2] = 3106;
    infantry.steer_init_encoder[STEER3] = 5739  ;
    infantry.steer_init_encoder[STEER4] = 1026;

    // 轮毂电机安装方向
    infantry.steer_wheel_install_direction[STEER1] = 1;
    infantry.steer_wheel_install_direction[STEER2] = 1;
    infantry.steer_wheel_install_direction[STEER3] = 1;
    infantry.steer_wheel_install_direction[STEER4] = -1;
    // 轮子控制PID
    PID_Init(&infantry.wheels_pid[STEER1], C620_MAX_SEND_CURRENT, 600, 0, 10, 0.3, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.wheels_pid[STEER2], C620_MAX_SEND_CURRENT, 600, 0, 10, 0.3, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.wheels_pid[STEER3], C620_MAX_SEND_CURRENT, 600, 0, 10, 0.3, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.wheels_pid[STEER4], C620_MAX_SEND_CURRENT, 600, 0, 10, 0.3, 0, 0, 0, 0, 0, 1, Integral_Limit);

    //    // 舵向控制PID
    //    PID_Init(&infantry.steers_angle_pid[STEER1], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    //    PID_Init(&infantry.steers_angle_pid[STEER2], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    //    PID_Init(&infantry.steers_angle_pid[STEER3], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    //    PID_Init(&infantry.steers_angle_pid[STEER4], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);

    //    PID_Init(&infantry.steers_speed_pid[STEER1], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);
    //    PID_Init(&infantry.steers_speed_pid[STEER2], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);
    //    PID_Init(&infantry.steers_speed_pid[STEER3], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);
    //    PID_Init(&infantry.steers_speed_pid[STEER4], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);

    //    // 6020前馈初始化
    //    infantry.Steer_6020_FF_Coefficient[0] = 10.0f;
    //    infantry.Steer_6020_FF_Coefficient[1] = 0.0f;
    //    infantry.Steer_6020_FF_Coefficient[2] = 0.0f;
    //    Feedforward_Init(&infantry.Steer_6020_FF, 7500, infantry.Steer_6020_FF_Coefficient, 0.004, 0, 0); // 15000
    // 舵向控制PID
    PID_Init(&infantry.steers_angle_pid[STEER1], 720, 0, 0.05, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.steers_angle_pid[STEER2], 720, 0, 0.05, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.steers_angle_pid[STEER3], 720, 0, 0.05, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.steers_angle_pid[STEER4], 720, 0, 0.05, 30, 0, 0, 0, 0, 0, 0, 1, NONE);

    PID_Init(&infantry.steers_speed_pid[STEER1], GM6020_MAX_CURRENT * 4 / 5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER2], GM6020_MAX_CURRENT * 4 / 5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER3], GM6020_MAX_CURRENT * 4 / 5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER4], GM6020_MAX_CURRENT * 4 / 5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);

    // 6020前馈初始化
    // infantry.Steer_6020_FF_Coefficient[0] = 8.0f;
    // infantry.Steer_6020_FF_Coefficient[1] = 8.0f;
    // infantry.Steer_6020_FF_Coefficient[2] = 8.0f;
    // infantry.Steer_6020_FF_Coefficient[3] = 8.0f;
    Feedforward_Init(&infantry.Steer_6020_FF, 6000, infantry.Steer_6020_FF_Coefficient, 0.004, 0, 0); // 15000

    // 底盘前后跟随 输出旋转角速度rad/s  输入弧度制角度
    PID_Init(&infantry.turn_pid, 3.0, 0, 0.05f, 3.0f, 0, 0.05f, 0, 0, 0.001, 0.009, 1, DerivativeFilter | OutputFilter);

    TD_Init(&infantry.steer_angle_td[0], 40000, 0.01);
    TD_Init(&infantry.steer_angle_td[1], 40000, 0.01);
    TD_Init(&infantry.steer_angle_td[2], 40000, 0.01);
    TD_Init(&infantry.steer_angle_td[3], 40000, 0.01);

#ifdef STEER_TORQUE_FEEDFORWARD
    /* === 整车速度外环PID初始化（扭矩前馈） ===
     * 参数推导依据：
     *   - 现有轮电机速度PID: Kp=10, Ki=0.3, 输出±16000 (C620电流值)
     *   - 当1 m/s速度误差时，轮速误差≈1005 deg/s，轮PID输出≈10050（接近满量程）
     *   - 整车外环应更温和，输出"力"而非"电流"，避免与轮速PID冲突
     *
     * 平动外环PID参数推导：
     *   - 整车质量 m=25kg，期望1 m/s误差→0.6 m/s²加速度→15N力
     *   - Kp=15, Ki=3(消除稳态误差), Kd=0.5(轻微阻尼)+微分滤波
     *   - max_out=80N(限制最大加速度3.2 m/s²)，integral_limit=20
     *
     * 旋转外环PID参数推导：
     *   - 转动惯量 I=0.8 kg·m²，期望1 rad/s误差→1 rad/s²角加速度→0.8 N·m
     *   - Kp=0.8, Ki=0.15, Kd=0.03+微分滤波
     *   - max_out=5 N·m(限制最大角加速度6.25 rad/s²)，integral_limit=1.5
     */
    PID_Init(&infantry.chassis_translate_x_pid,
             STEER_FF_MAX_FORCE,    /* max_out = 80N */
             20.0f,                  /* integral_limit */
             0.05f,                  /* deadband = 0.05 m/s */
             15.0f,                  /* Kp = 15 */
             3.0f,                   /* Ki = 3 */
             0.5f,                   /* Kd = 0.5 */
             0, 0,                   /* A, B（不用变速积分） */
             0,                      /* output_lpf_rc（不用输出滤波） */
             0.005f,                 /* derivative_lpf_rc = 5ms */
             1,                      /* ols_order */
             Integral_Limit | DerivativeFilter);

    PID_Init(&infantry.chassis_translate_y_pid,
             STEER_FF_MAX_FORCE,    /* max_out = 80N */
             20.0f,                  /* integral_limit */
             0.05f,                  /* deadband = 0.05 m/s */
             15.0f,                  /* Kp = 15 */
             3.0f,                   /* Ki = 3 */
             0.5f,                   /* Kd = 0.5 */
             0, 0,                   /* A, B */
             0,                      /* output_lpf_rc */
             0.005f,                 /* derivative_lpf_rc = 5ms */
             1,                      /* ols_order */
             Integral_Limit | DerivativeFilter);

    PID_Init(&infantry.chassis_rotate_pid,
             STEER_FF_MAX_TORQUE,   /* max_out = 5 N·m */
             1.5f,                   /* integral_limit */
             0.05f,                  /* deadband = 0.05 rad/s */
             0.8f,                   /* Kp = 0.8 */
             0.15f,                  /* Ki = 0.15 */
             0.03f,                  /* Kd = 0.03 */
             0, 0,                   /* A, B */
             0,                      /* output_lpf_rc */
             0.005f,                 /* derivative_lpf_rc = 5ms */
             1,                      /* ols_order */
             Integral_Limit | DerivativeFilter);

    /* 初始化前馈电流数组 */
    for (int i = 0; i < 4; i++)
        infantry.wheels_ff_current[i] = 0.0f;
#endif /* STEER_TORQUE_FEEDFORWARD */
}

/**********************************************************************************************************
 *函 数 名: add_vector
 *功能说明: 两个向量相加,返回向量为-180到180角度，输入输出均为degree
 *形    参: vector1 vector2
 *返 回 值: result = vector1 + vector2(向量加法)
 **********************************************************************************************************/
Vector add_vector(Vector *v1, Vector *v2)
{
    Vector result;
    float v1_angle = DEG2R_RATIO * v1->angle;
    float v2_angle = DEG2R_RATIO * v2->angle;
    float x = v1->module * arm_cos_f32(v1_angle) + v2->module * arm_cos_f32(v2_angle);
    float y = v1->module * arm_sin_f32(v1_angle) + v2->module * arm_sin_f32(v2_angle);
    arm_sqrt_f32(x * x + y * y, &(result.module));
    result.angle = atan2f(y, x) * R2DEG_RATIO;
    return result;
}

/*舵向电机控制运动优化*/
float steer_moving_optimization(uint8_t steer_num)
{
    /*顺逆时针转动角度计算*/
    float min_angle;
    float flipped_angle_set;
    float invert_flag = 1.0; // 速度向量反转标志
    float angle_now = (infantry.sensors_info.steer_recv[steer_num].angle - infantry.steer_init_encoder[steer_num]) / 8192.0f * 360.0f;
    angle_now += 90.0f; // 根据安装误差添加偏移，使angle_now与angle_set都从y正方向开始计算
    if (angle_now < 0)
        angle_now += 360; // 把转动方向统一到相对初始角度正方向
    float angle_set = fmodf(infantry.steer_vector[steer_num].angle, 360);
    if (angle_set < 0)
        angle_set += 360;                                              // 把转动方向统一到相对初始角度正方向
    float angle_clockwise = fmodf((angle_set - angle_now + 360), 360); // 转动方向统一到相对现在位置正方向
    float angle_counter_clockwise = 360 - angle_clockwise;
    /*比较顺逆时针转动角度大小*/
    if (angle_clockwise <= angle_counter_clockwise)
    {
        min_angle = angle_clockwise;
    }
    else
    {
        min_angle = -angle_counter_clockwise;
    }

    /*180度边界特殊处理：恰好对面时不翻转，避免浮点精度问题导致抖动*/
    if (fabsf(fabsf(min_angle) - 180.0f) < 0.5f)
    {
        // 目标恰好对面180°，保持当前策略，不做翻转优化
        infantry.steer_vector[steer_num].module *= invert_flag;
        return min_angle;
    }

    /*若最小旋转角大于90°，则翻转轮组的向量坐标重新计算*/
    if (fabsf(min_angle) > 90)
    {
        float flipped_angle_now = angle_now + 180;
        if (flipped_angle_now >= 360)
        {
            flipped_angle_now -= 360;
        } // 把翻转后的旋转方向统一到相对初始角正方向

        // 简化翻转角度计算：直接计算翻转后的最小旋转角度
        float flipped_angle_clockwise = fmodf((angle_set - flipped_angle_now + 360), 360);
        float flipped_angle_counter_clockwise = 360 - flipped_angle_clockwise;

        if (flipped_angle_clockwise <= flipped_angle_counter_clockwise)
        {
            flipped_angle_set = flipped_angle_clockwise;
        }
        else
        {
            flipped_angle_set = -flipped_angle_counter_clockwise;
        }

        invert_flag = -1.0; // 改变轮子转向
        min_angle = flipped_angle_set;
    }
    infantry.steer_vector[steer_num].module *= invert_flag;
    return min_angle;
}

#ifdef STEER_TORQUE_FEEDFORWARD
/**
 * @brief 整车速度外环PID + 动力学扭矩前馈分配
 *
 * 三层架构的"中层 + 下层"实现：
 *
 * 【中层 - 整车速度闭环】
 *   - 平动外环PID：target_x_v ↔ x_v → F_x_g (N, 云台坐标系)
 *   - 平动外环PID：target_y_v ↔ y_v → F_y_g (N, 云台坐标系)
 *   - 旋转外环PID：target_yaw_v ↔ yaw_v → T_yaw (N·m)
 *   - 叠加滚动阻力前馈：F_roll = mu * m * g * sign(v_target)
 *
 * 【下层 - 动力学分配】
 *   - 坐标变换：F_x_g/F_y_g（云台系）→ F_x_c/F_y_c（底盘系）
 *   - 每轮平动力：F_i_trans = (F_x_c * cos(steer_angle) + F_y_c * sin(steer_angle)) / 4
 *   - 每轮旋转力：F_i_rot = T_yaw / (4 * R)（等分，假设舵角已对齐切线方向）
 *   - 轮扭矩：T_wheel = (F_i_trans + F_i_rot) * r_wheel
 *   - 转C620电流：I_ff = T_wheel * STEER_FF_TORQUE_TO_C620
 *   - 叠加到 wheels_set_current[i]
 *
 * @note 在 steer_chassis_control() 的4轮PID计算完成后调用
 *       此时 steer_pos_kinematics() 已更新 x_v/y_v/yaw_v（在ChasisControlTask中先调用）
 *
 * @note y_v 符号约定：
 *   steer_pos_kinematics() 中 y_v = -(vy*cos + vx*sin)，负号为0x09A协议约定
 *   前馈中取 -y_v 作为物理速度（向左为正）
 *   ⚠ 若调试发现y轴前馈方向反了，把下方 -infantry.y_v 改成 infantry.y_v
 */
static void steer_torque_feedforward_apply(void)
{
    /* === 第一步：整车速度外环PID === */
    /* 平动x（云台坐标系，向前为正）
     * PID_Calculate(measure, ref) → 误差 = ref - measure = target_x_v - x_v */
    float F_x_g = PID_Calculate(&infantry.chassis_translate_x_pid,
                                infantry.x_v, infantry.target_x_v);

    /* 平动y（云台坐标系，向左为正）
     * y_v 带负号约定，取 -y_v 作为物理速度 */
    float F_y_g = PID_Calculate(&infantry.chassis_translate_y_pid,
                                -infantry.y_v, infantry.target_y_v);

    /* 旋转 */
    float T_yaw = PID_Calculate(&infantry.chassis_rotate_pid,
                                infantry.yaw_v, infantry.target_yaw_v);

    /* === 第二步：阻力前馈（滚动阻力） ===
     * F_roll = mu * m * g，沿目标速度方向施加
     * 克服静摩擦/滚动阻力，让车动起来 */
    float F_roll = STEER_FF_ROLL_RESISTANCE_COEF * STEER_FF_CHASSIS_MASS * STEER_FF_GRAVITY;
    float target_v_mag = sqrtf(infantry.target_x_v * infantry.target_x_v +
                               infantry.target_y_v * infantry.target_y_v);
    if (target_v_mag > 0.01f)
    {
        /* 沿目标速度方向加阻力前馈 */
        float dir_x = infantry.target_x_v / target_v_mag;
        float dir_y = infantry.target_y_v / target_v_mag;
        F_x_g += F_roll * dir_x;
        F_y_g += F_roll * dir_y;
    }

    /* === 第三步：坐标变换（云台系 → 底盘系） ===
     * 正运动学：x_v = vx*c - vy*s,  -y_v = vy*c + vx*s
     * 逆变换（力同为矢量）：F_x_c = c*F_x_g - s*F_y_g
     *                      F_y_c = -s*F_x_g + c*F_y_g */
    float F_x_c = infantry.cos_dir * F_x_g - infantry.sin_dir * F_y_g;
    float F_y_c = -infantry.sin_dir * F_x_g + infantry.cos_dir * F_y_g;

    /* === 第四步：动力学分配到各轮 ===
     * 对每个轮，基于当前实际舵角分解力 */
    for (int i = 0; i < 4; i++)
    {
        /* 当前舵角（弧度，与正运动学 steer_pos_kinematics 一致的定义） */
        float steer_angle_deg = (infantry.sensors_info.steer_recv[i].angle -
                                 infantry.steer_init_encoder[i]) / 8192.0f * 360.0f;
        float steer_angle_rad = steer_angle_deg * DEG2R_RATIO;

        /* 平动分量：力在轮子方向的投影，4轮等分 */
        float F_i_trans = (F_x_c * arm_cos_f32(steer_angle_rad) +
                           F_y_c * arm_sin_f32(steer_angle_rad)) / 4.0f;

        /* 旋转分量：每轮切线方向 F_rot = T / (4 * R)
         * 假设舵角已对齐切线方向（稳态准确，瞬态由PID补偿） */
        float F_i_rot = T_yaw / (4.0f * STEER_INFANTRY_RADIUS);

        /* 总轮扭矩 = (平动力 + 旋转力) * 轮半径 */
        float T_wheel = (F_i_trans + F_i_rot) * STEER_WHEEL_RADIUS;

        /* 转C620电流值 */
        float I_ff = T_wheel * STEER_FF_TORQUE_TO_C620;

        /* 限幅（保守值，防止前馈过强干扰PID） */
        I_ff = LIMIT_MAX_MIN(I_ff, STEER_FF_MAX_CURRENT_PER_WHEEL,
                            -STEER_FF_MAX_CURRENT_PER_WHEEL);

        /* 保存前馈电流（调试/Ozone观察用） */
        infantry.wheels_ff_current[i] = I_ff;

        /* 叠加到轮电机电流（在PID输出基础上加） */
        infantry.excute_info.wheels_set_current[i] += I_ff;
    }
}
#endif /* STEER_TORQUE_FEEDFORWARD */

/**********************************************************************************************************
 *函 数 名: steer_control
 *功能说明: 舵轮底盘控制计算,跟随、陀螺、平动集成在一个函数
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/

uint8_t move_symbol;
void steer_chassis_control(void)
{
    /*如要底盘跟随,计算旋转速度*/
    if (remote_controller.control_mode_action == FOLLOW_GIMBAL || remote_controller.control_mode_action == SPEED_FOLLOW)
    {
        // 添加死区判断，避免小角度误差时的震荡
        if (fabsf(infantry.error_angle) > 0.05f) // 死区：约2.86度
        {
            infantry.target_yaw_v = GIMBAL_MOTOR_SIGN * (PID_Calculate(&infantry.turn_pid, infantry.error_angle, 0.0f)); // 单位rad/s
        }
        else
        {
            infantry.target_yaw_v = 0.0f; // 在死区内，停止旋转
            // 清除PID积分，防止误差累积
            infantry.turn_pid.Iout = 0.0f;
        }
    }

    // 拨杆回正后（云台板传下来的v_x从1变为0），TD算完infantry.target_x_v会有e-14量级的微小值，不是0.0f，导致atan2函数不是0而是某个值，故target_x_y小于e-5后直接赋0.0f
    if (fabsf(infantry.target_x_v) < 1e-5f)
        infantry.target_x_v = 0.0f;
    if (fabsf(infantry.target_y_v) < 1e-5f)
        infantry.target_y_v = 0.0f;
    if (fabsf(infantry.target_yaw_v) < 1e-5f)
        infantry.target_yaw_v = 0.0f;
    if (fabsf(infantry.target_x_v) < 1e-5f && fabsf(infantry.target_y_v) < 1e-5f)
        move_symbol = 0; // 底盘转到位标志位
    else if (fabsf(infantry.error_angle) < 0.1f && speed_follow_enable_flag == 1)
        move_symbol = 1;
    if (remote_controller.control_mode_action == SPEED_FOLLOW)
    {
        float speed_follow_optimize_k = arm_cos_f32(infantry.error_angle) + 1e-5;
        speed_follow_optimize_k = fabsf(speed_follow_optimize_k);
        if (move_symbol == 0)
        {
            infantry.turn_pid.Kp = 6;
            infantry.turn_pid.MaxOut = 6;
            // 启动转向时减少

            if (speed_follow_optimize_k < 0.6)
                speed_follow_optimize_k = 0.01f;
            speed_follow_optimize_k = LIMIT_MAX_MIN(speed_follow_optimize_k, 1.0f, 0.01f);
            infantry.set_x_v = speed_follow_optimize_k * (infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir);
            infantry.set_y_v = speed_follow_optimize_k * (infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir);
        }
        else
        {
            infantry.turn_pid.Kp = 4.0;
            infantry.turn_pid.MaxOut = 3.5;
            speed_follow_optimize_k = LIMIT_MAX_MIN(speed_follow_optimize_k, 1.0f, 0.2f);
            float y_opti;
            if (speed_follow_optimize_k < 0.8f)
                y_opti = 0.8f;
            else
                y_opti = speed_follow_optimize_k;

            // 高速转向进行小幅削减,只削减x
            infantry.set_x_v = speed_follow_optimize_k * (infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir);
            infantry.set_y_v = (infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir);
        }
    }
    if (infantry.target_x_v != 0 || infantry.target_y_v != 0 || infantry.target_yaw_v != 0 || gimbal_receiver_pack1.through_hole_flag)
    {
        if (remote_controller.control_mode_action != SPEED_FOLLOW) // 非速度跟随下速度矢量计算
        {
            /*计算平动向量的速度模值m/s、方向degree*/
            // 根据底盘跟随方向添加对应偏移角度
            // error_angle是相对于所选方向的角度，需要转换到统一坐标系
            float direction_offset = 0.0f;
            if (infantry.chassis_direction == CHASSIS_BACK)
                direction_offset = 180.0f;
            else if (infantry.chassis_direction == CHASSIS_LEFT)
                direction_offset = -90.0f;
            else if (infantry.chassis_direction == CHASSIS_RIGHT)
                direction_offset = 90.0f;
            // CHASSIS_FRONT: direction_offset = 0.0f

            // 角度计算：atan2(y, -x)匹配舵轮物理基准
            infantry.robot_vector.angle = direction_offset + R2DEG_RATIO * atan2f(infantry.target_y_v, -infantry.target_x_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;

            arm_sqrt_f32(infantry.target_y_v * infantry.target_y_v + infantry.target_x_v * infantry.target_x_v, &infantry.robot_vector.module); // 计算速度模值
        }
        else
        {
            // 角度计算：atan2(y, -x)匹配舵轮物理基准
            infantry.robot_vector.angle = R2DEG_RATIO * atan2f(infantry.set_y_v, -infantry.set_x_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;

            arm_sqrt_f32(infantry.set_y_v * infantry.set_y_v + infantry.set_x_v * infantry.set_x_v, &infantry.robot_vector.module); // 计算速度模值
            if (fabsf(infantry.robot_vector.module) < 0.001)
                infantry.target_yaw_v = 0; // 改善速度跟随模式
        }

        /*平动向量赋值,以底盘建立坐标系*/
        for (int i = 0; i < 4; i++)
        {
            infantry.steer_vector[i].angle = infantry.robot_vector.angle;   // degree 角度转换到底盘坐标系
            infantry.steer_vector[i].module = infantry.robot_vector.module; // m/s
        }

        /*转动向量赋值*/
        Vector w_vector[4];

        if (remote_controller.control_mode_action == CV_ROTATE || remote_controller.control_mode_action == FOLLOW_GIMBAL || remote_controller.control_mode_action == NOT_FOLLOW_GIMBAL || remote_controller.control_mode_action == SPEED_FOLLOW)
        {
            // 底盘旋转3508线速度大小赋值 m/s，统一用绝对值，方向由角度控制
            float rotation_speed = fabsf(infantry.target_yaw_v) * STEER_INFANTRY_RADIUS; // 从角速度rad/s到线速度m/s
            w_vector[STEER1].module = rotation_speed;
            w_vector[STEER2].module = rotation_speed;
            w_vector[STEER3].module = rotation_speed;
            w_vector[STEER4].module = rotation_speed;
            // 底盘旋转6020角度朝向赋值 degree，根据旋转方向调整角度
            // 切线方向 = 安装位置角度 + 90°（逆时针旋转时）
            // 安装位置：STEER1=135°(左前), STEER2=45°(右前), STEER3=-135°(左后), STEER4=-45°(右后)
            // 逆时针旋转时切线方向：STEER1=45°, STEER2=135°, STEER3=-45°, STEER4=-135°
            float angle_offset = (infantry.target_yaw_v >= 0) ? 0.0f : 180.0f; // 反转时角度翻转180度
            w_vector[STEER1].angle = -135.0f + angle_offset;
            w_vector[STEER2].angle = 135.0f + angle_offset;
            w_vector[STEER3].angle = -45.0f + angle_offset;
            w_vector[STEER4].angle = 45.0f + angle_offset;

            /*执行向量加法,把转动、平动向量相加*/
            for (int u = 0; u < 4; u++)
            {
                infantry.steer_vector[u] = add_vector(&(infantry.steer_vector[u]), &(w_vector[u]));
            }
        }

        for (int i = 0; i < 4; i++)
        {
            /*6020转角最小和3508反向转动策略*/
            infantry.steer_vector[i].angle = infantry.sensors_info.steer_decode[i].angle + steer_moving_optimization(i);

            /*电机安装方向修正*/
            infantry.steer_vector[i].module *= infantry.steer_wheel_install_direction[i];

            /*单位转换 从线速度m/s -> 轴degree/s*/
            infantry.steer_vector[i].module *= STEER_SPEED_TO_DEGEREE_S;

            /*pid计算*/
            infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.steer_vector[i].module);
            #ifdef STEER_ANGLE_KP_LINEAR_GAIN
            /* 舵电机角度环 Kp 自适应: 根据角度误差线性调整 Kp
             * steer_vector[i].angle = steer_decode[i].angle + steer_moving_optimization(i)
             * 故 steer_vector[i].angle - steer_decode[i].angle 即最短路径误差,范围 [-180,180]
             * - 误差大时提高 Kp 加快响应
             * - 误差小时降低 Kp 抑制超调抖动 */
            {
                float steer_err_abs = fabsf(infantry.steer_vector[i].angle - infantry.sensors_info.steer_decode[i].angle);
                float kp_scale;
                if (steer_err_abs <= STEER_ANGLE_ERR_LOW_DEG)
                    kp_scale = STEER_ANGLE_KP_LOW_SCALE;
                else if (steer_err_abs >= STEER_ANGLE_ERR_HIGH_DEG)
                    kp_scale = STEER_ANGLE_KP_HIGH_SCALE;
                else
                    kp_scale = STEER_ANGLE_KP_LOW_SCALE +
                              (STEER_ANGLE_KP_HIGH_SCALE - STEER_ANGLE_KP_LOW_SCALE) *
                              (steer_err_abs - STEER_ANGLE_ERR_LOW_DEG) /
                              (STEER_ANGLE_ERR_HIGH_DEG - STEER_ANGLE_ERR_LOW_DEG);
                infantry.steers_angle_pid[i].Kp = STEER_ANGLE_KP_BASE * kp_scale;
            }
            #endif
            // 加一层td滤波作缓冲
            infantry.Steer_Speed_Setpoint[i] = PID_Calculate(&infantry.steers_angle_pid[i], infantry.sensors_info.steer_decode[i].angle, TD_Calculate(&infantry.steer_angle_td[i], infantry.steer_vector[i].angle));
            infantry.steers_set_current = PID_Calculate(&infantry.steers_speed_pid[i], infantry.sensors_info.steer_decode[i].speed, infantry.Steer_Speed_Setpoint[i]) + Feedforward_Calculate(&infantry.Steer_6020_FF, infantry.Steer_Speed_Setpoint[i]);
            infantry.excute_info.steers_set_current[i] = infantry.steers_set_current;
        }

        #ifdef STEER_TORQUE_FEEDFORWARD
        /* 整车扭矩前馈：在轮速PID输出基础上叠加动力学前馈电流
         * 在4轮速度PID计算完成后调用，此时x_v/y_v/yaw_v已由steer_pos_kinematics()更新 */
        steer_torque_feedforward_apply();
        #endif
    }
    else
    {
        for (int m = 0; m < 4; m++)
        {
            infantry.excute_info.steers_set_current[m] = 0.0f;
            infantry.excute_info.wheels_set_current[m] = PID_Calculate(&infantry.wheels_pid[m], infantry.sensors_info.wheels_decode[m].speed, 0.0f);
        }
        #ifdef STEER_TORQUE_FEEDFORWARD
        /* 停车时清零前馈PID积分项，防止下次启动时积分突变 */
        infantry.chassis_translate_x_pid.Iout = 0.0f;
        infantry.chassis_translate_x_pid.ITerm = 0.0f;
        infantry.chassis_translate_x_pid.Last_ITerm = 0.0f;
        infantry.chassis_translate_y_pid.Iout = 0.0f;
        infantry.chassis_translate_y_pid.ITerm = 0.0f;
        infantry.chassis_translate_y_pid.Last_ITerm = 0.0f;
        infantry.chassis_rotate_pid.Iout = 0.0f;
        infantry.chassis_rotate_pid.ITerm = 0.0f;
        infantry.chassis_rotate_pid.Last_ITerm = 0.0f;
        for (int m = 0; m < 4; m++)
            infantry.wheels_ff_current[m] = 0.0f;
        #endif
    }
}

/**
 * @brief 舵轮正运动学：从舵电机角度和轮电机速度反解底盘相对于云台的实际速度
 * @note 完全对应逆运动学 steer_chassis_control 的逻辑，并包含坐标变换
 *
 *       【逆运动学】steer_vector[i] = add_vector(robot_vector, w_vector[i])
 *       【正运动学】先平均得到平动分量，再从轮速中减去平动，最后计算yaw
 *
 *       【yaw切线角度】与逆运动学w_vector.angle一致
 *       STEER1=135°, STEER2=45°, STEER3=-135°, STEER4=-45°
 *
 *       【坐标变换】
 *       云台→底盘：chassis_x = target_x*cos + target_y*sin
 *                  chassis_y = target_y*cos - target_x*sin
 *       底盘→云台：gimbal_x = chassis_x*cos - chassis_y*sin
 *                  gimbal_y = chassis_x*sin + chassis_y*cos
 */
void steer_pos_kinematics(void)
{
    float wheel_speed_m_s[4]; // 四个轮的物理线速度
    float steer_angle_rad[4]; // 四个舵轮的物理角度

    // 四个舵轮yaw切线方向角度（与逆运动学w_vector.angle一致）
    // [FIX bug②] 原为 {-45, 45, -135, 135}，与逆运动学 w_vector 的 {-135, 135, -45, 45} 每个偏 90°，
    //             导致纯自转时 angle_diff=90° → cos≈0 → yaw_v≈0。现改为与逆运动学一致。
    float tangent_angle_rad[4] = {-135.0f * DEG2R_RATIO, 135.0f * DEG2R_RATIO, -45.0f * DEG2R_RATIO, 45.0f * DEG2R_RATIO};

    // 第一步：计算每个轮的物理线速度和舵轮角度
    for (int i = 0; i < 4; i++)
    {
        // 单位转换：度/s → m/s
        float wheel_speed_deg_s = infantry.sensors_info.wheels_decode[i].speed;
        wheel_speed_m_s[i] = wheel_speed_deg_s * STEER_DEGEREE_S_TO_MS;

        // 安装方向修正
        wheel_speed_m_s[i] *= infantry.steer_wheel_install_direction[i];

        // 计算舵轮物理角度
        float encoder_offset_deg = (infantry.sensors_info.steer_recv[i].angle - infantry.steer_init_encoder[i]) / 8192.0f * 360.0f;
        steer_angle_rad[i] = encoder_offset_deg * DEG2R_RATIO;
    }

    // 第二步：平均计算平动分量（yaw分量平均为0）
    // [FIX bug①] 原为 sin 给 x、cos 给 y，与逆运动学 add_vector 的 cos 给 x、sin 给 y 相反。
    //             现改为 cos 给 x、sin 给 y，与逆运动学一致。
    float vx_chassis_sum = 0.0f;
    float vy_chassis_sum = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        vx_chassis_sum += wheel_speed_m_s[i] * arm_cos_f32(steer_angle_rad[i]);
        vy_chassis_sum += wheel_speed_m_s[i] * arm_sin_f32(steer_angle_rad[i]);
    }
    float vx_chassis = vx_chassis_sum / 4.0f;
    float vy_chassis = vy_chassis_sum / 4.0f;

    // 第三步：计算yaw贡献（从轮速中减去平动分量，再投影到切线）
    float yaw_sum = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        // 平动分量：robot_vector 在该舵轮方向的分量
        // [FIX bug①] 同步修改：cos 给 x、sin 给 y
        float robot_speed_contribution = vx_chassis * arm_cos_f32(steer_angle_rad[i]) + vy_chassis * arm_sin_f32(steer_angle_rad[i]);

        // yaw分量：轮速 - 平动分量
        float yaw_speed_component = wheel_speed_m_s[i] - robot_speed_contribution;

        // yaw贡献 = yaw分量 * cos(steer_angle - tangent_angle) / radius
        float angle_diff = steer_angle_rad[i] - tangent_angle_rad[i];
        float yaw_contribution = yaw_speed_component * arm_cos_f32(angle_diff) / STEER_INFANTRY_RADIUS;
        yaw_sum += yaw_contribution;
    }

    // 第四步：输出结果
    infantry.yaw_v = yaw_sum / 4.0f;

    // 第五步：底盘坐标系 → 云台坐标系
    // [FIX bug①] vx_chassis/vy_chassis 含义已修正（vx=true_x, vy=true_y），坐标变换同步调整
    //   逆运动学(云台→底盘): chassis_x = gimbal_x*cos + gimbal_y*sin, chassis_y = gimbal_y*cos - gimbal_x*sin
    //   正运动学(底盘→云台): gimbal_x = chassis_x*cos - chassis_y*sin, gimbal_y = chassis_x*sin + chassis_y*cos
    //   y_v 保留原有负号（与下游 0x09A 协议约定一致）
    infantry.x_v = vx_chassis * infantry.cos_dir - vy_chassis * infantry.sin_dir;   // 向前
    infantry.y_v = -(vy_chassis * infantry.cos_dir + vx_chassis * infantry.sin_dir); // 向左
}

/**
 * @brief 舵轮角度调试函数：方波跳变调参
 * @note 调参时只动一个舵轮，其他舵电机和全部轮电机不动
 *       方波跳变用于测试PID阶跃响应
 *       使用方法：
 *       1. 在ChasisControlTask.c中取消注释steer_angle_debug()调用，注释掉main_control()
 *       2. 修改STEER_DEBUG_TARGET_STEER选择调试的舵轮
 *       3. 观察舵轮响应，调整steer_pid_init中的PID参数
 *       4. 调参完成后恢复main_control()调用，注释掉steer_angle_debug()
 */
void steer_angle_debug(void)
{
    static uint32_t debug_time_cnt = 0;    // 时间计数器(ms)
    static uint8_t square_state = 0;       // 方波状态：0=低，1=高
    static int16_t init_encoder_value = 0; // 初始编码器值（原始值，不累积）
    static uint8_t init_flag = 0;          // 初始化标志
    float target_angle;                    // 目标角度
    float current_angle_offset;            // 当前相对偏移角度

    // 第一次调用时记录初始编码器值
    if (init_flag == 0)
    {
        init_encoder_value = infantry.sensors_info.steer_recv[STEER_DEBUG_TARGET_STEER].angle;
        init_flag = 1;
    }

    // 时间计数，假设任务周期1ms
    debug_time_cnt += 1;

    // 方波周期计算，每STEER_DEBUG_SQUARE_PERIOD毫秒翻转一次
    if (debug_time_cnt >= STEER_DEBUG_SQUARE_PERIOD)
    {
        debug_time_cnt = 0;
        square_state = !square_state; // 翻转方波状态
    }

    // 计算目标角度：基准角度 ± 幅度（相对于初始位置）
    if (square_state == 0)
    {
        target_angle = STEER_DEBUG_ANGLE_BASE - STEER_DEBUG_ANGLE_AMPLITUDE; // -45°
    }
    else
    {
        target_angle = STEER_DEBUG_ANGLE_BASE + STEER_DEBUG_ANGLE_AMPLITUDE; // +45°
    }

    // 所有轮电机输出为0（底盘不移动）
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = 0.0f;
    }

    // 目标舵轮执行角度控制，其他舵电机输出为0
    for (int i = 0; i < 4; i++)
    {
        if (i == STEER_DEBUG_TARGET_STEER)
        {
            // 使用原始编码器值计算相对角度（单圈范围内）
            int16_t current_encoder = infantry.sensors_info.steer_recv[i].angle;
            int16_t encoder_offset = current_encoder - init_encoder_value;

            // 处理编码器过零（单圈范围内，±半圈）
            if (encoder_offset > 4096) // 超过半圈正向
                encoder_offset -= 8192;
            else if (encoder_offset < -4096) // 超过半圈负向
                encoder_offset += 8192;

            // 转换为角度
            current_angle_offset = encoder_offset / 8192.0f * 360.0f;

            // 计算速度设定点（角度PID输出）
            infantry.Steer_Speed_Setpoint[i] = PID_Calculate(&infantry.steers_angle_pid[i],
                                                             current_angle_offset,
                                                             target_angle);
            // 计算电流输出（速度PID + 前馈）
            infantry.steers_set_current = PID_Calculate(&infantry.steers_speed_pid[i],
                                                        infantry.sensors_info.steer_decode[i].speed,
                                                        infantry.Steer_Speed_Setpoint[i]) +
                                          Feedforward_Calculate(&infantry.Steer_6020_FF, infantry.Steer_Speed_Setpoint[i]);
            infantry.excute_info.steers_set_current[i] = infantry.steers_set_current;
        }
        else
        {
            // 其他舵轮：输出为0
            infantry.excute_info.steers_set_current[i] = 0.0f;
        }
    }
}
