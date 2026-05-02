#include "ChasisController.h"
#include "PowerLimit.h"

extern uint8_t speed_follow_enable_flag;
void steer_pid_init()
{
    // 舵向初始编码值设定
    infantry.steer_init_encoder[STEER1] = 3753;
    infantry.steer_init_encoder[STEER2] = 3116;
    infantry.steer_init_encoder[STEER3] = 5700;
    infantry.steer_init_encoder[STEER4] = 1000;

    // 轮毂电机安装方向
    infantry.steer_wheel_install_direction[STEER1] = 1;
    infantry.steer_wheel_install_direction[STEER2] = 1;
    infantry.steer_wheel_install_direction[STEER3] = -1;
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

    PID_Init(&infantry.steers_speed_pid[STEER1], GM6020_MAX_CURRENT*4/5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER2], GM6020_MAX_CURRENT*4/5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER3], GM6020_MAX_CURRENT*4/5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER4], GM6020_MAX_CURRENT*4/5, 8000, 1.0, 10, 0, 0, 0, 0, 0, 0, 1, Integral_Limit);

    // 6020前馈初始化
    infantry.Steer_6020_FF_Coefficient[0] = 8.0f;
    infantry.Steer_6020_FF_Coefficient[1] = 0.0f;
    infantry.Steer_6020_FF_Coefficient[2] = 0.0f;
    Feedforward_Init(&infantry.Steer_6020_FF, 6000, infantry.Steer_6020_FF_Coefficient, 0.004, 0, 0); // 15000

    // 底盘前后跟随 输出旋转角速度rad/s  输入弧度制角度
    // 降低Kp以减少震荡，增加死区稳定性
    PID_Init(&infantry.turn_pid, 3.0, 0, 0.05f, 2.0f, 0, 0.05f, 0, 0, 0.001, 0.009, 1, DerivativeFilter | OutputFilter);

		TD_Init(&infantry.steer_angle_td[0], 40000, 0.01);
		TD_Init(&infantry.steer_angle_td[1], 40000, 0.01);
		TD_Init(&infantry.steer_angle_td[2], 40000, 0.01);
		TD_Init(&infantry.steer_angle_td[3], 40000, 0.01);
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
    if (angle_now < 0)
        angle_now += 360; // 把转动方向统一到相对初始角度正方向
    float angle_set = fmodf(infantry.steer_vector[steer_num].angle, 360);
    if (angle_set < 0)
        angle_set += 360;                                              // 把转动方向统一到相对初始角度正方向
    float angle_clockwise = fmodf((angle_set - angle_now + 360), 360); // 转动方向统一到相对现在位置正方向
    float angle_counter_clockwise = 360 - angle_clockwise;
    /*比较顺逆时针转动角度大小*/
    if (angle_clockwise < angle_counter_clockwise)
    {
        min_angle = angle_clockwise;
    }
    else
    {
        min_angle = -angle_counter_clockwise;
    }
    /*若最小旋转角大于90°，则翻转轮组的向量坐标重新计算*/
    if (abs(min_angle) > 90)
    {
        float flipped_angle_now = angle_now + 180;
        if (flipped_angle_now >= 360)
        {
            flipped_angle_now -= 360;
        } // 把翻转后的旋转方向统一到相对初始角正方向
        if (angle_clockwise > angle_counter_clockwise)
        {
            flipped_angle_set = fmodf((angle_set - flipped_angle_now + 360), 360);
        }
        else
        {
            flipped_angle_set = -fmodf((flipped_angle_now - angle_set + 360), 360);
        }
        invert_flag = -1.0; // 改变轮子转向
        min_angle = flipped_angle_set;
    }
    infantry.steer_vector[steer_num].module *= invert_flag;
    return min_angle;
}

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
    if (remote_controller.control_mode_action == FOLLOW_GIMBAL||remote_controller.control_mode_action == SPEED_FOLLOW)
    {
        // 添加死区判断，避免小角度误差时的震荡
        if (fabsf(infantry.error_angle) > 0.05f)  // 死区：约2.86度
        {
            infantry.target_yaw_v = GIMBAL_MOTOR_SIGN * (PID_Calculate(&infantry.turn_pid, infantry.error_angle, 0.0f)); // 单位rad/s
        }
        else
        {
            infantry.target_yaw_v = 0.0f;  // 在死区内，停止旋转
            // 清除PID积分，防止误差累积
            infantry.turn_pid.Iout = 0.0f;
        }
    }

			
    // 拨杆回正后（云台板传下来的v_x从1变为0），TD算完infantry.target_x_v会有e-14量级的微小值，不是0.0f，导致atan2函数不是0而是某个值，故target_x_y小于e-5后直接赋0.0f
    if (abs(infantry.target_x_v) < 1e-5f)
        infantry.target_x_v = 0.0f;
    if (abs(infantry.target_y_v) < 1e-5f)
        infantry.target_y_v = 0.0f;
    if (abs(infantry.target_yaw_v) < 1e-5f)
        infantry.target_yaw_v = 0.0f;
    if(abs(infantry.target_x_v) < 1e-5f && abs(infantry.target_y_v) < 1e-5f)
    move_symbol = 0;//底盘转到位标志位
    else if (fabsf(infantry.error_angle) < 0.1f&&speed_follow_enable_flag == 1) move_symbol = 1;
    if(remote_controller.control_mode_action == SPEED_FOLLOW)
    {
			float speed_follow_optimize_k = arm_cos_f32(infantry.error_angle) + 1e-5;
      speed_follow_optimize_k = fabsf(speed_follow_optimize_k);
        if(move_symbol == 0)
        {
					infantry.turn_pid.Kp = 6;
					infantry.turn_pid.MaxOut = 6;
            //启动转向时减少
            
					if(speed_follow_optimize_k < 0.6) speed_follow_optimize_k = 0.01f;
            speed_follow_optimize_k = LIMIT_MAX_MIN(speed_follow_optimize_k,1.0f,0.01f);
            infantry.set_x_v = speed_follow_optimize_k *(infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir);
            infantry.set_y_v = speed_follow_optimize_k * (infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir);

        }
        else
        {
					infantry.turn_pid.Kp = 4.0;
					infantry.turn_pid.MaxOut = 3.5;
					speed_follow_optimize_k = LIMIT_MAX_MIN(speed_follow_optimize_k,1.0f,0.2f);
					float y_opti;
					if(speed_follow_optimize_k<0.8f) y_opti = 0.8f;
					else y_opti = speed_follow_optimize_k;
					
            //高速转向进行小幅削减,只削减x
            infantry.set_x_v = speed_follow_optimize_k*(infantry.target_x_v * infantry.cos_dir + infantry.target_y_v * infantry.sin_dir);
            infantry.set_y_v = (infantry.target_y_v * infantry.cos_dir - infantry.target_x_v * infantry.sin_dir);

        }
        
    }
    if (infantry.target_x_v != 0 || infantry.target_y_v != 0 || infantry.target_yaw_v != 0 || gimbal_receiver_pack1.through_hole_flag)
    {
        if(remote_controller.control_mode_action != SPEED_FOLLOW)//非速度跟随下速度矢量计算
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

            infantry.robot_vector.angle = direction_offset + R2DEG_RATIO * atan2f(infantry.target_x_v, infantry.target_y_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;

            arm_sqrt_f32(infantry.target_y_v * infantry.target_y_v + infantry.target_x_v * infantry.target_x_v, &infantry.robot_vector.module); // 计算速度模值
            

        }
        else{
             /*计算平动向量的速度模值m/s、方向degree*/
            //因为速度跟随有跟随方向，所以这里不用加180度
            float new_vector_angle = R2DEG_RATIO * atan2f(infantry.set_x_v, infantry.set_y_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;
            iir(&infantry.robot_vector.angle,new_vector_angle,0.95);//过一个低通
            //infantry.robot_vector.angle = R2DEG_RATIO * atan2f(infantry.set_x_v, infantry.set_y_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;

            arm_sqrt_f32(infantry.set_y_v * infantry.set_y_v + infantry.set_x_v * infantry.set_x_v, &infantry.robot_vector.module); // 计算速度模值
            if(fabsf(infantry.robot_vector.module)<0.001) infantry.target_yaw_v = 0;//改善速度跟随模式
        }
        
        /*平动向量赋值,以底盘建立坐标系*/
        for (int i = 0; i < 4; i++)
        {
            infantry.steer_vector[i].angle = infantry.robot_vector.angle;   // degree 角度转换到底盘坐标系
            infantry.steer_vector[i].module = infantry.robot_vector.module; // m/s
        }

        /*转动向量赋值*/
        Vector w_vector[4];
        
        if (remote_controller.control_mode_action == CV_ROTATE || remote_controller.control_mode_action == FOLLOW_GIMBAL || remote_controller.control_mode_action == NOT_FOLLOW_GIMBAL||remote_controller.control_mode_action == SPEED_FOLLOW)
        {
            // 底盘旋转3508线速度大小赋值 m/s，统一用绝对值，方向由角度控制
            float rotation_speed = fabsf(infantry.target_yaw_v) * STEER_INFANTRY_RADIUS; // 从角速度rad/s到线速度m/s
            w_vector[STEER1].module = rotation_speed;
            w_vector[STEER2].module = rotation_speed;
            w_vector[STEER3].module = rotation_speed;
            w_vector[STEER4].module = rotation_speed;
            // 底盘旋转6020角度朝向赋值 degree，根据旋转方向调整角度
            float angle_offset = (infantry.target_yaw_v >= 0) ? 0.0f : 180.0f; // 反转时角度翻转180度
            w_vector[STEER1].angle = 135.0f + angle_offset;
            w_vector[STEER2].angle = 45.0f + angle_offset;
            w_vector[STEER3].angle = -135.0f + angle_offset;
            w_vector[STEER4].angle = -45.0f + angle_offset;

					
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
						// 加一层td滤波作缓冲
            infantry.Steer_Speed_Setpoint[i] = PID_Calculate(&infantry.steers_angle_pid[i], infantry.sensors_info.steer_decode[i].angle, TD_Calculate(&infantry.steer_angle_td[i], infantry.steer_vector[i].angle));
            infantry.steers_set_current = PID_Calculate(&infantry.steers_speed_pid[i], infantry.sensors_info.steer_decode[i].speed, infantry.Steer_Speed_Setpoint[i]) + Feedforward_Calculate(&infantry.Steer_6020_FF, infantry.Steer_Speed_Setpoint[i]);
            infantry.excute_info.steers_set_current[i] = infantry.steers_set_current;
        }
    }
    else
    {
        for (int m = 0; m < 4; m++)
        {
            infantry.excute_info.steers_set_current[m] = 0.0f;
            infantry.excute_info.wheels_set_current[m] = PID_Calculate(&infantry.wheels_pid[m], infantry.sensors_info.wheels_decode[m].speed, 0.0f);
        }
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
    float wheel_speed_m_s[4];     // 四个轮的物理线速度
    float steer_angle_rad[4];     // 四个舵轮的物理角度

    // 四个舵轮yaw切线方向角度（与逆运动学w_vector.angle一致）
    // STEER1=135°, STEER2=45°, STEER3=-135°, STEER4=-45°
    float tangent_angle_rad[4] = {135.0f * DEG2R_RATIO, 45.0f * DEG2R_RATIO, -135.0f * DEG2R_RATIO, -45.0f * DEG2R_RATIO};

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
    float vx_chassis_sum = 0.0f;
    float vy_chassis_sum = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        vx_chassis_sum += wheel_speed_m_s[i] * arm_sin_f32(steer_angle_rad[i]);
        vy_chassis_sum += wheel_speed_m_s[i] * arm_cos_f32(steer_angle_rad[i]);
    }
    float vx_chassis = vx_chassis_sum / 4.0f;
    float vy_chassis = vy_chassis_sum / 4.0f;

    // 第三步：计算yaw贡献（从轮速中减去平动分量，再投影到切线）
    float yaw_sum = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        // 平动分量：robot_vector 在该舵轮方向的分量
        float robot_speed_contribution = vx_chassis * arm_sin_f32(steer_angle_rad[i]) + vy_chassis * arm_cos_f32(steer_angle_rad[i]);

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
    infantry.x_v = vx_chassis * infantry.cos_dir - vy_chassis * infantry.sin_dir;
    infantry.y_v = vx_chassis * infantry.sin_dir + vy_chassis * infantry.cos_dir;
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
    static uint32_t debug_time_cnt = 0;      // 时间计数器(ms)
    static uint8_t square_state = 0;         // 方波状态：0=低，1=高
    static int16_t init_encoder_value = 0;   // 初始编码器值（原始值，不累积）
    static uint8_t init_flag = 0;            // 初始化标志
    float target_angle;                       // 目标角度
    float current_angle_offset;               // 当前相对偏移角度

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
        square_state = !square_state;  // 翻转方波状态
    }

    // 计算目标角度：基准角度 ± 幅度（相对于初始位置）
    if (square_state == 0)
    {
        target_angle = STEER_DEBUG_ANGLE_BASE - STEER_DEBUG_ANGLE_AMPLITUDE;  // -45°
    }
    else
    {
        target_angle = STEER_DEBUG_ANGLE_BASE + STEER_DEBUG_ANGLE_AMPLITUDE;  // +45°
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
            if (encoder_offset > 4096)       // 超过半圈正向
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
                                                        infantry.Steer_Speed_Setpoint[i])
                                          + Feedforward_Calculate(&infantry.Steer_6020_FF, infantry.Steer_Speed_Setpoint[i]);
            infantry.excute_info.steers_set_current[i] = infantry.steers_set_current;
        }
        else
        {
            // 其他舵轮：输出为0
            infantry.excute_info.steers_set_current[i] = 0.0f;
        }
    }
}
