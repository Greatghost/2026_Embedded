#include "ChasisController.h"

Infantry infantry;
uint8_t speed_follow_enable_flag = 0;
static float last_valid_velocity_angle = 0.0f;

/*
 * Feedback recovery is deliberately separated from actuator recovery:
 *   1. all required feedback must stay healthy for CHASSIS_RECOVERY_COUNT;
 *   2. commands and final motor currents then ramp from 0 to 100%.
 */
static float chassis_recovery_scale = 0.0f;
static void chassis_pid_integral_clear(void);
static void chassis_control_state_clear(void);

void InfantryInit(Infantry *infantry)
{
    // 功率控制初始化
    PowerLimitInit(&infantry->power_limiter, 4, M3508, infantry->power_limit_method);

	//加速快一点
	TD_Init(&infantry->x_v_td, 8000, 0.02);//0.5上升
    TD_Init(&infantry->y_v_td, 8000, 0.02);
    TD_Init(&infantry->yaw_v_td, 5000, 0.03);
	
//	TD_Init(&infantry->x_v_td, 10000, 0.02);//0.5上升
//    TD_Init(&infantry->y_v_td, 10000, 0.02);
//    TD_Init(&infantry->yaw_v_td, 10000, 0.03);
}

/**
 * @brief  转角限制到±180度
 * @param  输入转角
 * @retval 输出转角
 */
float limit_pi(float in)
{
    while (in < -180.0f || in > 180.0f)
    {
        if (fabs(in - 180.0f) < 1e-4)
        {
            in = 180.0f;
            break;
        }
        else if (in < -180.0f)
        {
            in = in + 360.0f;
        }
        else if (in > 180.0f)
        {
            in = in - 360.0f;
        }
    }
    return in;
}

/**
 * @brief 计算速度矢量的方向角度（单位：度）
 * @return 方向角度，范围 [-180, 180]
 */
float target_ang_speed = 0.0f;

#define SPEED_FOLLOW_VELOCITY_EPSILON 0.01f
#define CHASSIS_DIRECTION_SWITCH_HYSTERESIS_DEG 8.0f

#if ROBOT == GOBLIN
float speed_angle_bias = -28.0f;
#elif ROBOT == TIGER
float speed_angle_bias = -90.0f;
#endif
float calculate_velocity_angle(void)
{
    /*
     * set_x_v/set_y_v 是已经按当前云台/底盘夹角变换后的底盘系速度。
     * SPEED_FOLLOW 必须使用这个速度矢量方向作为闭环目标；不能替换成
     * 云台 yaw 编码器角度 target_ang。
     */
    float x = infantry.set_x_v;
    float y = infantry.set_y_v;
    float magnitude_sq = x * x + y * y;
    float velocity_epsilon_sq =
        SPEED_FOLLOW_VELOCITY_EPSILON * SPEED_FOLLOW_VELOCITY_EPSILON;

    if (magnitude_sq < velocity_epsilon_sq)
    {
        speed_follow_enable_flag = 0U;
        return last_valid_velocity_angle;
    }

    last_valid_velocity_angle =
        limit_pi(arm_atan2_f32(y, x) * 180.0f / PI);
    speed_follow_enable_flag = 1U;
    return last_valid_velocity_angle;
}

void chassis_manual_takeover_reset(void)
{
    /*
     * A PC-to-remote handover must not inherit the PC velocity direction,
     * direction hysteresis or controller history. Reinitializing these states
     * is the runtime equivalent of the reboot that previously cleared the bug.
     */
    taskENTER_CRITICAL();
    last_valid_velocity_angle = 0.0f;
    target_ang_speed = 0.0f;
    speed_follow_enable_flag = 0U;
    infantry.chassis_direction = CHASSIS_FRONT;
    chassis_control_state_clear();
    taskEXIT_CRITICAL();
}

/**
 * @brief  底盘方向偏差获取
 * @param  目标方向(单位为角度)
 * @retval 方向偏差
 */

float angle_z_err_get(float target_ang, float zeros_angle)
{
    float errors[4];
    uint8_t allowed[4] = {0U, 0U, 0U, 0U};
    float angle_bias =
        (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45) ? 45.0f : 0.0f;
    float yaw_scale =
        (infantry.yaw_motor_type == YAW_GM6020)
            ? 22.755555556f
            : YAW_DM_ANGLE_SCALE;
    float front_error;
    chassis_direction_e best_direction = CHASSIS_FRONT;
    float best_abs;

    if (remote_controller.control_mode_action == SPEED_FOLLOW)
        target_ang_speed = calculate_velocity_angle();
    else
        speed_follow_enable_flag = 0U;

    /*
     * SPEED_FOLLOW 的目标是速度矢量方向（单位已经是度）；其他跟随模式
     * 使用云台编码器角度。两种 yaw 电机仅编码器比例不同。
     */
    if (remote_controller.control_mode_action == SPEED_FOLLOW &&
        speed_follow_enable_flag != 0U)
    {
        front_error = limit_pi(zeros_angle / yaw_scale -
                               target_ang_speed +
                               speed_angle_bias +
                               angle_bias);
    }
    else
    {
        front_error = limit_pi(zeros_angle / yaw_scale -
                               target_ang / yaw_scale +
                               angle_bias);
    }

    errors[CHASSIS_FRONT] = front_error;
    errors[CHASSIS_BACK] = limit_pi(front_error + 180.0f);
    errors[CHASSIS_LEFT] =
        limit_pi(front_error + GIMBAL_MOTOR_SIGN * 90.0f);
    errors[CHASSIS_RIGHT] =
        limit_pi(front_error - GIMBAL_MOTOR_SIGN * 90.0f);

    if (infantry.chassis_follow_type == TWO_SIDES_FOLLOW)
    {
        allowed[CHASSIS_FRONT] = 1U;
        allowed[CHASSIS_BACK] = 1U;
    }
    else if (infantry.chassis_follow_type == TWO_SIDES_LEFT_RIGHT)
    {
        allowed[CHASSIS_LEFT] = 1U;
        allowed[CHASSIS_RIGHT] = 1U;
        best_direction = CHASSIS_LEFT;
    }
    else
    {
        for (int i = 0; i < 4; i++)
            allowed[i] = 1U;
    }

    best_abs = fabsf(errors[best_direction]);
    for (int i = 0; i < 4; i++)
    {
        if (allowed[i] != 0U && fabsf(errors[i]) < best_abs)
        {
            best_abs = fabsf(errors[i]);
            best_direction = (chassis_direction_e)i;
        }
    }

    /*
     * 当前方向只要没有比最佳方向差 8° 以上就继续保持，避免在四个边的
     * 分界处频繁切换，引发所有舵轮目标同时跳变。
     */
    if ((uint32_t)infantry.chassis_direction < 4U &&
        allowed[infantry.chassis_direction] != 0U &&
        fabsf(errors[infantry.chassis_direction]) <=
            best_abs + CHASSIS_DIRECTION_SWITCH_HYSTERESIS_DEG)
    {
        best_direction = infantry.chassis_direction;
    }

    infantry.chassis_direction = best_direction;
    return errors[best_direction];
}

// 获取控制方向
void getDir()
{
    if (infantry.yaw_motor_type == YAW_GM6020)
    {
        // 计算与正对情况的夹角，22.755555556f = 8192 / 360.0f，结果转弧度
        float AngErr_front = limit_pi(GIMBAL_FOLLOW_ZERO / 22.755555556f - gimbal_receiver_pack1.yaw_motor_angle / 22.755555556f) * ANGLE_TO_RAD_COEF;
        infantry.sin_dir = arm_sin_f32(AngErr_front);
        infantry.cos_dir = arm_cos_f32(AngErr_front);
			
				UI_FRONT_ERR = limit_pi((GIMBAL_FOLLOW_ZERO + UI_FRONT_BIAS) / 22.755555556f - gimbal_receiver_pack1.yaw_motor_angle / 22.755555556f) * ANGLE_TO_RAD_COEF;
				UI_FRONT_SIN = arm_sin_f32(UI_FRONT_ERR);
				UI_FRONT_COS = arm_cos_f32(UI_FRONT_ERR);
    }
    else if (infantry.yaw_motor_type == YAW_DM_MOTOR)
    {
        float AngErr_front = limit_pi(GIMBAL_FOLLOW_ZERO / YAW_DM_ANGLE_SCALE - gimbal_receiver_pack1.yaw_motor_angle / YAW_DM_ANGLE_SCALE) * ANGLE_TO_RAD_COEF;
        infantry.sin_dir = arm_sin_f32(AngErr_front);
        infantry.cos_dir = arm_cos_f32(AngErr_front);
			
				UI_FRONT_ERR = limit_pi((GIMBAL_FOLLOW_ZERO + UI_FRONT_BIAS) / YAW_DM_ANGLE_SCALE - gimbal_receiver_pack1.yaw_motor_angle / YAW_DM_ANGLE_SCALE) * ANGLE_TO_RAD_COEF;
				UI_FRONT_SIN = arm_sin_f32(UI_FRONT_ERR);
				UI_FRONT_COS = arm_cos_f32(UI_FRONT_ERR);
    }
}

void get_sensors_info(Sensors *sensors_info)
{
    if (infantry.chassis_type == STEER_WHEEL)
    {
        for (int i = 0; i < 4; i++)
        {
            // 舵电机解码，速度没经过滤波
            GM6020_Decode(&sensors_info->steer_recv[i], &sensors_info->steer_decode[i]);

            // 轮电机解码
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
        }
    }
    // 麦轮和全向轮无舵向电机
    else if (infantry.chassis_type == MECANUM_WHEEL || infantry.chassis_type == OMNI_WHEEL)
    {
        for (int i = 0; i < 4; i++)
        {
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
        }
    }

    infantry.error_angle = angle_z_err_get(gimbal_receiver_pack1.yaw_motor_angle, GIMBAL_FOLLOW_ZERO) * ANGLE_TO_RAD_COEF;
    getDir();
}

void wheels_power_limit(Infantry *infantry)
{
    // 功率控制
    // limiter赋值
    float w_error = 0.0f;
    float steer_w_error = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        infantry->power_limiter.wheels_scaler.motor_w[i] = infantry->sensors_info.wheels_decode_raw[i].speed * ANGLE_TO_RAD_COEF;
        infantry->power_limiter.Steer_scaler.motor_w[i] = infantry->sensors_info.steer_decode[i].speed * ANGLE_TO_RAD_COEF;

        //设置为电机反馈可作拟合
//        infantry->power_limiter.wheels_scaler.motor_I[i] = infantry->sensors_info.wheels_decode_raw[i].torque_current;
//        if(infantry->chassis_type == STEER_WHEEL)
//        {
//            infantry->power_limiter.Steer_scaler.motor_I[i] = infantry->sensors_info.steer_decode[i].torque_current;
//        }
         



//        // // 设置为发送电流可做削减
        infantry->power_limiter.wheels_scaler.motor_I[i] = infantry->excute_info.wheels_set_current[i] / C620_CURRENT_SEND_TRANS;
        

        // 设置实际轮子转速与设定转速差
        w_error = fabs(infantry->wheels_set_v[i] - infantry->sensors_info.wheels_decode[i].speed) * ANGLE_TO_RAD_COEF;
        

        infantry->power_limiter.wheels_scaler.motor_w_error[i] = w_error * w_error; // 功率分配函数
        

        if(infantry->chassis_type == STEER_WHEEL)
        {
//            //  // 电流削减，拟合时需注释第一行
           infantry->power_limiter.Steer_scaler.motor_I[i] = infantry->excute_info.steers_set_current[i] / GM6020_CURRENT_SEND_TRANS;
            steer_w_error = fabs(infantry->Steer_Speed_Setpoint[i] - infantry->sensors_info.steer_decode[i].speed) * ANGLE_TO_RAD_COEF;
            infantry->power_limiter.Steer_scaler.motor_w_error[i] = steer_w_error * steer_w_error;
        }
    }

    // 限制功率

    PowerLimit(&infantry->power_limiter, infantry->set_power);
    // PowerLimit(&infantry->power_limiter, 80);

    // 作功率削减
    for (int i = 0; i < 4; i++)
    {
        infantry->excute_info.wheels_set_current[i] *= infantry->power_limiter.wheels_scaler.send_torque_lower_scale[i];
        infantry->wheels_send_current[i] = infantry->excute_info.wheels_set_current[i];
        if(infantry->chassis_type == STEER_WHEEL)
        {infantry->excute_info.steers_set_current[i] *= infantry->power_limiter.Steer_scaler.send_torque_lower_scale[i];}
    }
}

// 设置机器人功率以及控制其速度
void set_robot_speed(Infantry *infantry)
{
    infantry->set_power = cap_controller.set_power;

    // 根据设置的功率计算出设定速度，注意保持speed_x_max 与 speed_y_max 与 speed_yaw_max * wheel_radius基本同值
    // 因为该参数需要关联到功率控制部分，所以要保证在跑满功率的前提下给大，但过大会导致部分机器人轮子打滑，所以需要控制
    if (infantry->chassis_type == STEER_WHEEL)
    {
        infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.06f + 4.8f;
        infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.06f + 4.8f;
        infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.21f + 16.0f;
    }
    else if (infantry->chassis_type == MECANUM_WHEEL)
    {
        infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.06f + 4.8f; // 因为后面还有功率控制，这个设定值本质上是为了控制不同功率速度的大致给定
        infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.06f + 4.8f;
        infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.21f + 16.0f;
    }
    else if (infantry->chassis_type == OMNI_WHEEL)
    {
			infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.06f + 4.8f; // 经测试，不同功率时，平地、上坡以及飞坡均能吃满set_power
        infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.06f + 4.8f;
        infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.21f + 16.0f;
    }
}

// 加速策略
void wheels_accel(Infantry *infantry)
{
    /*
     * Do not let integral terms accumulate while the final actuator command is
     * intentionally attenuated by the recovery ramp.
     */
    if (chassis_recovery_scale < 1.0f)
    {
        chassis_pid_integral_clear();
    }

    // infantry->speed_yaw_max = 10.0f;
    // infantry->target_x_v = TD_Calculate(&infantry->x_v_td, infantry->receive_x_v);
    // infantry->target_y_v = TD_Calculate(&infantry->y_v_td, infantry->receive_y_v);

    infantry->target_x_v = infantry->receive_x_v * chassis_recovery_scale; //TD控制器有点问题，怎么调参都会导致速度抖动，暂时不使用TD
    infantry->target_y_v = infantry->receive_y_v * chassis_recovery_scale;


    if (remote_controller.control_mode_action == NOT_FOLLOW_GIMBAL || remote_controller.control_mode_action == CV_ROTATE) // 检录陀螺要变向
    {
        // infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, 2.0f*infantry->receive_yaw_v);
        infantry->target_yaw_v = infantry->receive_yaw_v * chassis_recovery_scale;
    }
    else
    {
        // infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, infantry->speed_yaw_max);
        infantry->target_yaw_v = infantry->speed_yaw_max * chassis_recovery_scale;
    }
}

void chassis_powerdown_control(Infantry *infantry)
{
    infantry->set_x_v = 0;
    infantry->set_y_v = 0;
    infantry->set_yaw_v = 0;
    infantry->target_x_v = 0;
    infantry->target_y_v = 0;
    infantry->target_yaw_v = 0;

    for (int i = 0; i < 4; i++)
    {
        infantry->excute_info.steers_set_current[i] = 0;
        infantry->excute_info.wheels_set_current[i] = 0;
    }

    TD_Clear(&infantry->x_v_td, 0);
    TD_Clear(&infantry->y_v_td, 0);
    TD_Clear(&infantry->yaw_v_td, 0);
}

void chassis_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_follow_control();
        break;
    case OMNI_WHEEL:
        omni_follow_control();
        break;
    case STEER_WHEEL:
        steer_chassis_control();
        break;
    default:
        break;
    }
}

void chassis_not_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_chassis_control();
        break;
    case OMNI_WHEEL:
        omni_chassis_control();
        break;
    case STEER_WHEEL:
        steer_chassis_control();
        break;
    default:
        break;
    }
}

void chassis_rotate_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_rotate_control();
        break;
    case STEER_WHEEL:
        steer_chassis_control();
        break;
    case OMNI_WHEEL:
        omni_rotate_control();
        break;
    default:
        break;
    }
}

void chassis_speed_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        //mecanum_rotate_control();
        break;
    case STEER_WHEEL:
        steer_chassis_control();
        break;
    case OMNI_WHEEL:
        omni_speed_follow_control();
        break;
    default:
        break;
    }
}

void main_control(Infantry *infantry)
{
    switch (remote_controller.control_mode_action)
    {
    case FOLLOW_GIMBAL:
        chassis_follow_control(infantry);
        break;
    case NOT_FOLLOW_GIMBAL:
        chassis_not_follow_control(infantry);
        break;
    case CV_ROTATE:
        chassis_rotate_control(infantry);
        break;
    case SPEED_FOLLOW:
        chassis_speed_follow_control(infantry);
        break;

    default:
        chassis_powerdown_control(infantry);
        break;
    }
}
#define CHASSIS_FEEDBACK_TIMEOUT_COUNT 50
#define CHASSIS_RECOVERY_COUNT         500U
#define CHASSIS_SOFT_START_COUNT       500U

uint32_t online_count;
static uint32_t chassis_soft_start_count;

static void chassis_pid_integral_clear(void)
{
    infantry.turn_pid.Iout = 0.0f;
    infantry.turn_pid.ITerm = 0.0f;
    infantry.turn_pid.Last_ITerm = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        infantry.wheels_pid[i].Iout = 0.0f;
        infantry.wheels_pid[i].ITerm = 0.0f;
        infantry.wheels_pid[i].Last_ITerm = 0.0f;

        if (infantry.chassis_type == STEER_WHEEL)
        {
            infantry.steers_angle_pid[i].Iout = 0.0f;
            infantry.steers_angle_pid[i].ITerm = 0.0f;
            infantry.steers_angle_pid[i].Last_ITerm = 0.0f;
            infantry.steers_speed_pid[i].Iout = 0.0f;
            infantry.steers_speed_pid[i].ITerm = 0.0f;
            infantry.steers_speed_pid[i].Last_ITerm = 0.0f;
        }
    }

    #ifdef STEER_TORQUE_FEEDFORWARD
    /* 清零整车扭矩前馈PID积分项，与轮PID同步清零 */
    infantry.chassis_translate_x_pid.Iout = 0.0f;
    infantry.chassis_translate_x_pid.ITerm = 0.0f;
    infantry.chassis_translate_x_pid.Last_ITerm = 0.0f;
    infantry.chassis_translate_y_pid.Iout = 0.0f;
    infantry.chassis_translate_y_pid.ITerm = 0.0f;
    infantry.chassis_translate_y_pid.Last_ITerm = 0.0f;
    infantry.chassis_rotate_pid.Iout = 0.0f;
    infantry.chassis_rotate_pid.ITerm = 0.0f;
    infantry.chassis_rotate_pid.Last_ITerm = 0.0f;
    for (int i = 0; i < 4; i++)
        infantry.wheels_ff_current[i] = 0.0f;
    #endif
}

static void chassis_recovery_reset(void)
{
    online_count = 0U;
    chassis_soft_start_count = 0U;
    chassis_recovery_scale = 0.0f;
    chassis_control_state_clear();
}

static void chassis_control_state_clear(void)
{
    infantry.set_x_v = 0.0f;
    infantry.set_y_v = 0.0f;
    infantry.set_yaw_v = 0.0f;
    infantry.target_x_v = 0.0f;
    infantry.target_y_v = 0.0f;
    infantry.target_yaw_v = 0.0f;

    PID_Clear(&infantry.turn_pid);
    TD_Clear(&infantry.x_v_td, 0.0f);
    TD_Clear(&infantry.y_v_td, 0.0f);
    TD_Clear(&infantry.yaw_v_td, 0.0f);

    for (int i = 0; i < 4; i++)
    {
        infantry.wheels_set_v[i] = 0.0f;
        infantry.excute_info.wheels_set_current[i] = 0.0f;
        infantry.excute_info.steers_set_current[i] = 0.0f;
        PID_Clear(&infantry.wheels_pid[i]);

        if (infantry.chassis_type == STEER_WHEEL)
        {
            infantry.Steer_Speed_Setpoint[i] = 0.0f;
            PID_Clear(&infantry.steers_angle_pid[i]);
            PID_Clear(&infantry.steers_speed_pid[i]);
            TD_Clear(&infantry.steer_angle_td[i],
                     infantry.sensors_info.steer_decode[i].angle);
        }
    }

    if (infantry.chassis_type == STEER_WHEEL)
    {
        for (int i = 0; i < 4; i++)
            Feedforward_Clear(&infantry.Steer_6020_FF[i]);
        steer_control_state_reset();
    }
    else if (infantry.chassis_type == MECANUM_WHEEL)
    {
        Feedforward_Clear(&infantry.Mecanum_Follow_FF);
    }
}

static uint8_t feedback_counter_expired(volatile int16_t *off_time)
{
    int16_t feedback_age;

    /* CAN接收中断会把计数清零；在上限处饱和，避免int16_t溢出后误判在线。 */
    /* 临界区避免任务的读-改-写覆盖CAN中断刚写入的0。 */
    taskENTER_CRITICAL();
    feedback_age = *off_time;
    if (feedback_age <= CHASSIS_FEEDBACK_TIMEOUT_COUNT)
    {
        feedback_age++;
        *off_time = feedback_age;
    }
    taskEXIT_CRITICAL();

    return (feedback_age > CHASSIS_FEEDBACK_TIMEOUT_COUNT);
}

uint8_t chassis_control_check()
{
    /* 板间控制帧与电机反馈任一超过约50 ms未更新，立即保持零电流。 */
    if (feedback_counter_expired(&offline_detector.gimbal_comm_off_time))
    {
        chassis_recovery_reset();
        return 0;
    }

    for (int i = 0; i < 4; i++)
    {
        if (feedback_counter_expired(&offline_detector.wheel_3508_off_time[i]))
        {
            chassis_recovery_reset();
            return 0;
        }

        if (infantry.chassis_type == STEER_WHEEL &&
            feedback_counter_expired(&offline_detector.steer_6020_off_time[i]))
        {
            chassis_recovery_reset();
            return 0;
        }
    }

    /* 所有反馈连续正常约500 ms后再恢复，避免接触不良时反复启停。 */
    if (online_count <= CHASSIS_RECOVERY_COUNT)
    {
        online_count++;
    }

    if (online_count <= CHASSIS_RECOVERY_COUNT)
    {
        chassis_soft_start_count = 0U;
        chassis_recovery_scale = 0.0f;
        chassis_control_state_clear();
        return 0;
    }

    /*
     * The first enabled cycle is still exactly zero.  Subsequent healthy
     * cycles ramp linearly to full command in approximately 500 ms.
     */
    if (chassis_soft_start_count < CHASSIS_SOFT_START_COUNT)
    {
        chassis_recovery_scale =
            (float)chassis_soft_start_count / (float)CHASSIS_SOFT_START_COUNT;
        chassis_soft_start_count++;
    }
    else
    {
        chassis_recovery_scale = 1.0f;
    }

    return 1;
}

void execute_control(ExcuteTorque *torque)
{
	uint8_t if_can_control = chassis_control_check();
	if (if_can_control == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			torque->steers_set_current[i] = 0.0f;
			torque->wheels_set_current[i] = 0.0f;
		}
	}
	else
	{
		/* Final safety ramp applies identically to debug and normal builds. */
		for (int i = 0; i < 4; i++)
		{
			torque->steers_set_current[i] *= chassis_recovery_scale;
			torque->wheels_set_current[i] *= chassis_recovery_scale;
		}
	}

	#if CHASSIS_DEBUG ==1
	if (infantry.chassis_type == STEER_WHEEL)
    {
        // 底盘6020电机掉线处理，如果6020掉了，给对应的3508发0。
        for (int i = 0; i < 4; i++)
        {
//            if (offline_detector.steer_6020_state[i] == STEER_6020_OFF)
//                infantry.excute_info.wheels_set_current[i] = 0.0f;
        }
       
        // 舵
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_1 - 0x204, (int16_t)torque->steers_set_current[0], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_2 - 0x204, (int16_t)torque->steers_set_current[1], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_3 - 0x204, (int16_t)torque->steers_set_current[2], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_4 - 0x204, (int16_t)torque->steers_set_current[3], GM6020_CUR_MODE);
        
        CanSend(DJI_STEERS_CAN, torque->steers_send_data, GM6020_STD_CUR_ID_1_4, 8);

//        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
        
        
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
    else if (infantry.chassis_type == MECANUM_WHEEL || infantry.chassis_type == OMNI_WHEEL)
    {
        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
		
                
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
	#else
    if (infantry.chassis_type == STEER_WHEEL)
    {
        // 舵
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_1 - 0x204, (int16_t)torque->steers_set_current[0], GM6020_CUR_MODE);
        //GM6020_SendPack(0, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_2 - 0x204, (int16_t)torque->steers_set_current[1], GM6020_CUR_MODE);
				GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_2 - 0x204, (int16_t)torque->steers_set_current[1], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_3 - 0x204, (int16_t)torque->steers_set_current[2], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_4 - 0x204, (int16_t)torque->steers_set_current[3], GM6020_CUR_MODE);
        if (if_can_control == 0)
			{
			GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_1 - 0x204, 0, GM6020_CUR_MODE);
            GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_2 - 0x204, 0, GM6020_CUR_MODE);
            GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_3 - 0x204, 0, GM6020_CUR_MODE);
            GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_4 - 0x204, 0, GM6020_CUR_MODE);
	
            }
        CanSend(DJI_STEERS_CAN, torque->steers_send_data, GM6020_STD_CUR_ID_1_4, 8);

        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);

        
        if (if_can_control == 0)
			{
				M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, 0, SEND_CURRENT);
                M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, 0, SEND_CURRENT);
                M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, 0, SEND_CURRENT);
                M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, 0, SEND_CURRENT);
			}
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
    else if (infantry.chassis_type == MECANUM_WHEEL || infantry.chassis_type == OMNI_WHEEL)
    {
        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
        if (if_can_control == 0)
			{
				M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, 0, SEND_CURRENT);
                M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, 0, SEND_CURRENT);
                M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, 0, SEND_CURRENT);
                M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, 0, SEND_CURRENT);
			}
                
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
	#endif
}
