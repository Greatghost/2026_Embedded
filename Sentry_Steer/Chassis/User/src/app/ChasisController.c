#include "ChasisController.h"

Infantry infantry;
uint8_t speed_follow_enable_flag = 0;

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

#if ROBOT == GOBLIN
float speed_angle_bias = -28.0f;
#elif ROBOT == TIGER
float speed_angle_bias = -90.0f;  // 默认值，SPEED_FOLLOW模式会动态修改为31
#endif
float calculate_velocity_angle() {
    static float last_valid_angle = 0.0f;  // 保存上次有效角度（线程不安全，需根据场景加锁）
    const float VELOCITY_EPSILON = 0.01f; // 速度小量阈值（可配置）

    float x = infantry.set_x_v;
    float y = infantry.set_y_v;
    float magnitude_sq = x * x + y * y;

    if (magnitude_sq < VELOCITY_EPSILON * VELOCITY_EPSILON) {
        // 速度过小，返回上次有效角度（保持角度连续性）
        speed_follow_enable_flag = 0;
        return last_valid_angle;
    } else {
        // 计算并更新有效角度
        float angle_rad = arm_atan2_f32(y, x);  // 注意参数顺序：y（纵坐标）在前，x（横坐标）在后
        float angle_deg = angle_rad * 180.0f / PI;
        last_valid_angle = limit_pi(angle_deg);  // 保存最新有效角度
        speed_follow_enable_flag = 1;
        return last_valid_angle;
    }
}

/**
 * @brief  底盘方向偏差获取
 * @param  目标方向(单位为角度)
 * @retval 方向偏差
 */

float angle_z_err_get(float target_ang, float zeros_angle)
{
    float AngErr_front, AngErr_back, AngErr_left, AngErr_right, minAngle, angleBias = 0.0f;
		
    if(remote_controller.control_mode_action == SPEED_FOLLOW) target_ang_speed = calculate_velocity_angle();  //目标角度为速度方向
    



    // 不同电机的计算不一样，但要保证最终的输出error_angle定义一致
    if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45) // 增加45度计算夹角
    {
        angleBias = 45.0f;
    }
    else angleBias = 0.f;

    // 根据模式动态设置speed_angle_bias
    float current_speed_angle_bias = speed_angle_bias;  // 默认使用全局设置
    #if ROBOT == TIGER
    if(remote_controller.control_mode_action == SPEED_FOLLOW && speed_follow_enable_flag == 1)
    {
        current_speed_angle_bias = 31.0f;  // SPEED_FOLLOW模式使用31
    }
    else
    {
        current_speed_angle_bias = -90.0f;  // 其他模式使用-90
    }
    #endif

     if(remote_controller.control_mode_action == SPEED_FOLLOW && speed_follow_enable_flag == 1)
    //if(remote_controller.control_mode_action == SPEED_FOLLOW ) // 舵轮启动已经优化，此处不需要二者混合
    {
        // if (infantry.yaw_motor_type == YAW_GM6020) {
        //     AngErr_front = limit_pi(zeros_angle / 22.755555556f - target_ang_speed / 22.755555556f + angleBias);
        //     AngErr_back = limit_pi(AngErr_front + 180.0f);
        //     AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
        //     AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
        // }
        if (infantry.yaw_motor_type == YAW_DM_MOTOR) {
            AngErr_front = limit_pi(zeros_angle / YAW_DM_ANGLE_SCALE - target_ang_speed + current_speed_angle_bias);
            AngErr_back = limit_pi(AngErr_front + 180.0f);
            AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
            AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
        }
    }
    else{
        if (infantry.yaw_motor_type == YAW_GM6020)
        {
            AngErr_front = limit_pi(zeros_angle / 22.755555556f - target_ang / 22.755555556f + angleBias);
            AngErr_back = limit_pi(AngErr_front + 180.0f);
            AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
            AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
        }
        else if (infantry.yaw_motor_type == YAW_DM_MOTOR)
        {
            AngErr_front = limit_pi(zeros_angle / YAW_DM_ANGLE_SCALE - target_ang / YAW_DM_ANGLE_SCALE + angleBias);
            AngErr_back = limit_pi(AngErr_front + 180.0f);
            AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
            AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
        }
    }
    

    // 判断跟随
    if (infantry.chassis_follow_type == TWO_SIDES_FOLLOW)
    {
        if (fabs(AngErr_front) > fabs(AngErr_back))
        {
            infantry.chassis_direction = CHASSIS_BACK;
            return AngErr_back;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_FRONT;
            return AngErr_front;
        }
    }
    else if (infantry.chassis_follow_type == TWO_SIDES_LEFT_RIGHT)
    {
        if (fabs(AngErr_left) > fabs(AngErr_right))
        {
            infantry.chassis_direction = CHASSIS_RIGHT;
            return AngErr_right;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_LEFT;
            return AngErr_left;
        }
    }
    else if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW || infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45)
    {
        minAngle = MIN(fabs(AngErr_front), MIN(fabs(AngErr_back), MIN(fabs(AngErr_left), fabs(AngErr_right))));
        if (fabs(fabs(AngErr_front) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_FRONT;
            return AngErr_front;
        }
        else if (fabs(fabs(AngErr_back) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_BACK;
            return AngErr_back;
        }
        else if (fabs(fabs(AngErr_left) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_LEFT;
            return AngErr_left;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_RIGHT;
            return AngErr_right;
        }
    }
    return 0;
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
    // infantry->speed_yaw_max = 10.0f;
    // infantry->target_x_v = TD_Calculate(&infantry->x_v_td, infantry->receive_x_v);
    // infantry->target_y_v = TD_Calculate(&infantry->y_v_td, infantry->receive_y_v);

    infantry->target_x_v = infantry->receive_x_v; //TD控制器有点问题，怎么调参都会导致速度抖动，暂时不使用TD
    infantry->target_y_v = infantry->receive_y_v;


    if (remote_controller.control_mode_action == NOT_FOLLOW_GIMBAL || remote_controller.control_mode_action == CV_ROTATE) // 检录陀螺要变向
    {
        // infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, 2.0f*infantry->receive_yaw_v);
        infantry->target_yaw_v = infantry->receive_yaw_v;
    }
    else
    {
        // infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, infantry->speed_yaw_max);
        infantry->target_yaw_v = infantry->speed_yaw_max;
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
uint32_t online_count;
uint8_t chassis_control_check(){

    //  检查板间通信
    if (offline_detector.comm_state[0] == COMM_OFF) 
    {
        online_count = 0;
        return 0;
    }
    // 检查电机上电情况,off_time只有电机通信在的时候会清零
    for(int i=0;i<4;i++)
    {
        offline_detector.wheel_3508_off_time[i]++;
        if(offline_detector.wheel_3508_off_time[i] > 50)
        {
            online_count = 0;
            return 0;
        }
        if(infantry.chassis_type == STEER_WHEEL)
        {
            offline_detector.steer_6020_off_time[i]++;
            if(offline_detector.steer_6020_off_time[i] > 50)
            {
                online_count = 0;
                return 0;
            }
        }
        
    }
    online_count++;
    // 过检测超过500ms，可以底盘上电跑
    if(online_count > 500)return 1;
    else return 0;
}

void execute_control(ExcuteTorque *torque)
{	
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
        // 底盘6020电机掉线处理，如果6020掉了，给对应的3508发0。
        for (int i = 0; i < 4; i++)
        {
            if (offline_detector.steer_6020_state[i] == STEER_6020_OFF)
                infantry.excute_info.wheels_set_current[i] = 0.0f;
        }
        uint8_t if_can_control  = 	chassis_control_check();
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
		uint8_t if_can_control  = 	chassis_control_check();
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
