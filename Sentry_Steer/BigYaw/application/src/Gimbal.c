#include "Gimbal.h"
#include "robot_config.h"

GimbalController gimbal_controller;

FIRFilter big_yaw_fir_filter;
uint8_t gimbal_enable_flag;
uint16_t gimbal_enable_cnt = 0;
#define GIMBAL_INIT_WAIT_TIME  150

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit()
{

#if ROBOT == GOBLIN
    // pitch VOL LOOP
    PID_Init(&gimbal_controller.pitch_angle_pid, 500.0f, 0, 0.1f, 22.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Init(&gimbal_controller.pitch_speed_pid, 5000, 1600, 0.0f, 20.0f, 2.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
    // yaw GM6020 CURRENT LOOP
	PID_Init(&gimbal_controller.small_yaw_angle_pid, 900.0, 0, 0.1f, 28.0f, 0, 0.15f, 0, 0, 0.0, 0.0f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.small_yaw_speed_pid, GM6020_MAX_CURRENT, 4800, 0.1f, 110.0f, 1.0f, 0, 0, 0, 0.f, 0, 1, Integral_Limit | Trapezoid_Intergral);
    // yaw DM MOTOR CURRENT LOOP
	PID_Init(&gimbal_controller.big_yaw_angle_pid, 180.0, 0, 0, 3.0f, 0.f, 0.6f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.big_yaw_speed_pid, 1000, 300, 0.0, 15.0f, 2.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // 跟踪微分器
//    TD_Init(&gimbal_controller.pos_big_yaw_td, 10000, 0.01);
//    TD_Init(&gimbal_controller.speed_big_yaw_td, 90000, 0.01);
//    TD_Init(&gimbal_controller.pos_small_yaw_td, 30000, 0.003);
//    TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);
	TD_Init(&gimbal_controller.pos_big_yaw_td, 20000, 0.01);
    TD_Init(&gimbal_controller.speed_big_yaw_td, 90000, 0.01);
    TD_Init(&gimbal_controller.pos_small_yaw_td, 10000, 0.01);
    TD_Init(&gimbal_controller.speed_small_yaw_td, 90000, 0.01);

    float small_yaw_speed_ff_c[3] = {0.4f, 0.1f, 0.0f};//小yaw前馈参数向量
    // Feedforward_Init( Feedforward_t,float max_out, float *c, float lpf_rc, uint16_t ref_dot_ols_order, uint16_t ref_ddot_ols_orde)
    Feedforward_Init(&gimbal_controller.small_yaw_speed_forward,500.0f, small_yaw_speed_ff_c, 0.01f, 5, 5);
    // 云台、机械臂的最小二乘法阶数经验值通常为3~5，LPF_rc在0.01~0.05

    float big_yaw_angle_ff_c[3] = {0.f, 0.15f, 0.0f};//大yaw前馈参数向量
    Feedforward_Init(&gimbal_controller.big_yaw_angle_forward,100.0f, big_yaw_angle_ff_c, 0.01f, 5, 5);
	float big_yaw_speed_ff_c[3] = {1.0f, 0.4f, 0.0f};
    Feedforward_Init(&gimbal_controller.big_yaw_speed_forward,500.0f, big_yaw_speed_ff_c, 0.01f, 5, 5);

    // 滤波器，阶数，截止频率，采样频率,FIR有bug
   // FIRFilter_Init(&big_yaw_fir_filter, 10, 100, 500);
#endif
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
float Gimbal_Pitch_Calculate(float set_point)
{
    // pitch 三环
    //gimbal_controller.set_pitch_angle = set_point;
		iir(&set_point,gimbal_controller.set_pitch_angle,0.8);
    gimbal_controller.set_pitch_speed = PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, set_point);
    gimbal_controller.set_pitch_current = GIMBAL_PITCH_MOTOR_SIGN * PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.set_pitch_speed);
    return gimbal_controller.set_pitch_current;
}



//新的大小yaw控制方法，大yaw带着小yaw转，然后大yaw稍微超调一点点，在末程继续给小yaw力矩
//暂时先用分阶段控制的方法
float Gimbal_Small_Yaw_Calculate(float set_point)
{
    if(gimbal_controller.small_yaw_recv.angle > GIMBAL_SMALL_YAW_LIMIT_RIGHT && gimbal_controller.small_yaw_recv.angle < GIMBAL_SMALL_YAW_LIMIT_LEFG)
    {
     gimbal_controller.set_small_yaw_angle = TD_Calculate(&gimbal_controller.pos_small_yaw_td, set_point);
//去掉角度环前馈        //gimbal_controller.set_small_yaw_speed = PID_Calculate(&gimbal_controller.small_yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point) + GIMBAL_SMALL_YAW_POS_FORWARD_COEF * gimbal_controller.pos_small_yaw_td.dx;
        gimbal_controller.set_small_yaw_speed = PID_Calculate(&gimbal_controller.small_yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point);

		TD_Calculate(&gimbal_controller.speed_small_yaw_td, gimbal_controller.set_small_yaw_speed);
        //速度环增设前馈
        gimbal_controller.set_small_yaw_current = GIMBAL_SMALL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.small_yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_small_yaw_speed) + Feedforward_Calculate(&gimbal_controller.small_yaw_speed_forward,gimbal_controller.set_small_yaw_speed));
        gimbal_controller.set_small_yaw_current = LIMIT_MAX_MIN(gimbal_controller.set_small_yaw_current,GM6020_MAX_CURRENT,-GM6020_MAX_CURRENT);
        return gimbal_controller.set_small_yaw_current;

    }
    else{
					gimbal_controller.set_small_yaw_angle = TD_Calculate(&gimbal_controller.pos_small_yaw_td, set_point);
//去掉角度环前馈        //gimbal_controller.set_small_yaw_speed = PID_Calculate(&gimbal_controller.small_yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point) + GIMBAL_SMALL_YAW_POS_FORWARD_COEF * gimbal_controller.pos_small_yaw_td.dx;
        gimbal_controller.set_small_yaw_speed = PID_Calculate(&gimbal_controller.small_yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point);

		TD_Calculate(&gimbal_controller.speed_small_yaw_td, gimbal_controller.set_small_yaw_speed);
        //速度环增设前馈
        gimbal_controller.set_small_yaw_current = GIMBAL_SMALL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.small_yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_small_yaw_speed) + Feedforward_Calculate(&gimbal_controller.small_yaw_speed_forward,gimbal_controller.set_small_yaw_speed));
        gimbal_controller.set_small_yaw_current = 0;//卸力
			return gimbal_controller.set_small_yaw_current;
    }
    
}

uint8_t bigyaw_turn_flag = 0,bigyaw_is_turning_flag = 0;
float small_yaw_IMU_err,small_yaw_motor_err;
ZeroCheck_Typedef big_yaw_angle_zero_check;
float big_yaw_angle_after_zero_check;
float angle_diff;

void BigYawZeroCheck(void)
{
    big_yaw_angle_after_zero_check = ZeroCheck(&big_yaw_angle_zero_check,gimbal_controller.DM_Big_Yaw_Motor.P_Receive,360.0f);
}
//此处过零检测放到接收函数
void BigYawTurnCheck(void)
{	
	small_yaw_IMU_err = gimbal_controller.target_small_yaw_angle - gimbal_controller.gyro_yaw_angle ;//小yaw的陀螺仪差值角
	small_yaw_motor_err = (float)(gimbal_controller.small_yaw_recv.angle - GIMBAL_SMALL_YAW_ZERO_POINT)*360.0f/8192.0f;//小yaw的电机误差角
}

float current_angle_big;
float target_angle_big;
float dynamic_angle_big;//根据小yaw的角度差值动态设置大yaw的设定角度，包括电机角和陀螺仪角
void BigYawSetpointSet(void){
	current_angle_big = big_yaw_angle_after_zero_check;//gimbal_controller.DM_Big_Yaw_Motor.P_Receive; // 当前电机位置	
	dynamic_angle_big = 1.6f * small_yaw_IMU_err + 0.6f * small_yaw_motor_err; //线性相加，避免大yaw异动
	target_angle_big = current_angle_big + dynamic_angle_big; // 根据小yaw转角实时调大yaw
	// 设置目标角度
	gimbal_controller.target_big_yaw_angle = target_angle_big;
	angle_diff = fabsf(target_angle_big - current_angle_big);

}



float big_yaw_gyro_raw;
float big_yaw_speed_raw;
float big_yaw_gyro_bias = 0.0f;
float dealed_big_yaw_gyro;
uint8_t big_yaw_mode;
float Gimbal_Big_Yaw_Gyro_cal(float set_point)
{
    if(fabs(dealed_big_yaw_gyro-(gimbal_controller.gyro_yaw_angle+small_yaw_motor_err))>3.5f)
    {
        big_yaw_gyro_bias +=dealed_big_yaw_gyro-(gimbal_controller.gyro_yaw_angle+small_yaw_motor_err);
    }
    dealed_big_yaw_gyro = big_yaw_gyro_raw + big_yaw_gyro_bias;//融合上下板陀螺仪零漂
    if (big_yaw_mode == 0)//跟随模式
    {
        gimbal_controller.target_big_yaw_angle = dealed_big_yaw_gyro + small_yaw_motor_err;
        gimbal_controller.set_big_yaw_angle = TD_Calculate(&gimbal_controller.pos_big_yaw_td, gimbal_controller.target_big_yaw_angle);
        gimbal_controller.set_big_yaw_speed = PID_Calculate(&gimbal_controller.big_yaw_angle_pid, dealed_big_yaw_gyro,gimbal_controller.set_big_yaw_angle) + Feedforward_Calculate(&gimbal_controller.big_yaw_angle_forward,gimbal_controller.set_big_yaw_angle);
        TD_Calculate(&gimbal_controller.speed_big_yaw_td, gimbal_controller.set_big_yaw_speed);
        //此处也需要增加一个前馈，用以处理小陀螺状态
        gimbal_controller.set_big_yaw_current = GIMBAL_BIG_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.big_yaw_speed_pid, big_yaw_speed_raw, gimbal_controller.set_big_yaw_speed) + Feedforward_Calculate(&gimbal_controller.big_yaw_speed_forward,gimbal_controller.set_big_yaw_speed));
        if(gimbal_controller.small_yaw_recv.angle != 0)//防止小yaw掉线，大yaw疯转
        {
            return gimbal_controller.set_big_yaw_current;
        }
        else return 0.f;
    }
    else{//大yaw不动
        gimbal_controller.target_big_yaw_angle = dealed_big_yaw_gyro;
        gimbal_controller.set_big_yaw_angle = TD_Calculate(&gimbal_controller.pos_big_yaw_td, gimbal_controller.target_big_yaw_angle);
        gimbal_controller.set_big_yaw_speed = PID_Calculate(&gimbal_controller.big_yaw_angle_pid, dealed_big_yaw_gyro,gimbal_controller.set_big_yaw_angle) + Feedforward_Calculate(&gimbal_controller.big_yaw_angle_forward,gimbal_controller.set_big_yaw_angle);
        TD_Calculate(&gimbal_controller.speed_big_yaw_td, gimbal_controller.set_big_yaw_speed);
        //此处也需要增加一个前馈，用以处理小陀螺状态
        gimbal_controller.set_big_yaw_current = GIMBAL_BIG_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.big_yaw_speed_pid, big_yaw_speed_raw, gimbal_controller.set_big_yaw_speed) + Feedforward_Calculate(&gimbal_controller.big_yaw_speed_forward,gimbal_controller.set_big_yaw_speed));
        if(gimbal_controller.small_yaw_recv.angle != 0)//防止小yaw掉线，大yaw疯转
        {
            return gimbal_controller.set_big_yaw_current;
        }
        else return 0.f;
    }
    
    

}
float big_yaw_angle_after_iir;
float Gimbal_Big_Yaw_Calculate(float set_point)
{
		
   // BigYawZeroCheck();移到接收函数中
    BigYawTurnCheck();
    BigYawSetpointSet();
	
    gimbal_controller.set_big_yaw_angle = TD_Calculate(&gimbal_controller.pos_big_yaw_td, gimbal_controller.target_big_yaw_angle);
   	iir(&big_yaw_angle_after_iir,gimbal_controller.set_big_yaw_angle,0.8f);
    gimbal_controller.set_big_yaw_speed = PID_Calculate(&gimbal_controller.big_yaw_angle_pid, big_yaw_angle_after_zero_check,big_yaw_angle_after_iir) + Feedforward_Calculate(&gimbal_controller.big_yaw_angle_forward,big_yaw_angle_after_iir);
    TD_Calculate(&gimbal_controller.speed_big_yaw_td, gimbal_controller.set_big_yaw_speed);
    //此处也需要增加一个前馈，用以处理小陀螺状态
    gimbal_controller.set_big_yaw_current = GIMBAL_BIG_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.big_yaw_speed_pid, gimbal_controller.DM_Big_Yaw_Motor.V_Receive, gimbal_controller.set_big_yaw_speed) + Feedforward_Calculate(&gimbal_controller.big_yaw_speed_forward,gimbal_controller.set_big_yaw_speed));
    
    // 大yaw缓启动
    
    if(fabsf(dynamic_angle_big) < 5.0f)
    {
        
        gimbal_enable_cnt++;
        if(gimbal_enable_cnt > GIMBAL_INIT_WAIT_TIME )
        {
            gimbal_controller.big_yaw_angle_pid.MaxOut = 180;
            gimbal_enable_flag = 1;
        }

    }
    if(gimbal_enable_flag == 0)
    {
        gimbal_controller.set_big_yaw_current = LIMIT_MAX_MIN(gimbal_controller.set_big_yaw_current,300.0f,-300.0f);
    }
    if(gimbal_controller.small_yaw_recv.angle != 0)//防止小yaw掉线，大yaw疯转
    {
        return gimbal_controller.set_big_yaw_current;
    }
    else return 0.f;
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
    TD_Clear(&gimbal_controller.pos_big_yaw_td, big_yaw_angle_after_zero_check);

    gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_small_yaw_speed = 0;
    gimbal_controller.set_small_yaw_current = 0;

 //   gimbal_controller.target_big_yaw_angle = gimbal_controller.gyro_yaw_angle;
 //   gimbal_controller.set_big_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_big_yaw_speed = 0;
    gimbal_controller.set_big_yaw_current = 0;
		
	bigyaw_turn_flag = 0;//大yaw转到位，再判断是否满足小yaw需求
    bigyaw_is_turning_flag = 0;
   // FIRFilter_Free(&big_yaw_fir_filter);
   gimbal_enable_flag = 0;
   gimbal_enable_cnt = 0;
   gimbal_controller.big_yaw_angle_pid.MaxOut = 30;
   

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
#if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG // PITCH安装位置刚好过圈，下方代替一个过零检测
        if (cur_motor_angle > 180)
        {
            cur_motor_angle -= 360;
        }
#endif
        cur_motor_angle /= 2; // 有2:1减速比
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
    //iir(&gimbal_controller.gyro_yaw_speed, speed, 0.6);//改滤波
    gimbal_controller.gyro_last_yaw_angle = gimbal_controller.gyro_yaw_angle;
}

/**
 * @brief 由于重力补偿的作用，云台需要施加一个非线性力抵消重力影响，该力需要根据实际来进行测定
 */
float GimbalPitchComp()
{
    // //记得每调一台车都需要重新更新参数
    // const static float pitch_comp[5] = {0.1399, -0.9144, -10.2, 9.038, -3337};
    // float x[4];

    // //解析静止时的非线性函数，只能大致补偿，然后靠PID的I使最终无静差
    // //低于一定角度或高于一定角度，根据测量结果，输出应大致不变
    // x[3] = LIMIT_MAX_MIN(gimbal_controller.gyro_pitch_angle, 8, -12);
    // x[2] = x[3] * x[3];
    // x[1] = x[2] * x[3];
    // x[0] = x[1] * x[3];

    // float sum = pitch_comp[4];
    // for (int i = 0; i < 4; i++)
    // {
    //     sum += x[i] * pitch_comp[i];
    // }
    // iir(&gimbal_controller.comp_pitch_current, sum * GIMBAL_PITCH_COMP_COEF, 0.7);
    // return gimbal_controller.comp_pitch_current;
    iir(&gimbal_controller.comp_pitch_current, GIMBAL_PITCH_COMP * arm_cos_f32(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) * GIMBAL_PITCH_COMP_COEF, 0.7);
    return gimbal_controller.comp_pitch_current;
}

/**
 * @brief 云台摩擦力模型，只使用库伦摩擦力，因粘性摩擦力在辨识中表现不明显，故忽略
 * @param[in] void
 */
//float GimbalFrictionModel()
//{
//    // 根据转速判断符号
//    if (fabsf(gimbal_controller.gyro_yaw_speed) < BORDER_FRICTION_SPEED)
//    {
//        // 部分补偿
//        return gimbal_controller.gyro_yaw_speed / BORDER_FRICTION_SPEED * FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF;
//    }
//    // 全补偿
//    return FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF * sign(gimbal_controller.gyro_yaw_speed);
//}
