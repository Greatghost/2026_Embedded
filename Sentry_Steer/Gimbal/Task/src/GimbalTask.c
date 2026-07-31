#include "GimbalTask.h"
#include "iwdgTask.h"

uint8_t motor_send_data[2][SEND_ID_NUMS][8];
uint8_t is_has_motor_data[2][SEND_ID_NUMS];

SawToothWave saw_tooth_wave;
extern BigYawController big_yaw_controller;

#define PC_GIMBAL_RECOVERY_PITCH_RATE_DPS 100.0f
#define PC_GIMBAL_RECOVERY_YAW_RATE_DPS   180.0f
#define PC_GIMBAL_RECOVERY_DEFAULT_DT       0.002f
#define PC_GIMBAL_RECOVERY_MAX_DT           0.020f

static uint8_t pc_gimbal_target_armed = 0U;
static uint8_t pc_gimbal_recovery_active = 1U;

/**
 * @brief 手动模式接管云台时执行无扰切换
 *
 * 上位机模式会持续覆盖 target_pitch_angle/target_big_yaw_angle。切回手动
 * 模式时，必须先用当前反馈角重新建立目标，并清除控制器中属于上一个目标
 * 的历史状态，否则手动模式会继续追赶最后一帧上位机目标。
 */
static void Gimbal_Manual_Mode_Enter(void)
{
    const float current_pitch = gimbal_controller.gyro_pitch_angle;
    const float current_yaw = big_yaw_controller.dealed_big_yaw_gyro;

    gimbal_controller.target_pitch_angle = current_pitch;
    gimbal_controller.target_pitch_speed = 0.0f;
    gimbal_controller.target_pitch_acceleration = 0.0f;
    gimbal_controller.set_pitch_angle = current_pitch;
    gimbal_controller.set_pitch_speed = 0.0f;
    gimbal_controller.set_pitch_current = 0.0f;

    PID_Clear(&gimbal_controller.pitch_angle_pid);
    PID_Clear(&gimbal_controller.pitch_speed_pid);
    PID_Clear(&gimbal_controller.pitch_current_pid);
    Feedforward_Reset(&gimbal_controller.pitch_angle_forward, current_pitch);
    Feedforward_Reset(&gimbal_controller.pitch_speed_forward, 0.0f);

    gimbal_controller.target_big_yaw_angle = current_yaw;
    gimbal_controller.target_big_yaw_speed = 0.0f;
    gimbal_controller.target_big_yaw_acceleration = 0.0f;
    gimbal_controller.set_big_yaw_angle = current_yaw;
    gimbal_controller.set_big_yaw_speed = 0.0f;
    gimbal_controller.set_big_yaw_current = 0.0f;

    PID_Clear(&gimbal_controller.big_yaw_angle_pid);
    PID_Clear(&gimbal_controller.big_yaw_speed_pid);
    Feedforward_Reset(&gimbal_controller.big_yaw_angle_forward, current_yaw);
    Feedforward_Reset(&gimbal_controller.big_yaw_speed_forward, 0.0f);
    TD_Clear(&gimbal_controller.pos_big_yaw_td, current_yaw);
}

static float Gimbal_PC_Target_Slew(float current, float target, float max_step)
{
    const float error = target - current;

    if (error > max_step)
    {
        return current + max_step;
    }
    if (error < -max_step)
    {
        return current - max_step;
    }
    return target;
}

static void Gimbal_PC_Target_Disarm(void)
{
    pc_gimbal_target_armed = 0U;
    pc_gimbal_recovery_active = 1U;
}

/* PC目标只从同一帧快照更新；离线时立即无扰同步到当前反馈。 */
static void Gimbal_PC_Target_Update(void)
{
    PCControlSnapshot_t pc_control;
    float recovery_dt;
    float pitch_step;
    float yaw_step;

    if (PCControlGetSnapshot(&pc_control) == 0U)
    {
        if (pc_gimbal_target_armed != 0U)
        {
            Gimbal_Manual_Mode_Enter();
        }
        else
        {
            gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
            gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
			gimbal_controller.target_pitch_speed = 0.0f;
			gimbal_controller.target_pitch_acceleration = 0.0f;
			gimbal_controller.target_big_yaw_speed = 0.0f;
			gimbal_controller.target_big_yaw_acceleration = 0.0f;
        }
        Gimbal_PC_Target_Disarm();
        return;
    }

    if (pc_gimbal_target_armed == 0U)
    {
        Gimbal_Manual_Mode_Enter();
        pc_gimbal_target_armed = 1U;
        pc_gimbal_recovery_active = 1U;
    }

    if (fabsf(gimbal_controller.gyro_pitch_angle - pc_control.pitch) > 60.0f ||
        fabsf(big_yaw_controller.dealed_big_yaw_gyro - pc_control.yaw) > 70.0f)
    {
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
		gimbal_controller.target_pitch_speed = 0.0f;
		gimbal_controller.target_pitch_acceleration = 0.0f;
		gimbal_controller.target_big_yaw_speed = 0.0f;
		gimbal_controller.target_big_yaw_acceleration = 0.0f;
        pc_gimbal_recovery_active = 1U;
        return;
    }

    if (pc_gimbal_recovery_active == 0U)
    {
        gimbal_controller.target_pitch_angle = pc_control.pitch;
        gimbal_controller.target_big_yaw_angle = pc_control.yaw;
		gimbal_controller.target_pitch_speed = pc_control.pitch_omega;
		gimbal_controller.target_pitch_acceleration = pc_control.pitch_alpha;
		gimbal_controller.target_big_yaw_speed = pc_control.yaw_omega;
		gimbal_controller.target_big_yaw_acceleration = pc_control.yaw_alpha;
        return;
    }

    recovery_dt = gimbal_controller.delta_t;
    if (!(recovery_dt > 0.0f && recovery_dt <= PC_GIMBAL_RECOVERY_MAX_DT))
    {
        recovery_dt = PC_GIMBAL_RECOVERY_DEFAULT_DT;
    }
    pitch_step = PC_GIMBAL_RECOVERY_PITCH_RATE_DPS * recovery_dt;
    yaw_step = PC_GIMBAL_RECOVERY_YAW_RATE_DPS * recovery_dt;

    gimbal_controller.target_pitch_angle = Gimbal_PC_Target_Slew(
        gimbal_controller.target_pitch_angle, pc_control.pitch, pitch_step);
    gimbal_controller.target_big_yaw_angle = Gimbal_PC_Target_Slew(
        gimbal_controller.target_big_yaw_angle, pc_control.yaw, yaw_step);
	gimbal_controller.target_pitch_speed = 0.0f;
	gimbal_controller.target_pitch_acceleration = 0.0f;
	gimbal_controller.target_big_yaw_speed = 0.0f;
	gimbal_controller.target_big_yaw_acceleration = 0.0f;

    if (gimbal_controller.target_pitch_angle == pc_control.pitch &&
        gimbal_controller.target_big_yaw_angle == pc_control.yaw)
    {
        pc_gimbal_recovery_active = 0U;
    }
}

void Gimbal_Powerdown_Cal()
{
    limitPitchAngle();
    GimbalClear();

    motor_communication[PITCH_MOTOR].control = 0;
    motor_communication[BIG_YAW_MOTOR].control = 0;
    // [SMALL_YAW_REMOVED] 小Yaw电机控制已删除
    // motor_communication[SMALL_YAW_MOTOR].control = 0;
    big_yaw_controller.gimbal_last_mode = 0;

    // TEST
    // SawToothInit(&saw_tooth_wave, 16.0f, 10, 200, gimbal_controller.gyro_yaw_angle);
}

void Gimbal_Autoaim_Cal()
{
    float pitch_before_limit;

    Gimbal_PC_Target_Update();

    // 额外加一层保护
    if (fabsf(gimbal_controller.target_pitch_angle) > 60.0f)
    {
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
		gimbal_controller.target_pitch_speed = 0.0f;
		gimbal_controller.target_pitch_acceleration = 0.0f;
    }
    // [SMALL_YAW_REMOVED] target_big_yaw_angle保护 (原target_small_yaw)
    if (fabsf(gimbal_controller.target_big_yaw_angle - 999.0f) < 1e-4)
    {
        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
		gimbal_controller.target_big_yaw_speed = 0.0f;
		gimbal_controller.target_big_yaw_acceleration = 0.0f;
    }

    // pitch限制幅值
	pitch_before_limit = gimbal_controller.target_pitch_angle;
    limitPitchAngle();
	if (gimbal_controller.target_pitch_angle != pitch_before_limit)
	{
		gimbal_controller.target_pitch_speed = 0.0f;
		gimbal_controller.target_pitch_acceleration = 0.0f;
	}
    motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_CalculateFeedforward(
		gimbal_controller.target_pitch_angle,
		gimbal_controller.target_pitch_speed,
		gimbal_controller.target_pitch_acceleration);
    // yaw计算
    // [SMALL_YAW_REMOVED] 小Yaw控制已删除，仅大Yaw
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_CalculateFeedforward(
		gimbal_controller.target_big_yaw_angle,
		gimbal_controller.target_big_yaw_speed,
		gimbal_controller.target_big_yaw_acceleration);
    // motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
    big_yaw_controller.gimbal_last_mode = 1;
}

void Gimbal_Small_Buff_Cal()
{
    Gimbal_PC_Target_Update();

    // 额外加一层保护
    if (fabsf(gimbal_controller.target_pitch_angle) > 60.0f)
    {
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    }
    // [SMALL_YAW_REMOVED] target_big_yaw_angle保护 (原target_small_yaw)
    if (fabsf(gimbal_controller.target_big_yaw_angle - 999.0f) < 1e-4)
    {
        gimbal_controller.target_big_yaw_angle = big_yaw_controller.dealed_big_yaw_gyro;
    }

    // pitch限制幅值
    limitPitchAngle();
    motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    // [SMALL_YAW_REMOVED] 小Yaw控制已删除，大Yaw直接承载bias补偿
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);
    // motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
    big_yaw_controller.gimbal_last_mode = 1;
}

void Gimbal_Big_Buff_Cal()
{
    /* 大符与小符共用PC角度闭环，只在上行mode_want中区分模式。 */
    Gimbal_Small_Buff_Cal();
}

void Gimbal_Act_Cal()
{
    GimbalTest_t *test = &gimbal_controller.gimbal_test;

    // === Pitch控制 ===
#if (GIMBAL_TEST_CONFIG == GIMBAL_CONFIG_PITCH_SQUARE)
    // Pitch方波测试模式
    gimbal_controller.target_pitch_angle = SquareWaveRun(&test->pitch_square, gimbal_controller.delta_t);
#elif (GIMBAL_CONTROL_DISCONNECT == 0)
    // 正常模式：目标角度由遥控器/上位机设置（ChassisSolver.c）
#else
    // 控制断开模式：目标角度保持不变，用于debug手动设置
#endif

    limitPitchAngle();

    // 计算Pitch控制输出
    float pitch_control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    // 记录目标函数（仅Pitch方波模式下）
#if (GIMBAL_TEST_CONFIG == GIMBAL_CONFIG_PITCH_SQUARE)
    float error = gimbal_controller.target_pitch_angle - gimbal_controller.gyro_pitch_angle;
    GimbalTestRunPitchCost(test, error, gimbal_controller.set_pitch_current, gimbal_controller.delta_t);
#endif

    motor_communication[PITCH_MOTOR].control = pitch_control;

    // [SMALL_YAW_REMOVED] 小Yaw控制已全部删除，大Yaw独立承载Yaw控制
    // // === Yaw控制 ===
    // #if (GIMBAL_TEST_CONFIG == GIMBAL_CONFIG_SMALLYAW_SQUARE)
    // gimbal_controller.target_small_yaw_angle = SquareWaveRun(&test->small_yaw_square, gimbal_controller.delta_t);
    // #elif (GIMBAL_CONTROL_DISCONNECT == 0)
    // #else
    // #endif
    // motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);

    // 大Yaw动态跟随
    // float delta_angle = fabsf(gimbal_controller.target_big_yaw_angle - (big_yaw_controller.dealed_big_yaw_gyro + 0.6 * big_yaw_controller.big_yaw_gyro_bias));
    // float dynamic_gain = -0.0004233 * delta_angle * delta_angle - 0.000567 * delta_angle + 0.999f;
    // dynamic_gain = LIMIT_MAX_MIN(dynamic_gain, 1.0f, 0.5f);
    // iir(&gimbal_controller.target_big_yaw_angle, big_yaw_controller.dealed_big_yaw_gyro + 0.6 * big_yaw_controller.big_yaw_gyro_bias, dynamic_gain);
    // motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);
    
    // [SINGLE_YAW] 目标角度由遥控器/上位机直接控制，不再通过IIR动态跟随
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);
    
    big_yaw_controller.gimbal_last_mode = 1;
}

extern SawToothWave gimbal_saw_tooth;
extern StepFunction gimbal_step;
float test_yaw_angle;

void YawSawTest()
{
    // yaw 10度响应测试

    // pitch限制幅值
    limitPitchAngle();
    //  motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(SawWaveRun(&saw_tooth_wave, gimbal_controller.delta_t));
    motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    // yaw计算
    // 锯齿波
    test_yaw_angle = SawWaveRun(&gimbal_saw_tooth, gimbal_controller.delta_t);
    // [SMALL_YAW_REMOVED] 小Yaw控制已删除，锯齿波测试仅用大Yaw
    // motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(test_yaw_angle);
    float delta_angle = fabsf(gimbal_controller.target_big_yaw_angle - test_yaw_angle);
    // 动态增益，y = -0.0004233^2 + 0.000567 + 0.998,对应点(0,0.998),(10,0.95),(30,0.6),在正半轴单调递减
    float dynamic_gain = -0.0004233 * delta_angle * delta_angle - 0.000567 * delta_angle + 0.999f;
    dynamic_gain = LIMIT_MAX_MIN(dynamic_gain, 1.0f, 0.5f);
    iir(&gimbal_controller.target_big_yaw_angle, test_yaw_angle, dynamic_gain);
    // 阶跃
    //	motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(StepRun(&gimbal_step, gimbal_controller.delta_t));
    // 大YAW目标加上bias补偿
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(big_yaw_controller.dealed_big_yaw_gyro + big_yaw_controller.big_yaw_gyro_bias);
}

void Gimbal_Test_Cal()
{
    //    // pitch限制幅值
    //    // limitPitchAngle();
    //    // Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    //    // GM6020_SendPack(dji_motors_send_data, GM6020_STD_VOL_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    //    // yaw计算
    //    // Gimbal_Yaw_Calculate(SawWaveRun(&saw_tooth_wave, gimbal_controller.delta_t));
    //    // Gimbal_Speed_Calculate(90.0f);
    //    // GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_VOL_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
    //
    //		// pitch限制幅值
    //    limitPitchAngle();
    //		//  motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(SawWaveRun(&saw_tooth_wave, gimbal_controller.delta_t));
    //		motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    //    // yaw计算
    //    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(SawWaveRun(&saw_tooth_wave, gimbal_controller.delta_t));
    //    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);

    YawSawTest();
    big_yaw_controller.gimbal_last_mode = 1;
}

void Gimbal_Backturn_Cal()
{
    // pitch限制幅值
    limitPitchAngle();
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    // GM6020_SendPack(dji_motors_send_data, GM6020_STD_VOL_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    // Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    // GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_VOL_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Motor_Data_Pack()
{
    memset(is_has_motor_data[0], 0, SEND_ID_NUMS * 2);
    uint8_t can_select = 0;

    for (int i = 0; i < 5; i++)
    {
        can_select = motor_communication[i].can == CAN1 ? 0 : 1;
        switch (motor_communication[i].motor_type)
        {
        case GM6020:
            GM6020_SendPack(motor_send_data[can_select][motor_communication[i].motor_id_type], motor_communication[i].std_id, motor_communication[i].motor_id - 0x204, (int16_t)motor_communication[i].control, GM6020_CUR_MODE);
            is_has_motor_data[can_select][motor_communication[i].motor_id_type] = TRUE;
            break;
        case M3508:
            M3508_SendPack(motor_send_data[can_select][motor_communication[i].motor_id_type], motor_communication[i].std_id, motor_communication[i].motor_id - 0x200, motor_communication[i].control, SEND_CURRENT);
            is_has_motor_data[can_select][motor_communication[i].motor_id_type] = TRUE;
            break;
        case M2006:
            M2006_SendPack(motor_send_data[can_select][motor_communication[i].motor_id_type], motor_communication[i].std_id, motor_communication[i].motor_id - 0x200, motor_communication[i].control);
            is_has_motor_data[can_select][motor_communication[i].motor_id_type] = TRUE;
            break;
        case DM_MOTOR:
            if (i == BIG_YAW_MOTOR)
            {
                gimbal_controller.DM_Big_Yaw_Motor.t_ff = motor_communication[i].control;
                DM_Motor_Control(&gimbal_controller.DM_Big_Yaw_Motor, motor_send_data[can_select][motor_communication[i].motor_id_type], DM_MIT_CONTROL);
                is_has_motor_data[can_select][motor_communication[i].motor_id_type] = TRUE;
            }
            else if (i == PITCH_MOTOR)
            {
                gimbal_controller.DM_Pitch_Motor.t_ff = motor_communication[i].control;
                DM_Motor_Control(&gimbal_controller.DM_Pitch_Motor, motor_send_data[can_select][motor_communication[i].motor_id_type], DM_MIT_CONTROL);
                is_has_motor_data[can_select][motor_communication[i].motor_id_type] = TRUE;
            }
            break;
        default:
            break;
        }
    }
}

void Motor_Data_Send()
{
    int send_ID_max_1 = 3;
    // 先发送0到2
    for (int i = 0; i < SEND_ID_NUMS; i++)
    {
        if (is_has_motor_data[0][i])
        {
            CanSend(&hcan1, motor_send_data[0][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[0][i], &motor_wait_time[0][i]);
        }
        if (is_has_motor_data[1][i])
        {
            CanSend(&hcan2, motor_send_data[1][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[1][i], &motor_wait_time[1][i]);
        }
    }
}

void Motor_Data_Send_0()
{
    int send_ID_max_1 = 3;
    // 先发送0到2
    for (int i = 0; i < send_ID_max_1; i++)
    {
        if (is_has_motor_data[0][i])
        {
            CanSend(&hcan1, motor_send_data[0][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[0][i], &motor_wait_time[0][i]);
        }
        if (is_has_motor_data[1][i])
        {
            CanSend(&hcan2, motor_send_data[1][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[1][i], &motor_wait_time[1][i]);
        }
    }
}

void Motor_Data_Send_1()
{
    int send_ID_max_1 = 5;
    // 达妙满频率
    for (int i = send_ID_max_1; i < SEND_ID_NUMS; i++)
    {
        if (is_has_motor_data[0][i])
        {
            CanSend(&hcan1, motor_send_data[0][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[0][i], &motor_wait_time[0][i]);
        }
        if (is_has_motor_data[1][i])
        {
            CanSend(&hcan2, motor_send_data[1][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[1][i], &motor_wait_time[1][i]);
        }
    }
}

void Motor_Data_Send_2()
{
    int friction_num = 4;
    // 摩擦轮和拨盘
    int i = friction_num;
    if (is_has_motor_data[0][i])
    {
        CanSend(&hcan1, motor_send_data[0][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[0][i], &motor_wait_time[0][i]);
    }
    if (is_has_motor_data[1][i])
    {
        CanSend(&hcan2, motor_send_data[1][i], MOTOR_STD_ID_LIST[i], &motor_tx_header[1][i], &motor_wait_time[1][i]);
    }
}
void updataSensors()
{
    // 更新传感器信息
    updateGyro();

    // PITCH电机解码  TODO:扩展到所有类型电机
    if (motor_communication[PITCH_MOTOR].motor_type == GM6020)
    {
        GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    }

    // [SMALL_YAW_REMOVED] 小Yaw电机解码已删除
    // if (motor_communication[SMALL_YAW_MOTOR].motor_type == GM6020)
    // {
    //     GM6020_Decode(&gimbal_controller.small_yaw_recv, &gimbal_controller.small_yaw_info);
    // }

    // 摩擦轮电机解码  TODO:扩展到所有类型电机
    if (motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_type == M3508)
    {
        for (int i = 0; i < 2; i++)
        {
            M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
        }
    }

    // 拨盘电机解码 TODO:扩展到所有类型电机
    if (motor_communication[TOGGLE_MOTOR].motor_type == M2006)
    {
        M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.70);
    }

    // 弹舱盖电机解码 TODO:扩展到所有类型电机
    //    if (motor_communication[BAY_MOTOR].motor_type == M2006)
    //    {
    //        M2006_Decode(&bomb_bay_controller.bomb_bay_recv, &bomb_bay_controller.bomb_bay_info, WITH_REDUCTION, 0.95);
    //    }
}

void PC_Send(uint32_t index)
{
//    // [DEBUG] 临时: 只发TypeID=0，验证SendToPC_Buff缓冲冲突
//    if (index % 2 == 0) // 250HZ
//    {
//      SendtoPC(USUAL_PC_DATA);
//    }
//    return;

//    // === 以下暂时屏蔽 ===
    if (JudgeData_update)
    {

        if (index % 20 == 1) // 25hz
        {
            SendtoPC(JUDGE_PC_DATA);
            JudgeData_update = 0;
        }
    }
    if (Blood_update)
    {
        if (index % 50 == 1) // 10hz
        {
            SendtoPC(JUDGE_PC_DATA_BLOOD_1);
        }
        else if (index % 50 == 15)
        {
            SendtoPC(JUDGE_PC_DATA_BLOOD_2);
            Blood_update = 0;
        }
    }
    if (index % 50 == 13) // 10Hz, 避开USUAL_PC_DATA的偶数index
    {
        SendtoPC(JUDGE_PC_DATA_EXTENDED);
    }
    // 新增: TypeID 7 哨兵信息发送 (10Hz)
    if (index % 50 == 5)
    {
        SendtoPC(JUDGE_PC_DATA_SENTRY_DATA);
    }
    // 新增: TypeID 8 弹量数据+RFID扩展发送 (10Hz)
    if (index % 50 == 25)
    {
        SendtoPC(JUDGE_PC_DATA_BULLET_DATA_AND_RFID2);
    }
    #ifdef DEBUG_ROBOT_CMD_SEND
	// 调试: TypeID 9 小地图下发指令发送 (0.5Hz)
	if (index % 1000 == 7)
	{
		SendtoPC(JUDGE_PC_DATA_ROBOT_COMMAND);
	}
    #endif

    // 新增: TypeID 10 哨兵姿态时长发送 (10Hz)
    if (index % 50 == 45)
    {
        SendtoPC(JUDGE_PC_DATA_SENTRY_DURATION);
    }

    if (index % 2 == 0) // 250HZ
    {
    #if COMMUNICATION_CHOOSE == COMMUNICATION_OF_IFANTRY
        SendtoPC(); // 将信息发送给上位机
    #elif COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY
        SendtoPC(USUAL_PC_DATA); // 将信息发送给上位机

    #endif
    }
	// TypeID 11 云台动态反馈约90~100Hz；避开定位包和0.5Hz调试包的发送时隙。
	if (((index % 10 == 7) && (index % 50 != 37) && (index % 1000 != 7)) ||
		(index % 10 == 9))
	{
		SendtoPC(JUDGE_PC_DATA_GIMBAL_DYNAMICS);
	}
    if (index % 100 == 3) // 5hz
    {
        SendtoPC(JUDGE_PC_DATA_RFID_BUFF);
    }
    if (index % 50 == 37) // 10hz
    {
        SendtoPC(JUDGE_PC_DATA_POS);
    }
}

/**
 * @brief 云台控制任务
 * @param[in] void
 */

float dt_max = 100.0f, dt;
void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 2; // 500HZ
    enum GIMBAL_ACTION previous_gimbal_action = GIMBAL_POWERDOWN;
    uint8_t shoot_feedback_was_ready = 0U;

    FrictionWheel_Init();
    GimbalPidInit();
    TogglePidInit();
    DM_Motor_Init(&gimbal_controller.DM_Big_Yaw_Motor, 3.141593f, 20, 45);
    DM_Motor_Init(&gimbal_controller.DM_Pitch_Motor, 3.141593f, 10, 30);

    /* crossing_hole-main 自动 G/B/C/J 系统辨识模块。 */
    GimbalSystemID_Init(&gimbal_controller);

    /* 云台测试模块初始化 */
    GimbalTestInit(&gimbal_controller.gimbal_test);
    // SawToothInit(&saw_tooth_wave, 10, 1, 200, 0);

    vTaskDelay(2000);

    static uint32_t index = 0;

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        global_debugger.robot_debugger.dt = DWT_GetDeltaT(&global_debugger.robot_debugger.last_cnt);

        // 更新传感器信息
        updataSensors();
        Big_Yaw_Bias_Cal();
        Schmitt_PID_changer();

#if GIMBAL_SYSID
        /* 与 crossing_hole-main 相同：传感器更新后、控制模式计算前推进状态机。 */
        if (!gimbal_sysid.yaw.sysid_done || !gimbal_sysid.pitch.sysid_done)
        {
            GimbalSystemID_Run();
        }
#endif

        /* 使用快照保证本控制周期内模式一致，并在手动模式进入沿执行无扰接管。 */
        const enum GIMBAL_ACTION current_gimbal_action = remote_controller.gimbal_action;
        const uint8_t current_action_uses_pc_target =
            current_gimbal_action == GIMBAL_AUTO_AIM_MODE ||
            current_gimbal_action == GIMBAL_SMALL_BUFF_MODE ||
            current_gimbal_action == GIMBAL_BIG_BUFF_MODE;
        if (current_action_uses_pc_target == 0U)
        {
            /* 即使掉线状态先被底盘任务切成手动，下一次PC接管仍从反馈限速恢复。 */
            Gimbal_PC_Target_Disarm();
        }
        if (current_gimbal_action == GIMBAL_ACT_MODE &&
            previous_gimbal_action != GIMBAL_ACT_MODE)
        {
            Gimbal_Manual_Mode_Enter();
        }
        previous_gimbal_action = current_gimbal_action;

        switch (current_gimbal_action)
        {
        case GIMBAL_POWERDOWN: // 掉电模式
            Gimbal_Powerdown_Cal();
            break;
        case GIMBAL_ACT_MODE: // 云台运动模式
            Gimbal_Act_Cal();
            break;
        case GIMBAL_AUTO_AIM_MODE: // 自瞄模式
            Gimbal_Autoaim_Cal();
            break;
        case GIMBAL_SMALL_BUFF_MODE:
            Gimbal_Small_Buff_Cal();
            break;
        case GIMBAL_BIG_BUFF_MODE:
            Gimbal_Big_Buff_Cal();
            break;
        case GIMBAL_TEST_MODE:
            Gimbal_Test_Cal();
            break;
        default:
            Gimbal_Powerdown_Cal();
            break;
        }

        /*
         * 所有模式（含SI/Test旁路）最终都经过反馈有效性闸门，防止旧反馈
         * 仍被打包成非零力矩。各轴独立掉线时只切断对应轴；IMU掉线切断两轴。
         */
        const uint8_t imu_feedback_ready =
            offline_detector.imu_state[0] == IMU_ON &&
            offline_detector.imu_state[1] == IMU_ON;
        if (!imu_feedback_ready ||
            offline_detector.pitch_motor_state != PITCH_MOTOR_ON)
        {
            motor_communication[PITCH_MOTOR].control = 0.0f;
        }
        if (!imu_feedback_ready ||
            offline_detector.yaw_motor_state != YAW_MOTOR_ON)
        {
            motor_communication[BIG_YAW_MOTOR].control = 0.0f;
        }

        /*
         * 射击机构也必须消费掉线状态。任一反馈不稳定时整套射击输出切零；
         * 恢复后的首周期只同步控制器，下一周期才重新进入射击状态机。
         */
        const uint8_t shoot_feedback_ready =
            offline_detector.friction_motor_state[LEFT_FRICTION_WHEEL] == FRICTION_WHEEL_MOTOR_ON &&
            offline_detector.friction_motor_state[RIGHT_FRICTION_WHEEL] == FRICTION_WHEEL_MOTOR_ON &&
            offline_detector.toggle_motor_state == TOGGLE_MOTOR_ON;

        if (!shoot_feedback_ready || !shoot_feedback_was_ready)
        {
            Shoot_FeedbackSafeStop();
        }
        else
        {
            Shoot_Cal();
        }
        shoot_feedback_was_ready = shoot_feedback_ready;

#if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG
        // 弹舱盖控制函数
        // BombBayControl();
#endif
        Motor_Data_Pack(); // 电机数据打包
                           //			Motor_Data_Send();
        if (index % 2 == 0)
            Motor_Data_Send_0();
        //				 Motor_Data_Send_0();
        //			else if(index % 3 == 0) Motor_Data_Send_2();
        if (index % 3 == 0)
            Motor_Data_Send_2();
        Motor_Data_Send_1();
        // 电机数据发送

        PC_Send(index);
        index++;

        /* Recover a genuinely stuck transfer by cleanly re-enumerating USB.
         * A one-second timeout avoids resetting for ordinary host latency. */
        CDC_Transmit_FS_Watchdog(1000U);

        Iwdg_ReportAlive(IWDG_HEARTBEAT_GIMBAL);
        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
