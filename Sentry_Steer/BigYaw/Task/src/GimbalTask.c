#include "GimbalTask.h"

uint8_t motor_send_data[2][SEND_ID_NUMS][8];
uint8_t is_has_motor_data[2][SEND_ID_NUMS];

SI_t SI_obeject;
SawToothWave saw_tooth_wave;


void Gimbal_Powerdown_Cal()
{
    limitPitchAngle();
    GimbalClear();

    motor_communication[PITCH_MOTOR].control = 0;
    motor_communication[BIG_YAW_MOTOR].control = 0;
    motor_communication[SMALL_YAW_MOTOR].control = 0;
    

    // TEST
    // SawToothInit(&saw_tooth_wave, 16.0f, 10, 200, gimbal_controller.gyro_yaw_angle);
}

void Gimbal_Autoaim_Cal()
{
		PCRecvData pc_recv_data_temp = pc_recv_data;
    // 设置目标角度
    if (pc_recv_data_temp.enemy_id != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data_temp.pitch) < 60.0f && fabsf(gimbal_controller.target_small_yaw_angle - pc_recv_data_temp.yaw) < 70.0f)
    {
        if (offline_detector.pc_state == PC_ON)
        {
            gimbal_controller.target_pitch_angle = pc_recv_data_temp.pitch;
            //PCyaw作为小yaw输入
            gimbal_controller.target_small_yaw_angle = pc_recv_data_temp.yaw;
        }
    }
		
    // 额外加一层保护
    if (fabsf(gimbal_controller.target_pitch_angle) > 60.0f)
    {
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    }
    if (fabsf(gimbal_controller.target_small_yaw_angle - 999.0f) < 1e-4)
    {
        gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    }

    // pitch限制幅值
    limitPitchAngle();
    motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    // yaw计算
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);
    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);

}

void Gimbal_Small_Buff_Cal()
{
		PCRecvData pc_recv_data_temp = pc_recv_data;
    // 设置目标角度
    if (pc_recv_data_temp.enemy_id != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data_temp.pitch) < 60.0f && fabsf(gimbal_controller.target_small_yaw_angle - pc_recv_data_temp.yaw) < 70.0f)
    {
        gimbal_controller.target_pitch_angle = pc_recv_data_temp.pitch;
        gimbal_controller.target_small_yaw_angle = pc_recv_data_temp.yaw;
    }

    // 额外加一层保护
    if (fabsf(gimbal_controller.target_pitch_angle) > 60.0f)
    {
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    }
    if (fabsf(gimbal_controller.target_small_yaw_angle - 999.0f) < 1e-4)
    {
        //PCyaw作为小yaw输入
        gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    }
		
    // pitch限制幅值
    limitPitchAngle();
    motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    // yaw计算
    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);

}

void Gimbal_Big_Buff_Cal()
{
		PCRecvData pc_recv_data_temp = pc_recv_data;
    // 设置目标角度
    if (pc_recv_data_temp.enemy_id != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data_temp.pitch) < 60.0f && fabsf(gimbal_controller.target_small_yaw_angle - pc_recv_data_temp.yaw) < 70.0f)
    {
        gimbal_controller.target_pitch_angle = pc_recv_data_temp.pitch;
        gimbal_controller.target_small_yaw_angle = pc_recv_data_temp.yaw;
    }

    // 额外加一层保护
    if (fabsf(gimbal_controller.target_pitch_angle) > 60.0f)
    {
        gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    }
    if (fabsf(gimbal_controller.target_small_yaw_angle - 999.0f) < 1e-4)
    {
        gimbal_controller.target_small_yaw_angle = gimbal_controller.gyro_yaw_angle;
    }	
		
    // pitch限制幅值
    limitPitchAngle();
    motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    // yaw计算
    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
    motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);

}

void Gimbal_SI_Cal()
{
}


void Gimbal_Act_Cal()
{
	// pitch限制幅值
	limitPitchAngle();
	motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

	// yaw计算
	 //	motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
	motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(gimbal_controller.target_small_yaw_angle);
	motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);//先调小yaw，保护一下大yaw
	
}


extern SawToothWave gimbal_saw_tooth;
extern StepFunction gimbal_step;

void YawSawTest()
{
	//yaw 10度响应测试
	   
		// pitch限制幅值
    limitPitchAngle();
		//  motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(SawWaveRun(&saw_tooth_wave, gimbal_controller.delta_t));
		motor_communication[PITCH_MOTOR].control = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    // yaw计算
		// 锯齿波
    motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(SawWaveRun(&gimbal_saw_tooth, gimbal_controller.delta_t));
		// 阶跃
	//	motor_communication[SMALL_YAW_MOTOR].control = Gimbal_Small_Yaw_Calculate(StepRun(&gimbal_step, gimbal_controller.delta_t));
 motor_communication[BIG_YAW_MOTOR].control = Gimbal_Big_Yaw_Calculate(gimbal_controller.target_big_yaw_angle);


	
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
	

//	YawSawTest();
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
    for (int i = 0; i < MOTOR_APP_NUMS; i++)
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
            if (i == BIG_YAW_MOTOR) // 目前仅仅适配yaw轴电机，TODO:适配到所有电机
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

void updataSensors()
{
    // 更新传感器信息
    updateGyro();

    // PITCH电机解码  TODO:扩展到所有类型电机
    if (motor_communication[PITCH_MOTOR].motor_type == GM6020)
    {
        GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    }

    // YAW电机解码  TODO:扩展到所有类型电机
    if (motor_communication[SMALL_YAW_MOTOR].motor_type == GM6020)
    {
        GM6020_Decode(&gimbal_controller.small_yaw_recv, &gimbal_controller.small_yaw_info);
    }

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

/**
 * @brief 云台控制任务
 * @param[in] void
 */
void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 2; // 500HZ

    FrictionWheel_Init();
    GimbalPidInit();
    TogglePidInit();
    DM_Motor_Init(&gimbal_controller.DM_Big_Yaw_Motor, 3.141593f, 20, 45);
    DM_Motor_Init(&gimbal_controller.DM_Pitch_Motor, 3.141593f, 10, 30);

    /* 系统辨识以及测试 */
    SIInit(&SI_obeject, 10, 160.0f);
		//SawToothInit(&saw_tooth_wave, 10, 1, 200, 0);

    vTaskDelay(2000);

    static int index = 0;

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        global_debugger.robot_debugger.dt = DWT_GetDeltaT(&global_debugger.robot_debugger.last_cnt);

        // 更新传感器信息
        updataSensors();

        switch (remote_controller.gimbal_action)
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
        case GIMBAL_SI_MODE:
            Gimbal_SI_Cal();
            break;
        case GIMBAL_TEST_MODE:
            Gimbal_Test_Cal();
            break;
        default:
            Gimbal_Powerdown_Cal();
            break;
        }

        
#if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG
        // 弹舱盖控制函数
        //BombBayControl();
#endif	
        Motor_Data_Pack(); // 电机数据打包
        Motor_Data_Send(); // 电机数据发送

        if (index % 2 == 0)//250HZ
        {
					#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_IFANTRY
            SendtoPC(); // 将信息发送给上位机
					#elif COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY
					SendtoPC(USUAL_PC_DATA); // 将信息发送给上位机
					// 后续补充裁判系统数据发送给NUC
					#endif
        }
        index++;

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
