#include "ShootTask.h"

/**
  * @brief  打蛋数量估计
  * @param  None
  * @retval None
  */
 uint32_t ShootCountTime_last=0;
 float ShootCount_dt=0;
 float ShootCount_IntervalTime = 0;//上次射击数量估计距今的时间
 int ShootCount_Number = 0;//射击的总子弹数
 int ShootCount_Number_1 = 0; 
 #define ShootInterval_Min_time 50
 void ShootCount_Cal(void)
 {
     static short ShootMode_last = 0;
     ShootCount_dt = DWT_GetDeltaT(&ShootCountTime_last);
     ShootCount_IntervalTime+=ShootCount_dt*1000;
     
     if(remote_controller.last_shoot_action != remote_controller.shoot_action){
         ShootCount_IntervalTime = -200;//切换摩擦轮模式后设置一段时间保护，防止被认为射击
     }
     if(friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL].Err>80 && friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL].Err < 1000.0)//满足掉速要求
     {
         if(ShootCount_IntervalTime>ShootInterval_Min_time){//连射的最小间隔
             //ShootAble = 0;
             ShootCount_Number++;//总射击子弹数
             //Shoot.HeatControl.CurShootNumber++;//不再需要
             //ShootCount_Number_1 = Shoot.HeatControl.CurShootNumber; 
             ShootCount_IntervalTime = 0;//只在此处重置时间
             //23.11.19 增加打蛋处理完成标志
             //24.12.27 暂时去除
             //Shoot_Cmd.Shoot_State_send = Shoot_Cmd.Shoot_State;
         }
     }
     remote_controller.last_shoot_action = remote_controller.shoot_action;
 }
 void Shoot_Powerdown_Cal()
 {
     // 摩擦轮高速时进行减速
     if (fabsf(friction_wheels.friction_motor_msgs[LEFT_FRICTION_WHEEL].speed) > 6000 || fabsf(friction_wheels.friction_motor_msgs[RIGHT_FRICTION_WHEEL].speed > 6000))
     {
         FrictionWheel_Set(0, 0);
     }
     // 减小到一定程度后设置为 0
     else
     {
         PID_Clear(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL]);
         PID_Clear(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL]);
 
         friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] = 0;
         friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] = 0;
     }
 
     motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
     motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];
 
     // 拨盘电机
     motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
 
     toggle_controller.toggle_state = TOGGLE_NORMAL;
     toggle_controller.reverse_counter = 0;
     toggle_controller.error_counter = 0;
 }
 
 void Shoot_Check_Cal() // 检录打弹模式
 {
     // 摩擦轮
     setFrictionSpeed(chassis_pack_get_1.bullet_level);
 
     // 测试
     FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
     motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
     motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];
 
     // 拨盘
     // 设置拨盘转动速度
         if(fabsf(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 300)
             toggle_controller.shoot_freq_speed = 500.0f;
         else
             toggle_controller.shoot_freq_speed = 0.0f;
         
     // 自动反拨检测
     autoReverse();
 
     // PID计算
     remote_controller.single_shoot_flag = FALSE; // 连发情况也将单发标志位清零
 
     switch (toggle_controller.toggle_state)
     {
     case TOGGLE_NORMAL: // 正常状态下
         motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * toggle_controller.shoot_freq_speed);
         break;
     case TOGGLE_REVERSE: // 反拨
         motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * 100.0f * (-1.0f));
         break;
     default: // 错误卸力
         motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
         break;
     }
 }
 
 void Shoot_Speed_Cal() // 速度模式
 {
    // 摩擦轮
    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];

    // 拨盘
    // 设置拨盘转动速度
    selectShootFreq(chassis_pack_get_1.robot_level, chassis_pack_get_1.buff_state);

    // 位置环自动反拨检测
    //autoReverse();


//	if(remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE)//辅瞄时上位机决定射击与否		
//		//		chassis_pack_get_1.is_shootable = chassis_pack_get_1.is_shootable & pc_recv_data.shoot_flag;
//	{
//		if(chassis_pack_get_1.is_shootable && pc_recv_data.shoot_flag)
//			chassis_pack_get_1.is_shootable = 1;
//		else
//			chassis_pack_get_1.is_shootable = 0;
//	}

    switch (toggle_controller.toggle_state)
    {
    case TOGGLE_NORMAL: // 正常状态下
        if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE || remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
        {
            // 大符模式，采用单发模式
            if (remote_controller.single_shoot_flag && chassis_pack_get_1.is_shootable) // 触发单发射击标志
            {
                ToggleAddGrid(&toggle_controller.set_pos, 1);
                remote_controller.single_shoot_flag = FALSE;
            }
            motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
        }
        //else if (chassis_pack_get_1.is_shootable && toggle_controller.is_shoot)
   else if (toggle_controller.is_shoot)
                {
            remote_controller.single_shoot_flag = FALSE; // 连发情况也将单发标志位清零
            motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * toggle_controller.shoot_freq_speed);
        }
        else
        {
            remote_controller.single_shoot_flag = FALSE; // 不打弹情况将打击标志位清零
                        //motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED,0);
    motor_communication[TOGGLE_MOTOR].control = 0;

                    //机械拨盘测试
                }
        break;
    case TOGGLE_REVERSE:                             // 反拨
        remote_controller.single_shoot_flag = FALSE; // 反转状态下也将单发标记清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * 100.0f * (-1.0f));
        break;
    default:                                         // 错误卸力
        remote_controller.single_shoot_flag = FALSE; // 异常反转状态下也将单发标记清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
        break;
    }
 }
 
 void Shoot_Pos_Cal() // 位置模式
 {
    static int bodanLastPos; 
	static uint32_t last_time = 0;
	static int Shoot_IntervalTime = 40;//40ms拨一格，25hz,暂且先10hz实验
	static int delay_num = 0;
    static uint8_t multi_shoot_flag;
    // 摩擦轮
    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];

    delay_num++;

    // 位置环自动反拨检测
    //autoReverse();

    switch (toggle_controller.toggle_state)
    {
    case TOGGLE_NORMAL: // 正常状态下
        if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE || remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
        {
            // 大符模式，采用单发模式
            if (remote_controller.single_shoot_flag && chassis_pack_get_1.is_shootable) // 触发单发射击标志
            {
                ToggleAddGrid(&toggle_controller.set_pos, 1);
                remote_controller.single_shoot_flag = FALSE;
            }
            motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
        }
        if (delay_num > Shoot_IntervalTime/2)
        {
            if(toggle_controller.is_shoot)  
            {
                multi_shoot_flag = 1;//连续开火
            }
            else
            {
                multi_shoot_flag = 0;//停止连续开火
            }
        }
        if(delay_num>Shoot_IntervalTime)
        {
            if(toggle_controller.is_shoot)
            {
                ToggleAddGrid(&toggle_controller.set_pos, 1);
                delay_num = 0;
            }
            
            
        }
            remote_controller.single_shoot_flag = FALSE; // 不打弹情况将打击标志位清零
                //motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED,0);
            motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);

        
        //else if (chassis_pack_get_1.is_shootable && toggle_controller.is_shoot)
        // else if (toggle_controller.is_shoot)
        // {
        //     if(delay_num > Shoot_IntervalTime/2)
        //     {
        //         multi_shoot_flag = 1;//连续开火
        //     }

        //     if(delay_num > Shoot_IntervalTime)//后续需要加热量控制！！！！
		//     {
        //         //	bodanLastPos = Bodan_Pos;
        //         delay_num = 0;
        //         ToggleAddGrid(&toggle_controller.set_pos, 1);
        //         motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);

			
		//     }
        //     remote_controller.single_shoot_flag = FALSE; // 连发情况也将单发标志位清零
        // }
        // else
        // {
        //     remote_controller.single_shoot_flag = FALSE; // 不打弹情况将打击标志位清零
        //         //motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED,0);
        //     motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);

        //         //机械拨盘测试
        // }
        break;
    case TOGGLE_REVERSE:                             // 反拨
        remote_controller.single_shoot_flag = FALSE; // 反转状态下也将单发标记清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * 100.0f * (-1.0f));
        break;
    default:                                         // 错误卸力
        remote_controller.single_shoot_flag = FALSE; // 异常反转状态下也将单发标记清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
        break;
    }



 }
 #define TOGGLE_SPEED_MODE 0
 #define TOGGLE_POS_MODE 1
 #define TOGGLE_MODE TOGGLE_SPEED_MODE
 void Shoot_Fire_Cal()
 {
    #if TOGGLE_MODE == TOGGLE_POS_MODE
        Shoot_Pos_Cal();
    #else
        Shoot_Speed_Cal();
    #endif
    
     
 }
 
 void Shoot_Test_Cal()
 {
     // 摩擦轮
     setFrictionSpeed(chassis_pack_get_1.bullet_level);
     FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
     motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
     motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];
 
     // 拨盘
     motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, -150.0f);
 }
 
 void Shoot_Autoaim_Cal()
 {
     // 弹舱盖
 
     // 拨盘
 
     // 摩擦轮
 }
 
 void Shoot_Supply_Cal()
 {
     FrictionWheel_Set(0, 0);
     motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
     motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];
 
     motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, 0.0f);
 }

void ShootTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1000hz

    FrictionWheel_Init();
    TogglePidInit();

    vTaskDelay(2000);

    static int index = 0;

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

       // global_debugger.robot_debugger.dt = DWT_GetDeltaT(&global_debugger.robot_debugger.last_cnt);

        
        

        switch (remote_controller.shoot_action)
        {
        case SHOOT_POWERDOWN_MODE: // 掉电模式
            Shoot_Powerdown_Cal();
            break;
        case SHOOT_CHECK_MODE: // 自检模式
            Shoot_Check_Cal();
            break;
        case SHOOT_FIRE_MODE: // 开火模式
            Shoot_Fire_Cal();
            break;
        case SHOOT_TEST_MODE: // 弹道测试模式
            Shoot_Test_Cal();
            break;
        case SHOOT_AUTO_AIM_MODE: // 自瞄模式
            Shoot_Autoaim_Cal();
            break;
        case SHOOT_SUPPLY_MODE: // 补给模式
            Shoot_Supply_Cal();
            break;
        default:
            Shoot_Powerdown_Cal();
            break;
        }


        

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}