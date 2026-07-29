/**
 ******************************************************************************
 * @file    ChassisSolver.c
 * @brief   底盘解算器
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChassisSolver.h"

ChassisSolver chassis_solver;

extern BigYawController big_yaw_controller;
extern Nav_Cmd_t NAV_cmd;
extern PC_StateControl PC_statecontrol;
extern JudgeData_1_t JudgeRecieveData;

static void PCNavigationSafeStop(void)
{
    offline_detector.pc_state = PC_OFF;

    NAV_cmd.Nav_Speed_x = 0.0f;
    NAV_cmd.Nav_Speed_y = 0.0f;
    NAV_cmd.Nav_Speed_w = 0.0f;
    PC_statecontrol.CapState = 0U;
    PC_statecontrol.if_through_hole = 0U;
    PC_statecontrol.RotateState = 0U;
    big_yaw_controller.big_yaw_mode = 0U;

    chassis_solver.chassis_speed_x = 0.0f;
    chassis_solver.chassis_speed_y = 0.0f;
    chassis_solver.chassis_speed_w = 0.0f;
    toggle_controller.is_shoot = FALSE;

    setRobotState(CONTROL_MODE);
    setControlModeAction(NOT_FOLLOW_GIMBAL);
    setShootAction(SHOOT_POWERDOWN_MODE);
    setGimbalAction(GIMBAL_ACT_MODE);
    setSuperPower(POWER_TO_BATTERY);
}

/*
 * 比赛/辅瞄档的PC数据失效时，底盘、云台和拨盘仍保持安全停止，但摩擦轮
 * 继续预转。具体的无数据拨盘锁止由Shoot_Autoaim_Cal()负责。
 */
static void PCAutoAimSafeStop(void)
{
    PCNavigationSafeStop();
    setShootAction(SHOOT_AUTO_AIM_MODE);
}

void changeFricAction()
{
    if (remote_controller.shoot_action != SHOOT_POWERDOWN_MODE)
        setShootAction(SHOOT_POWERDOWN_MODE);
    else
        setShootAction(SHOOT_FIRE_MODE);
}

void changeSupplyMode()
{
    if (remote_controller.shoot_action != SHOOT_POWERDOWN_MODE)
        setShootAction(SHOOT_POWERDOWN_MODE);
    else
        setShootAction(SHOOT_SUPPLY_MODE);
}

void PCStateControl() // 比赛专用
{
    PCControlSnapshot_t pc_control;

    if (PCControlGetSnapshot(&pc_control) == 0U)
    {
        PCAutoAimSafeStop();
        return;
    }
    offline_detector.pc_state = PC_ON;

    // 暂时注释比赛开始检查
    // if(JudgeRecieveData.is_game_start ==0)
    // {
    // 	setRobotState(CONTROL_MODE);
    //     setControlModeAction(NOT_FOLLOW_GIMBAL);
    //     setShootAction(SHOOT_FIRE_MODE);
    //     setGimbalAction(GIMBAL_ACT_MODE);
    //     setSuperPower(POWER_TO_BATTERY);
    // 	chassis_solver.chassis_speed_x = 0.f;
    // 	chassis_solver.chassis_speed_y = 0.f;
    // 	chassis_solver.chassis_speed_w = 0.f;
    // }
    // else
    {
        setRobotState(CONTROL_MODE);

        // 暂时注释血量检查
        // if(JudgeRecieveData2.Self_blood == 0)
        // {
        // 	setControlModeAction(NOT_CONTROL_MODE);
        // 	chassis_solver.chassis_speed_x = 0;
        // 	chassis_solver.chassis_speed_y = 0;
        // 	chassis_solver.chassis_speed_w = 0;
        // 	return;
        // }

        if (fabsf(INS.Pitch) > 80.0f || fabsf(INS.Roll) > 60.0f)
        {
            setAllModeOff(); // 翻车检测
            return;
        }

        // 删除过洞模式检查，直接根据 RotateState 设置速度
        // [FIX] FollowMode (FireCode bit4) 优先于 Rotate
        //   FollowMode=1 → SPEED_FOLLOW (底盘 yaw 速度跟随云台 yaw)
        //   FollowMode=0 且 rotate_state==0 → NOT_FOLLOW_GIMBAL
        //   FollowMode=0 且 rotate_state!=0 → CV_ROTATE (小陀螺分档)
        if (pc_control.follow_mode == 1U)
        {
            setControlModeAction(SPEED_FOLLOW);
            chassis_solver.chassis_speed_w = 0.f;
        }
        else if (pc_control.rotate_state == 0U)
        {
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            chassis_solver.chassis_speed_w = 0.f;
        }
        else
        {
            setControlModeAction(CV_ROTATE);
            switch (pc_control.rotate_state)
            {
            case 1:
                chassis_solver.chassis_speed_w = 0.5f * MAX_YAW_SPEED;
                break;
            case 2:
                chassis_solver.chassis_speed_w = 0.75f * MAX_YAW_SPEED;
                break;
            case 3:
                chassis_solver.chassis_speed_w = 1.0f * MAX_YAW_SPEED;
                break;

            default:
                chassis_solver.chassis_speed_w = 0.0f * MAX_YAW_SPEED;
                break;
            }
        }

        chassis_solver.chassis_speed_x = pc_control.nav_speed_x;
        chassis_solver.chassis_speed_y = pc_control.nav_speed_y;

        if (pc_control.cap_state == 1U)
        {
            setSuperPower(POWER_TO_SuperPower);
        }
        else
        {
            setSuperPower(POWER_TO_BATTERY);
        }

        // 射击模式: 强化进攻姿态 (current_posture==4, sentry_cmd bit21-23 提取) 进入
        //          SHOOT_UNSTOPPABLE_AUTO_AIM_MODE (辅瞄判定 + 高弹频);
        //          其他姿态维持 SHOOT_AUTO_AIM_MODE
        if (current_posture == 4U)
            setShootAction(SHOOT_UNSTOPPABLE_AUTO_AIM_MODE);
        else
            setShootAction(SHOOT_AUTO_AIM_MODE); // 辅瞄爽打
        setGimbalAction(GIMBAL_AUTO_AIM_MODE);

        // 摇杆推上/下 → 手动打弹（自瞄模式下也可手动触发）
        if (fabsf(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 330)
            toggle_controller.is_shoot = TRUE;
        else
            toggle_controller.is_shoot = FALSE;
    }
}

void DJIKeyMouseUpdate(ChassisSolver *infantry)
{
    uint8_t R_flag = 0;
    uint8_t Hole_flag = 0;
    const uint8_t pc_control_online = PCControlIsOnline();
    if (offline_detector.remote_state == REMOTE_OFF)
    {
        setAllModeOff();
        return;
    }

    if (pc_control_online == 0U &&
        (remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE ||
         remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE ||
         remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE))
    {
        setGimbalAction(GIMBAL_ACT_MODE);
    }
    // 记得设置好接口可以跳转回遥控器模式
    if (remote_controller.dji_remote.rc.s[RIGHT_SW] != Mid)
    {
        initRemoteControl(DJI_REMOTE_CONTROL);
        // setControlMode(DJI_REMOTE_CONTROL);
    }
    else
    {

#if ROBOT == GOBLIN
        if (remote_controller.dji_remote.rc.s[LEFT_SW] == Up) // 高自由度活动
        {
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_FIRE_MODE);
            // setGimbalAction(GIMBAL_ACT_MODE);//为了大、小符
            if (remote_controller.gimbal_action == GIMBAL_POWERDOWN) // 复位
            {
                setGimbalAction(GIMBAL_ACT_MODE); // 为了大、小符
            }
        }
        else if (remote_controller.dji_remote.rc.s[LEFT_SW] == Mid)
        {
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_FIRE_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
        }
        else
        {
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE); // 可环顾四周
        }

#elif ROBOT == TIGER

        if (remote_controller.dji_remote.rc.s[LEFT_SW] == Up) // 高自由度活动
        {
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_FIRE_MODE);
            // setGimbalAction(GIMBAL_ACT_MODE);//为了大、小符
            if (remote_controller.gimbal_action == GIMBAL_POWERDOWN) // 复位
            {
                setGimbalAction(GIMBAL_ACT_MODE); // 为了大、小符
            }
        }
        else if (remote_controller.dji_remote.rc.s[LEFT_SW] == Mid)
        {
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_FIRE_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
        }
        else
        {
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE); // 可环顾四周
        }

#endif

        // 飞坡标志位 & 被动电容标志位
        SetFlyMode(NOT_FLY);             // 默认关闭飞坡
        setSuperPower(POWER_TO_BATTERY); // 默认关闭被动电容

        // 按键检测
        volatile uint16_t keyValue = remote_controller.dji_remote.keyValue;
        remote_controller.dji_remote.keyChangeOn = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) &
                                                   remote_controller.dji_remote.keyValue;
        remote_controller.dji_remote.keyChangeOff = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) &
                                                    remote_controller.dji_remote.last_keyValue;
        remote_controller.dji_remote.last_keyValue = keyValue;

        /* 按键操作 */

        float speed_x = 0.0f, speed_y = 0.0f;
        for (uint16_t i = 1; i > 0; i <<= 1)
        {
            uint16_t key_and = keyValue & i;
            switch (key_and) // 按键
            {
            case KEY_B:
                break;
            case KEY_V:
                Hole_flag = 1;
                // setControlModeAction(FOLLOW_GIMBAL);
                break;
            case KEY_SHIFT:
                setSuperPower(POWER_TO_SuperPower);
                break;
            case KEY_CTRL:
                setControlModeAction(CV_ROTATE);
                break;
            case KEY_Q:
                break;
            case KEY_E:
                break;
            case KEY_R:
                R_flag = 1; // 反拨、大符、小符
                break;
            case KEY_F:
                break;
            case KEY_G:
                SetFlyMode(IS_FLY);
                break;
            case KEY_Z:
                break;
            case KEY_X:
                break;
            case KEY_C:
                Hole_flag = 1;
                // setControlModeAction(FOLLOW_GIMBAL);
                break;
            case KEY_D:
                speed_x += 1.0f;
                break;
            case KEY_A:
                speed_x -= 1.0f;
                break;
            case KEY_S:
                speed_y -= 1.0f;
                break;
            case KEY_W:
                speed_y += 1.0f;
                break;
            default:
                break;
            }
            key_and = remote_controller.dji_remote.keyChangeOn & i;
            switch (key_and) // 按键上升沿
            {
            case KEY_B:
                //								if(R_flag)
                //
                //								//	bomb_bay_controller.is_motor_init = 0;//卡死过，重新初始化
                //								else
                //								{
                //                if (bomb_bay_controller.cover_state == BOMB_BAY_COVER_ON)
                //                 //   CloseCoverCommand();
                //                else
                //                    OpenCoverCommand();
                //								}
                ;
                break;
            case KEY_V:
                break;
            case KEY_SHIFT:
                break;
            case KEY_CTRL:

                break;
            case KEY_Q: // 切换底盘形态
                if (remote_controller.chassis_format == CROSS_MODE)
                    remote_controller.chassis_format = X_MODE;
                else
                    remote_controller.chassis_format = CROSS_MODE;
                break;

            case KEY_E:
                break;

            case KEY_R:
                break;

            case KEY_F:
                break;

            case KEY_G:
                break;
            case KEY_Z:
                if (R_flag && pc_control_online != 0U) // 小符
                {
                    setGimbalAction(GIMBAL_SMALL_BUFF_MODE);
                }
                else
                {
                    setGimbalAction(GIMBAL_ACT_MODE);
                }

                break;
            case KEY_X:
                if (R_flag && pc_control_online != 0U) // 大符
                {
                    setGimbalAction(GIMBAL_BIG_BUFF_MODE);
                }
                else
                {
                    setGimbalAction(GIMBAL_ACT_MODE);
                }
                break;
            case KEY_C:
                break;
            case KEY_D:
                break;
            case KEY_A:
                break;
            case KEY_S:
                break;
            case KEY_W:
                break;
            default:
                break;
            }
            key_and = remote_controller.dji_remote.keyChangeOff & i;

            switch (key_and) // 按键下降沿
            {
            case KEY_B:
                break;
            case KEY_V:
                break;
            case KEY_SHIFT:
                break;
            case KEY_CTRL:
                chassis_solver.Rotate_Counter++;
                break;
            case KEY_Q:
                break;
            case KEY_E:

                gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ += 180;

                break;
            case KEY_R:
                break;
            case KEY_F:
                break;
            case KEY_G:
                break;
            case KEY_Z:
                break;
            case KEY_X:
                break;
            case KEY_C:
                break;
            case KEY_D:
                break;
            case KEY_A:
                break;
            case KEY_S:
                break;
            case KEY_W:
                break;
            default:
                break;
            }
        }

        chassis_solver.chassis_speed_x = speed_x;
        chassis_solver.chassis_speed_y = speed_y;

        // 小陀螺变向
        if (remote_controller.control_mode_action == CV_ROTATE)
        {
            chassis_solver.chassis_speed_w = 1.0f;
            // 两次按Ctrl，陀螺变方向
            if (chassis_solver.Rotate_Counter % 2 == 1)
                chassis_solver.chassis_speed_w *= -1;
        }
        else
            chassis_solver.chassis_speed_w = 0.0f;

        // 过洞底盘跟随 并 限制功率从而缓速移动
        if (Hole_flag)
        {
            setControlModeAction(SPEED_FOLLOW);
            chassis_send_pack1.through_hole_flag = 1;
        }
        else
            chassis_send_pack1.through_hole_flag = 0;

        // 鼠标操作
        if (remote_controller.gimbal_action == GIMBAL_ACT_MODE || remote_controller.gimbal_action == GIMBAL_TEST_MODE)
        {
            // Yaw控制：仅在正常模式下响应遥控器
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= remote_controller.dji_remote.mouse.x * 0.005f;
#endif
        }

        // Pitch控制：仅在正常模式下响应遥控器
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
        gimbal_controller.target_pitch_angle -= remote_controller.dji_remote.mouse.y * 0.005f;
        gimbal_controller.target_pitch_angle -= remote_controller.dji_remote.mouse.z * 0.001f;
#endif

        // 鼠标左键检测
        volatile unsigned char press_l = remote_controller.dji_remote.mouse.press_l;
        remote_controller.dji_remote.mouse.mouseChangeOn_l = (remote_controller.dji_remote.mouse.last_press_l ^ press_l) &
                                                             press_l;
        remote_controller.dji_remote.mouse.mouseChangeOff_l = (remote_controller.dji_remote.mouse.last_press_l ^ press_l) &
                                                              remote_controller.dji_remote.mouse.last_press_l;
        remote_controller.dji_remote.mouse.last_press_l = press_l;

        // 按键操作，连发情况
        if (press_l)
        {
            toggle_controller.is_shoot = TRUE;
        }
        else
        {
            toggle_controller.is_shoot = FALSE;
        }
        if (remote_controller.dji_remote.mouse.mouseChangeOn_l)
        {
            remote_controller.single_shoot_flag = TRUE;
        }

        volatile unsigned char press_r = remote_controller.dji_remote.mouse.press_r;
        remote_controller.dji_remote.mouse.mouseChangeOn_r = (remote_controller.dji_remote.mouse.last_press_r ^ press_r) &
                                                             press_r;
        remote_controller.dji_remote.mouse.mouseChangeOff_r = (remote_controller.dji_remote.mouse.last_press_r ^ press_r) &
                                                              remote_controller.dji_remote.mouse.last_press_r;
        remote_controller.dji_remote.mouse.last_press_r = press_r;

        // 辅瞄
        if (press_r && pc_control_online != 0U &&
            remote_controller.gimbal_action == GIMBAL_ACT_MODE)
        {
            setGimbalAction(GIMBAL_AUTO_AIM_MODE);
        }

        // 退出辅瞄
        if (remote_controller.dji_remote.mouse.mouseChangeOff_r && remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE)
        {
            setGimbalAction(GIMBAL_ACT_MODE);
        }

        if (offline_detector.pitch_motor_state == PITCH_MOTOR_OFF && offline_detector.yaw_motor_state == YAW_MOTOR_OFF)
        {
            setGimbalAction(GIMBAL_POWERDOWN); // 可环顾四周
        }
    }
}

void blueToothStateUpdate(ChassisSolver *infantry)
{
    // switch (remote_controller.blue_tooth_key)
    // {
    // case BLUE_TOOTH_UP:
    //     infantry->target_v = 0.5f;
    //     break;
    // case BLUE_TOOTH_DOWN:
    //     infantry->target_v = -0.5f;
    //     break;
    // case BLUE_TOOTH_STABLE:
    //     infantry->target_v = 0;
    //     infantry->target_yaw_v = 0;
    //     break;
    // case BLUE_TOOTH_LEFT:
    //     infantry->target_yaw_v += 0.5f;
    //     break;
    // case BLUE_TOOTH_RIGHT:
    //     infantry->target_yaw_v -= 0.5f;
    //     break;
    // default:
    //     break;
    // }
    // remote_controller.blue_tooth_key = BLUE_TOOTH_NO_ACTION;
}

void setAllModeOff()
{
    setRobotState(OFFLINE_MODE);
    setControlModeAction(NOT_CONTROL_MODE);
    setShootAction(SHOOT_POWERDOWN_MODE);
    setGimbalAction(GIMBAL_POWERDOWN);
    chassis_solver.chassis_speed_x = 0.0f;
    chassis_solver.chassis_speed_y = 0.0f;
    chassis_solver.chassis_speed_w = 0.0f;
    NAV_cmd.Nav_Speed_x = 0.0f;
    NAV_cmd.Nav_Speed_y = 0.0f;
    NAV_cmd.Nav_Speed_w = 0.0f;
    toggle_controller.is_shoot = FALSE;
    // DisableCoverCommand();
    big_yaw_controller.gimbal_last_mode = 0;
}

// 遥控器模式部分保留，其余调整为哨兵专用模式
// 锯齿波测试
SawToothWave gimbal_saw_tooth;
StepFunction gimbal_step;
int saw_tooth_init_flag = 0, step_init_flag = 0;

/*
    遥控器操作模式：
        左上：
            右上：打弹测试
            右中：云台检录
            右下：普通移动
        左中：
            右上：辅瞄测试
            右中：比赛模式，会小陀螺
            右下：导航模式，不会小陀螺
        左下：
            右上：小陀螺,另一个方向
            右中：小陀螺
            右下：下电
*/

void DJIRemoteUpdate(ChassisSolver *infantry)
{
    PCControlSnapshot_t pc_control;
    const uint8_t pc_control_online = PCControlGetSnapshot(&pc_control);
    int leg_len_switch = 0;
    // 判断状态
    switch (remote_controller.dji_remote.rc.s[LEFT_SW])
    {
    case Down:
        switch (remote_controller.dji_remote.rc.s[RIGHT_SW])
        {
        case Down:
            setAllModeOff();
            // 测试锯齿波用
            saw_tooth_init_flag = 0;
            step_init_flag = 0;
            break;
        case Mid:

            // 检录小陀螺
            setRobotState(CONTROL_MODE);
            setControlModeAction(CV_ROTATE);
            setShootAction(SHOOT_POWERDOWN_MODE); // 先不打弹
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_BATTERY);

            // 云台控制 - Yaw
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 云台控制 - Pitch
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_X_SPEED;
            chassis_solver.chassis_speed_y = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_Y_SPEED;
            chassis_solver.chassis_speed_w = -0.5f * MAX_YAW_SPEED;
            // 检录要求变向小陀螺
            //   if (remote_controller.control_mode_action == CV_ROTATE)
            //   {
            //       chassis_solver.chassis_speed_w = 0.5f * MAX_YAW_SPEED;
            //       // 陀螺变向
            //       if (remote_controller.dji_remote.rc.Previous_rc_Right_SW != Mid)
            //           chassis_solver.Rotate_Counter++;
            //       if (chassis_solver.Rotate_Counter % 2 == 1)
            //           chassis_solver.chassis_speed_w *= -1;

            //   }
            // 			else
            // 				chassis_solver.chassis_speed_w = 0.0f;

            break;
        case Up:
            // 底盘不动，检录射击模式
            setRobotState(CONTROL_MODE);
            setControlModeAction(CV_ROTATE);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_SuperPower);

            // OpenCoverCommand();

            // 云台控制 - Yaw
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 云台控制 - Pitch
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_X_SPEED;
            chassis_solver.chassis_speed_y = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_Y_SPEED;
            chassis_solver.chassis_speed_w = 0.75f * MAX_YAW_SPEED;

            //            if((remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 330)
            //            {
            //                toggle_controller.is_shoot = TRUE;
            //            }
            //            else
            //            {
            //                toggle_controller.is_shoot = FALSE;
            //
            //            }

            break;
        default:
            setAllModeOff();
            // 测试锯齿波用
            saw_tooth_init_flag = 0;
            step_init_flag = 0;
            break;
        }

        break;
    case Mid:
        // 测导航模式：右拨杆之前在Down位置时触发
        if (remote_controller.dji_remote.rc.Previous_rc_Right_SW == Down)
        {
            if (pc_control_online == 0U)
            {
                PCNavigationSafeStop();
                break;
            }
            offline_detector.pc_state = PC_ON;

            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_BATTERY);

            // 云台保持当前角度不变
            // 底盘执行NUC发来的命令
            chassis_solver.chassis_speed_x = pc_control.nav_speed_x;
            chassis_solver.chassis_speed_y = pc_control.nav_speed_y;
            chassis_solver.chassis_speed_w = 0.f;
        }
        else
        {
            switch (remote_controller.dji_remote.rc.s[RIGHT_SW])
            {
            case Down:
                PCStateControl(); // NUC模式，允许旋转
                break;
            case Mid:
                PCStateControl(); // 比赛专用
                break;
            case Up:
                // 辅瞄测试
                if (pc_control_online == 0U)
                {
                    PCAutoAimSafeStop();
                    break;
                }
                offline_detector.pc_state = PC_ON;

                setRobotState(CONTROL_MODE);
                setControlModeAction(NOT_CONTROL_MODE);
                setShootAction(SHOOT_AUTO_AIM_MODE);
                setGimbalAction(GIMBAL_AUTO_AIM_MODE);
                setSuperPower(POWER_TO_BATTERY);

                // OpenCoverCommand();

                // 云台控制
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
                gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
#endif
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
                gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
#endif

                // 底盘控制
                chassis_solver.chassis_speed_x = 0; //(remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE;
                chassis_solver.chassis_speed_y = 0; //(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE;
                chassis_solver.chassis_speed_w = 0;

                // 摇杆推上/下 → 手动打弹（自瞄模式下也可手动触发）
                if (fabsf(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 0)
                    toggle_controller.is_shoot = TRUE;
                else
                    toggle_controller.is_shoot = FALSE;

                break;
            default:
                setAllModeOff();
                break;
            }
        }

        break;
    case Up:
        // 左上为遥控器控制
        switch (remote_controller.dji_remote.rc.s[RIGHT_SW])
        {
        case Down:
            // 整车运动，底盘跟随，不打弹
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_BATTERY);
            big_yaw_controller.big_yaw_mode = 0;
            saw_tooth_init_flag = 0;

            // OpenCoverCommand();

            // 云台控制
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
#endif

#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_X_SPEED;
            chassis_solver.chassis_speed_y = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_Y_SPEED;
            chassis_solver.chassis_speed_w = 0.f;
            break;
        case Mid:

            // 超电或者小陀螺，现在为锯齿波测试
            setRobotState(CONTROL_MODE);
            setControlModeAction(SPEED_FOLLOW);
            setShootAction(SHOOT_POWERDOWN_MODE); // 先不打弹
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_SuperPower);
            big_yaw_controller.big_yaw_mode = 0;

            // CloseCoverCommand();
            //
            if (0 == saw_tooth_init_flag)
            {
                // 10度
                SawToothInit(&gimbal_saw_tooth, 6.0f, 1, 250, gimbal_controller.gyro_yaw_angle);
                // yaw 30度响应测试，t单位为ms
                // sawtoothinit(&gimbal_saw_tooth,30.0f, 1, 1000, gimbal_controller.gyro_yaw_angle);
                saw_tooth_init_flag = 1;
            }
            // if(0 == step_init_flag)
            // {
            //     //yaw 30度响应测试
            //     StepInit(&gimbal_step,gimbal_controller.gyro_yaw_angle,30,1);
            // }

            // saw_tooth_init_flag = 1;
            // step_init_flag = 1;

            // 云台控制
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
#endif

#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_X_SPEED;
            chassis_solver.chassis_speed_y = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_Y_SPEED;
            chassis_solver.chassis_speed_w = 0; // 0.5f*MAX_YAW_SPEED;

            break;
        case Up:
            // 底盘不动，打弹
            setRobotState(CONTROL_MODE);
            setControlModeAction(SPEED_FOLLOW);
            setGimbalAction(GIMBAL_ACT_MODE);

            // 射击模式选择: 仅由左摇杆(物理, =RIGHT_CH)决定, 右摇杆(物理, =LEFT_CH)专心控云台瞄准
            //   - 左摇杆上推 (> 0) → UNSTOPPABLE 连续高速射击
            //   - 左摇杆居中                → FIRE 模式, is_shoot=FALSE (待机)
            //   - 左摇杆下推 (< 0) → FIRE 模式, is_shoot=TRUE (单发)
            if (((int)remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 0)
            {
                setShootAction(SHOOT_UNSTOPPABLE_MODE);
            }
            else
            {
                setShootAction(SHOOT_FIRE_MODE);
            }
            setSuperPower(POWER_TO_BATTERY);
            big_yaw_controller.big_yaw_mode = 0;

            // OpenCoverCommand();

            // 云台控制
#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_SMALLYAW_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_big_yaw_angle /* [SMALL_YAW_REMOVED] 原为target_small_yaw */ -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
#endif

#if (GIMBAL_TEST_CONFIG != GIMBAL_CONFIG_PITCH_SQUARE && GIMBAL_CONTROL_DISCONNECT == 0)
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
#endif

            // 底盘控制
            //             chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_X_SPEED;
            //            chassis_solver.chassis_speed_y = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE * MAX_Y_SPEED;
            chassis_solver.chassis_speed_w = 0.f * MAX_YAW_SPEED;
            if (fabsf(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 330)
            {
                toggle_controller.is_shoot = TRUE;
            }
            else
            {
                toggle_controller.is_shoot = FALSE;
            }

            break;
        default:
            setAllModeOff();
            break;
        }
        break;
    default:
        setAllModeOff();
        break;
    }

    remote_controller.dji_remote.rc.Previous_rc_Right_SW = remote_controller.dji_remote.rc.s[RIGHT_SW]; // 给小陀螺变向用

    // if (offline_detector.remote_state == REMOTE_OFF)
    // {
    //     setAllModeOff();
    // }
    // 哨兵会关遥控器
}

/**
 * @brief 根据遥控器或者蓝牙更新控制状态
 * @param[in] infantry
 */
void get_control_info(ChassisSolver *infantry)
{
    switch (remote_controller.control_type)
    {
    case BLUE_TOOTH:
        blueToothStateUpdate(infantry);
        break;
    case DJI_REMOTE_CONTROL:
        DJIRemoteUpdate(infantry);
        break;
    case KEY_MOUSE:
        DJIKeyMouseUpdate(infantry);
    default:
        break;
    }
}
