#include "ShootTask.h"

Shoot_Cmd_t Shoot_Cmd;

/// @brief 辅瞄的开火请求等级
/// @details 当辅瞄请求开火时，此变量会立即来到最大值，随时间递减，最小为 0，此时表示辅瞄打弹请求过期，不再开火
uint8_t aa_fire_req_lvl = 0;

// 最大辅瞄开火频率是 50Hz, 目前辅瞄帧率是 62.5fps，满足此需求
#define AA_Shoot_KeepTime_ms 20
#define AA_Shoot_IterationDuration_ms 2
#define AA_FIRE_LVL_MAX (AA_Shoot_KeepTime_ms / AA_Shoot_IterationDuration_ms)

/// @brief [boolean] 经过校验，确定当前是否开火
/// @details 当有辅瞄开火请求，且满足瞄准、热量等条件后，此值为非零值，否则为 0(false)
uint8_t AA_Shootable = 0;

// namespace Firecode {

/// @brief 上一次辅瞄的开火代码
uint8_t last_AA_firecode = 0;
static int shoot_pos_delay_num = 0;
static uint8_t current_AA_firecode = 0;
static uint8_t pc_shoot_was_online = 0;

/**
 * @brief 射击机构反馈失效时清空闭环状态并保持零电流
 * @note  调用期间不执行正常射击状态机，避免掉线时累计待执行的拨弹位置。
 */
void Shoot_FeedbackSafeStop(void)
{
    PID_Clear(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL]);
    PID_Clear(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL]);
    friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] = 0.0f;
    friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] = 0.0f;

    (void)Toggle_Calculate(TOGGLE_STOP, 0.0f);
    toggle_controller.toggle_state = TOGGLE_NORMAL;
    toggle_controller.reverse_counter = 0;
    toggle_controller.error_counter = 0;
    toggle_controller.is_shoot = 0;

    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = 0.0f;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = 0.0f;
    motor_communication[TOGGLE_MOTOR].control = 0.0f;

    remote_controller.single_shoot_flag = FALSE;
    AA_Shootable = 0;
    aa_fire_req_lvl = 0;
    last_AA_firecode = current_AA_firecode;
    pc_shoot_was_online = 0;
    shoot_pos_delay_num = 0;
}

// uint8_t Firecode_FlipShootState(const uint8_t code) {
//     return (code == 0b00) ? 0b11 : 0b00;
// }

uint8_t Firecode_CheckFireRequest(const uint8_t last, const uint8_t now)
{
    return (last == 0x0 && now == 0x3) || (last == 0x3 && now == 0x0);
}

// }

void Shoot_Powerdown_Cal()
{
    // 摩擦轮高速时进行减速
    if (fabsf(friction_wheels.friction_motor_msgs[LEFT_FRICTION_WHEEL].speed) > 6000 ||
        fabsf(friction_wheels.friction_motor_msgs[RIGHT_FRICTION_WHEEL].speed) > 6000)
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
    if (fabsf(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) > 300)
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
    selectShootFreq();

    // 位置环自动反拨检测
    autoReverse();

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
        // else if (chassis_pack_get_1.is_shootable && toggle_controller.is_shoot)
        else if (toggle_controller.is_shoot)
        {
            remote_controller.single_shoot_flag = FALSE; // 连发情况也将单发标志位清零
            motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * toggle_controller.shoot_freq_speed);
        }
        else
        {
            remote_controller.single_shoot_flag = FALSE; // 不打弹情况将打击标志位清零
                                                         // motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED,0);
            motor_communication[TOGGLE_MOTOR].control = 0;

            // 机械拨盘测试
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
    static int Shoot_IntervalTime = 25; // 2ms进行一次，自增至20次需要40ms，即25hz弹频
    // static int Shoot_IntervalTime =500;

    // 弹频分档 (GimbalTask 500Hz = 2ms 一次, Shoot_IntervalTime 为计数上限):
    //   SHOOT_AUTO_AIM_MODE              = 25  -> 50ms  -> 约 20Hz (常规辅瞄)
    //   SHOOT_UNSTOPPABLE_AUTO_AIM_MODE  = 25  -> 50ms  -> 约 20Hz (强化进攻, 与 UNSTOPPABLE 对齐)
    //   其他                            = 50  -> 100ms -> 约 10Hz (手动)
    if (remote_controller.shoot_action == SHOOT_AUTO_AIM_MODE)
        Shoot_IntervalTime = 25;
    else if (remote_controller.shoot_action == SHOOT_UNSTOPPABLE_AUTO_AIM_MODE)
        Shoot_IntervalTime = 25;
    else
        Shoot_IntervalTime = 50;

    // [FIX] 每周期清零 Shoot_State_send, 仅在实际拨弹时更新为 current_AA_firecode
    // 防止非辅瞄模式或未开火时残留旧值导致上位机误判仍在开火
    Shoot_Cmd.Shoot_State_send = 0;
    // 摩擦轮
    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];

    shoot_pos_delay_num++;

    // 位置环自动反拨检测
    autoReverse();

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

        if (shoot_pos_delay_num > Shoot_IntervalTime)
        {
            if (toggle_controller.is_shoot && (remote_controller.shoot_action == SHOOT_FIRE_MODE ||
                                               remote_controller.shoot_action == SHOOT_AUTO_AIM_MODE ||
                                               remote_controller.shoot_action == SHOOT_UNSTOPPABLE_AUTO_AIM_MODE))
            {
                ToggleAddGrid(&toggle_controller.set_pos, 1);
                shoot_pos_delay_num = 0;
            }
            else if (1 == AA_Shootable)
            {
                ToggleAddGrid(&toggle_controller.set_pos, 1);
                shoot_pos_delay_num = 0;
                Shoot_Cmd.Shoot_State_send = current_AA_firecode;
                last_AA_firecode = current_AA_firecode;
            }
        }
        remote_controller.single_shoot_flag = FALSE; // 不打弹情况将打击标志位清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);

        break;
    case TOGGLE_REVERSE:                             // 反拨
        remote_controller.single_shoot_flag = FALSE; // 反转状态下也将单发标记清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * 300.0f * (-1.0f));
        break;
    default:                                         // 错误卸力
        remote_controller.single_shoot_flag = FALSE; // 异常反转状态下也将单发标记清零
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
        break;
    }

    AA_Shootable = 0; // 重置变量防止浪费子弹
}
#define TOGGLE_SPEED_MODE 0
#define TOGGLE_POS_MODE 1
#define TOGGLE_MODE TOGGLE_POS_MODE
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
    motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, 150.0f);
}

/*
 * 自瞄控制帧无效时只禁止拨弹，不停止摩擦轮预转。这样比赛/辅瞄档不依赖
 * PC控制帧心跳即可保持发射准备，同时不会消费旧的开火沿。
 */
static void Shoot_AutoaimFrictionStandby_Cal(void)
{
    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];

    motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
    toggle_controller.toggle_state = TOGGLE_NORMAL;
    toggle_controller.reverse_counter = 0;
    toggle_controller.error_counter = 0;
    toggle_controller.is_shoot = 0;
    remote_controller.single_shoot_flag = FALSE;
}

void Shoot_Autoaim_Cal()
{
    PCControlSnapshot_t pc_control;

    /* 掉线及恢复首周期都不接受旧开火沿，也不保留待执行的拨弹请求。 */
    if (PCControlGetSnapshot(&pc_control) == 0U)
    {
        pc_shoot_was_online = 0U;
        current_AA_firecode = 0U;
        last_AA_firecode = 0U;
        aa_fire_req_lvl = 0U;
        AA_Shootable = 0U;
        Shoot_AutoaimFrictionStandby_Cal();
        return;
    }

    current_AA_firecode = pc_control.shoot_state;
    if (pc_shoot_was_online == 0U)
    {
        pc_shoot_was_online = 1U;
        last_AA_firecode = current_AA_firecode;
        aa_fire_req_lvl = 0U;
        AA_Shootable = 0U;
        Shoot_AutoaimFrictionStandby_Cal();
        return;
    }

    /*last0,pc3,state3,aashootable,然后send3，last3，pc0，state0，再进poscal，addgrid，如
    如果pc发了新的但是不能拨，state还是0，senddata没有还是0，还是接受到为3，继续进
    */

    if (aa_fire_req_lvl > 0)
        aa_fire_req_lvl--;

    if (Firecode_CheckFireRequest(last_AA_firecode, current_AA_firecode) > 0)
    {
        aa_fire_req_lvl = AA_FIRE_LVL_MAX;
    }

    if (chassis_pack_get_1.is_shootable > 0)
    {
        //        uint8_t i_aimed_target = fabsf(gimbal_controller.gyro_pitch_angle - pc_pitch) < 1.2f && fabsf(gimbal_controller.gyro_yaw_angle - pc_yaw) < 1.5f;
        uint8_t i_aimed_target =
            fabsf(gimbal_controller.gyro_pitch_angle - pc_control.pitch) < 1.0f &&
            fabsf(big_yaw_controller.dealed_big_yaw_gyro - pc_control.yaw) < 1.0f;

        AA_Shootable = i_aimed_target && aa_fire_req_lvl > 0;
    }
    Shoot_Pos_Cal();
}

void Shoot_Supply_Cal()
{
    FrictionWheel_Set(0, 0);
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];

    motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, 0.0f);
}

void Shoot_Unstoppable_Cal()
{
    // 弹频换算: 原 TOGGLE_SPEED 连续转动 900°/s ÷ ONE_GRID_ANGLE(45°/发) = 20Hz
    // GimbalTask 500Hz (2ms/次), 20Hz 周期 50ms → UNSTOPPABLE_SHOOT_INTERVAL = 25 个周期
    // 保持当前弹频不变, 仅把拨盘从连续转动改为位置环单发(一格一格)转动
    static int unstoppable_delay_num = 0;
    const int UNSTOPPABLE_SHOOT_INTERVAL = 25;

    // 摩擦轮: 高弹频下摩擦轮受子弹持续摩擦导致掉速, 提升目标转速以维持实际弹速。
    // setFrictionSpeed() 设置基础目标 (-BULLET_17MM_23MS_SPEED_L/R), 此处叠加补偿量。
    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    //friction_wheels.set_speed_l -= UNSTOPPABLE_FRICTION_BOOST;
    //friction_wheels.set_speed_r -= UNSTOPPABLE_FRICTION_BOOST;
    FrictionWheel_Set(-friction_wheels.set_speed_l, +friction_wheels.set_speed_r);
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL];
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].control = friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL];

    // 拨盘: 位置环单发模式, 由 is_shoot 触发, 每格间隔 UNSTOPPABLE_SHOOT_INTERVAL 个周期
    unstoppable_delay_num++;
    autoReverse();

    switch (toggle_controller.toggle_state)
    {
    case TOGGLE_NORMAL: // 正常状态下
        if (unstoppable_delay_num > UNSTOPPABLE_SHOOT_INTERVAL)
        {
            if (toggle_controller.is_shoot)
            {
                ToggleAddGrid(&toggle_controller.set_pos, 1);
                unstoppable_delay_num = 0;
            }
        }
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
        break;
    case TOGGLE_REVERSE: // 反拨
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * 300.0f * (-1.0f));
        break;
    default: // 错误卸力
        motor_communication[TOGGLE_MOTOR].control = Toggle_Calculate(TOGGLE_STOP, 0.0f);
        break;
    }
}

void Shoot_Cal(void)
{
    // 辅瞄类模式 (含 UNSTOPPABLE_AUTO_AIM) 共享 pc_shoot_was_online / aa_fire_req_lvl / AA_Shootable 上下文,
    // 切到非辅瞄模式时清零, 防止旧开火沿在下次切回辅瞄时被消费。
    if (remote_controller.shoot_action != SHOOT_AUTO_AIM_MODE &&
        remote_controller.shoot_action != SHOOT_UNSTOPPABLE_AUTO_AIM_MODE)
    {
        pc_shoot_was_online = 0U;
        aa_fire_req_lvl = 0U;
        AA_Shootable = 0U;
    }

    // 摩擦轮 Kp 动态调整: 两个 UNSTOPPABLE 模式下弹频较快, 摩擦轮负载扰动更大,
    // Kp 提升至默认值的 1.1 倍 (2.2 -> 2.42) 以加快转速恢复; 其他模式恢复默认值。
    // 每周期设置, 避免模式切换时残留。
    // {
    //     const float FRICTION_KP_DEFAULT = 2.2f;
    //     const float FRICTION_KP_UNSTOPPABLE = 2.2f; // 2.2f * 1.1f
    //     float kp_target = (remote_controller.shoot_action == SHOOT_UNSTOPPABLE_MODE ||
    //                        remote_controller.shoot_action == SHOOT_UNSTOPPABLE_AUTO_AIM_MODE)
    //                           ? FRICTION_KP_UNSTOPPABLE
    //                           : FRICTION_KP_DEFAULT;
    //     friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL].Kp = kp_target;
    //     friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL].Kp = kp_target;
    // }

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
    case SHOOT_UNSTOPPABLE_MODE: // UNSTOPPABLE模式
        Shoot_Unstoppable_Cal();
        break;
    case SHOOT_UNSTOPPABLE_AUTO_AIM_MODE: // 强化进攻姿态下的高速辅瞄射击
        // 复用 Shoot_Autoaim_Cal 的全部辅瞄判定逻辑 (FireCode 沿/AA_Shootable/掉线安全停),
        // 仅在 Shoot_Pos_Cal() 内部按本模式取更高弹频。
        Shoot_Autoaim_Cal();
        break;
    default:
        Shoot_Powerdown_Cal();
        break;
    }
}
