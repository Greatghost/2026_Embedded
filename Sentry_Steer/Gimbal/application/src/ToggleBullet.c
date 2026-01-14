#include "ToggleBullet.h"

ToggleController toggle_controller;

/* 根据机器人等级以及增益来确定射频 */
void selectShootFreq()
{
    // 外部会使用SIGN_ROTATE符号进行纠正，这里的正转全为正，反转为负号
    /* 分别为机器人的等级，以及有无增益 */
//#if ROBOT == GOBLIN
//    static float shoot_freq[10][2] = {{700, 500}, {700, 500}, {700, 500}, {700, 500}, {700, 500}, {700, 500}, {700, 500}, {700, 500}, {700, 500}, {700, 500}}; // omni
//#endif
//    uint8_t robot_level_ = robot_level > 10 ? 1 : (robot_level <= 0 ? 1 : robot_level);
//    uint8_t shoot_buff_ = (shoot_buff & 0x02) >> 1; // 取第二位冷却增益
    //toggle_controller.shoot_freq_speed = shoot_freq[robot_level_ - 1][shoot_buff_] * 1.2f;
toggle_controller.shoot_freq_speed = 500;

}

/**
 * @brief 拨盘转速检测&自动反转(拨盘正常后删)
 * @param[in] void
 */
void autoReverse(void)
{
    // 状态机切换状态
    switch (toggle_controller.toggle_state)
    {
    case TOGGLE_NORMAL:
        if ((SIGN_ROTATE * toggle_controller.toggle_info.torque_current - TOGGLE_START_CURRENT) >= 0.0f && fabs(toggle_controller.toggle_info.speed) < 10.0f)
        {
            toggle_controller.reverse_counter++; // 超过力且拨盘转不动，则累加
        }
        else
        {
            toggle_controller.reverse_counter = 0;
        }
        if (fabs(toggle_controller.toggle_speed_pid.Err) > 700.0f) // 转速发散保护
        {
            toggle_controller.error_counter++; // 超过拨盘转速，则累加
        }
        else
        {
            toggle_controller.error_counter = 0;
        }
        if (toggle_controller.reverse_counter >= 200 || toggle_controller.error_counter >= 50)
        {
            toggle_controller.toggle_state = TOGGLE_ERROR; // 状态错误
            toggle_controller.reverse_counter = 0;         // 清零状态
            toggle_controller.error_counter = 0;           // 清零状态
        }
        break;
    case TOGGLE_REVERSE:
        toggle_controller.reverse_counter++;
        if (toggle_controller.reverse_counter >= 30) // 反拨时间设置
        {
            toggle_controller.toggle_state = TOGGLE_NORMAL; // 状态切到正常
            toggle_controller.reverse_counter = 0;          // 清零状态
        }
        break;
    case TOGGLE_ERROR: // 错误状态，则进行卸力处理
        toggle_controller.reverse_counter++;
        if (toggle_controller.reverse_counter >= 40) // 卸力时间设置
        {
            toggle_controller.toggle_state = TOGGLE_REVERSE; // 状态错误
            toggle_controller.reverse_counter = 0;           // 清零状态
        }
        break;
    default:
        toggle_controller.toggle_state = TOGGLE_NORMAL;
        break;
    }
}

void TogglePidInit()
{
#if ROBOT == GOBLIN
    // OMNI 7_18
	PID_Init(&toggle_controller.toggle_pos_pid, 1400.0f, 0, 0, 12, 0, 0, 0, 0, 0, 0, 1, NONE);                                           // OMNI
//    PID_Init(&toggle_controller.toggle_speed_pid, 8.0f, 4.0f, 0, 0.08, 0.02, 0, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate); // omni
		PID_Init(&toggle_controller.toggle_speed_pid, 16.0f, 2.7f, 0, 0.0265, 0.0064, 0, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate); // omni
#elif ROBOT == TIGER
// 
	PID_Init(&toggle_controller.toggle_pos_pid, 1400.0f, 0, 0, 12, 0, 0, 0, 0, 0, 0, 1, NONE);                                           // OMNI
//    PID_Init(&toggle_controller.toggle_speed_pid, 8.0f, 4.0f, 0, 0.08, 0.02, 0, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate); // omni
	PID_Init(&toggle_controller.toggle_speed_pid, 16.0f, 2.7f, 0, 0.0265, 0.0064, 0, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate); // omni

	
	#endif

    toggle_controller.toggle_state = TOGGLE_NORMAL;
    toggle_controller.reverse_counter = 0;
    toggle_controller.error_counter = 0;
}

/**
 * @brief 拨弹电机控制计算
 * @param[in] control_mode 控制模式
 * @param[in] set_point 目的位置(位置控制)，目标速度(速度控制)
 */
float Toggle_Calculate(enum TOGGLE_CONTRL_MODE control_mode, float set_point)
{
    if (control_mode == TOGGLE_SPEED)
    {
        toggle_controller.set_pos = toggle_controller.toggle_info.angle;
        toggle_controller.set_speed = set_point;

        PID_Clear(&toggle_controller.toggle_pos_pid);
        return PID_Calculate(&toggle_controller.toggle_speed_pid, toggle_controller.toggle_info.speed, set_point);
    }
    else if (control_mode == TOGGLE_POS)
    {
        toggle_controller.set_speed = PID_Calculate(&toggle_controller.toggle_pos_pid, toggle_controller.toggle_info.angle, set_point);
        return PID_Calculate(&toggle_controller.toggle_speed_pid, toggle_controller.toggle_info.speed, toggle_controller.set_speed);
    }
    else
    {
        PID_Clear(&toggle_controller.toggle_pos_pid);
        PID_Clear(&toggle_controller.toggle_speed_pid);
        toggle_controller.set_pos = toggle_controller.toggle_info.angle;
        toggle_controller.set_speed = 0;
        return 0;
    }
}

/**
 * @brief 拨弹电机推动N格
 * @param[in] set_point 目标位置
 * @param[in] N 推动N格
 */
void ToggleAddGrid(float *set_point, float N)
{
	if(toggle_controller.toggle_state == TOGGLE_NORMAL)
  {
		// 设置值为当前值加上一格大小
    *set_point = toggle_controller.set_pos + ONE_GRID_ANGLE * N * SIGN_ROTATE;
	}
	else{
		*set_point = toggle_controller.toggle_info.angle;
		
	}
}
