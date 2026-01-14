// #include "BombBay.h"

// enum BOMB_BAY_STATE bomb_bay_state; // 记录弹舱盖状态

// BombBayController bomb_bay_controller;

// void BombBayPidInit(void)
// {
//     // 速度环
//     PID_Init(&bomb_bay_controller.bomb_bay_pos_pid, C610_MAX_CURRENT, 3.0f, 0, 0.03, 0.00, 0, 0, 0, 0, 0, 0, Integral_Limit | OutputFilter);
// }

// /**
//  * @brief 弹舱盖电机初始化
//  * @param[in] void
//  */
// void BombBayMotorInit(void)
// {
//     BombBayPidInit();
//     bomb_bay_controller.disable_bomb_bay = BombBayDisable;
//     bomb_bay_controller.open_bomb_bay = BombBayOpen;
//     bomb_bay_controller.close_bomb_bay = BombBayClose;
//     bomb_bay_controller.init_bomb_bay = BombBayInit;
//     bomb_bay_controller.cover_state = BOMB_BAY_COVER_NOT_INIT;
// }

// /**
//  * @brief 弹舱盖电机打开
//  * @param[in] void
//  */
// void BombBayOpen(void)
// {
//     if (bomb_bay_controller.cover_state != BOMB_BAY_COVER_ON)
//     {
//         bomb_bay_controller.set_speed = -ROTATE_SPEED;
//         bomb_bay_controller.send_current = -1.5f;

//         if (fabsf(bomb_bay_controller.bomb_bay_info.angle - bomb_bay_controller.inti_pos) > (BOMB_BAY_DELTA_POSITION + 5.0f) || fabsf(bomb_bay_controller.bomb_bay_info.angle - bomb_bay_controller.inti_pos) > (BOMB_BAY_DELTA_POSITION - 5.0f))
//         {
//             bomb_bay_controller.cover_state = BOMB_BAY_COVER_ON;
//             PID_Clear(&bomb_bay_controller.bomb_bay_pos_pid);
//         }
//     }
//     else
//     {
//         bomb_bay_controller.set_speed = 0;
//         bomb_bay_controller.send_current = PID_Calculate(&bomb_bay_controller.bomb_bay_pos_pid, bomb_bay_controller.bomb_bay_info.angle, bomb_bay_controller.inti_pos - BOMB_BAY_DELTA_POSITION);
//     }
// }

// /**
//  * @brief 弹舱盖电机关闭(d单环控制)
//  * @param[in] void
//  */
// void BombBayClose(void)
// {
//     if (bomb_bay_controller.cover_state != BOMB_BAY_COVER_OFF)
//     {
//         bomb_bay_controller.send_current = 1.5f;

//         if (fabsf(bomb_bay_controller.bomb_bay_info.angle - bomb_bay_controller.inti_pos) < 5.0f)
//         {
//             bomb_bay_controller.cover_state = BOMB_BAY_COVER_OFF;
//             PID_Clear(&bomb_bay_controller.bomb_bay_pos_pid);
//         }
//     }
//     else
//     {
//         bomb_bay_controller.send_current = PID_Calculate(&bomb_bay_controller.bomb_bay_pos_pid, bomb_bay_controller.bomb_bay_info.angle, bomb_bay_controller.inti_pos);
//     }
// }

// /**
//  * @brief 弹舱盖电机失能(单环控制)
//  * @param[in] void
//  */
// void BombBayDisable(void)
// {
//     bomb_bay_controller.send_current = 0;
// }

// /**
//  * @brief 弹舱盖位置初始化(d单环控制)
//  * @param[in] void
//  */
// void BombBayInit(void)
// {
//     bomb_bay_controller.send_current = 1.5f;
//     if (fabsf(bomb_bay_controller.bomb_bay_info.speed) < 3.0f)
//     {
//         bomb_bay_controller.static_t += bomb_bay_controller.delta_t;
//     }
//     else
//         bomb_bay_controller.static_t = 0;

//     if (bomb_bay_controller.static_t > 0.5f)
//     {
//         bomb_bay_controller.inti_pos = bomb_bay_controller.bomb_bay_info.angle;
//         bomb_bay_controller.is_motor_init = 1;
//         bomb_bay_controller.static_t = 0;
//         bomb_bay_controller.cover_state = BOMB_BAY_COVER_OFF;
//     }
// }

// /**
//  * @brief 弹舱盖电机关闭(d单环控制)
//  * @param[in] void
//  */
// void BombBayControl(void)
// {
//     if (!bomb_bay_controller.is_motor_init)
//     {
//         bomb_bay_controller.usr_input_cmd = INIT_POS; // 关闭
//     }

//     bomb_bay_controller.delta_t = DWT_GetDeltaT(&bomb_bay_controller.last_cnt);
//     bomb_bay_controller.delta_t = LIMIT_MAX_MIN(bomb_bay_controller.delta_t, 0.0025f, 0.0018f);

//     if (bomb_bay_controller.usr_input_cmd == CLOSE_COVER)
//         bomb_bay_controller.close_bomb_bay();
//     else if (bomb_bay_controller.usr_input_cmd == OPEN_COVER)
//         bomb_bay_controller.open_bomb_bay();
//     else if (bomb_bay_controller.usr_input_cmd == DISABLE_COVER)
//         bomb_bay_controller.disable_bomb_bay();
//     else if (bomb_bay_controller.usr_input_cmd == INIT_POS)
//         bomb_bay_controller.init_bomb_bay();
		
// 		motor_communication[BAY_MOTOR].control = bomb_bay_controller.send_current;
// }

// void OpenCoverCommand(void)
// {
//     if (bomb_bay_controller.usr_input_cmd != OPEN_COVER)
//         bomb_bay_controller.last_usr_input_cmd = bomb_bay_controller.usr_input_cmd;

//     bomb_bay_controller.usr_input_cmd = OPEN_COVER;
// }

// void CloseCoverCommand(void)
// {
//     if (bomb_bay_controller.last_usr_input_cmd != CLOSE_COVER && bomb_bay_controller.usr_input_cmd != CLOSE_COVER)
//         bomb_bay_controller.last_usr_input_cmd = bomb_bay_controller.usr_input_cmd;

//     bomb_bay_controller.usr_input_cmd = CLOSE_COVER;
// }

// void DisableCoverCommand(void)
// {

//     if (bomb_bay_controller.cover_state == BOMB_BAY_COVER_OFF)
//     {
//         bomb_bay_controller.usr_input_cmd = DISABLE_COVER;
//     }
//     else
//     {
//         if (bomb_bay_controller.usr_input_cmd != CLOSE_COVER)
//             bomb_bay_controller.last_usr_input_cmd = bomb_bay_controller.usr_input_cmd;
//         bomb_bay_controller.usr_input_cmd = CLOSE_COVER;
//     }
//     PID_Clear(&bomb_bay_controller.bomb_bay_pos_pid);
// }
