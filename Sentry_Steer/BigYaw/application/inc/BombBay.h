// #ifndef _BOMB_BAY_H
// #define _BOMB_BAY_H

// #include "pid.h"
// #include "M2006.h"
// #include "debug.h"
// #include "can_send_config.h"

// #define BOMB_BAY_DELTA_POSITION 115.0f

// #define BOMB_BAY_MOTOR_SPIN_DIR 1
// #define ROTATE_SPEED 25.0f

// enum BOMB_BAY_STATE
// {
//     BOMB_BAY_COVER_OFF,
//     BOMB_BAY_COVER_ON,
//     BOMB_BAY_COVER_NOT_INIT
// };

// enum BOMB_BAY_ACTION
// {
//     CLOSE_COVER,
//     OPEN_COVER,
//     DISABLE_COVER,
//     INIT_POS
// };

// /* 弹舱盖控制器 */
// typedef struct BombBayController
// {
//     PID_t bomb_bay_pos_pid;
//     M2006_Recv bomb_bay_recv;
//     M2006_Info bomb_bay_info;

//     float set_pos;
//     float set_speed;
//     float send_current; // 发送电流值

//     float is_motor_init;
//     float inti_pos; // 上电时电机编码器值

//     uint8_t if_bomb_bay_init;

//     float delta_t;
//     float static_t;
//     uint32_t last_cnt;

//     enum BOMB_BAY_ACTION usr_input_cmd;      // 用户输入命令
//     enum BOMB_BAY_ACTION last_usr_input_cmd; // 用户输入命令
//     enum BOMB_BAY_STATE cover_state;         // 弹舱盖状态

//     void (*disable_bomb_bay)(); // 初始化函数
//     void (*open_bomb_bay)();    // 打开弹舱盖
//     void (*close_bomb_bay)();   // 关闭弹舱盖
//     void (*init_bomb_bay)();    // 初始化

// } BombBayController;

// extern BombBayController bomb_bay_controller;
// extern enum BOMB_BAY_STATE bomb_bay_state;

// void BombBayPidInit(void);
// void BombBayOpen(void);
// void BombBayClose(void);
// void BombBayDisable(void);
// void BombBayMotorInit(void);
// void BombBayControl(void);
// void BombBayInit(void);

// void OpenCoverCommand(void);
// void CloseCoverCommand(void);
// void DisableCoverCommand(void);

// #endif
