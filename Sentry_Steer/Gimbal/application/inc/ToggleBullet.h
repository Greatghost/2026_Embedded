#ifndef _TOGGLE_BULLET_H
#define _TOGGLE_BULLET_H

#include "pid.h"
#include "M2006.h"
#include "toggle_config.h"

enum TOGGLE_CONTRL_MODE
{
  TOGGLE_SPEED, // 拨弹速度控制(连发)
  TOGGLE_POS,   // 拨弹单发控制(单发)
  TOGGLE_STOP
};

typedef enum ToggleState
{
  TOGGLE_NORMAL,  // 正常
  TOGGLE_ERROR,   // 拨弹错误
  TOGGLE_REVERSE, // 反拨
} ToggleState;

/* 拨弹控制器 */
typedef struct ToggleController
{
  PID_t toggle_pos_pid;
  PID_t toggle_speed_pid; // 拨弹速度环
  M2006_Recv toggle_recv;
  M2006_Info toggle_info;

  float set_pos;
  float set_speed;
  int8_t is_shoot; // 发射指令

  float shoot_freq_speed; // 根据机器人等级设置的拨盘速度

  //	反拨部分
  ToggleState toggle_state;
  uint8_t rev_shoot; // 速度环反拨指令
  int32_t reverse_counter;
  int32_t error_counter; // 拨弹错误计数
} ToggleController;

#if ROBOT == QI_TIAN_DA_SHENG || ROBOT == GOBLIN || ROBOT == TIGER
#define TOGGLE_START_CURRENT 15.5f // 拨弹开始时电流,超过此值但拨盘转不动即为堵转
#elif ROBOT == NIU_MO_SON || ROBOT == CHEN_JING_YUAN
#define TOGGLE_START_CURRENT 4.5f // 拨弹开始时电流,超过此值但拨盘转不动即为堵转
#endif

extern ToggleController toggle_controller;

void TogglePidInit(void);
float Toggle_Calculate(enum TOGGLE_CONTRL_MODE control_mode, float set_point);
void ToggleAddGrid(float *set_point, float N);
void selectShootFreq(void);
void autoReverse(void);

#endif // !_TOGGLE_BULLET_H
