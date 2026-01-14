#ifndef __GRAPHICS_SEND_TASK_H
#define __GRAPHICS_SEND_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#include "stdint.h"
#include "GimbalReceive.h"
#include "GimbalSend.h"
#include "SuperPower.h"

#include "bsp_referee.h"
#include "Referee.h"

#include "algorithmOfCRC.h"
#include "remote_control.h"
#include "ins.h"
#include "ChasisController.h"
#include "PowerControlTask.h"

#include "iwdg.h"

#define IMAGE_CENTER_X 1920 / 2 // 图像X中心
#define IMAGE_CENTER_Y 1080 / 2 // 图像Y中心

#define CAP_BAR_LENGTH 250 // 电容UI长度
#define CAP_BAR_WIDTH 20   // 电容UI高度

#define CAP_BAR_UI_START_X IMAGE_CENTER_X - CAP_BAR_LENGTH / 2 + CAP_BAR_WIDTH / 2 // 起始绘制X
#define CAP_BAR_UI_START_Y 20                                                      // 起始绘制Y

#define CHASSIS_POS_LENGTH 100 // 底盘UI长度
#define CHASSIS_POS_WIDTH 100   // 底盘UI高度

#define CHASSIS_POS_UI_START_X 1650 // 起始绘制X
#define CHASSIS_POS_UI_START_Y 700 // 起始绘制Y       

#define GIM_CHASSIS_ANGLE_LINE_LEN 150

//不同车底盘跟随模式不同，一些车有固定头的方向，加入一个偏置来画UI中云台方向
#if ROBOT == NIUNIU
#define UI_FRONT_BIAS 8160
#elif ROBOT == CHEN_JING_YUAN
#define UI_FRONT_BIAS 0
#elif ROBOT == NIU_MO_SON
#define UI_FRONT_BIAS 0
#elif ROBOT == QI_TIAN_DA_SHENG
#define UI_FRONT_BIAS 0
#elif ROBOT == TIGER 
#define UI_FRONT_BIAS 0
#endif

extern float UI_FRONT_ERR,UI_FRONT_SIN,UI_FRONT_COS;

void Refereetask(void *pvParameters);

#endif
