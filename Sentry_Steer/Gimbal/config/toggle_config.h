#ifndef _TOGGLE_CONFIG_H
#define _TOGGLE_CONFIG_H
#include "robot_config.h"

// 拨弹参数总览

#if ROBOT == GOBLIN
    #define ONE_GRID_ANGLE 45.0f // 度
    #define SIGN_ROTATE 1.0f    // 拨盘正转动方向 1.0f表示逆时针为正拨  -1.0表示逆时针为反拨
#elif ROBOT == TIGER
		#define ONE_GRID_ANGLE 45.0f // 度
    #define SIGN_ROTATE -1.0f    // 拨盘正转动方向 1.0f表示逆时针为正拨  -1.0表示逆时针为反拨
#endif

#endif