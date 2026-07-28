#ifndef _BUZZER_ALARM_TASK_H
#define _BUZZER_ALARM_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/* 蜂鸣器告警任务 (2026-07-19新增)
 * 监听底盘电机掉线状态 (CAN 0x0A1 → motor_offline_recv.motor_offline_bitmap)
 * - bit0-3 : 轮电机1-4 掉线 → 分别响 1~4 声
 * - bit4-7 : 舵电机1-4 掉线 → 分别响 5~8 声
 * - 多个电机同时掉线 → 按编号顺序依次响
 * - 无掉线 → 静音
 * - CAN 数据超过 3 秒未刷新 → 视为通信失联，停止响铃
 */
void BuzzerAlarmTask(void const * argument);

// 测试用,平时注释此行
//#define TestBuzzerAlarmTask

#endif // _BUZZER_ALARM_TASK_H
