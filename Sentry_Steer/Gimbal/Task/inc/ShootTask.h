#ifndef _SHOOTTASK_H
#define _SHOOTTASK_H


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ChassisSolver.h"
#include "can_config.h"

#include "ChassisSend.h"

#include "bsp_can.h"
#include "can.h"

/*
 * 遥控器向上射击参数:
 *   N: 每组连续发射的弹丸数
 *   f: 平均弹频，单位为发/秒(Hz)
 * 组周期 = 1000 * N / f，默认 3 发/组、24Hz，即 125ms/组。
 */
#define REMOTE_UP_BURST_SIZE_N       1U
#define REMOTE_UP_FIRE_RATE_F_HZ     16U

#if REMOTE_UP_BURST_SIZE_N == 0U
#error "REMOTE_UP_BURST_SIZE_N must be greater than zero"
#endif

#if REMOTE_UP_FIRE_RATE_F_HZ == 0U
#error "REMOTE_UP_FIRE_RATE_F_HZ must be greater than zero"
#endif

/* 四舍五入到最接近的整数毫秒。 */
#define REMOTE_UP_BURST_PERIOD_MS                                            \
    ((1000U * REMOTE_UP_BURST_SIZE_N + REMOTE_UP_FIRE_RATE_F_HZ / 2U) /     \
     REMOTE_UP_FIRE_RATE_F_HZ)

#if REMOTE_UP_BURST_PERIOD_MS == 0U
#error "The configured remote-up burst period is below the 1ms timer resolution"
#endif

typedef struct{
	uint8_t Friction_cmd;
	uint8_t Shoot_State; // only 2 ( at mask 0b11) bits are used
	uint8_t Shoot_State_send; // only 2 (at mask 0b11) bits are used
	uint8_t Shoot_Freq_cmd;
} Shoot_Cmd_t;

extern Shoot_Cmd_t shoot_cmd;
extern GimbalController gimbal_controller;
void ShootTask(void *pvParameters);
void Shoot_Cal(void);
void Shoot_FeedbackSafeStop(void);


#endif
