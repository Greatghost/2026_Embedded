#ifndef _POWER_LIMIT_H
#define _POWER_LIMIT_H

#include "SuperPower.h"
#include "Referee.h"

#include "remote_control.h"
#include "Motor_Typdef.h"

#include "robot_config.h"

// 预测功率 P = I^2*R + K*W*I + P0

// MF9025电机
#define MF9025_R 0.06437f
#define MF9025_K 0.002871f
#define MF9025_B 1.33f
#define MF9025_P0 7.675f

// M3508电机
#if ROBOT == CHEN_JING_YUAN
#define M3508_R 0.1761f
#define M3508_B 0.0057f
#define M3508_K 0.3262f
#define M3508_P0 1.33f

#elif ROBOT == NIU_MO_SON
#define M3508_R 0.1873f
#define M3508_B 0.0067f
#define M3508_K 0.3294f
#define M3508_P0 1.33f // 后续加上6020的一起

#elif ROBOT == NIUNIU
#define M3508_R 0.1250f
#define M3508_B 0.0058f
#define M3508_K 0.2513f
#define M3508_P0 1.217475f // 注意这个是平均每个轮子的功率

#elif ROBOT == QI_TIAN_DA_SHENG
//#define M3508_R 0.1253f
//#define M3508_B 0.0040f
//#define M3508_K 0.2178f
//#define M3508_P0 2.2f //多控4w
#define M3508_R 0.1165f
#define M3508_B 0.0064f
#define M3508_K 0.2506f
#define M3508_P0 2.8f //多控2w

#elif ROBOT == TIGER
//#define M3508_R 0.2281f
#define M3508_R 0.1901f
#define M3508_K 0.2638f
#define M3508_B 0.0040f
#define M3508_P0 2.0f //多控2w
#define GM6020_R 6.7981f
#define GM6020_K 0.1231f
#define GM6020_B 0.0113f



#endif

typedef enum Power_Arrange
{
	NOT_ARRANGE,	// 未进行功率分配
	NEG_ARRANGE,	// 负功率分配
	NEED_ARRANGE,	// 需要功率分配
	NORMAL_ARRANGE, // 正常分配
	ARRANGE_ERROR,	// 功率分配错误
} Power_Arrange;

typedef enum PowerLimitMethod
{
	TORQUE_REDUCE_METHOD, // 转矩缩减方法
	SPEED_ERROR_METHOD,	  // 转速误差方法
} PowerLimitMethod;

typedef struct
{
	float motor_w[4];
	float motor_I[4];


	float motor_a[4]; // 中间计算参量
	float motor_b[4];
	float motor_c[4];

	float a;
	float b;
	float c;//一元二次方程各项

	float motor_R;
	float motor_K;
	float motor_B; 
	float motor_P0; 


	Power_Arrange power_arrange_state[4];

	float motor_P[4];				  // 计算预测功率
	float send_torque_lower_scale[4]; // 功率限制之后电机缩放系数
	int8_t is_arrange_power[4];		  // 是否已经进行功率分配


	// float actual_ina260_power;	// 电流计获得的实际电机功率
	// float actual_referee_power; // 裁判系统反馈功率（若连接超级电容，不能代表电机）
	float predict_send_power;	// 发送转矩电流会造成电机的总输出功率

	float can_arrange_power;  // 计算除了负功率外需要分配的功率值
	float need_arrange_power; // 需要分配的功率(功率分配模式)

	float need_arrange_w; // 转速误差分配模式

	float set_power;  // 预期最大功率值
	int8_t motor_num; // 电机数
	MOTOR_TYPE motor_type;

	// sum_w_i sum_i_2
	float sum_w_i;
	float sum_i_2;
	float sum_w_2;


	float motor_w_error[4];

	PowerLimitMethod power_limit_method;


} Torque_Scaler;

typedef struct
{
	Torque_Scaler wheels_scaler;
	Torque_Scaler Steer_scaler;

	
	// float actual_ina260_power;	// 电流计获得的实际电机功率
	// float actual_referee_power; // 裁判系统反馈功率（若连接超级电容，不能代表电机）
} PowerLimiter;

// 限制最高四个电机
void PowerLimit(PowerLimiter *limitter, float set_power);
void PowerLimitInit(PowerLimiter *limitter, int motor_num, MOTOR_TYPE motor_type, PowerLimitMethod method);

#endif // !_POWER_LIMIT_H
