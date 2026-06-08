#ifndef _MOTOR_TYPEDEF
#define _MOTOR_TYPEDEF

#include "struct_typedef.h"
#include "can.h"

typedef enum MOTOR_ID_TYPE
{
    DJI_0x1FF,
    DJI_0x1FE,
    DJI_0x2FF,
    DJI_0x2FE,
    DJI_0x200,
    DM_MOTOR_1,
		DM_MOTOR_2,
    MOTOR_MF9025,
    SEND_ID_NUMS // 一定放到最后,指代所有可能发送的电机ID
} MOTOR_ID_TYPE;

typedef enum MOTOR_TYPE
{
    GM6020,
    M3508,
    M2006,
    DM_MOTOR
} MOTOR_TYPE;

typedef enum MOTOR_APP_TYPE // 电机应用类型
{
    LEFT_FRICTION_WHEEL_MOTOR ,
    RIGHT_FRICTION_WHEEL_MOTOR,
    PITCH_MOTOR,
    BIG_YAW_MOTOR,
    // [SMALL_YAW_REMOVED] 删除小Yaw, 保留枚举值占位以避免后续索引错乱
    // SMALL_YAW_MOTOR,
    TOGGLE_MOTOR,
		//BAY_MOTOR,
    MOTOR_APP_NUMS
} MOTOR_APP_TYPE;

// 电机通信结构体
typedef struct Motor_Communication
{
    CAN_TypeDef *can;            // 发送CAN选择
    uint32_t std_id;             // 发送帧ID
    uint32_t motor_id;           // 电机ID 0x201-0x208
    MOTOR_ID_TYPE motor_id_type; // 电机ID类型
    MOTOR_TYPE motor_type;       // 电机类型
    float control;               // 电机控制电流/电压
} Motor_Communication;

extern uint32_t MOTOR_STD_ID_LIST[SEND_ID_NUMS];
extern uint32_t motor_wait_time[2][SEND_ID_NUMS];
extern CAN_TxHeaderTypeDef motor_tx_header[2][SEND_ID_NUMS];

#endif