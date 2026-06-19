#include "can_send_config.h"

// 发送结构体配置(注意不影响接收)
Motor_Communication motor_communication[MOTOR_APP_NUMS];

void Motor_Config_Init()
{

#if ROBOT == GOBLIN
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_id = 0x202; // 电调实际ID
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID

    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].motor_id = 0x201; // 电调实际ID
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];

    motor_communication[PITCH_MOTOR].can = CAN2;
    motor_communication[PITCH_MOTOR].motor_id = 0x06;
    motor_communication[PITCH_MOTOR].motor_id_type = DM_MOTOR_2;
    motor_communication[PITCH_MOTOR].motor_type = DM_MOTOR;
    motor_communication[PITCH_MOTOR].std_id = MOTOR_STD_ID_LIST[DM_MOTOR_2];

    motor_communication[BIG_YAW_MOTOR].can = CAN1;
    motor_communication[BIG_YAW_MOTOR].motor_id = 0x05;
    motor_communication[BIG_YAW_MOTOR].motor_id_type = DM_MOTOR_1;
    motor_communication[BIG_YAW_MOTOR].motor_type = DM_MOTOR;
    motor_communication[BIG_YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DM_MOTOR_1];

    // [SMALL_YAW_REMOVED] 小Yaw电机通信配置已删除
    // motor_communication[SMALL_YAW_MOTOR].can = CAN2;
    // motor_communication[SMALL_YAW_MOTOR].motor_id = 0x205;
    // motor_communication[SMALL_YAW_MOTOR].motor_id_type = DJI_0x1FE;
    // motor_communication[SMALL_YAW_MOTOR].motor_type = GM6020;
    // motor_communication[SMALL_YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x1FE];

    motor_communication[TOGGLE_MOTOR].can = CAN1;
    motor_communication[TOGGLE_MOTOR].motor_id = 0x204;
    motor_communication[TOGGLE_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[TOGGLE_MOTOR].motor_type = M2006;
    motor_communication[TOGGLE_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];

    // motor_communication[BAY_MOTOR].can = CAN2;
    // motor_communication[BAY_MOTOR].motor_id = 0x204;
    // motor_communication[BAY_MOTOR].motor_id_type = DJI_0x200;
    // motor_communication[BAY_MOTOR].motor_type = M2006;
    // motor_communication[BAY_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];
		
#elif ROBOT == TIGER

		motor_communication[LEFT_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_id = 0x202; // 电调实际ID
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[LEFT_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID

    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].motor_id = 0x201; // 电调实际ID
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[RIGHT_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];

    motor_communication[PITCH_MOTOR].can = CAN2;
    motor_communication[PITCH_MOTOR].motor_id = 0x06;
    motor_communication[PITCH_MOTOR].motor_id_type = DM_MOTOR_2;
    motor_communication[PITCH_MOTOR].motor_type = DM_MOTOR;
    motor_communication[PITCH_MOTOR].std_id = MOTOR_STD_ID_LIST[DM_MOTOR_2];

    motor_communication[BIG_YAW_MOTOR].can = CAN1;
    motor_communication[BIG_YAW_MOTOR].motor_id = 0x05;
    motor_communication[BIG_YAW_MOTOR].motor_id_type = DM_MOTOR_1;
    motor_communication[BIG_YAW_MOTOR].motor_type = DM_MOTOR;
    motor_communication[BIG_YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DM_MOTOR_1];

    // [SMALL_YAW_REMOVED] 小Yaw电机通信配置已删除
    // motor_communication[SMALL_YAW_MOTOR].can = CAN2;
    // motor_communication[SMALL_YAW_MOTOR].motor_id = 0x205;
    // motor_communication[SMALL_YAW_MOTOR].motor_id_type = DJI_0x1FE;
    // motor_communication[SMALL_YAW_MOTOR].motor_type = GM6020;
    // motor_communication[SMALL_YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x1FE];

    motor_communication[TOGGLE_MOTOR].can = CAN1;
    motor_communication[TOGGLE_MOTOR].motor_id = 0x204;
    motor_communication[TOGGLE_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[TOGGLE_MOTOR].motor_type = M2006;
    motor_communication[TOGGLE_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];

    // motor_communication[BAY_MOTOR].can = CAN2;
    // motor_communication[BAY_MOTOR].motor_id = 0x204;
    // motor_communication[BAY_MOTOR].motor_id_type = DJI_0x200;
    // motor_communication[BAY_MOTOR].motor_type = M2006;
    // motor_communication[BAY_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];

#endif
}