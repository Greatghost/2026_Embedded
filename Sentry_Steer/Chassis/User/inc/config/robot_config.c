#include "ChasisController.h"

void setRobotType()
{

#if ROBOT == QI_TIAN_DA_SHENG
    infantry.chassis_type = OMNI_WHEEL;
    infantry.yaw_motor_type = YAW_DM_MOTOR;
    infantry.chassis_follow_type = FOUR_SIDES_FOLLOW;
    infantry.power_limit_method = SPEED_ERROR_METHOD;
#elif ROBOT == TIGER
	infantry.chassis_type = STEER_WHEEL;
    infantry.yaw_motor_type = YAW_DM_MOTOR;
    infantry.chassis_follow_type = TWO_SIDES_FOLLOW;
    infantry.power_limit_method = TORQUE_REDUCE_METHOD;
	
#endif

    if (infantry.chassis_type == MECANUM_WHEEL)
    {
        mecanum_pid_init();
    }
    else if (infantry.chassis_type == OMNI_WHEEL)
    {
        omni_pid_init();
    }
    else if (infantry.chassis_type == STEER_WHEEL)
    {
        steer_pid_init();
    }
}
