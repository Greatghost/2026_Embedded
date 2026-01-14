#ifndef GIMBAL_ESTIMATE_TASK_H
#define GIMBAL_ESTIMATE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BMI088driver.h"
#include "ins_task.h"

#include "debug.h"

void GimbalEstimate_task(void *pvParameters);

extern IMU_Data_t IMUData;

#endif // !GIMBAL_ESTIMATE_TASK_H
