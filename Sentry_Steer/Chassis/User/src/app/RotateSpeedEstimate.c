// 底盘小陀螺转速估计 
#include "RotateSpeedEstimate.h"
#include <string.h>

rotate_speed_estimate_t Rotate_Speed_EKF;
float wheels_speed_real[4]; 
float yaw_motor_raw_speed;
float imu_speed;
float rotate_speed_after_EKF;

// 状态转移矩阵 F（1x1单位矩阵，航向角速度自身传递）
const float KF_F[1] = {1.0f};

// 观测矩阵 H（2x1矩阵，两个观测量均直接观测角速度）
const float KF_H[2] = {1.0f, 1.0f};


void Rotate_Speed_EKF_Init(rotate_speed_estimate_t *estimator) {
    /* 初始化1维卡尔曼滤波器（无控制输入，2个观测）*/
    Kalman_Filter_Init(&estimator->Rotate_Speed_EKF, 1, 0, 2);
    estimator->Q1 = 1.0f;
   
    estimator->R1 = 0.01f;
    estimator->R2 = 1.0f;

    estimator.Rotate_Speed_EKF.xhat_data[0] = 

    /* 系统动态模型配置 --------------------------------------------*/
    // 状态转移矩阵F（单位矩阵）
    estimator->Rotate_Speed_EKF.F_data[0] = 1.0f;

    // 初始协方差矩阵P（高不确定性）
    estimator->Rotate_Speed_EKF.P_data[0] = 10.0f;

    // 过程噪声协方差Q（来自系统特性）
    estimator->Rotate_Speed_EKF.Q_data[0] = estimator->Q1;

    /* 观测模型配置 ----------------------------------------------*/
    // 测量映射：所有观测都对应唯一状态量（索引0）
    uint8_t measurement_map[5] = {0, 0, 0, 0, 0};
    memcpy(estimator->Rotate_Speed_EKF.MeasurementMap, 
           measurement_map, 
           sizeof(measurement_map));

    // 测量系数（四轮和云台都直接对应omega）
    float measurement_degree[5] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f}; // 修正点：云台系数改为1.0
    memcpy(estimator->Rotate_Speed_EKF.MeasurementDegree,
           measurement_degree,
           sizeof(measurement_degree));

    // 测量噪声配置（四轮用R1，云台用R2）
    float R_diag[5];
    for(int i = 0; i < 4; i++) R_diag[i] = estimator->R1;
    R_diag[4] = estimator->R2;
    memcpy(estimator->Rotate_Speed_EKF.MatR_DiagonalElements,
           R_diag,
           sizeof(R_diag));

    /* 滤波器稳定性配置 -----------------------------------------*/
    // 设置状态最小方差防止过度收敛
    estimator->Rotate_Speed_EKF.StateMinVariance[0] = 0.01f;

    // 启用自动矩阵调整
    estimator->Rotate_Speed_EKF.UseAutoAdjustment = 1;
}

float Rotate_Speed_EKF_Update(rotate_speed_estimate_t *estimator) {
    /* 更新四轮观测值 ------------------------------------------*/
    for(int i = 0; i < 4; i++) {
        estimator->Rotate_Speed_EKF.MeasuredVector[i] = 
            estimator->wheel_rotate_speed_average[i];
    }

    /* 处理云台观测值 ------------------------------------------*/
    // 云台差速观测公式：yaw_motor_speed - imu_speed = -2*omega
    // 因此omega = -0.5 * (yaw_motor_speed - imu_speed)
    estimator->Rotate_Speed_EKF.MeasuredVector[4] = 
        -0.5f * estimator->yaw_motor_speed_minus_imu;

    /* 执行卡尔曼滤波更新 --------------------------------------*/
    Kalman_Filter_Update(&estimator->Rotate_Speed_EKF);

    /* 返回滤波结果（符号已通过观测处理修正）-------------------*/
    return estimator->Rotate_Speed_EKF.FilteredValue[0]; // 修正点：移除不必要的负号??
}

float Rotate_Speed_Estimate(rotate_speed_estimate_t *estimator){
    estimator->wheel_rotate_speed_average = 0.f;
    for(i=0; i<4; i++){
        estimator->wheel_rotate_speed_average += estimator->wheels_speed_real[i];
    }
    estimator->wheel_rotate_speed_average /= 4.f;
    estimator->yaw_motor_speed_minus_imu = estimator->yaw_motor_raw_speed - estimator->imu_speed;
    estimator->rotate_speed_estimate = Rotate_Speed_EKF_Update(estimator);
    return estimator->rotate_speed_estimate;
    
}

void RotateEstimateTask(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(pdMS_TO_TICKS(1000));
    Rotate_Speed_EKF_Init(&Rotate_Speed_EKF);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        rotate_speed_after_EKF = RotateEstimate(&Rotate_Speed_EKF);

        //xEventGroupSetBits(xCreatedEventGroup, GIMBAL_TASK_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}