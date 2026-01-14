// RotateSpeedEstimate.h
#ifndef __ROTATE_SPEED_ESTIMATE_H
#define __ROTATE_SPEED_ESTIMATE_H

#include "kalman_filter.h"

//标准库中的FreeRTOS头文件
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    // 卡尔曼滤波器实例
    KalmanFilter_t Rotate_Speed_EKF;
    
    // 观测数据源
    float wheel_rotate_speed_average;  // 四轮转换后的角速度估计
    float yaw_motor_speed_minus_imu;      // 云台电机转速与IMU的差值
    
    // 噪声参数
    float Q1;   // 过程噪声方差
    float R1;   // 轮速测量噪声方差
    float R2;   // 云台测量噪声方差
    float rotate_speed_estimate;
} rotate_speed_estimate_t;

// 初始化函数声明
void Rotate_Speed_EKF_Init(rotate_speed_estimate_t *estimator);

// 更新函数声明
float Rotate_Speed_EKF_Update(rotate_speed_estimate_t *estimator);

#endif // __ROTATE_SPEED_ESTIMATE_H
