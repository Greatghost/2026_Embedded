#ifndef _FILTER_H
#define _FILTER_H

#include "user_lib.h"
#include "arm_math.h"
#include "stdlib.h"
#define MAX_ORDER 50 // 定义最大滤波器阶数

// FIR滤波器结构体
typedef struct {
    int order;          // 滤波器阶数
    float coeffs[MAX_ORDER + 1]; // 滤波器系数数组
    float buffer[MAX_ORDER + 1]; // 延迟线（用于存储历史输入）
    int buffer_index;   // 延迟线索引
} FIRFilter;


void FIRFilter_Init(FIRFilter* filter, int order, float cutoff_freq, float sampling_freq);
float FIRFilter_Update(FIRFilter* filter, float input);
void FIRFilter_Free(FIRFilter* filter);

void iir(float *raw_data, float new_data, float filter_value);

#endif // !_FILTER_H
