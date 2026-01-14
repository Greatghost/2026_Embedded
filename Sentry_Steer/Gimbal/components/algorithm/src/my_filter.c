#include "my_filter.h"

void iir(float *raw_data, float new_data, float filter_value)
{
    *raw_data = *raw_data * filter_value + new_data * (1 - filter_value);
}


// 初始化FIR滤波器
// 结构体，滤波器阶数，截止频率和采样频率
void FIRFilter_Init(FIRFilter* filter, int order, float cutoff_freq, float sampling_freq) {
    filter->order = order;
    filter->buffer_index = 0;

    // 计算滤波器系数（理想低通滤波器的脉冲响应）
    float fc = cutoff_freq / sampling_freq; // 归一化截止频率
    for (int n = 0; n <= order; n++) {
        if (n == order / 2) {
            filter->coeffs[n] = 2 * fc; // 中心点
        } else {
            filter->coeffs[n] = arm_sin_f32(2 * PI * fc * (n - order / 2)) / (PI * (n - order / 2));
        }
    }

    // 应用汉宁窗（减少频谱泄漏）
    for (int n = 0; n <= order; n++) {
        filter->coeffs[n] *= 0.5f * (1 - cos(2 * PI * n / order));
    }

    // 初始化延迟线
    for (int i = 0; i <= order; i++) {
        filter->buffer[i] = 0.0f;
    }
}

// 更新滤波器并返回滤波后的值
float FIRFilter_Update(FIRFilter* filter, float input) {
    // 将新输入存入延迟线
    filter->buffer[filter->buffer_index] = input;

    // 计算卷积（滤波器输出）
    float output = 0.0f;
    int index = filter->buffer_index;
    for (int i = 0; i <= filter->order; i++) {
        output += filter->coeffs[i] * filter->buffer[index];
        index--;
        if (index < 0) {
            index = filter->order;
        }
    }

    // 更新延迟线索引
    filter->buffer_index++;
    if (filter->buffer_index > filter->order) {
        filter->buffer_index = 0;
    }

    return output;
}

// 释放滤波器资源
void FIRFilter_Free(FIRFilter* filter) {
    free(filter->coeffs);
    free(filter->buffer);
}