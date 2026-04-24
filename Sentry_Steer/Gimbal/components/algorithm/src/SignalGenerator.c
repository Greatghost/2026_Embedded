/**
 ******************************************************************************
 * @file	 SignalGenerator.c
 * @brief    信号发生器定义
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "SignalGenerator.h"

void StepInit(StepFunction *step, float initial_value, float step_amplitude, float start_time)
{
    step->start_time = start_time;
    step->time = 0;
    step->initial_value = initial_value;
    step->step_amplitude = step_amplitude;
}
float StepRun(StepFunction *step, float delta_t)
{
    step->time += delta_t;
    if (step->time > step->start_time)
    {
        return step->initial_value + step->step_amplitude;
    }
    else
    {
        return step->initial_value;
    }
}

void SinInit(SinFunction *sin_function, float amplitude, float start_time, uint16_t T)
{
    sin_function->amplitude = amplitude;
    sin_function->start_time = start_time;
    sin_function->cycle_count = 0;
    sin_function->T = T;
    sin_function->w = 2 * PI * 1000.0f / (float)T;
    sin_function->time = 0;
}

float SinRun(SinFunction *sin_function, float delta_t)
{
    sin_function->time += delta_t;
    if (sin_function->time < sin_function->start_time)
    {
        return 0.0f;
    }
    float angle = sin_function->w * (sin_function->time - sin_function->start_time);

    if (angle > 2 * PI * (sin_function->cycle_count + 1))
    {
        sin_function->cycle_count++;
    }

    return sin_function->amplitude * arm_sin_f32(angle);
}

void SawToothInit(SawToothWave *saw_tooth_wave, float amplitude, float start_time, uint16_t T, float initial_value)
{
    saw_tooth_wave->T = T;
    saw_tooth_wave->amplitude = amplitude;
    saw_tooth_wave->start_time = start_time;
    saw_tooth_wave->time = 0;
    saw_tooth_wave->cycle_count = 0;
    saw_tooth_wave->initial_value = initial_value;
    saw_tooth_wave->out = initial_value;
    saw_tooth_wave->slope = amplitude * 1000.0f / (float)T;
}

float SawWaveRun(SawToothWave *saw_tooth_wave, float delta_t)
{
    saw_tooth_wave->time += delta_t;
    if (saw_tooth_wave->time < saw_tooth_wave->start_time)
    {
        saw_tooth_wave->out = saw_tooth_wave->initial_value;
        return saw_tooth_wave->initial_value;
    }

    if ((saw_tooth_wave->time - saw_tooth_wave->start_time) > (saw_tooth_wave->cycle_count + 1) * (float)saw_tooth_wave->T / 1000.0f)
    {
        saw_tooth_wave->cycle_count++;
        saw_tooth_wave->out = saw_tooth_wave->initial_value;
    }

    saw_tooth_wave->out += saw_tooth_wave->slope * delta_t;
    return saw_tooth_wave->out;
}

/**
 * @brief  方波信号初始化
 * @param  square_wave: 方波结构体指针
 * @param  low_value: 低值
 * @param  high_value: 高值
 * @param  start_time: 开始时间 (s)
 * @param  T: 周期 (ms)
 */
void SquareWaveInit(SquareWave *square_wave, float low_value, float high_value, float start_time, uint16_t T)
{
    square_wave->T = T;
    square_wave->low_value = low_value;
    square_wave->high_value = high_value;
    square_wave->start_time = start_time;
    square_wave->time = 0;
    square_wave->cycle_count = 0;
    square_wave->is_high = 0;
}

/**
 * @brief  方波信号运行
 * @param  square_wave: 方波结构体指针
 * @param  delta_t: 时间增量 (s)
 * @return 当前方波输出值
 */
float SquareWaveRun(SquareWave *square_wave, float delta_t)
{
    square_wave->time += delta_t;

    if (square_wave->time < square_wave->start_time)
    {
        square_wave->is_high = 0;
        return square_wave->low_value;
    }

    // 计算从启动后经过的总时间
    float elapsed_time = square_wave->time - square_wave->start_time;
    // 计算当前周期数
    float period_in_seconds = (float)square_wave->T / 1000.0f;
    square_wave->cycle_count = (uint16_t)(elapsed_time / period_in_seconds);

    // 计算在当前周期内的时间位置
    float time_in_cycle = elapsed_time - (float)square_wave->cycle_count * period_in_seconds;
    float half_period = period_in_seconds / 2.0f; // 半周期 (s)

    // 前半周期为低值，后半周期为高值
    if (time_in_cycle >= half_period)
    {
        square_wave->is_high = 1;
        return square_wave->high_value;
    }
    else
    {
        square_wave->is_high = 0;
        return square_wave->low_value;
    }
}
