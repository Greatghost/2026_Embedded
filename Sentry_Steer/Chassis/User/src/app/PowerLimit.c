#include "PowerLimit.h"
#include "ina260.h"
#include "SuperPower.h"
#include "ChasisController.h"

void PowerLimitInit(PowerLimiter *limitter, int motor_num, MOTOR_TYPE motor_type, PowerLimitMethod method)
{
    limitter->wheels_scaler.motor_num = LIMIT_MAX_MIN(motor_num, 4, 1);
    limitter->wheels_scaler.motor_type = motor_type;
    limitter->wheels_scaler.power_limit_method = method;
    limitter->wheels_scaler.motor_R = M3508_R;
    limitter->wheels_scaler.motor_K = M3508_K;
    limitter->wheels_scaler.motor_B = M3508_B;
    limitter->wheels_scaler.motor_P0 = M3508_P0;
    if(infantry.chassis_type == STEER_WHEEL)
    {
        limitter->Steer_scaler.motor_num = LIMIT_MAX_MIN(motor_num, 4, 1);
        limitter->Steer_scaler.motor_type = GM6020;
        limitter->Steer_scaler.power_limit_method = SPEED_ERROR_METHOD;//舵电机采用转速误差分配
        limitter->Steer_scaler.motor_R = GM6020_R;
        limitter->Steer_scaler.motor_K = GM6020_K;
        limitter->Steer_scaler.motor_B = GM6020_B;
				limitter->Steer_scaler.motor_P0 = M3508_P0;//常数项为同一项
        

    }

}

/**
 * @brief 计算Torque_Scaler对象中各电机的转矩缩放系数
 * @param scaler Torque_Scaler结构体指针（内部包含二次方程系数a/b/c及状态）
 */
void TorqueScaler_PowerScaleCal(Torque_Scaler *scaler)
{
    for (int i = 0; i < scaler->motor_num; i++)
    {
        // 处理无需分配或正常分配的状态
        if (scaler->power_arrange_state[i] == NEG_ARRANGE || scaler->power_arrange_state[i] == NORMAL_ARRANGE)
        {
            scaler->send_torque_lower_scale[i] = 1.0f;
        }
        // 处理需要分配功率的状态（使用结构体内部的a/b/c系数）
        else if (scaler->power_arrange_state[i] == NEED_ARRANGE)
        {
            // 从结构体获取二次方程系数（单电机的a/b/c）
            float a = scaler->motor_a[i];  // 注意：此处使用单电机的a，而非全局a
            float b = scaler->motor_b[i];
            float c = scaler->motor_c[i] + scaler->motor_P0 - scaler->motor_P[i];

            // 判断是否无有效解（转速过高）
            if (c > 0 && b > 0)
            {
                scaler->send_torque_lower_scale[i] = 0.0f;
                scaler->power_arrange_state[i] = ARRANGE_ERROR;
            }
            // a为0保护
            else if (fabs(a) < 1e-6f)
            {
                scaler->send_torque_lower_scale[i] = 0.0f;
                scaler->power_arrange_state[i] = ARRANGE_ERROR;
            }
            // 判别式小于0（无实根，取顶点值）
            else if ((b * b - 4 * c * a) < 0)
            {
                scaler->send_torque_lower_scale[i] = -b / a / 2;
                scaler->send_torque_lower_scale[i] = LIMIT_MAX_MIN(scaler->send_torque_lower_scale[i], 1.0f, 0.0f);
            }
            // 正常求解二次方程
            else
            {
                scaler->send_torque_lower_scale[i] = (-b + Sqrt(b * b - 4 * c * a)) / 2 / a;
                scaler->send_torque_lower_scale[i] = LIMIT_MAX_MIN(scaler->send_torque_lower_scale[i], 1.0f, 0.0f);
            }
        }
    }
}

/**
 * @brief 对单个Torque_Scaler对象进行功率限制计算，完成参数准备后调用缩放系数计算函数
 * @param scaler Torque_Scaler结构体指针（内部包含电机参数和状态）
 * @param set_power 设定的最大功率值
 * @param offline_states 电机离线状态数组（外部传入，对应wheel_3508_state/steer_6020_state）
 */
void TorqueScaler_PowerLimit(Torque_Scaler *scaler, float set_power, enum ROBOT_SENSORS_DETECT *offline_states)
{
    // 初始化累计变量
    float i_2, w_i, w_2;
    float sum_i_2 = 0.0f, sum_w_i = 0.0f, sum_w_2 = 0.0f;
    scaler->predict_send_power = 0.0f;

    // 从结构体获取电机参数（R/K/B/P0）
    float motor_R = scaler->motor_R;
    float motor_K = scaler->motor_K;
    float motor_B = scaler->motor_B;
    float motor_P0 = scaler->motor_P0;

    // 计算各电机的功率相关参数并累加
    for (int i = 0; i < scaler->motor_num; i++)
    {
        // 仅处理在线或未初始化的电机
        if (offline_states[i] == WHEEL_3508_ON || offline_states[i] == STEER_6020_ON || 
            offline_states[i] == NOT_INIT)
        {
            i_2 = scaler->motor_I[i] * scaler->motor_I[i];
            w_i = scaler->motor_w[i] * scaler->motor_I[i];
            w_2 = scaler->motor_w[i] * scaler->motor_w[i];

            sum_i_2 += i_2;
            sum_w_i += w_i;
            sum_w_2 += w_2;

            // 计算单电机的二次项系数（存储到结构体）
            scaler->motor_a[i] = i_2 * motor_R;
            scaler->motor_b[i] = w_i * motor_K;
            scaler->motor_c[i] = w_2 * motor_B;

            // 计算电机预测功率
            scaler->motor_P[i] = scaler->motor_a[i] + scaler->motor_b[i] + scaler->motor_c[i] + motor_P0;
            scaler->predict_send_power += scaler->motor_P[i];
        }
    }

    // 保存累计值和设定功率
    scaler->set_power = set_power;
    scaler->sum_w_i = sum_w_i;
    scaler->sum_i_2 = sum_i_2;
    scaler->sum_w_2 = sum_w_2;

    // 重置功率分配状态
    for (int i = 0; i < scaler->motor_num; i++)
    {
        scaler->power_arrange_state[i] = NOT_ARRANGE;
    }

    // 预测功率超过设定功率时，进行功率削减
    if (scaler->predict_send_power > set_power)
    {
        scaler->can_arrange_power = set_power;
        scaler->need_arrange_power = 0.0f;
        scaler->need_arrange_w = 0.0f;

        // 根据功率限制方法进行分配
        if (scaler->power_limit_method == TORQUE_REDUCE_METHOD)
        {
            // 转矩削减模式：统计需要分配的功率
            for (int i = 0; i < scaler->motor_num; i++)
            {
                scaler->need_arrange_power += fabsf(scaler->motor_P[i]);
                scaler->power_arrange_state[i] = NEED_ARRANGE;
            }

            // 二次分配：按比例削减功率
            if (scaler->need_arrange_power >= scaler->can_arrange_power)
            {
                for (int i = 0; i < scaler->motor_num; i++)
                {
                    if (scaler->power_arrange_state[i] == NEED_ARRANGE)
                    {
                        scaler->motor_P[i] *= scaler->can_arrange_power / scaler->need_arrange_power;
                    }
                }
            }
        }
        else if (scaler->power_limit_method == SPEED_ERROR_METHOD)
        {
            // 转速误差模式：统计需要分配的功率和转速误差
            for (int i = 0; i < scaler->motor_num; i++)
            {
                scaler->need_arrange_power += fabsf(scaler->motor_P[i]);
                scaler->need_arrange_w += scaler->motor_w_error[i];
                scaler->power_arrange_state[i] = NEED_ARRANGE;
            }

            // 二次分配：按转速误差比例削减功率
            if (scaler->need_arrange_power >= scaler->can_arrange_power)
            {
                for (int i = 0; i < scaler->motor_num; i++)
                {
                    if (scaler->power_arrange_state[i] == NEED_ARRANGE)
                    {
                        scaler->motor_P[i] = scaler->motor_w_error[i] * scaler->can_arrange_power / (scaler->need_arrange_w + 1e-6f);
                    }
                }
            }
        }
    }
    else
    {
        // 功率未超限，无需削减
        for (int i = 0; i < scaler->motor_num; i++)
        {
            scaler->power_arrange_state[i] = NORMAL_ARRANGE;
        }
    }

    // 关键调整：参数准备完成后，直接调用缩放系数计算函数
    TorqueScaler_PowerScaleCal(scaler);
}

void PowerLimit(PowerLimiter *limitter, float set_power)
{
    if(infantry.chassis_type == STEER_WHEEL)
    {
        //优先分配功率给舵轮
        float max_steer_power_rate = 1.4f;//舵轮功率上限占比 
        TorqueScaler_PowerLimit(&limitter->Steer_scaler, set_power*max_steer_power_rate,offline_detector.steer_6020_state);
        float wheels_power = set_power - limitter->Steer_scaler.predict_send_power;
        wheels_power = LIMIT_MAX_MIN(wheels_power, set_power, set_power*0.2f);//保证功率的最小占比
        TorqueScaler_PowerLimit(&limitter->wheels_scaler, wheels_power,offline_detector.wheel_3508_state);
    }
    else{
        TorqueScaler_PowerLimit(&limitter->wheels_scaler, set_power ,offline_detector.wheel_3508_state);
    }

}

void setINAPower(PowerLimiter *limitter, float ina_power)
{
    //limitter->actual_ina260_power = ina_power;
}
