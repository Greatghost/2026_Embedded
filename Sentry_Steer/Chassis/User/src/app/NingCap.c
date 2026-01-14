#include "NingCap.h"

// 不要直接远程使用.改变该变量，但可以远程利用该变量
SuperCapSendData cap_send_data;
SuperCapRecvData cap_recv_data;
NingCapController cap_controller;

void CapControllerInit()
{
    cap_controller.cap_energy_min = 0.5f * CAP_MIN_VOL * CAP_MIN_VOL * CAP_CAPACITY;
    cap_controller.cap_energy_max = 0.5f * CAP_MAX_VOL * CAP_MAX_VOL * CAP_CAPACITY;
    cap_controller.buffer_energy_state = BufferEnergy_High;
    cap_controller.cap_vol_state = CapVol_Low;
    cap_controller.set_power = 50.0f;
    cap_controller.cap_power = 50.0f;
}

/*  发送数据封装 */
void SendCapPack(SuperCapSendData *send_data, uint16_t P_ref)
{
    send_data->P_ref = P_ref * 100.0f;
}

void ReceiveCapDecode(uint8_t *recv_data, SuperCapRecvData *cap_recv_data)
{
    memcpy((uint8_t *)(cap_recv_data), recv_data, 8);
}

void getCapEnergy()
{
    cap_controller.cap_vol = cap_recv_data.cap_vol / 100.0f;
    cap_controller.cap_energy = 0.5f * cap_controller.cap_vol * cap_controller.cap_vol * CAP_CAPACITY;
    cap_controller.cap_energy_pecent = cap_controller.cap_energy - cap_controller.cap_energy_min > 0 ? (cap_controller.cap_energy - cap_controller.cap_energy_min) / (cap_controller.cap_energy_max - cap_controller.cap_energy_min) : 0.0f;
}

// 只跟缓冲能量有关，与其它均无关
void RefereeOutputControl(float buffer_energy, float max_power)
{
    switch (cap_controller.buffer_energy_state)
    {
    case BufferEnergy_High:
        // 状态机切换判断
        cap_controller.buffer_energy_state = buffer_energy < BUFFER_ENERGY_MID ? (buffer_energy < BUFFER_ENERGY_LOW ? BufferEnergy_Low : BufferEnergy_Middle) : BufferEnergy_High;
        break;
    case BufferEnergy_Middle:
        cap_controller.buffer_energy_state = buffer_energy > BUFFER_ENERGY_HIGH ? BufferEnergy_High : (buffer_energy < BUFFER_ENERGY_LOW ? BufferEnergy_Low : BufferEnergy_Middle);
        break;
    case BufferEnergy_Low:
        cap_controller.buffer_energy_state = buffer_energy > BUFFER_ENERGY_HIGH ? BufferEnergy_High : (buffer_energy > BUFFER_ENERGY_MID ? BufferEnergy_Middle : BufferEnergy_Low);
        break;
    default:
        cap_controller.buffer_energy_state = BufferEnergy_Low;
        break;
    }

    // 不同状态的冲电判断
    switch (cap_controller.buffer_energy_state)
    {
    case BufferEnergy_High:
        cap_controller.cap_power = max_power;
        break;
    case BufferEnergy_Middle:
        cap_controller.cap_power = max_power * 0.8f;
        break;
    default:
        cap_controller.cap_power = max_power * 0.6f;
        break;
    }
}

void ChassisOutputControl(float need_power, float max_power)
{
    switch (cap_controller.cap_vol_state)
    {
    case CapVol_Low:
        // 状态机切换判断
        cap_controller.cap_vol_state = cap_controller.cap_vol > CAP_VOL_MID ? (cap_controller.cap_vol > CAP_VOL_HIGH ? CapVol_High : CapVol_Middle) : CapVol_Low;
        break;
    case CapVol_Middle:
        cap_controller.cap_vol_state = cap_controller.cap_vol > CAP_VOL_HIGH ? CapVol_High : (cap_controller.cap_vol < CAP_VOL_LOW ? CapVol_Low : CapVol_Middle);
        break;
    case CapVol_High:
        cap_controller.cap_vol_state = cap_controller.cap_vol < CAP_VOL_MID ? (cap_controller.cap_vol < CAP_VOL_LOW ? CapVol_Low : CapVol_Middle) : CapVol_High;
        break;
    default:
        cap_controller.cap_vol_state = CapVol_Low;
        break;
    }

    switch (cap_controller.cap_vol_state)
    {
    // 电容能量正常，要多少，给多少
    case CapVol_Middle:
    case CapVol_High:
        cap_controller.set_power = need_power;
        break;
    default:
        // 电容能量低，不能要多少给多少
        cap_controller.set_power = max_power - 5.0f; // 去除部分能量损耗，确保不超功率
        break;
    }
}

// 输入缓冲能量、底盘最高功率，需要使用的功率，返回发送功率值，潜在返回电容功率发送值
float NingCapControl(float buffer_energy, float max_power, float need_power)
{
    // 计算能量
    getCapEnergy();

    // 根据缓冲能量计算裁判系统输出功率
    RefereeOutputControl(buffer_energy, max_power);

    // 根据电容电压来判断底盘设置的功率
    ChassisOutputControl(need_power, max_power);

    // 返回底盘设置功率
    return cap_controller.set_power;
}
