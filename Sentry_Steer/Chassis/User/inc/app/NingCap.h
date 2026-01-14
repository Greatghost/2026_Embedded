
/*  宁王第一版电容 2024.5.1 */

/*  使用示例：

// 发送
// 首先定义一个变量
uint8_t can_data[8]; //CAN发送数据
SuperCapSendData cap_send_data;
SendCapPack(&cap_send_data,60);  //给定60W充电
memcpy(can_data,(uint8_t*)&cap_send_data,8);
// 然后利用CAN将数据发出去即可  CAN_ID: 0x080;

// 接收解码
SuperCapRecvData recv_data;
ReceiveDecode(can_data,&recv_data);

*/

#ifndef _NING_CAP_H
#define _NING_CAP_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tools.h"

// 电容组以及电容板硬件参数
#define CAP_CAPACITY 4.54f // 电容组容值
#define CAP_MIN_VOL 8.00f  // 电容组放电最低电压
#define CAP_MAX_VOL 29.00f // 电容组最大电压

// 数据传输结构要和电容板匹配

#pragma pack(push, 1) // 不进行字节对齐

typedef struct SuperCapRecvData
{
    uint16_t cap_vol; // 剩余电压
    int16_t chassis_power;
    uint16_t referee_power;
    uint16_t V_buckin;
} SuperCapRecvData;

typedef struct SuperCapSendData
{
    uint16_t buffer_energy; // 缓冲能量
    uint16_t P_ref;         // 参考给定电压
    uint16_t __;
    uint16_t ___;
} SuperCapSendData;
#pragma pack(pop) // 不进行字节对齐

typedef enum BufferEnergyState
{
    BufferEnergy_Low = 0,
    BufferEnergy_Middle,
    BufferEnergy_High
} BufferEnergyState;

typedef enum CapVolState
{
    CapVol_Low = 0,
    CapVol_Middle,
    CapVol_High
} CapVolState;

#define BUFFER_ENERGY_HIGH 50.0f
#define BUFFER_ENERGY_MID 30.0f
#define BUFFER_ENERGY_LOW 10.0f

#define CAP_VOL_HIGH 20.0f
#define CAP_VOL_MID 15.0f
#define CAP_VOL_LOW 9.0f

typedef struct NingCapController
{
    float set_power; // 底盘电机的功率期望
    float cap_power; // 电容的充电期望
    float cap_vol;   // 电容的剩余电压 , 单位V
    float cap_energy;
    float cap_energy_pecent; // 电容的剩余能量
    float cap_energy_max;
    float cap_energy_min;

    // 缓冲能量状态
    BufferEnergyState buffer_energy_state;
    CapVolState cap_vol_state;
} NingCapController;

extern SuperCapSendData cap_send_data;
extern SuperCapRecvData cap_recv_data;
extern NingCapController cap_controller;

void ReceiveCapDecode(uint8_t *recv_data, SuperCapRecvData *cap_recv_data);
void SendCapPack(SuperCapSendData *send_data, uint16_t P_ref);
float NingCapControl(float buffer_energy, float max_power, float need_power);

void CapControllerInit(void);

#endif
