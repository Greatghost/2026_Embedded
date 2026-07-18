/**
  ******************************************************************************
  * @file    wbus_decoder.h
  * @brief   WBUS协议解码器头文件
  ******************************************************************************
  * @attention
  *
  * WBUS协议帧格式:
  * - 帧头: 0x0F
  * - 数据: 22字节 (包含16个11位通道数据 + 标志位)
  * - 帧长: 25字节
  *
  * 串口配置: 100000bps, 9位数据位, 2位停止位, 偶校验
  *
  ******************************************************************************
  */

#ifndef __WBUS_DECODER_H
#define __WBUS_DECODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <string.h>

/* WBUS协议定义 */
#define WBUS_FRAME_LENGTH       25      /* WBUS帧长度 */
#define WBUS_HEADER             0x0F    /* WBUS帧头 */
#define WBUS_END_BYTE           0x00    /* 标准WBUS/S.BUS帧尾 */
#define WBUS_CHANNEL_NUM        16      /* WBUS通道数量 */
#define WBUS_CHANNEL_MIN        172     /* 通道最小值 */
#define WBUS_CHANNEL_MAX        1811    /* 通道最大值 */
#define WBUS_CHANNEL_MID        992     /* 通道中间值 */
#define WBUS_CHANNEL_RANGE      820     /* 通道半范围 (1811-992 或 992-172) */

/*
 * WBUS正常帧周期远小于100ms。在线状态只由有效帧刷新，连续超时后进入
 * 安全态；掉线恢复时先等待若干连续有效帧，避免单个毛刺帧重新使能整车。
 */
#define WBUS_TIMEOUT_MS          100u
#define WBUS_RECOVERY_FRAMES       3u

/* 映射到DJI遥控器的定义 - 与原系统兼容 */
#define WBUS_CH_VALUE_MIN       ((uint16_t)364)
#define WBUS_CH_VALUE_MAX       ((uint16_t)1684)
#define WBUS_CH_VALUE_OFFSET    ((uint16_t)1024)  /* 对应DJI的CH_MIDDLE */

/* WBUS通道索引定义 (0-15) */
#define WBUS_CH1                0   /* 通道1 -> DJI RIGHT_CH_LR */
#define WBUS_CH2                1   /* 通道2 -> DJI RIGHT_CH_UD */
#define WBUS_CH3                2   /* 通道3 -> DJI LEFT_CH_LR */
#define WBUS_CH4                3   /* 通道4 -> DJI LEFT_CH_UD */
#define WBUS_CH5                4   /* 通道5 -> 替代CH6的双位拨杆 (NUC模式选择) */
#define WBUS_CH6                5   /* 通道6 (已损坏,不再使用) */
#define WBUS_CH7                6   /* 通道7 -> DJI RIGHT_SW (右拨杆) */
#define WBUS_CH8                7   /* 通道8 -> 替代CH6的双位拨杆 (遥控模式选择) */
#define WBUS_CH9                8   /* 通道9 */
#define WBUS_CH10               9   /* 通道10 */
#define WBUS_CH11               10  /* 通道11 */
#define WBUS_CH12               11  /* 通道12 */
#define WBUS_CH13               12  /* 通道13 */
#define WBUS_CH14               13  /* 通道14 */
#define WBUS_CH15               14  /* 通道15 */
#define WBUS_CH16               15  /* 通道16 */

/* WBUS标志位定义 */
#define WBUS_FLAG_CH17          0x01
#define WBUS_FLAG_CH18          0x02
#define WBUS_FLAG_FRAME_LOST    0x04
#define WBUS_FLAG_FAILSAFE      0x08

/* 拨杆位置定义 - 与DJI遥控器兼容 */
typedef enum {
    WBUS_SW_UP = 1,     /* 上 */
    WBUS_SW_MID = 3,    /* 中 */
    WBUS_SW_DOWN = 2    /* 下 */
} WBUS_SwitchPosition;

/* WBUS数据结构 */
typedef struct {
    uint16_t ch[WBUS_CHANNEL_NUM];  /* 16个通道数据 (11位, 172-1811) */
    uint8_t ch17;                    /* 通道17 (数字通道) */
    uint8_t ch18;                    /* 通道18 (数字通道) */
    uint8_t frame_lost;              /* 帧丢失标志 */
    uint8_t failsafe;                /* 失效保护标志 */
    uint8_t rssi;                    /* 信号强度 (部分接收机支持) */
} WBUS_Data_t;

/* WBUS接收机状态 */
typedef struct {
    WBUS_Data_t data;               /* 解码后的数据 */
    uint8_t rx_buffer[WBUS_FRAME_LENGTH * 2];  /* 接收缓冲区 */
    uint8_t is_connected;           /* 连接状态 */
    uint8_t valid_frame_streak;     /* 掉线恢复时的连续有效帧数 */
    uint32_t last_update_time;      /* 上次更新时间 */
    uint32_t frame_count;           /* 帧计数 */
    uint32_t error_count;           /* 错误计数 */
} WBUS_Receiver_t;

/* 全局变量声明 */
extern WBUS_Receiver_t wbus_receiver;

/* 函数声明 */
void WBUS_Init(void);
uint8_t WBUS_Decode(volatile uint8_t rx_buffer[]);
uint8_t WBUS_GetSwitchPosition(uint16_t ch_value);
uint8_t WBUS_GetTwoPositionSwitch(uint16_t ch_value);
uint8_t WBUS_GetLeftSwitchFromDualSwitches(uint16_t ch5_value, uint16_t ch8_value);
uint16_t WBUS_MapToDJIChannel(uint16_t wbus_value);
void WBUS_UpdateRemoteController(void);
uint8_t WBUS_IsConnected(void);
void WBUS_CheckTimeout(void);
void WBUS_ResetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __WBUS_DECODER_H */
