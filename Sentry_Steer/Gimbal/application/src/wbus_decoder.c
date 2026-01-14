/**
  ******************************************************************************
  * @file    wbus_decoder.c
  * @brief   WBUS协议解码器实现
  ******************************************************************************
  * @attention
  *
  * WBUS协议 (Futaba S.BUS兼容):
  * - 波特率: 100000bps
  * - 数据位: 8位 (HAL配置为9位带偶校验)
  * - 停止位: 2位
  * - 校验位: 偶校验
  * - 帧格式: [0x0F][22字节数据][标志位][帧尾0x00]
  * - 16个11位比例通道 + 2个数字通道
  *
  ******************************************************************************
  */

#include "wbus_decoder.h"
#include "remote_control.h"

/* 全局WBUS接收机实例 */
WBUS_Receiver_t wbus_receiver;

/* 外部引用 - remote_controller结构体 */
extern RemoteController remote_controller;

/**
  * @brief  WBUS解码器初始化
  * @retval None
  */
void WBUS_Init(void)
{
    memset(&wbus_receiver, 0, sizeof(WBUS_Receiver_t));
    
    /* 初始化通道为中间值 */
    for (int i = 0; i < WBUS_CHANNEL_NUM; i++) {
        wbus_receiver.data.ch[i] = WBUS_CHANNEL_MID;
    }
    
    wbus_receiver.is_connected = 0;
    wbus_receiver.frame_count = 0;
    wbus_receiver.error_count = 0;
}

/**
  * @brief  解码WBUS数据帧
  * @param  rx_buffer: 接收缓冲区指针 (25字节)
  * @retval None
  * @note   WBUS数据格式 (小端序, 11位每通道):
  *         Byte[0]  = Header (0x0F)
  *         Byte[1]  = CH1[7:0]
  *         Byte[2]  = CH2[4:0]:CH1[10:8]
  *         Byte[3]  = CH3[1:0]:CH2[10:5]
  *         Byte[4]  = CH4[7:0]:CH3[10:2]
  *         ... (以此类推)
  *         Byte[23] = Flags
  *         Byte[24] = End byte (0x00)
  */
void WBUS_Decode(volatile uint8_t rx_buffer[])
{
    /* 检查帧头 */
    if (rx_buffer[0] != WBUS_HEADER) {
        wbus_receiver.error_count++;
        return;
    }
    
    /* 解码16个11位通道 (大疆SBUS/WBUS格式) */
    /* 通道1-16从字节1-22解码 */
    wbus_receiver.data.ch[0]  = ((rx_buffer[1]      | rx_buffer[2]  << 8) & 0x07FF);
    wbus_receiver.data.ch[1]  = ((rx_buffer[2] >> 3 | rx_buffer[3]  << 5) & 0x07FF);
    wbus_receiver.data.ch[2]  = ((rx_buffer[3] >> 6 | rx_buffer[4]  << 2 | rx_buffer[5] << 10) & 0x07FF);
    wbus_receiver.data.ch[3]  = ((rx_buffer[5] >> 1 | rx_buffer[6]  << 7) & 0x07FF);
    wbus_receiver.data.ch[4]  = ((rx_buffer[6] >> 4 | rx_buffer[7]  << 4) & 0x07FF);
    wbus_receiver.data.ch[5]  = ((rx_buffer[7] >> 7 | rx_buffer[8]  << 1 | rx_buffer[9] << 9) & 0x07FF);
    wbus_receiver.data.ch[6]  = ((rx_buffer[9] >> 2 | rx_buffer[10] << 6) & 0x07FF);
    wbus_receiver.data.ch[7]  = ((rx_buffer[10] >> 5 | rx_buffer[11] << 3) & 0x07FF);
    wbus_receiver.data.ch[8]  = ((rx_buffer[12]      | rx_buffer[13] << 8) & 0x07FF);
    wbus_receiver.data.ch[9]  = ((rx_buffer[13] >> 3 | rx_buffer[14] << 5) & 0x07FF);
    wbus_receiver.data.ch[10] = ((rx_buffer[14] >> 6 | rx_buffer[15] << 2 | rx_buffer[16] << 10) & 0x07FF);
    wbus_receiver.data.ch[11] = ((rx_buffer[16] >> 1 | rx_buffer[17] << 7) & 0x07FF);
    wbus_receiver.data.ch[12] = ((rx_buffer[17] >> 4 | rx_buffer[18] << 4) & 0x07FF);
    wbus_receiver.data.ch[13] = ((rx_buffer[18] >> 7 | rx_buffer[19] << 1 | rx_buffer[20] << 9) & 0x07FF);
    wbus_receiver.data.ch[14] = ((rx_buffer[20] >> 2 | rx_buffer[21] << 6) & 0x07FF);
    wbus_receiver.data.ch[15] = ((rx_buffer[21] >> 5 | rx_buffer[22] << 3) & 0x07FF);
    
    /* 解码标志位 */
    wbus_receiver.data.ch17 = (rx_buffer[23] & WBUS_FLAG_CH17) ? 1 : 0;
    wbus_receiver.data.ch18 = (rx_buffer[23] & WBUS_FLAG_CH18) ? 1 : 0;
    wbus_receiver.data.frame_lost = (rx_buffer[23] & WBUS_FLAG_FRAME_LOST) ? 1 : 0;
    wbus_receiver.data.failsafe = (rx_buffer[23] & WBUS_FLAG_FAILSAFE) ? 1 : 0;
    
    /* 更新状态 */
    wbus_receiver.frame_count++;
    wbus_receiver.is_connected = !wbus_receiver.data.failsafe;
    
    /* 更新remote_controller结构体,保持与DJI遥控器兼容 */
    WBUS_UpdateRemoteController();
}

/**
  * @brief  将WBUS拨杆通道值转换为DJI拨杆位置
  * @param  ch_value: WBUS通道值 (172-1811)
  * @retval 拨杆位置 (Up=1, Mid=3, Down=2)
  * @note   将连续值离散化为三档位置
  */
uint8_t WBUS_GetSwitchPosition(uint16_t ch_value)
{
    /* 三档拨杆位置判断阈值 */
    /* 上: 172-500, 中: 500-1500, 下: 1500-1811 */
    if (ch_value < 500) {
        return WBUS_SW_UP;      /* 对应DJI的 Up (1) */
    } else if (ch_value > 1500) {
        return WBUS_SW_DOWN;    /* 对应DJI的 Down (2) */
    } else {
        return WBUS_SW_MID;     /* 对应DJI的 Mid (3) */
    }
}

/**
  * @brief  将WBUS通道值映射到DJI通道值范围
  * @param  wbus_value: WBUS通道值 (172-1811, 中间992)
  * @retval DJI通道值 (364-1684, 中间1024)
  */
uint16_t WBUS_MapToDJIChannel(uint16_t wbus_value)
{
    /* 限幅 */
    if (wbus_value < WBUS_CHANNEL_MIN) wbus_value = WBUS_CHANNEL_MIN;
    if (wbus_value > WBUS_CHANNEL_MAX) wbus_value = WBUS_CHANNEL_MAX;
    
    /* 线性映射: WBUS (172-1811) -> DJI (364-1684) */
    /* 公式: dji = (wbus - 172) * (1684 - 364) / (1811 - 172) + 364 */
    /* 简化: dji = (wbus - 172) * 1320 / 1639 + 364 */
    int32_t temp = (int32_t)(wbus_value - WBUS_CHANNEL_MIN);
    temp = temp * (WBUS_CH_VALUE_MAX - WBUS_CH_VALUE_MIN) / (WBUS_CHANNEL_MAX - WBUS_CHANNEL_MIN);
    temp += WBUS_CH_VALUE_MIN;
    
    return (uint16_t)temp;
}

/**
  * @brief  更新remote_controller结构体 (兼容DJI遥控器格式)
  * @retval None
  * @note   将WBUS数据映射到DJI remote_controller结构:
  *         - WBUS CH1 -> rc.ch[RIGHT_CH_LR] (右摇杆左右)
  *         - WBUS CH2 -> rc.ch[RIGHT_CH_UD] (右摇杆上下)
  *         - WBUS CH3 -> rc.ch[LEFT_CH_LR] (左摇杆左右)
  *         - WBUS CH4 -> rc.ch[LEFT_CH_UD] (左摇杆上下)
  *         - WBUS CH6 -> rc.s[RIGHT_SW] (右拨杆)
  *         - WBUS CH7 -> rc.s[LEFT_SW] (左拨杆)
  */
void WBUS_UpdateRemoteController(void)
{
    /* 映射摇杆通道 (1-4) */
    remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH4]);
    remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH3]);
    remote_controller.dji_remote.rc.ch[LEFT_CH_LR]  = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH1]);
    remote_controller.dji_remote.rc.ch[LEFT_CH_UD]  = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH2]);
    
    /* 映射拨杆通道 (6/7 -> 左右拨杆) */
    remote_controller.dji_remote.rc.s[RIGHT_SW] = WBUS_GetSwitchPosition(wbus_receiver.data.ch[WBUS_CH7]);
    remote_controller.dji_remote.rc.s[LEFT_SW]  = WBUS_GetSwitchPosition(wbus_receiver.data.ch[WBUS_CH6]);
    
    /* 应用摇杆死区 (与原DJI遥控器相同) */
    for (int i = 0; i < 4; i++) {
        RemoteLimit(&remote_controller.dji_remote.rc.ch[i], 30);
    }
    
    /* 可选: 映射其他WBUS通道到扩展功能 */
    /* remote_controller.dji_remote.rc.ch[4] = ... */
}

/**
  * @brief  检查WBUS连接状态
  * @retval 1: 已连接, 0: 未连接或失效保护
  */
uint8_t WBUS_IsConnected(void)
{
    return wbus_receiver.is_connected && !wbus_receiver.data.failsafe;
}

/**
  * @brief  重置WBUS数据 (用于离线检测)
  * @retval None
  */
void WBUS_ResetData(void)
{
    /* 重置通道为中间值 */
    for (int i = 0; i < WBUS_CHANNEL_NUM; i++) {
        wbus_receiver.data.ch[i] = WBUS_CHANNEL_MID;
    }
    
    /* 重置拨杆为中间位置 */
    remote_controller.dji_remote.rc.s[RIGHT_SW] = WBUS_SW_MID;
    remote_controller.dji_remote.rc.s[LEFT_SW] = WBUS_SW_MID;
    
    /* 重置摇杆为中间值 */
    for (int i = 0; i < 4; i++) {
        remote_controller.dji_remote.rc.ch[i] = WBUS_CH_VALUE_OFFSET;
    }
    
    wbus_receiver.is_connected = 0;
}
