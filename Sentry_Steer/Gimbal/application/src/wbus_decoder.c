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
  * @brief  将遥控输入和整车状态切到确定的安全值
  * @note   与连接状态分开处理，恢复等待期间不能反复清零连续有效帧计数。
  */
static void WBUS_ApplySafeState(void)
{
    for (int i = 0; i < 4; i++) {
        remote_controller.dji_remote.rc.ch[i] = CH_MIDDLE;
    }

    remote_controller.dji_remote.rc.s[LEFT_SW] = Down;
    remote_controller.dji_remote.rc.s[RIGHT_SW] = Down;
    remote_controller.dji_remote.rc.Previous_rc_Right_SW = Down;
    remote_controller.dji_remote.rc.poke = 0;
    remote_controller.dji_remote.mouse.x = 0;
    remote_controller.dji_remote.mouse.y = 0;
    remote_controller.dji_remote.mouse.z = 0;
    remote_controller.dji_remote.mouse.press_l = 0;
    remote_controller.dji_remote.mouse.press_r = 0;
    remote_controller.dji_remote.keyValue = 0;

    setRobotState(OFFLINE_MODE);
    setGimbalAction(GIMBAL_POWERDOWN);
    setShootAction(SHOOT_POWERDOWN_MODE);
}

static void WBUS_Disconnect(void)
{
    wbus_receiver.is_connected = 0;
    wbus_receiver.valid_frame_streak = 0;
    WBUS_ApplySafeState();
}

/**
  * @brief  校验本项目实际使用的WBUS通道
  * @note   必须在原始WBUS域校验。映射后的合法最小值为364，不能再用
  *         “小于400”猜测掉线，否则摇杆打满负方向会被误判。
  */
static uint8_t WBUS_UsedChannelsAreValid(const WBUS_Data_t *data)
{
    static const uint8_t used_channels[] = {
        WBUS_CH1, WBUS_CH2, WBUS_CH3, WBUS_CH4,
        WBUS_CH5, WBUS_CH7, WBUS_CH8
    };

    for (uint32_t i = 0;
         i < sizeof(used_channels) / sizeof(used_channels[0]);
         i++) {
        const uint16_t value = data->ch[used_channels[i]];
        if (value < WBUS_CHANNEL_MIN || value > WBUS_CHANNEL_MAX) {
            return 0;
        }
    }
    return 1;
}

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
    wbus_receiver.valid_frame_streak = 0;
    wbus_receiver.last_update_time = 0;
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
uint8_t WBUS_Decode(volatile uint8_t rx_buffer[])
{
    WBUS_Data_t decoded = {0};

    /* 帧头、帧尾都必须匹配，避免DMA错位后碰巧解出合法通道。 */
    if (rx_buffer[0] != WBUS_HEADER ||
        rx_buffer[WBUS_FRAME_LENGTH - 1U] != WBUS_END_BYTE) {
        wbus_receiver.error_count++;
        if (!wbus_receiver.is_connected) {
            wbus_receiver.valid_frame_streak = 0;
        }
        return 0;
    }
    
    /* 解码16个11位通道 (大疆SBUS/WBUS格式) */
    /* 通道1-16从字节1-22解码 */
    decoded.ch[0]  = ((rx_buffer[1]      | rx_buffer[2]  << 8) & 0x07FF);
    decoded.ch[1]  = ((rx_buffer[2] >> 3 | rx_buffer[3]  << 5) & 0x07FF);
    decoded.ch[2]  = ((rx_buffer[3] >> 6 | rx_buffer[4]  << 2 | rx_buffer[5] << 10) & 0x07FF);
    decoded.ch[3]  = ((rx_buffer[5] >> 1 | rx_buffer[6]  << 7) & 0x07FF);
    decoded.ch[4]  = ((rx_buffer[6] >> 4 | rx_buffer[7]  << 4) & 0x07FF);
    decoded.ch[5]  = ((rx_buffer[7] >> 7 | rx_buffer[8]  << 1 | rx_buffer[9] << 9) & 0x07FF);
    decoded.ch[6]  = ((rx_buffer[9] >> 2 | rx_buffer[10] << 6) & 0x07FF);
    decoded.ch[7]  = ((rx_buffer[10] >> 5 | rx_buffer[11] << 3) & 0x07FF);
    decoded.ch[8]  = ((rx_buffer[12]      | rx_buffer[13] << 8) & 0x07FF);
    decoded.ch[9]  = ((rx_buffer[13] >> 3 | rx_buffer[14] << 5) & 0x07FF);
    decoded.ch[10] = ((rx_buffer[14] >> 6 | rx_buffer[15] << 2 | rx_buffer[16] << 10) & 0x07FF);
    decoded.ch[11] = ((rx_buffer[16] >> 1 | rx_buffer[17] << 7) & 0x07FF);
    decoded.ch[12] = ((rx_buffer[17] >> 4 | rx_buffer[18] << 4) & 0x07FF);
    decoded.ch[13] = ((rx_buffer[18] >> 7 | rx_buffer[19] << 1 | rx_buffer[20] << 9) & 0x07FF);
    decoded.ch[14] = ((rx_buffer[20] >> 2 | rx_buffer[21] << 6) & 0x07FF);
    decoded.ch[15] = ((rx_buffer[21] >> 5 | rx_buffer[22] << 3) & 0x07FF);
    
    /* 解码标志位 */
    decoded.ch17 = (rx_buffer[23] & WBUS_FLAG_CH17) ? 1 : 0;
    decoded.ch18 = (rx_buffer[23] & WBUS_FLAG_CH18) ? 1 : 0;
    decoded.frame_lost = (rx_buffer[23] & WBUS_FLAG_FRAME_LOST) ? 1 : 0;
    decoded.failsafe = (rx_buffer[23] & WBUS_FLAG_FAILSAFE) ? 1 : 0;

    /* 接收机明确置failsafe时立即下电，即使其通道同时被填成0。 */
    if (decoded.failsafe) {
        wbus_receiver.data = decoded;
        wbus_receiver.error_count++;
        WBUS_Disconnect();
        return 0;
    }

    /*
     * frame_lost表示本帧通道可能是接收机保持的旧值。它不能刷新有效
     * 心跳；连续丢帧会由WBUS_CheckTimeout在100ms后进入安全态。
     */
    if (decoded.frame_lost) {
        wbus_receiver.error_count++;
        if (!wbus_receiver.is_connected) {
            wbus_receiver.valid_frame_streak = 0;
        }
        return 0;
    }

    if (!WBUS_UsedChannelsAreValid(&decoded)) {
        wbus_receiver.error_count++;
        if (!wbus_receiver.is_connected) {
            wbus_receiver.valid_frame_streak = 0;
        }
        return 0;
    }

    wbus_receiver.data = decoded;
    
    /* 更新状态 */
    wbus_receiver.frame_count++;
    wbus_receiver.last_update_time = HAL_GetTick();
    if (wbus_receiver.valid_frame_streak < WBUS_RECOVERY_FRAMES) {
        wbus_receiver.valid_frame_streak++;
    }

    if (wbus_receiver.valid_frame_streak < WBUS_RECOVERY_FRAMES) {
        wbus_receiver.is_connected = 0;
        WBUS_ApplySafeState();
        return 1;
    }

    wbus_receiver.is_connected = 1;
    
    /* 更新remote_controller结构体,保持与DJI遥控器兼容 */
    WBUS_UpdateRemoteController();
    return 1;
}

/**
  * @brief  将WBUS双位拨杆通道值转换为开关位置
  * @param  ch_value: WBUS通道值 (172-1811)
  * @retval 开关位置 (0=Down, 1=Up)
  * @note   CH5/CH8双位拨杆实际值: 向下~353, 向上~1694
  *         以1000为分界进行判断
  */
uint8_t WBUS_GetTwoPositionSwitch(uint16_t ch_value)
{
    /* 双位拨杆位置判断阈值 */
    /* 实测: 向下约353, 向上约1694 */
    /* 以1000为分界 */
    if (ch_value < 1000) {
        return 0;  /* Down */
    } else {
        return 1;  /* Up */
    }
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
  * @brief  根据CH5和CH8双位拨杆组合确定LEFT_SW位置
  * @param  ch5_value: WBUS CH5通道值 (NUC模式选择)
  * @param  ch8_value: WBUS CH8通道值 (遥控模式选择)
  * @retval 拨杆位置 (Up=1, Mid=3, Down=2)
  * @note   新映射逻辑：
  *         - CH5向下(Down): 下电 (LEFT_SW = Down)
  *         - CH5向上 + CH8向上: 遥控模式 (LEFT_SW = Up)
  *         - CH5向上 + CH8向下: NUC模式 (LEFT_SW = Mid)
  */
uint8_t WBUS_GetLeftSwitchFromDualSwitches(uint16_t ch5_value, uint16_t ch8_value)
{
    uint8_t ch5_pos = WBUS_GetTwoPositionSwitch(ch5_value);
    uint8_t ch8_pos = WBUS_GetTwoPositionSwitch(ch8_value);

    /* CH5向下优先下电 */
    if (ch5_pos == 0) {
        return WBUS_SW_DOWN;  /* 下电 */
    }

    /* CH5向上时，根据CH8决定模式 */
    if (ch8_pos == 1) {
        return WBUS_SW_UP;    /* 遥控模式 */
    } else {
        return WBUS_SW_MID;   /* NUC模式 */
    }
}

/**
  * @brief  更新remote_controller结构体 (兼容DJI遥控器格式)
  * @retval None
  * @note   将WBUS数据映射到DJI remote_controller结构:
  *         - WBUS CH1 -> rc.ch[RIGHT_CH_LR] (右摇杆左右)
  *         - WBUS CH2 -> rc.ch[RIGHT_CH_UD] (右摇杆上下)
  *         - WBUS CH3 -> rc.ch[LEFT_CH_LR] (左摇杆左右)
  *         - WBUS CH4 -> rc.ch[LEFT_CH_UD] (左摇杆上下)
  *         - WBUS CH7 -> rc.s[RIGHT_SW] (右拨杆, 三位拨杆)
  *         - WBUS CH5+CH8 -> rc.s[LEFT_SW] (左拨杆, 由双位拨杆组合替代)
  */
void WBUS_UpdateRemoteController(void)
{
    /* 映射摇杆通道 (1-4) */
    remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH4]);
    remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH3]);
    remote_controller.dji_remote.rc.ch[LEFT_CH_LR]  = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH1]);
    remote_controller.dji_remote.rc.ch[LEFT_CH_UD]  = WBUS_MapToDJIChannel(wbus_receiver.data.ch[WBUS_CH2]);

    /* 映射右拨杆 (CH7, 三位拨杆) */
    remote_controller.dji_remote.rc.s[RIGHT_SW] = WBUS_GetSwitchPosition(wbus_receiver.data.ch[WBUS_CH7]);

    /* 映射左拨杆 (CH5+CH8组合, 替代已损坏的CH6三位拨杆) */
    remote_controller.dji_remote.rc.s[LEFT_SW]  = WBUS_GetLeftSwitchFromDualSwitches(
                                                      wbus_receiver.data.ch[WBUS_CH5],
                                                      wbus_receiver.data.ch[WBUS_CH8]);

    /* 应用摇杆死区 (与原DJI遥控器相同) */
    for (int i = 0; i < 4; i++) {
        RemoteLimit(&remote_controller.dji_remote.rc.ch[i], 30);
    }

    /* 更新WBUS扩展通道数据 (用于调试) */
    for (int i = 0; i < WBUS_CHANNEL_NUM; i++) {
        remote_controller.dji_remote.rc.wbus_ch[i] = wbus_receiver.data.ch[i];
    }
    remote_controller.dji_remote.rc.wbus_ch5_pos = WBUS_GetTwoPositionSwitch(wbus_receiver.data.ch[WBUS_CH5]);
    remote_controller.dji_remote.rc.wbus_ch8_pos = WBUS_GetTwoPositionSwitch(wbus_receiver.data.ch[WBUS_CH8]);

    /* 可选: 映射其他WBUS通道到扩展功能 */
    /* remote_controller.dji_remote.rc.ch[4] = ... */
}

/**
  * @brief  检查WBUS连接状态
  * @retval 1: 已连接, 0: 未连接或失效保护
  */
uint8_t WBUS_IsConnected(void)
{
    if (!wbus_receiver.is_connected || wbus_receiver.data.failsafe ||
        wbus_receiver.last_update_time == 0) {
        return 0;
    }

    return (uint32_t)(HAL_GetTick() - wbus_receiver.last_update_time) <= WBUS_TIMEOUT_MS;
}

/**
  * @brief  周期检查真正的“无帧掉线”
  * @note   应由任务周期调用；uint32_t减法天然兼容HAL tick回绕。
  */
void WBUS_CheckTimeout(void)
{
    if (wbus_receiver.last_update_time == 0 ||
        (uint32_t)(HAL_GetTick() - wbus_receiver.last_update_time) > WBUS_TIMEOUT_MS) {
        WBUS_Disconnect();
    }
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
    
    wbus_receiver.last_update_time = 0;
    WBUS_Disconnect();
}
