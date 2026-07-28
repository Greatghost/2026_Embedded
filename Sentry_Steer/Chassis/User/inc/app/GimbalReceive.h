#ifndef _GIMBAL_RECEIVE_H
#define _GIMBAL_RECEIVE_H

#include "stdint.h"

#include "remote_control.h"

#pragma pack(push, 1)

//typedef struct GimbalReceivePack1
//{
//  uint16_t robot_state : 1;
//  uint16_t control_type : 2;
//  uint16_t control_mode_action : 4;
//  uint16_t gimbal_mode : 3;
//  uint16_t shoot_mode : 3;
//  uint16_t chassis_fromat : 2;
//  uint16_t is_pc_on : 1;
//  uint8_t autoaim_id; // 自瞄ID

//  uint8_t __;
//  uint8_t cover_state : 1;
//  uint8_t reserved : 7;

//  int8_t robot_speed_x; // * 10 描述 x方向为云台正方向
//  int8_t robot_speed_y; // * 10描述
//  int8_t robot_speed_w;
//} GimbalReceivePack1;

typedef struct GimbalReceivePack2 // 上位机下发哨兵坐标 (CAN 0x151)
{
  int16_t sentry_x_cm;       // 哨兵 X 坐标 (cm)，来自上位机定位，范围 0~2800
  int16_t sentry_y_cm;       // 哨兵 Y 坐标 (cm)，来自上位机定位，范围 0~1500
  uint8_t reserved[4];       // 预留 0x00
} GimbalReceivePack2;

typedef struct GimbalReceivePack1
{
  // 第一个uint16_t位域: 16位 = 2字节
  uint16_t robot_state : 1;
  uint16_t control_type : 2;
  uint16_t control_mode_action : 3;
  uint16_t gimbal_mode : 3;
  uint16_t shoot_mode : 3;
  uint16_t chassis_fromat : 1;
  uint16_t is_pc_on : 1;
  uint16_t super_power : 1;
  uint16_t fly_state : 1;        // 共16位

  // 第二个uint8_t位域: 8位 = 1字节
  uint8_t through_hole_flag : 1;
  uint8_t sentry_posture : 3;    // 哨兵姿态: 1=进攻, 2=防御, 3=移动, 4=强化进攻, 5=强化防御, 6=强化移动, 0=未知
  uint8_t reserved_bits : 4;     // 共8位

  // yaw_motor_angle: 2字节
  int16_t yaw_motor_angle;       // 云台yaw轴电机角度

  // 速度数据: 3字节
  int8_t robot_speed_x;          // x方向速度
  int8_t robot_speed_y;          // y方向速度
  int8_t robot_speed_w;          // 小陀螺旋转速度

  // 总计: 2 + 1 + 2 + 3 = 8字节
} GimbalReceivePack1;

#pragma pack(pop)

extern GimbalReceivePack1 gimbal_receiver_pack1;
extern GimbalReceivePack2 gimbal_receiver_pack2;

extern int8_t gimbal_receive_1_update;
extern int8_t gimbal_receive_2_update;

// 上位机下发哨兵坐标 (CAN 0x151)，单位 cm
extern int16_t sentry_coord_x_cm;
extern int16_t sentry_coord_y_cm;

void Gimbal_msgs_Decode1(void);
void Gimbal_msgs_Decode2(void);

/* ===== 0x152 map_data 分段重组状态机 (2026 V2.0新增) ===== */

#define MAP_PATH_PAYLOAD_SIZE    105U  /* map_data_t payload 字节数 */
#define MAP_PATH_SEGMENT_COUNT   15U   /* CAN 0x152 分段总数 */
#define MAP_PATH_SEGMENT_PAYLOAD 7U    /* 每帧 CAN payload 字节数 */
#define MAP_PATH_TIMEOUT_MS      50U   /* 重组超时阈值 */

/*
 * 底盘端CAN 0x152接收诊断计数器，可直接加入Keil Watch。
 * 注意：底盘接收的是15个CAN分片，不是NUC到云台的两个USB分片。
 */
typedef struct
{
    volatile uint32_t can_frame_rx_count;       /* 收到的0x152 CAN帧总数 */
    volatile uint32_t segment0_rx_count;        /* 收到序号0的起始分片数量 */
    volatile uint32_t segment14_rx_count;       /* 收到序号14的末尾分片数量 */
    volatile uint32_t invalid_segment_count;    /* 分片序号超出0~14 */
    volatile uint32_t timeout_count;            /* 15帧未在50ms内收齐 */
    volatile uint32_t order_drop_count;         /* 无起始帧、乱序或重复分片 */
    volatile uint32_t invalid_payload_count;    /* intention不是1/2/3 */
    volatile uint32_t reassembly_success_count; /* 成功重组105B payload */
    volatile uint32_t ready_overwrite_count;    /* 上一条尚未发送就被新路径覆盖 */
    volatile uint32_t ready_taken_count;        /* Refereetask成功取走完整路径 */
    volatile uint8_t last_segment_rx;            /* 最近收到的分片序号 */
    volatile uint8_t next_expected_segment;      /* 当前期望的下一分片序号 */
} ChassisMapPathRxDebug_t;

extern ChassisMapPathRxDebug_t g_chassis_map_path_rx_debug;

/* 底盘侧 map_data 分段重组 pending buffer */
typedef struct
{
    uint8_t  active;        /* 是否有进行中的重组 */
    uint8_t  next_index;    /* 下一个期望收到的 segment_index */
    uint8_t  payload[MAP_PATH_PAYLOAD_SIZE]; /* 105B map_data_t payload */
    uint32_t started_ms;    /* 本次重组起始时间戳 */
} chassis_map_path_pending_t;

/**
  * @brief  CAN 0x152 map_data 分段接收状态机
  * @param  data:     8B CAN 帧 (data[0]=segment_index, data[1..7]=7B payload)
  * @param  now_ms:   当前系统时间戳 (ms)，用于超时判定
  * @retval 无
  * @note   重组完成后只置 ready flag，不直接调用 Referee_SendMapData0x0307
  *         实际转发由 Refereetask 任务上下文执行（避免在 ISR 中阻塞）
  */
void MapPath_OnCanReceive(const uint8_t data[8], uint32_t now_ms);

/**
  * @brief  原子取得一份完整的0x0307 payload并清除ready标志
  * @param  out_payload_105: 任务上下文使用的105B快照缓冲区
  * @retval 1=已取得，0=当前没有完整payload
  * @note   短暂屏蔽中断，避免CAN ISR在任务复制期间覆盖ready缓冲区
  */
uint8_t MapPath_TakeReady(uint8_t out_payload_105[MAP_PATH_PAYLOAD_SIZE]);

/* ===== 0x153 custom_info 分段重组状态机 (2026 V2.0新增) ===== */

#define CUSTOM_INFO_PAYLOAD_SIZE    34U  /* custom_info_t payload 字节数 */
#define CUSTOM_INFO_SEGMENT_COUNT   5U   /* CAN 0x153 分段总数 */
#define CUSTOM_INFO_SEGMENT_PAYLOAD 7U   /* 每帧 CAN payload 字节数 (最后一帧仅 6B 有效) */
#define CUSTOM_INFO_TIMEOUT_MS      50U  /* 重组超时阈值 */

/* 底盘侧 custom_info 分段重组 pending buffer */
typedef struct
{
    uint8_t  active;        /* 是否有进行中的重组 */
    uint8_t  next_index;    /* 下一个期望收到的 segment_index */
    uint8_t  payload[CUSTOM_INFO_PAYLOAD_SIZE]; /* 34B custom_info_t payload */
    uint32_t started_ms;    /* 本次重组起始时间戳 */
} chassis_custom_info_pending_t;

/**
  * @brief  CAN 0x153 custom_info 分段接收状态机
  * @param  data:     8B CAN 帧 (data[0]=segment_index, data[1..7]=7B payload)
  * @param  now_ms:   当前系统时间戳 (ms)，用于超时判定
  * @retval 无
  * @note   重组完成后只置 ready flag，不直接调用 Referee_SendCustomInfo0x0308
  *         实际转发由 Refereetask 任务上下文执行（避免在 ISR 中阻塞）
  */
void CustomInfo_OnCanReceive(const uint8_t data[8], uint32_t now_ms);
uint8_t CustomInfo_TakeReady(uint8_t out_payload_34[CUSTOM_INFO_PAYLOAD_SIZE]);

/* ===== 转发任务接口（ISR → Refereetask） =====
 * ISR 中重组完成后，将 payload 拷贝到 ready 缓冲区并置 flag，
 * Refereetask 检测到 flag 后调用 Referee_SendMapData0x0307 / Referee_SendCustomInfo0x0308
 */
extern volatile uint8_t map_data_ready;          /* 0x152 重组完成标志 */
extern volatile uint8_t custom_info_ready;       /* 0x153 重组完成标志 */
extern uint8_t map_data_payload[MAP_PATH_PAYLOAD_SIZE];        /* 0x152 ready payload 副本 */
extern uint8_t custom_info_payload[CUSTOM_INFO_PAYLOAD_SIZE];  /* 0x153 ready payload 副本 */

#endif // !_GIMBAL_RECEIVE_H
