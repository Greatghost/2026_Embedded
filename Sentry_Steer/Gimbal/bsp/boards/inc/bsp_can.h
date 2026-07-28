#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"
#include "can_config.h"

#include "FrictionWheel.h"
#include "Gimbal.h"
#include "debug.h"
#include "ToggleBullet.h"
#include "ChassisGet.h"
#include "receive_data.h"


extern volatile uint8_t JudgeData_update;
extern volatile uint8_t Blood_update;
extern JudgeData_1_t JudgeRecieveData;
extern JudgeData_2_t JudgeRecieveData2;
extern JudgeBloodData_ForSend1_t JudgeBlood_F,JudgeBlood_E;
extern JudgeData_Buff_t JudgeData_Buff;
extern JudgeData_RFID_t JudgeData_RFID;
extern JudgeData_position_t JudgeData_position;

// chassis_pack_get_1 和 chassis_speed_recv 已在ChassisGet.h中声明
// shoot_data_recv, sentry_info_recv, bullet_extended_recv 已在ChassisGet.h中声明

void can_filter_init(void);

int8_t CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id, CAN_TxHeaderTypeDef *Motor_Send, uint32_t *wait_time);

// 新增: 发送SentryCmd给底盘 (2026-05-06协议)
void Can1SendSentryCmd(uint32_t sentry_cmd);

// 转发0x0307地图路径 (DownlinkTypeID=0x02)。
// Can1SendMapPath仅发布最新105B快照，可从USB中断调用；
// Can1ServiceMapPathTx必须由ChassisTask单一调用者周期执行。
// 105B map_data 通过 CAN 0x152 分 15 帧传输
// 每帧 8B: byte0 = segment_index(0~14), byte1-7 = 7B payload
// 底盘需在 CAN 0x152 上接收 15 帧并重组为 105B, 然后封装 0x0307 发给裁判系统
// 详见 底盘侧map_data转发实现说明.md
typedef struct
{
    volatile uint32_t snapshot_queued_count; // USB重组结果进入CAN待发送快照
    volatile uint32_t batch_started_count;   // 开始发送一组15帧
    volatile uint32_t can_frame_sent_count;  // 成功放入CAN邮箱的0x152帧总数
    volatile uint32_t can_send_retry_count;  // CanSend失败、等待下周期重试的次数
    volatile uint32_t batch_completed_count; // 15帧全部成功发出的整包数量
    volatile uint8_t last_segment_sent;       // 最近成功发出的分片序号0~14
} MapPathTxDebug_t;

extern MapPathTxDebug_t g_map_path_tx_debug;

void Can1SendMapPath(const uint8_t *map_data_105);
void Can1ServiceMapPathTx(void);

// 新增: 转发0x0308自定义信息 (2026-07-11协议 DownlinkTypeID=0x03)
// 34B custom_data 通过 CAN 0x153 分 5 帧传输
// 每帧 8B: byte0 = segment_index(0~4), byte1-7 = 7B payload (最后一帧仅6B有效)
// 底盘需在 CAN 0x153 上接收 5 帧并重组为 34B, 然后封装 0x0308 发给裁判系统
void Can1SendCustomInfo(const uint8_t *custom_data_34);
void Can1ServiceCustomInfoTx(void);

#endif
