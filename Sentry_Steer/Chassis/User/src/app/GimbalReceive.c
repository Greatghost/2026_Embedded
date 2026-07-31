/**
 ******************************************************************************
 * @file    Judge.c
 * @brief   云台数据接收
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalReceive.h"
#include "ChasisController.h"
#include "Referee.h"  // 用于访问sentry_decision_referee
#include "string.h"

GimbalReceivePack1 gimbal_receiver_pack1;
GimbalReceivePack2 gimbal_receiver_pack2;
int8_t gimbal_receive_1_update; // 更新标志，说明收到了一帧消息
int8_t gimbal_receive_2_update;
int8_t gimbal_receive_3_update;
int16_t jump_up_cnt = 0;

// 上位机下发哨兵坐标 (CAN 0x151)，单位 cm
int16_t sentry_coord_x_cm = 0;
int16_t sentry_coord_y_cm = 0;

float transition_mode_counter; // 计时器

/* ===== 转发任务接口（ISR → Refereetask） =====
 * ISR 中重组完成后，将 payload 拷贝到 ready 缓冲区并置 flag，
 * Refereetask 检测到 flag 后调用 Referee_SendMapData0x0307 / Referee_SendCustomInfo0x0308
 * volatile 保证 ISR 与任务间的可见性
 */
volatile uint8_t map_data_ready = 0;
volatile uint8_t custom_info_ready = 0;
uint8_t map_data_payload[MAP_PATH_PAYLOAD_SIZE];
uint8_t custom_info_payload[CUSTOM_INFO_PAYLOAD_SIZE];
ChassisMapPathRxDebug_t g_chassis_map_path_rx_debug;

void Gimbal_msgs_Decode1()
{
  static uint8_t last_pc_control_active = 0U;
  enum ROBOT_STATE robot_state = (enum ROBOT_STATE)gimbal_receiver_pack1.robot_state;
  enum CONTROL_TYPE contro_type = (enum CONTROL_TYPE)gimbal_receiver_pack1.control_type;
  enum CONTROL_MODE_ACTION control_mode_action = (enum CONTROL_MODE_ACTION)gimbal_receiver_pack1.control_mode_action;
  enum GIMBAL_ACTION gimbal_action = (enum GIMBAL_ACTION)gimbal_receiver_pack1.gimbal_mode;
  enum SHOOT_ACTION shoot_action = (enum SHOOT_ACTION)gimbal_receiver_pack1.shoot_mode;
  enum CHASSIS_FORMAT chassis_format = (enum CHASSIS_FORMAT)gimbal_receiver_pack1.chassis_fromat;

  enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack1.super_power;
  enum FlyControlState fly_or_not = (enum FlyControlState)gimbal_receiver_pack1.fly_state;

  setRobotState(robot_state);
  setControlMode(contro_type);
  setControlModeAction(control_mode_action);
  setGimbalAction(gimbal_action);
  setShootAction(shoot_action);
	setChassisFormat(chassis_format);

  setSuperPower(power_state);
  setFlyMode(fly_or_not);

  if (last_pc_control_active != 0U &&
      gimbal_receiver_pack1.pc_control_active == 0U)
  {
    chassis_manual_takeover_reset();
  }
  last_pc_control_active = gimbal_receiver_pack1.pc_control_active;

  // 运动百分比
  infantry.receive_x_v = gimbal_receiver_pack1.robot_speed_x / 30.0f;
  infantry.receive_y_v= gimbal_receiver_pack1.robot_speed_y / 30.0f;
  infantry.receive_yaw_v = gimbal_receiver_pack1.robot_speed_w / 7.0f;

  // 更新哨兵姿态到裁判系统数据包 (1=进攻, 2=防御, 3=移动, 4=强化进攻, 5=强化防御, 6=强化移动, 0=未知)
  if(gimbal_receiver_pack1.sentry_posture >= 1 && gimbal_receiver_pack1.sentry_posture <= 6)
  {
    sentry_decision_referee.sentry_posture = gimbal_receiver_pack1.sentry_posture;
  }
}

void Gimbal_msgs_Decode2()
{
  // 存储上位机下发的哨兵坐标 (CAN 0x151)
  sentry_coord_x_cm = gimbal_receiver_pack2.sentry_x_cm;
  sentry_coord_y_cm = gimbal_receiver_pack2.sentry_y_cm;

  // 更新接收标志
  gimbal_receive_2_update = 1;
}

/* ===== 0x152 map_data 分段重组状态机 (2026 V2.0新增) ===== */

static chassis_map_path_pending_t map_pending; /* 文件级 pending buffer */

/**
 * @brief  CAN 0x152 map_data 分段接收状态机
 * @param  data:    8B CAN 帧 (data[0]=segment_index 0~14, data[1..7]=7B payload)
 * @param  now_ms:  当前系统时间戳 (ms)，用于超时判定
 * @retval 无
 * @note   - segment_index == 0 启动新 pending
 *         - segment_index == next_index 累积 payload
 *         - next_index == 15 重组完成: 校验 intention ∈ {1,2,3} 后置 ready flag
 *         - 50ms 超时/乱序/重复: 清空 pending, 丢弃 (不重传)
 *         - 实际转发由 Refereetask 任务上下文执行（避免在 ISR 中阻塞）
 */
void MapPath_OnCanReceive(const uint8_t data[8], uint32_t now_ms)
{
    uint8_t seg;

    if (data == NULL)
    {
        g_chassis_map_path_rx_debug.invalid_segment_count++;
        return;
    }

    seg = data[0];
    g_chassis_map_path_rx_debug.can_frame_rx_count++;
    g_chassis_map_path_rx_debug.last_segment_rx = seg;

    if (seg >= MAP_PATH_SEGMENT_COUNT)
    {
        g_chassis_map_path_rx_debug.invalid_segment_count++;
        memset(&map_pending, 0, sizeof(map_pending));
        g_chassis_map_path_rx_debug.next_expected_segment = 0U;
        return;
    }

    if (seg == 0U)
    {
        g_chassis_map_path_rx_debug.segment0_rx_count++;
    }
    else if (seg == (MAP_PATH_SEGMENT_COUNT - 1U))
    {
        g_chassis_map_path_rx_debug.segment14_rx_count++;
    }

    /* 1. 超时清空: 距 started_ms 超过 50ms 视为旧路径污染 */
    if (map_pending.active &&
        (now_ms - map_pending.started_ms) > MAP_PATH_TIMEOUT_MS)
    {
        g_chassis_map_path_rx_debug.timeout_count++;
        memset(&map_pending, 0, sizeof(map_pending));
        g_chassis_map_path_rx_debug.next_expected_segment = 0U;
    }

    /* 2. segment_index == 0: 新路径起始, 清空旧 pending 并启动新 pending */
    if (seg == 0U)
    {
        memset(&map_pending, 0, sizeof(map_pending));
        map_pending.active = 1;
        map_pending.started_ms = now_ms;
        map_pending.next_index = 1;
        g_chassis_map_path_rx_debug.next_expected_segment = 1U;
        memcpy(&map_pending.payload[0], &data[1], MAP_PATH_SEGMENT_PAYLOAD);
        return;
    }

    /* 3. 乱序或无起始帧: 清空 pending, 丢弃 */
    if (!map_pending.active || seg != map_pending.next_index)
    {
        g_chassis_map_path_rx_debug.order_drop_count++;
        memset(&map_pending, 0, sizeof(map_pending));
        g_chassis_map_path_rx_debug.next_expected_segment = 0U;
        return;
    }

    /* 4. 顺序到达: 累积 payload */
    memcpy(&map_pending.payload[seg * MAP_PATH_SEGMENT_PAYLOAD],
           &data[1],
           MAP_PATH_SEGMENT_PAYLOAD);
    map_pending.next_index++;
    g_chassis_map_path_rx_debug.next_expected_segment = map_pending.next_index;

    /* 5. 重组完成: 校验 intention 后置 ready flag, 清空 pending
     *    不在 ISR 中调用 Referee_SendMapData0x0307，避免阻塞 */
    if (map_pending.next_index == MAP_PATH_SEGMENT_COUNT)
    {
        uint8_t intention = map_pending.payload[0];
        if (intention >= 1U && intention <= 3U)
        {
            /* 拷贝到 ready 缓冲区并置 flag，由 Refereetask 转发 */
            if (map_data_ready != 0U)
            {
                g_chassis_map_path_rx_debug.ready_overwrite_count++;
            }
            memcpy(map_data_payload, map_pending.payload, MAP_PATH_PAYLOAD_SIZE);
            map_data_ready = 1;
            g_chassis_map_path_rx_debug.reassembly_success_count++;
        }
        else
        {
            g_chassis_map_path_rx_debug.invalid_payload_count++;
        }
        memset(&map_pending, 0, sizeof(map_pending));
        g_chassis_map_path_rx_debug.next_expected_segment = 0U;
    }
}

uint8_t MapPath_TakeReady(uint8_t out_payload_105[MAP_PATH_PAYLOAD_SIZE])
{
    uint32_t primask;
    uint8_t has_payload = 0U;

    if (out_payload_105 == NULL)
    {
        return 0U;
    }

    /* CAN2 RX使用最高优先级中断，FreeRTOS的taskENTER_CRITICAL不能保证屏蔽它。
     * 保存PRIMASK并短暂全局关中断，105B复制完成后恢复原状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (map_data_ready != 0U)
    {
        memcpy(out_payload_105, map_data_payload, MAP_PATH_PAYLOAD_SIZE);
        map_data_ready = 0U;
        has_payload = 1U;
        g_chassis_map_path_rx_debug.ready_taken_count++;
    }
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    return has_payload;
}

/* ===== 0x153 custom_info 分段重组状态机 (2026 V2.0新增) ===== */

static chassis_custom_info_pending_t custom_info_pending; /* 文件级 pending buffer */

/**
 * @brief  CAN 0x153 custom_info 分段接收状态机
 * @param  data:    8B CAN 帧 (data[0]=segment_index 0~4, data[1..7]=7B payload)
 * @param  now_ms:  当前系统时间戳 (ms)，用于超时判定
 * @retval 无
 * @note   - segment_index == 0 启动新 pending
 *         - segment_index == next_index 累积 payload
 *         - 最后一帧 (seg 4) 仅 6B 有效, data[7] 为 padding 忽略
 *         - next_index == 5 重组完成: 置 ready flag
 *         - 50ms 超时/乱序/重复: 清空 pending, 丢弃 (不重传)
 *         - 实际转发由 Refereetask 任务上下文执行（避免在 ISR 中阻塞）
 */
void CustomInfo_OnCanReceive(const uint8_t data[8], uint32_t now_ms)
{
    uint8_t seg = data[0];

    /* 1. 超时清空: 距 started_ms 超过 50ms 视为旧数据污染 */
    if (custom_info_pending.active &&
        (now_ms - custom_info_pending.started_ms) > CUSTOM_INFO_TIMEOUT_MS)
    {
        memset(&custom_info_pending, 0, sizeof(custom_info_pending));
    }

    /* 2. segment_index == 0: 新路径起始, 清空旧 pending 并启动新 pending */
    if (seg == 0U)
    {
        memset(&custom_info_pending, 0, sizeof(custom_info_pending));
        custom_info_pending.active = 1;
        custom_info_pending.started_ms = now_ms;
        custom_info_pending.next_index = 1;
        memcpy(&custom_info_pending.payload[0], &data[1], CUSTOM_INFO_SEGMENT_PAYLOAD);
        return;
    }

    /* 3. 乱序或无起始帧: 清空 pending, 丢弃 */
    if (!custom_info_pending.active || seg != custom_info_pending.next_index)
    {
        memset(&custom_info_pending, 0, sizeof(custom_info_pending));
        return;
    }

    /* 4. 顺序到达: 累积 payload (最后一帧 seg==4 仅 6B 有效) */
    if (seg == (CUSTOM_INFO_SEGMENT_COUNT - 1U))
    {
        /* 最后一帧: 34 - 4*7 = 6B 有效 */
        memcpy(&custom_info_pending.payload[seg * CUSTOM_INFO_SEGMENT_PAYLOAD],
               &data[1],
               CUSTOM_INFO_PAYLOAD_SIZE - seg * CUSTOM_INFO_SEGMENT_PAYLOAD);
    }
    else
    {
        memcpy(&custom_info_pending.payload[seg * CUSTOM_INFO_SEGMENT_PAYLOAD],
               &data[1],
               CUSTOM_INFO_SEGMENT_PAYLOAD);
    }
    custom_info_pending.next_index++;

    /* 5. 重组完成: 置 ready flag, 清空 pending
     *    不在 ISR 中调用 Referee_SendCustomInfo0x0308，避免阻塞 */
    if (custom_info_pending.next_index == CUSTOM_INFO_SEGMENT_COUNT)
    {
        /* 拷贝到 ready 缓冲区并置 flag，由 Refereetask 转发 */
        memcpy(custom_info_payload, custom_info_pending.payload, CUSTOM_INFO_PAYLOAD_SIZE);
        custom_info_ready = 1;
        memset(&custom_info_pending, 0, sizeof(custom_info_pending));
    }
}

uint8_t CustomInfo_TakeReady(uint8_t out_payload_34[CUSTOM_INFO_PAYLOAD_SIZE])
{
    uint32_t primask;
    uint8_t has_payload = 0U;

    if (out_payload_34 == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (custom_info_ready != 0U)
    {
        memcpy(out_payload_34, custom_info_payload, CUSTOM_INFO_PAYLOAD_SIZE);
        custom_info_ready = 0U;
        has_payload = 1U;
    }
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    return has_payload;
}
