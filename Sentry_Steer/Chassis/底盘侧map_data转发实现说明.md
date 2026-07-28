# 底盘侧 map_data 转发实现说明（云台 → 底盘 → 裁判系统）

Updated: 2026-07-18

## 1. 背景

2026-07-18 协议变更后，上位机下发 `DownlinkTypeID=0x02` 的地图路径由原来的单次 107B 串口帧
改为两个固定 64B 物理分片（USB FS 单包 64B 限制）。云台板接收两段后重组为 105B
`map_data_t` payload，再通过 **CAN 多帧分段** 转发到底盘。底盘侧负责将 105B payload
封装为裁判系统 `cmd_id=0x0307` 帧并通过裁判串口发送。

完整链路：

```
上位机 ROS
  -> /ly/game/path (完整 50 点 MapPath)
  -> gimbal_driver
  -> 0x02 fragment 0 (64B) + fragment 1 (64B)  [USB CDC]
  -> 云台板重组为 105B map_data_t payload
  -> CAN 0x152 分 15 帧分段传输              [CAN1]
  -> 底盘重组 15 帧为 105B
  -> 封装裁判系统 0x0307 帧
  -> 裁判系统串口发送
```

上位机侧协议见 [map_path_fragment_reassembly.md](map_path_fragment_reassembly.md)。
本文档只描述 **底盘侧** 需要实现的工作。

## 2. 云台板 → 底盘 CAN 协议

### 2.1 CAN ID

| CAN ID | 方向 | 说明 |
|---|---|---|
| `0x152` | 云台 → 底盘 | map_data 分段传输，固定 15 帧/次 |

`0x152` 与既有 `0x150`(pack1)、`0x151`(坐标)、`0x15A`(SentryCmd) 不冲突。
底盘侧 CAN 滤波器需新增接收 `0x152`。

### 2.2 帧格式

每帧 8B，DLC=8：

| byte | 字段 | 说明 |
|---:|---|---|
| 0 | `segment_index` | `0` ~ `14`，单调递增 |
| 1-7 | `payload[7]` | 7B map_data 分段 |

15 帧总有效负载 = 15 × 7B = **105B**，与 `map_data_t` payload 完全一致。

> 注意：第 14 帧（最后 1 帧）的 7B 全部有效（105 - 14×7 = 7），不需要补零。

### 2.3 重组 buffer 布局

底盘收到 segment `i` 时，把 `payload[7]` 复制到重组 buffer 的 `[i*7 .. i*7+6]`：

```
segment 0  -> buffer[0..6]
segment 1  -> buffer[7..13]
...
segment 14 -> buffer[98..104]
```

收齐 15 帧后，`buffer[0..104]` 即为 105B `map_data_t` payload。

## 3. 105B map_data_t payload 字段

重组后的 105B 按下表解码（所有整数 little-endian）：

| payload byte | 字段 | 类型 / 数量 | 说明 |
|---:|---|---|---|
| 0 | `intention` | `uint8_t` | `1` 到点攻击、`2` 到点防守、`3` 移动到点；其它值非法 |
| 1-2 | `start_position_x` | `uint16_t` | 小地图起点 x，dm |
| 3-4 | `start_position_y` | `uint16_t` | 小地图起点 y，dm |
| 5-53 | `delta_x` | `int8_t[49]` | 相对上一点的 x 增量，dm |
| 54-102 | `delta_y` | `int8_t[49]` | 相对上一点的 y 增量，dm |
| 103-104 | `sender_id` | `uint16_t` | 机器人 ID，红方哨兵=7，蓝方哨兵=107 |

> `sender_id` 必须是机器人自身 ID，不是选手端 ID。上位机/云台板传来的值仅作提示；底盘在封装
> `0x0307` 前以裁判系统 `0x0201` 下发的自身 `robot_id`（红方哨兵 7、蓝方哨兵 107）覆盖该字段，
> 未取得有效自身 ID 时不上传。

## 4. 底盘侧实现要求

### 4.1 CAN 接收状态机

建议底盘侧在 CAN 接收任务里维护一份 pending buffer：

```c
typedef struct {
    uint8_t active;
    uint8_t next_index;       /* 下一个期望收到的 segment_index */
    uint8_t payload[105];
    uint32_t started_ms;
} chassis_map_path_pending_t;
```

处理逻辑：

1. 收到 CAN ID `0x152` 的 8B 帧。
2. 读取 `segment_index = data[0]`。
3. **超时清空**：若 pending 已存在且距 `started_ms` 超过 **50ms**，清空 pending。
4. **segment_index == 0**：清空旧 pending，新建 pending，`next_index = 1`，
   `memcpy(payload[0], &data[1], 7)`。
5. **segment_index == next_index**：`memcpy(payload[next_index*7], &data[1], 7)`，
   `next_index++`。
   - 若 `next_index == 15`：重组完成，进入第 6 步。
6. **重组完成**：校验 `intention ∈ {1,2,3}`，封装 0x0307 发送（见 4.2），清空 pending。
7. **其它情况**（segment_index 跳跃、回退、重复）：清空 pending，丢弃本次数据。
   不向裁判系统补发或重传。

> **超时建议 50ms**：云台板 15 帧在 1Mbps CAN 上约 2-3ms 发完，50ms 足够容忍总线抖动。
> 不要无限等待，否则旧 fragment 会污染下次路径。

### 4.2 封装并发送 0x0307

底盘侧已有的裁判系统串口发送器负责封装 `0x0307`：

1. 按 RM2026 V2.0 裁判系统协议封装：
   - 帧头 `0xA5` + `data_length`(=105) + `seq` + `CRC8`
   - `cmd_id = 0x0307`
   - `data` = 105B `map_data_t` payload
   - `CRC16` (reflected 0x1021, init 0xFFFF)
2. 通过裁判系统串口（通常是 UART4/USART6）发送一次完整帧。
3. **每次完整重组只发送一次**，不得重复发送。
4. CRC8/CRC16 由裁判发送器计算，**不要**复用云台板 fragment 层的 CRC。

### 4.3 边界条件

- 只收到部分 segment（如 0~10）后超时：**不得**发送 `0x0307`，直接丢弃。
- segment_index 乱序：清空 pending，丢弃。
- 同一 segment 重复：清空 pending，丢弃（保守策略，避免拼接错误数据）。
- `intention` 非 1/2/3：不发送 `0x0307`，丢弃。
- 新路径（segment 0）到达时立即淘汰未完成的旧路径。

## 5. 参考实现（伪代码）

```c
static chassis_map_path_pending_t map_pending;

void on_can_0x152_receive(const uint8_t data[8], uint32_t now_ms)
{
    uint8_t seg = data[0];

    /* 超时清空 */
    if (map_pending.active &&
        (now_ms - map_pending.started_ms) > 50U) {
        memset(&map_pending, 0, sizeof(map_pending));
    }

    if (seg == 0U) {
        /* 新路径起始 */
        memset(&map_pending, 0, sizeof(map_pending));
        map_pending.active = 1;
        map_pending.started_ms = now_ms;
        map_pending.next_index = 1;
        memcpy(&map_pending.payload[0], &data[1], 7);
        return;
    }

    if (!map_pending.active || seg != map_pending.next_index) {
        /* 乱序或无起始帧 */
        memset(&map_pending, 0, sizeof(map_pending));
        return;
    }

    memcpy(&map_pending.payload[seg * 7U], &data[1], 7);
    map_pending.next_index++;

    if (map_pending.next_index == 15U) {
        /* 重组完成 */
        uint8_t intention = map_pending.payload[0];
        if (intention >= 1U && intention <= 3U) {
            referee_send_map_data_0307(map_pending.payload);
        }
        memset(&map_pending, 0, sizeof(map_pending));
    }
}
```

## 6. 联调验收

1. **CAN 抓包**：上位机下发一条 50 点路径，云台板应在 CAN1 `0x152` 上连续发出 15 帧，
   `segment_index` 0→14 单调递增，每帧 8B。
2. **底盘重组**：底盘打印重组后的 `intention`、起点、49 个 `delta_x`、49 个 `delta_y`、
   `sender_id`，与上位机原始 50 点路径逐字段一致。
3. **裁判系统输出**：底盘裁判串口输出一次标准 `0x0307` 帧，帧头/CRC8/cmd_id/CRC16 正确。
4. **选手端小地图**：选手端小地图显示完整 50 点路径。
5. **异常测试**：
   - 只发 segment 0~10，等待超时：确认裁判系统无 `0x0307` 输出。
   - 故意篡改 segment_index 顺序：确认裁判系统无 `0x0307` 输出。
   - 连续发两条路径（segment 0 到达时淘汰旧路径）：确认裁判系统只输出最新路径。

## 7. 相关文件

- 云台板 USB 接收与重组：`Sentry_Steer/Gimbal/application/src/pc_serial.c` (`MapPath_OnFragment`)
- 云台板 CAN 分段发送：`Sentry_Steer/Gimbal/bsp/boards/src/bsp_can.c`
  (`Can1SendMapPath` 发布快照，`Can1ServiceMapPathTx` 在任务上下文逐段发送/失败重试)
- 云台板 CAN ID 定义：`Sentry_Steer/Gimbal/config/can_config.h` (`SEND_TO_CHASSIS_MAP_PATH_CAN_ID`)
- 上位机分片协议：[map_path_fragment_reassembly.md](map_path_fragment_reassembly.md)
- 上位机下行总协议：[downlink_control_frame.md](../downlink_control_frame.md)
