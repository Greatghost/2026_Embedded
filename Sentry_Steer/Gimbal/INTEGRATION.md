# 哨兵位置下发 — 嵌软组对接文档

> 华中科技大学狼牙队 | 状态：仿真验证通过

---

## 一、做了什么

新增一条 ROS 数据链路：**behavior_tree 发布哨兵坐标 → gimbal_driver 编码为 17 字节帧 → 串口下发 STM32**。

不修改任何云台控制、行为树决策、导航解算代码。

---

## 二、17 字节帧格式

STM32 收到的每一帧固定 17 字节，TypeID = 10（0x0A）：

```
Byte:  0     1      2-3       4-5       6-15      16
      0x21  0x0A   X(int16)   Y(int16)  0x00*10   CRC8
      head  type   LE cm      LE cm     reserved
```

### CRC8 参数

- 多项式：0x31
- 初值：0xFF
- 计算范围：字节 0~15（共 16 字节，不含 CRC 自身）
- 参考输入：`01 02 03 04` → `CRC8 = 0x8B`

### 坐标编码

- 单位：厘米 (cm)
- 类型：int16_t，小端序
- X 范围：0 ~ 2800（可配），超出截断
- Y 范围：0 ~ 1500（可配），超出截断
- 无效值（NaN/Inf）：回调过滤，不下发
- 超时 2s 未更新：停止发送

### 示例帧

```
X=245cm  Y=750cm  →  21 0a f5 00 ee 02 00 00 00 00 00 00 00 00 00 00 c2
X=1400cm Y=750cm  →  21 0a 78 05 ee 02 00 00 00 00 00 00 00 00 00 00 xx
```

---

## 三、STM32 解析代码

```c
#pragma pack(1)
typedef struct {
    uint8_t  head;          // 0x21
    uint8_t  type_id;       // 10
    int16_t  x_cm;          // X 坐标 (cm)，小端
    int16_t  y_cm;          // Y 坐标 (cm)，小端
    uint8_t  reserved[10];  // 预留，当前全 0
    uint8_t  crc8;          // CRC8 校验值
} sentry_coord_t;
#pragma pack()

// CRC8 计算：poly=0x31, init=0xFF
static uint8_t crc8_calc(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
    }
    return crc;
}

// 在串口接收中断里调用
void parse_serial_byte(uint8_t byte) {
    static uint8_t buf[32], len = 0;
    if (byte == 0x21) len = 0;
    if (len < sizeof(buf)) buf[len++] = byte;
    if (len >= sizeof(sentry_coord_t)) {
        sentry_coord_t *f = (sentry_coord_t*)buf;
        if (f->head == 0x21 && f->type_id == 10) {
            if (crc8_calc(buf, 16) == f->crc8) {
                update_sentry_position(f->x_cm, f->y_cm);
            }
            len = 0;  // 帧处理完毕，重置
        }
    }
}
```

---

## 四、ROS 话题

| 话题                       | 类型                           | 方向                            | 说明                      |
| ------------------------ | ---------------------------- | ----------------------------- | ----------------------- |
| `/ly/bt/sentry_position` | `geometry_msgs/PointStamped` | behavior_tree → gimbal_driver | 哨兵坐标（单位 m，frame_id=map） |

gimbal_driver 订阅后做：m→cm 转换 → 边界截断 → 新鲜度检查 → 组装 17B 帧 → `WriteRaw` 串口发送。

---

## 五、可配置参数

所有参数通过 ROS 2 params 传入，运行时可调：

| 参数                                        | 默认             | 说明                 |
| ----------------------------------------- | -------------- | ------------------ |
| `io_config.device_name`                   | `/dev/ttyACM0` | 串口设备               |
| `io_config.baud_rate`                     | `115200`       | 波特率                |
| `io_config.sentry_coord_send_interval_ms` | `100`          | 发送间隔（10Hz），最低 20ms |
| `io_config.sentry_coord_field_width_x`    | `2800`         | X 方向场地宽度 (cm)      |
| `io_config.sentry_coord_field_width_y`    | `1500`         | Y 方向场地宽度 (cm)      |
| `io_config.sentry_coord_fresh_timeout_ms` | `2000`         | 坐标超时停止发送           |

---

## 六、编译与启动

### 环境要求

Ubuntu 22.04 + ROS 2 Humble + `libboost-all-dev libfmt-dev libeigen3-dev`

### 编译

```bash
cd <项目根目录>
source /opt/ros/humble/setup.bash
colcon build --executor sequential
# 核心 11 包通过（detector/predictor 缺相机 SDK，但不影响本功能）
```

### 仿真验证

```bash
# 终端1
cd <项目根目录> && source ./install/setup.bash
pip3 install -e ./src/simulator
python3 -m simulator.start --offline-decision

# 终端2：开始比赛 + 监听坐标
echo '{"command":"start"}' >> /tmp/simulator_match_control.jsonl
source ./install/setup.bash && ros2 topic echo /ly/bt/sentry_position
```

### 实机启动

```bash
ros2 run behavior_tree behavior_tree_node --ros-args -r __ns:=/sentry
ros2 run gimbal_driver gimbal_driver_node --ros-args \
  -p io_config.device_name:="/dev/ttyACM0" \
  -p io_config.baud_rate:=115200 \
  -p io_config.sentry_coord_send_interval_ms:=100 \
  -r __ns:=/sentry
```

---

## 七、实机适配清单

| 项目      | 操作                                                              |
| ------- | --------------------------------------------------------------- |
| 串口设备名   | `ls /dev/tty*` 确认，改 `io_config.device_name`                     |
| 串口权限    | `sudo chmod 666 /dev/ttyXXX` 或 `sudo usermod -aG dialout $USER` |
| 场地尺寸    | 确认 X/Y 范围与 `io_config.sentry_coord_field_width_*` 一致            |
| 发送频率    | 默认 10Hz，如需调高改 `sentry_coord_send_interval_ms`                   |
| STM32 端 | 使用第三章 C 代码解析 TypeID=10 帧，CRC8 校验通过后取坐标                          |

---

## 八、代码改动清单（仅 8 个文件）

| 文件                                                  | 改动                             |
| --------------------------------------------------- | ------------------------------ |
| `src/behavior_tree/include/Application.hpp`         | +include, +publisher, +函数声明    |
| `src/behavior_tree/src/Application.cpp`             | +1 行创建 publisher               |
| `src/behavior_tree/src/PublishMessage.cpp`          | +1 行调用, +20 行实现                |
| `src/gimbal_driver/main.cpp`                        | +include, +成员变量, +60 行订阅/编码/发送 |
| `src/simulator/simulator/control_bus.py`            | +1 命令                          |
| `src/simulator/simulator/interactive_inputs.py`     | +1 属性, +7 行处理                  |
| `src/simulator/simulator/mock_inputs.py`            | +import, +publisher, +12 行方法   |
| `src/navi_tf_bridge/launch/map_aim_point.launch.py` | try/except 修复（原有bug）           |

**未涉及修改的模块：** behavior_tree 决策、gimbal_driver 云台控制、navi_tf_bridge 导航解算。

---

## 附录：开发验证记录（原 `SENTRY_POS.md` 合并，2026-06）

### 编译验证

`colcon build --executor sequential` 6 包通过（auto_aim_common / gimbal_driver / sentry_msgs / simulator / behavior_tree / navi_tf_bridge），0 errors / 0 warnings。

### 17 字节帧合规性验证

- 字节分布：head(1) + type(1) + X(2) + Y(2) + zeros(10) + CRC8(1) = 17B ✓
- 编解码测试：

  | 场景 | 坐标 (m) | 坐标 (cm) | 帧 X 字节 | 帧 Y 字节 |
  |------|---------|----------|----------|----------|
  | 场地中心 | (14.00, 7.50) | (1400, 750) | 0x78 0x05 | 0xEE 0x02 |
  | 场地原点 | (0.00, 0.00) | (0, 0) | 0x00 0x00 | 0x00 0x00 |
  | 场地右下 | (28.00, 15.00) | (2800, 1500) | 0xF0 0x0A | 0xDC 0x05 |

- 异常值：X>2800 截断 2800；X<0 截断 0；NaN/Inf 过滤不缓存；2s 超时停止发送。

### 可回滚说明

```bash
cd <项目根目录>
rm -rf src && cp -r ../backup/stage2/src_stage2_start_backup src
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash && colcon build --executor sequential
```

### 编号说明

本文档中哨兵坐标下行串行帧的 `TypeID` 在后续 2026-07-11 下行协议重构中被重新编号为 **`DownlinkTypeID=0x04`**（见 `2026-07-11_上位机下发协议总览.md`）。帧布局（17B、CRC8 poly=0x31/init=0xFF）与 ROS 链路（`/ly/bt/sentry_position` → gimbal_driver → 串口）保持不变。
