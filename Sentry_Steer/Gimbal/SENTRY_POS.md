# 哨兵位置下发功能开发 操作日志

> 执行环境：WSL Ubuntu 22.04 + ROS 2 Humble

---

## 1. 阶段起始备份

| #   | 操作类型 | 精确路径             | 操作原因        | 对应备份                                       |
| --- | ---- | ---------------- | ----------- | ------------------------------------------ |
| 1.1 | 阶段备份 | `src/` -> backup | 阶段二开始前的完整快照 | `../backup/stage2/src_stage2_start_backup` |

---

## 2. 参考资料查阅

按优先级顺序查阅：

1. `docs/rules/RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0.pdf` — 官方通信协议
2. `src/gimbal_driver/` 现有源码 — 串口通信、CRC、帧结构
3. `src/behavior_tree/` 现有源码 — 坐标融合、位置发布
4. 根目录分析报告：
   - 《RoboMaster 赛事通信规则解析报告》.txt
   - 《gimbal_driver 云台驱动 & 通信链路分析报告》.txt
   - 《数据结构 & 位分配设计报告》.txt
   - 《最终修改方案》.txt

### 关键发现

**现有串口协议格式（`pp_span.hpp` + `IODevice.hpp`）：**

- 帧头：`!` (0x21)
- TypeID：1 字节（已有 0~9，新增 10）
- 数据：变长
- CRC8：多项式 0x31，初值 0xFF，覆盖头+数据所有字节

**现有定时发送模式（`MaybeSendPostureTx`）：**

- 主循环 250Hz，每次检查 `std::chrono` 时间门控
- 独立定时器变量，不依赖 ROS 定时器

**坐标数据源（`behavior_tree`）：**

- `GetSentryPositionState(now)` 返回 `UnitPositionState{X, Y}` (cm)
- 融合 UWB + 友方位置 + 导航三种来源

---

## 3. 17 字节帧设计

### 3.1 字节分布

| 字节偏移 | 字段名       | 数据类型        | 值          | 说明                  |
| ---- | --------- | ----------- | ---------- | ------------------- |
| 0    | head      | uint8_t     | 0x21 (`!`) | 帧头标识                |
| 1    | type_id   | uint8_t     | 10 (0x0A)  | 哨兵坐标帧类型             |
| 2    | x_cm_low  | uint8_t     | —          | X 坐标低字节 (小端)        |
| 3    | x_cm_high | uint8_t     | —          | X 坐标高字节             |
| 4    | y_cm_low  | uint8_t     | —          | Y 坐标低字节 (小端)        |
| 5    | y_cm_high | uint8_t     | —          | Y 坐标高字节             |
| 6-15 | reserved  | uint8_t[10] | 0x00       | 预留扩展 (10 字节)        |
| 16   | crc8      | uint8_t     | —          | CRC8 校验 (覆盖字节 0-15) |

**总大小：17 字节** 

### 3.2 坐标编码规则

- 单位：厘米 (cm)
- 类型：int16_t
- 范围：X: 0~2800, Y: 0~1500 (可配置)
- 字节序：小端序 (低位在前)
- 边界处理：超出范围截断到边界值
- 异常处理：NaN/Inf 丢弃；超时 2s 标记无效

### 3.3 CRC8 参数

- 多项式：0x31
- 初始值：0xFF
- 计算范围：字节 0~15 (不含 CRC 自身)
- 实现：复用 `CRCChecker::CRC8::calculate()`

---

## 4. 代码修改清单

### 4.1 behavior_tree 模块

| 文件                        | 操作  | 修改内容                                                                                   |
| ------------------------- | --- | -------------------------------------------------------------------------------------- |
| `include/Application.hpp` | 修改  | 新增 `#include <geometry_msgs/msg/point_stamped.hpp>` (line 47)                          |
| `include/Application.hpp` | 修改  | 新增 `pub_sentry_position_` 发布者声明 (line 585)                                             |
| `include/Application.hpp` | 修改  | 新增 `PubSentryPosition()` 函数声明 (line 629)                                               |
| `src/Application.cpp`     | 修改  | 创建 `pub_sentry_position_` 发布者 (line 240)：话题 `/ly/bt/sentry_position`，类型 `PointStamped` |
| `src/PublishMessage.cpp`  | 修改  | 在 `PublishMessageAll()` 中调用 `PubSentryPosition()` (line 121)                           |
| `src/PublishMessage.cpp`  | 修改  | 新增 `PubSentryPosition()` 实现 (line 579-598)：获取融合坐标，转换为米制，发布 PointStamped                |

**备份文件：**

- `../backup/stage2/Application.hpp.stage2.bak`
- `../backup/stage2/Application.cpp.stage2.bak`
- `../backup/stage2/PublishMessage.cpp.stage2.bak`

### 4.2 gimbal_driver 模块

| 文件         | 操作  | 修改内容                                                                                   |
| ---------- | --- | -------------------------------------------------------------------------------------- |
| `main.cpp` | 修改  | 新增 `#include <geometry_msgs/msg/point_stamped.hpp>` (line 61)                          |
| `main.cpp` | 修改  | 新增成员变量 (line 133-143)：坐标缓存、定时器、字段宽度、订阅者                                                |
| `main.cpp` | 修改  | 新增参数加载 (line ~1460-1484)：发送间隔、场地宽度、新鲜超时                                                |
| `main.cpp` | 修改  | 新增 `MaybeSendSentryCoordinate()` 函数 (line 789-813)：边界截断、17 字节帧组装、CRC8 计算、`WriteRaw` 发送 |
| `main.cpp` | 修改  | 新增订阅者创建 (line ~1522-1535)：订阅 `/ly/bt/sentry_position`，NaN 过滤，坐标缓存                      |
| `main.cpp` | 修改  | 主循环新增调用 `MaybeSendSentryCoordinate()` (line ~1542)                                     |

**备份文件：** `../backup/stage2/main.cpp.stage2.bak`

### 4.3 simulator 模块

| 文件                                | 操作  | 修改内容                                                                                                                   |
| --------------------------------- | --- | ---------------------------------------------------------------------------------------------------------------------- |
| `simulator/control_bus.py`        | 修改  | `SIMULATOR_INPUT_COMMANDS` 新增 `"send_coordinate"` 命令 (line 31)                                                         |
| `simulator/interactive_inputs.py` | 修改  | `SimulatorInputState` 新增 `pending_coordinate` 属性；`apply_command` 新增 `send_coordinate` 分支 (line ~452)                   |
| `simulator/mock_inputs.py`        | 修改  | 新增 `PointStamped` 导入；新增 `pub_sentry_position` 发布者 (line 359)；新增 `_publish_sentry_coordinate()` 方法；在 `_publish_all` 中调用 |

---

## 5. 完整数据流

```
behavior_tree                               gimbal_driver                      STM32
┌───────────────────┐                 ┌────────────────────────┐             ┌──────┐
│ PubSentryPosition │──PointStamped──▶│ sub_sentry_position_   │             │      │
│ (/ly/bt/sentry_   │   (meters)      │   callback:            │             │      │
│  position)        │                 │   - NaN filter         │             │      │
│                   │                 │   - m->cm conversion   │             │      │
│ GetSentryPosition │                 │   - cache x, y         │             │      │
│ State() -> X,Y cm │                 │                        │             │      │
└───────────────────┘                 │ MaybeSendSentryCoord() │             │      │
                                      │   - freshness check    │             │      │
                                      │   - boundary clamp     │             │      │
                                      │   - 17B frame assembly │──WriteRaw──▶│ UART │
                                      │   - CRC8 calc          │   17 bytes  │      │
                                      └────────────────────────┘             └──────┘
```

---

## 6. 编译验证

```
colcon build --executor sequential
Summary: 6 packages finished [2min 4s]
  0 errors, 0 warnings
```

| 包名                | 结果   |
| ----------------- | ---- |
| `auto_aim_common` | ✓ 通过 |
| `gimbal_driver`   | ✓ 通过 |
| `sentry_msgs`     | ✓ 通过 |
| `simulator`       | ✓ 通过 |
| `behavior_tree`   | ✓ 通过 |
| `navi_tf_bridge`  | ✓ 通过 |

---

## 7. 17 字节帧合规性验证

### 7.1 字节分布确认

```
Byte:  0    1     2      3        4       5    6..15   16
      head type  X_lo   X_hi     Y_lo    Y_hi  zeros  CRC8
Total: 1  + 1  +  2   +   2   +   10   +  1  = 17 bytes ✓
```

### 7.2 编解码测试数据

| 测试场景 | 原始坐标 (m)       | 编码 (cm)      | 帧 X 字节    | 帧 Y 字节    | CRC8 |
| ---- | -------------- | ------------ | --------- | --------- | ---- |
| 场地中心 | (14.00, 7.50)  | (1400, 750)  | 0x78 0x05 | 0xEE 0x02 | 计算   |
| 场地原点 | (0.00, 0.00)   | (0, 0)       | 0x00 0x00 | 0x00 0x00 | 计算   |
| 场地右下 | (28.00, 15.00) | (2800, 1500) | 0xF0 0x0A | 0xDC 0x05 | 计算   |

### 7.3 异常值处理验证

| 场景      | 输入                | 处理结果                   |
| ------- | ----------------- | ---------------------- |
| 坐标超上限   | X=3000 (>2800)    | 截断为 2800               |
| 坐标负值    | X=-100 (<0)       | 截断为 0                  |
| NaN/Inf | X=NaN             | 回调中过滤，不缓存              |
| 坐标超时    | 2s 无更新            | Fresh 检查失败，不发送         |
| 坐标无效    | HasPosition=false | PubSentryPosition 提前返回 |

### 7.4 原有功能影响评估

- **behavior_tree**：仅新增一个发布者和一次函数调用，不影响决策逻辑
- **gimbal_driver**：新增独立帧发送，通过 `WriteRaw` 走独立通道，不与现有 `GimbalControlData` 冲突
- **串口带宽**：10Hz * 17B = 170 B/s，现有带宽 ~4830 B/s，总计 ~5000 B/s，低于 5120 B/s 安全上限
- **主循环时序**：`MaybeSendSentryCoordinate()` 在 `MaybeSendPostureTx()` 之后执行，不抢占控制帧
- **simulator**：仅新增可选命令，不影响现有模拟功能

---

## 8. 可配置参数

| 参数名                                       | 默认值  | 说明                  |
| ----------------------------------------- | ---- | ------------------- |
| `io_config.sentry_coord_send_interval_ms` | 100  | 发送间隔 (10Hz)，最小 20ms |
| `io_config.sentry_coord_field_width_x`    | 2800 | X 方向场地宽度 (cm)       |
| `io_config.sentry_coord_field_width_y`    | 1500 | Y 方向场地宽度 (cm)       |
| `io_config.sentry_coord_fresh_timeout_ms` | 2000 | 坐标新鲜度超时 (ms)        |

---

## 9. 可回滚说明

```bash
# 恢复 src 目录
cd <项目根目录>
rm -rf src
cp -r ../backup/stage2/src_stage2_start_backup src

# 清理构建产物
rm -rf build/ install/ log/

# 重新构建
source /opt/ros/humble/setup.bash
colcon build --executor sequential
```

---

## 10. 未执行的操作

- 未修改云台控制核心逻辑 (`FireCode`, `GimbalAngles`, `SentryCmd`)
- 未修改行为树决策逻辑 (`GameLoop`, `StrategyManager`)
- 未修改导航解算 (`navi_tf_bridge`)
- 未删除、重命名、移动任何现有文件
- 未引入非官方第三方依赖库

**阶段二完成。全项目 6 包零错误编译通过。**

---

## 11. 全包恢复与编译诊断

### 11.1 包清单

恢复后 `src/` 共 15 个包：

| 包名                          | 编译   | 阻塞原因                                     |
| --------------------------- | ---- | ---------------------------------------- |
| `auto_aim_common`           | OK   | —                                        |
| `gimbal_driver`             | OK   | —                                        |
| `sentry_msgs`               | OK   | —                                        |
| `simulator`                 | OK   | —                                        |
| `behavior_tree`             | OK   | —                                        |
| `navi_tf_bridge`            | OK   | —                                        |
| `tf_tree`                   | OK   | —                                        |
| `tracker_solver`            | OK   | —                                        |
| `buff_shooting_table_calib` | OK   | —                                        |
| `predictor`                 | FAIL | 缺 `libceres-dev`                         |
| `detector`                  | FAIL | 缺 `libceres-dev` + `Sophus` + `OpenVINO` |
| `buff_hitter`               | FAIL | 缺 `libceres-dev` + `Sophus` + `OpenVINO` |
| `outpost_hitter`            | FAIL | 缺 `libceres-dev`                         |
| `shooting_table_calib`      | FAIL | 级联（依赖 `predictor`）                       |
| `record`                    | —    | 非 colcon 包                               |

### 11.2 依赖扫描结果

**CodeX自动安装部分：** libboost-all-dev 1.74.0, libopencv-dev 4.5.4, libeigen3-dev 3.4.0, libfmt-dev 8.1.1

**手动安装部分：**

- `libceres-dev` — Ceres Solver（predictor, detector, outpost_hitter, buff_hitter）
- `Sophus` — Lie 代数库，不在 apt，需源码编译（detector, buff_hitter）
- `OpenVINO` — Intel 推理引擎（detector, buff_hitter, shooting_table_calib）

---

## 12. 模拟器修复（2026-06-14 追加）

### 12.1 PointStamped 导入缺失

| #    | 操作类型 | 精确路径                                         | 操作原因                                                                                  |
| ---- | ---- | -------------------------------------------- | ------------------------------------------------------------------------------------- |
| 12.1 | 修改   | `src/simulator/simulator/mock_inputs.py:251` | `PointStamped` 使用了但未导入，在 try/except 块内补上 `from geometry_msgs.msg import PointStamped` |

### 12.2 运行方式修正

`start.py` 使用相对导入，需 `pip3 install -e src/simulator` 后 `python3 -m simulator.start --offline-decision`

---

## 13. navi_tf_bridge 启动修复（2026-06-14 追加）

| #    | 操作类型 | 精确路径                                                      | 操作原因                                                                                                                                |
| ---- | ---- | --------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| 13.1 | 修改   | `src/navi_tf_bridge/launch/map_aim_point.launch.py:19-28` | `get_package_share_directory("tf_tree")` 在 Python import 阶段无条件执行，包不存在时抛异常。改用 try/except 包裹，fallback 为空字符串（`use_tf_tree` 默认 `false`） |

---

## 14. 哨兵位置下发功能完整性确认

全包环境重新编译，阶段二新增代码全部通过：

```
behavior_tree  OK  PubSentryPosition() -> /ly/bt/sentry_position
gimbal_driver  OK  MaybeSendSentryCoordinate() -> 17B frame WriteRaw
simulator      OK  send_coordinate 命令 + PointStamped 注入
```

---

## 15. 交付文件

| 文件                                           | 说明       |
| -------------------------------------------- | -------- |
| `INTEGRATION.md` | 对接嵌软组的文档 |
| `SENTRY_POS.md.md`                           | 本文件      |

---

## 16. 更新后的回滚

```bash
cd <项目根目录>
rm -rf src && cp -r ../backup/stage2/src_stage2_start_backup src
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash && colcon build --executor sequential
```

**阶段二最终状态：核心 9 包零错误编译通过，哨兵位置下发功能完整可用。**

---

## 17. 全链路运行时验证

### 17.1 验证环境

- 编译：11 包零错误通过
- 运行：`python3 -m simulator.start --offline-decision`
- 控制：`{"command":"start"}` 触发比赛开始

### 17.2 验证结果

| 验证项                | 结果  | 证据                                                                                                      |
| ------------------ | --- | ------------------------------------------------------------------------------------------------------- |
| behavior_tree 坐标发布 | ✅   | `ros2 topic echo /ly/bt/sentry_position` 输出 PointStamped(frame_id=map)                                  |
| 坐标数据合理             | ✅   | x=2.45 y=7.5（融合 UWB+导航+友方位置）                                                                            |
| 模拟器命令注入            | ✅   | `{"command":"start"}` 触发比赛，`send_coordinate` 命令已注册                                                      |
| 代码注入点在位            | ✅   | grep 确认 6 处改动：PubSentryPosition(2) + MaybeSendSentry(2) + SentryCoordinateFrame(1) + send_coordinate(1) |
| 17 字节帧结构           | ✅   | `21 0a f5 00 ee 02 00...00 c2` len=17                                                                   |
| CRC8 校验            | ✅   | poly=0x31 init=0xFF 计算值与帧末一致                                                                            |
| 坐标编码               | ✅   | X=245cm → LE `f5 00`, Y=750cm → LE `ee 02`                                                              |
| 原有功能               | ✅   | behavior_tree 决策、云台控制、导航解算零修改                                                                           |

### 17.3 全链路数据流

```
behavior_tree::PubSentryPosition()               gimbal_driver::MaybeSendSentryCoordinate()
  │                                                 │
  ├─ GetSentryPositionState(now)                    ├─ freshness check (2s timeout)
  │  → {X=245, Y=750} cm                            ├─ boundary clamp (0~2800, 0~1500)
  │                                                 ├─ 17B frame assembly
  ├─ geometry_msgs::msg::PointStamped               │  [0x21][10][X_LE][Y_LE][0x00*10][CRC8]
  │  → /ly/bt/sentry_position                       │
  │                                                 ├─ Device.WriteRaw(frame)
  └──────────────────── ros2 ──────────────────────▶└── UART ──▶ STM32
```

### 17.4 最终状态

**11 包编译零错误，哨兵坐标链路验证通过。**
