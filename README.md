2026哨兵，新的车！新的！王朝！

## 目录结构

- `Sentry_Steer/Chassis` — 底盘板嵌入式工程（STM32）
- `Sentry_Steer/Gimbal` — 云台板嵌入式工程（STM32）
- `Sentry_Steer/docs/` — 文档中心
  - `protocol/` — 现行通信协议规范（通信链路完整梳理、云台上位机通信协议总览、底盘云台CAN通信协议、底盘裁判系统详解等）
  - `architecture/` — 工程架构与代码分析（Gimbal/Chassis 项目架构文档、功率控制与超级电容分析）
  - `debug/` — 调试记录与问题清单（最高优先级问题、DEBUG_RECORD、哨兵秘诀等）
  - `reference/` — 官方规则手册、RM 通信协议 PDF、SD/芯片规范
  - `data/` — Ozone/Pitch 轴调试 CSV 数据
- `custom_client_path_viewer/` — 自定义客户端路径查看工具（Python）
