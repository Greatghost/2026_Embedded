#include "ChassisGet.h"

ChassisGetPack_1 chassis_pack_get_1;
ChassisSpeedRecv_t chassis_speed_recv;  // 底盘速度接收数据
//ChassisGetPack_2 chassis_pack_get_2;

// 新增: TypeID 7/8数据接收全局变量定义 (2026-05-06协议)
ShootDataRecv_t shoot_data_recv;
SentryInfoRecv_t sentry_info_recv;
BulletExtendedRecv_t bullet_extended_recv;
RobotCommand_ForSend_t robot_command_recv;  // 小地图下发指令接收 (0x09E)
SentryDuration_t sentry_duration;            // 哨兵姿态时长接收 (0x09F)
DamageDiff_t damage_diff;                    // 伤害值差接收 (0x0A0)