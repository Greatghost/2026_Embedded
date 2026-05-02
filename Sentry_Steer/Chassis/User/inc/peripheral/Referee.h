/**
 ******************************************************************************
 * @file    referee.h
 * @author  Karolance Future
 * @version V1.3.0
 * @date    2022/03/21
 * @brief   Header file of referee.c
 ******************************************************************************
 * @attention
 *
 *   依据裁判系统 串口协议附录 V1.3
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREE_H__
#define __REFEREE_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "protocol.h"
#include "algorithmOfCRC.h"
#include "debug.h"
#include "counter.h"

#include "HeatControl.h"

/* Referee Defines -----------------------------------------------------------*/
/* 比赛类型 */
#define Game_Type_RMUC 1	 // 超级对抗赛
#define Game_Type_RMUT 2	 // 单项赛
#define Game_Type_RMUA 3	 // 人工智能挑战赛
#define Game_Type_RMUL_3V3 4 // 高校联盟赛3V3
#define Game_Type_RMUL_1V1 5 // 高校联盟赛1V1

/* 比赛阶段 */
#define Game_Progress_Unstart 0	  // 未开始比赛
#define Game_Progress_Prepare 1	  // 准备阶段
#define Game_Progress_SelfCheck 2 // 自检阶段
#define Game_Progress_5sCount 3	  // 5s倒计时
#define Game_Progress_Battle 4	  // 对战中
#define Game_Progress_Calculate 5 // 比赛结算中

/* 比赛结果 */
#define Game_Result_Draw 0	  // 平局
#define Game_Result_RedWin 1  // 红方胜利
#define Game_Result_BlueWin 2 // 蓝方胜利

/* 警告信息 */
#define Warning_Yellow 1  // 黄牌警告
#define Warning_Red 2	  // 红牌警告
#define Warning_Failure 3 // 判负

/* 机器人ID */
#define Robot_ID_Red_Hero 1			// 红方英雄
#define Robot_ID_Red_Engineer 2		// 红方工程
#define Robot_ID_Red_Infantry3 3	// 红方步兵3
#define Robot_ID_Red_Infantry4 4	// 红方步兵4
#define Robot_ID_Red_Infantry5 5	// 红方步兵5
#define Robot_ID_Red_Aerial 6		// 红方无人机
#define Robot_ID_Red_Sentry 7		// 红方哨兵
#define Robot_ID_Red_Darts 8		// 红方飞镖
#define Robot_ID_Red_Radar 9		// 红方雷达
#define Robot_ID_Blue_Hero 101		// 蓝方英雄
#define Robot_ID_Blue_Engineer 102	// 蓝方工程
#define Robot_ID_Blue_Infantry3 103 // 蓝方步兵3
#define Robot_ID_Blue_Infantry4 104 // 蓝方步兵4
#define Robot_ID_Blue_Infantry5 105 // 蓝方步兵5
#define Robot_ID_Blue_Aerial 106	// 蓝方无人机
#define Robot_ID_Blue_Sentry 107	// 蓝方哨兵
#define Robot_ID_Blue_Darts 108		// 蓝方飞镖
#define Robot_ID_Blue_Radar 109		// 蓝方雷达

/* 机器人等级 */
#define Robot_Level_1 1 // 1级
#define Robot_Level_2 2 // 2级
#define Robot_Level_3 3 // 3级

/* 扣血类型 */
#define Hurt_Type_ArmoredPlate 0	 // 装甲板伤害
#define Hurt_Type_ModuleOffline 1	 // 模块离线
#define Hurt_Type_OverShootSpeed 2	 // 枪口超射速
#define Hurt_Type_OverShootHeat 3	 // 枪管超热量
#define Hurt_Type_OverChassisPower 4 // 底盘超功率
#define Hurt_Type_Collision 5		 // 装甲撞击

/* 发射机构编号 */
#define Shooter_ID1_17mm 1 // 1号17mm发射机构
#define Shooter_ID2_17mm 2 // 2号17mm发射机构
#define Shooter_ID1_42mm 3 // 1号42mm发射机构

/* 飞镖信息 */
#define Dart_State_Open 0	  // 飞镖闸门开启
#define Dart_State_Close 1	  // 飞镖闸门关闭
#define Dart_State_Changing 2 // 正在开启或者关闭中
#define Dart_Target_Outpost 0 // 飞镖目标为前哨站
#define Dart_Target_Base 1	  // 飞镖目标为基地

/* 操作手ID */
#define Cilent_ID_Red_Hero 0x0101		// 红方英雄操作手
#define Cilent_ID_Red_Engineer 0x0102	// 红方工程操作手
#define Cilent_ID_Red_Infantry3 0x0103	// 红方步兵3操作手
#define Cilent_ID_Red_Infantry4 0x0104	// 红方步兵4操作手
#define Cilent_ID_Red_Infantry5 0x0105	// 红方步兵5操作手
#define Cilent_ID_Red_Aerial 0x0106		// 红方飞手
#define Cilent_ID_Blue_Hero 0x0165		// 蓝方英雄操作手
#define Cilent_ID_Blue_Engineer 0x0166	// 蓝方工程操作手
#define Cilent_ID_Blue_Infantry3 0x0167 // 蓝方步兵3操作手
#define Cilent_ID_Blue_Infantry4 0x0168 // 蓝方步兵4操作手
#define Cilent_ID_Blue_Infantry5 0x0169 // 蓝方步兵5操作手
#define Cilent_ID_Blue_Aerial 0x016A	// 蓝方飞手

/* UI绘制内容cmdID */
#define UI_DataID_Delete 0x100	 // 客户端删除图形
#define UI_DataID_Draw1 0x101	 // 客户端绘制1个图形
#define UI_DataID_Draw2 0x102	 // 客户端绘制2个图形
#define UI_DataID_Draw5 0x103	 // 客户端绘制5个图形
#define UI_DataID_Draw7 0x104	 // 客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 // 客户端绘制字符图形

/* UI删除操作 */
#define UI_Delete_Invalid 0 // 空操作
#define UI_Delete_Layer 1	// 删除图层
#define UI_Delete_All 2		// 删除所有

/* UI图形操作 */
#define UI_Graph_invalid 0 // 空操作
#define UI_Graph_Add 1	   // 增加图形
#define UI_Graph_Change 2  // 修改图形
#define UI_Graph_Delete 3  // 删除图形

/* UI图形类型 */
#define UI_Graph_Line 0		 // 直线
#define UI_Graph_Rectangle 1 // 矩形
#define UI_Graph_Circle 2	 // 整圆
#define UI_Graph_Ellipse 3	 // 椭圆
#define UI_Graph_Arc 4		 // 圆弧
#define UI_Graph_Float 5	 // 浮点型
#define UI_Graph_Int 6		 // 整形
#define UI_Graph_String 7	 // 字符型

/* UI图形颜色 */
#define UI_Color_Main 0	  // 红蓝主色
#define UI_Color_Yellow 1 // 黄色
#define UI_Color_Green 2  // 绿色
#define UI_Color_Orange 3 // 橙色
#define UI_Color_Purple 4 // 紫色
#define UI_Color_Pink 5	  // 粉色
#define UI_Color_Cyan 6	  // 青色
#define UI_Color_Black 7  // 黑色
#define UI_Color_White 8  // 白色

// 哨兵决策
#define SENTRY_DECISION_SIZE 19 // 上传数据最大的长度为21？？实际使用19
#define HEADER_LEN 5			// 帧头长度
#define CMD_LEN 2				// 命令码长度
#define CRC_LEN 2				// 尾部CRC16校验
#define SENTRY_DECISION_ID 0x120

#pragma pack(push, 1)

/* 0x000X --------------------------------------------------------------------*/
typedef struct // 0x0001 比赛状态数据
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef struct // 0x0002 比赛结果数据
{
	uint8_t winner;
} ext_game_result_t;

typedef struct // 0x0003 友方机器人血量数据
{
	uint16_t friend_1_robot_HP;
	uint16_t friend_2_robot_HP;
	uint16_t friend_3_robot_HP;
	uint16_t friend_4_robot_HP;
	//uint16_t friend_5_robot_HP;
	uint16_t friend_7_robot_HP;
	uint16_t friend_outpost_HP;
	uint16_t friend_base_HP;
} robot_HP_friend_t;

/* 0x010X --------------------------------------------------------------------*/
typedef struct // 0x0101 场地事件数据
{
	// uint32_t event_type;
	uint8_t self_supply_status : 3;	  // 己方补给站状态：0-未激活/1-已激活/2-已使用
	uint8_t self_Buff_status : 3;	  // 己方Buff状态：0-无/1-攻击/2-防御/3-恢复/4-冷却
	uint8_t self_highland_status : 6; // 己方高地状态：0-未占领/1-已占领
	uint8_t self_BaseShield : 7;	  // 己方基地护盾状态：0-无/1-有
	uint16_t last_dart_time : 9;	  // 己方飞镖剩余发射时间（秒）
	uint8_t dart_target : 2;		  // 飞镖目标：0-前哨站/1-基地
	uint8_t gain_point_statuss : 2;	  // 得分状态：0-无/1-击杀/2-占领
									  // uint8_t _ : 3;
} ext_event_data_t;

typedef struct // 0x0102 补给站动作标识
{
	uint8_t reserved;				// 保留字节
	uint8_t supply_robot_id;		// 补给站对应机器人ID
	uint8_t supply_projectile_step; // 补弹步骤：0-未开始/1-进行中/2-完成
	uint8_t supply_projectile_num;	// 补弹数量
} ext_supply_projectile_action_t;

typedef struct // 0x0103 请求补给站补弹数据，由参赛队发送（RM 对抗赛尚未开放）
{
	uint8_t supply_projectile_id; // 补给站ID
	uint8_t supply_robot_id;	  // 请求补弹的机器人ID
	uint8_t supply_num;			  // 请求补弹数量
} ext_supply_projectile_booking_t;

typedef struct // 0x0104 裁判警告信息
{
	uint8_t level;		   // 警告等级：1-黄牌/2-红牌/3-判负
	uint8_t foul_robot_id; // 犯规机器人ID
	uint8_t count;		   // 犯规次数
} ext_referee_warning_t;

typedef struct // 0x0105 飞镖发射口倒计时
{
	uint8_t dart_remaining_time; // 飞镖发射口剩余开启时间（秒）
	uint16_t dart_info;			 // 飞镖信息：bit0-1闸门状态/bit2-3目标
} ext_dart_remaining_time_t;

// 0x120 哨兵自主决策
typedef struct
{
	uint16_t data_cmd_id;			 // 数据命令ID
	uint16_t send_ID;				 // 发送者ID
	uint16_t receiver_ID;			 // 接收者ID（0x8080）
} Student_interactive_header_data_t; // 交互数据

typedef struct
{
	uint32_t sentry_if_revive : 1;				   // 是否允许复活：0-否/1-是
	uint32_t sentry_immediate_revive : 1;		   // 是否申请立即复活：0-否/1-是
	uint32_t sentry_bullet_claim : 11;			   // 申请购买弹丸数量
	uint32_t sentry_remote_bullet_claim_times : 4; // 远程兑换弹丸次数
	uint32_t sentry_remote_HP_claim_times : 4;	   // 远程兑换血量次数
	uint32_t sentry_posture : 2;				   // 哨兵姿态：0-未知/1-进攻/2-防御/3-移动
	uint32_t reserve : 9;						   // 保留位
} Sentry_decision_referee_t;

/* 0x020X --------------------------------------------------------------------*/
typedef struct // 0x0201 比赛机器人状态
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef struct // 0x0202 实时功率热量数据
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef struct // 0x0203 机器人位置
{
	float x;
	float y;
	float z;
	float yaw;
} ext_game_robot_pos_t;

typedef struct // 0x0204 机器人增益
{
	uint8_t recovery_buff;		// 恢复Buff等级：0-无/1-5级
	uint8_t cooling_buff;		// 冷却Buff等级：0-无/1-5级
	uint8_t defence_buff;		// 防御Buff等级：0-无/1-5级
	uint8_t vulnerability_buff; // 易伤Debuff等级：0-无/1-5级
	uint16_t attack_buff;		// 攻击Buff值
	uint8_t remaining_energy;	// 剩余能量
} ext_buff_musk_t;

typedef struct // 0x205 空中支援状态
{
	uint8_t airforce_status; // 空中支援状态：0-未激活/1-已激活/2-已使用
	uint8_t time_remain;	 // 空中支援剩余时间（秒）
} air_support_data_t;

typedef struct // 0x0206 伤害状态
{
	uint8_t armor_type : 4; // 被击中装甲板类型：0-前/1-后/2-左/3-右/4-上
	uint8_t hurt_type : 4;	// 伤害类型：0-装甲板/1-模块离线/2-超射速/3-超热量/4-超功率/5-撞击
} ext_robot_hurt_t;

typedef struct // 0x0207 实时射击信息
{
	uint8_t bullet_type; // 弹丸类型：1-17mm/2-42mm
	uint8_t shooter_id;	 // 发射机构ID：1-1号17mm/2-2号17mm/3-1号42mm
	uint8_t bullet_freq; // 发射频率（发/秒）
	float bullet_speed;	 // 弹丸初速度（m/s）
} ext_shoot_data_t;

typedef struct // 0x0208 子弹剩余发射数
{
	uint16_t bullet_remaining_num_17mm; // 17mm弹丸剩余发射数
	uint16_t bullet_remaining_num_42mm; // 42mm弹丸剩余发射数
	uint16_t coin_remaining_num;		// 剩余金币数量
} ext_bullet_remaining_t;

typedef struct // 0x0209 机器人RFID状态
{
	uint32_t rfid_status; // RFID状态：bit0-补给区/bit1-己方高地/bit2-对方高地/bit3-能量机关/bit4-飞镖区
} ext_rfid_status_t;

// typedef struct // 0x020A 飞镖机器人客户端指令数据
// {
// 	uint8_t dart_launch_opening_status;
// 	uint8_t dart_attack_target;
// 	uint16_t target_change_time;
// 	uint16_t operate_launch_cmd_time;
// } ext_dart_client_cmd_t;

typedef struct // 0x20B 地面机器人位置
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
} ground_robot_position_t;

// 0x020D 哨兵信息 (RoboMaster 2026协议 V1.2.0)
typedef struct
{
	// 字节0-3: sentry_info (4字节)
	uint32_t sentry_bullet_claimed : 11;	  // bit 0-10: 成功兑换的允许发弹量
	uint32_t sentry_remote_bullet_times : 4;  // bit 11-14: 远程兑换发弹量次数
	uint32_t sentry_remote_hp_times : 4;	  // bit 15-18: 远程兑换血量次数
	uint32_t sentry_can_free_revive : 1;	  // bit 19: 是否可以免费复活：0-否/1-是
	uint32_t sentry_can_instant_revive : 1;	  // bit 20: 是否可以兑换立即复活：0-否/1-是
	uint32_t sentry_instant_revive_cost : 10; // bit 21-30: 立即复活需要的金币数
	uint32_t sentry_reserved : 1;			  // bit 31: 保留

	// 字节4-5: sentry_info_2 (2字节)
	uint16_t sentry_disengaged : 1;			  // bit 0: 是否处于脱战状态：0-否/1-是
	uint16_t team_17mm_bullet_remaining : 11; // bit 1-11: 17mm弹量剩余可兑换数
	uint16_t sentry_posture : 2;			  // bit 12-13: 哨兵姿态：0-未知/1-进攻/2-防御/3-移动
	uint16_t rune_can_activate : 1;			  // bit 14: 能量机关可激活：0-否/1-是
	uint16_t sentry_reserved2 : 1;			  // bit 15: 保留
} sentry_info_t;

typedef struct
{
	uint8_t radar_double_hurt_chance : 2; // 雷达双倍伤害机会次数
	uint8_t radar_if_double_hurt : 1;	  // 是否激活双倍伤害：0-否/1-是
	uint8_t _ : 5;						  // 保留位
} radar_info_t;

/* 0x030X --------------------------------------------------------------------*/
typedef struct // 0x0301 机器人间通信 头结构体
{
	uint16_t data_cmd_id; // 数据命令ID
	uint16_t sender_ID;	  // 发送者机器人ID
	uint16_t receiver_ID; // 接收者机器人ID
						  // uint8_t *data;
} ext_student_interactive_header_data_t;

/*老版机器人坐标结构体
typedef struct Radar_map_info
{
	uint16_t hero_position_x;          // 英雄机器人X坐标
	uint16_t hero_position_y;          // 英雄机器人Y坐标
	uint16_t engineer_position_x;      // 工程机器人X坐标
	uint16_t engineer_position_y;      // 工程机器人Y坐标
	uint16_t infantry_3_position_x;    // 步兵3机器人X坐标
	uint16_t infantry_3_position_y;    // 步兵3机器人Y坐标
	uint16_t infantry_4_position_x;    // 步兵4机器人X坐标
	uint16_t infantry_4_position_y;    // 步兵4机器人Y坐标
	uint16_t infantry_5_position_x;    // 步兵5机器人X坐标
	uint16_t infantry_5_position_y;    // 步兵5机器人Y坐标
	uint16_t sentry_position_x;        // 哨兵机器人X坐标
	uint16_t sentry_position_y;        // 哨兵机器人Y坐标

}Radar_map_info_t;
*/

// 雷达发送的哨兵预警信息 (data_cmd_id = 0x0201)
// 格式: carID(2) + distance(4字节float) + quadrant(2)
typedef struct
{
    uint16_t car_id;      // 敌方车辆ID: 1-5,7红方; 101-105,107蓝方
    float distance;       // 距离，单位m
    uint16_t quadrant;    // 象限: 0-7，对应八个方向
} radar_sentinel_alert_t;

// 雷达发送的哨兵赛场坐标信息 (data_cmd_id = 0x0202)
// 格式: 7个坐标(哨兵自身 + 敌方1,2,3,4,6,7号)，每个坐标为float x + float y
typedef struct
{
    float sentry_x;           // 哨兵自身x坐标
    float sentry_y;           // 哨兵自身y坐标
    float enemy1_hero_x;      // 敌方1号英雄x
    float enemy1_hero_y;      // 敌方1号英雄y
    float enemy2_engineer_x;  // 敌方2号工程x
    float enemy2_engineer_y;  // 敌方2号工程y
    float enemy3_infantry_x;  // 敌方3号步兵x
    float enemy3_infantry_y;  // 敌方3号步兵y
    float enemy4_infantry_x;  // 敌方4号步兵x
    float enemy4_infantry_y;  // 敌方4号步兵y
    float enemy6_drone_x;     // 敌方6号无人机x
    float enemy6_drone_y;     // 敌方6号无人机y
    float enemy7_sentry_x;    // 敌方7号哨兵x
    float enemy7_sentry_y;    // 敌方7号哨兵y
} radar_sentry_position_t;

// 雷达发送的敌方血量信息 (data_cmd_id = 0x0205)
// 格式: 5个uint16血量，顺序为敌方1,2,3,4,7号
typedef struct
{
    uint16_t enemy1_hero_hp;      // 敌方1号英雄血量
    uint16_t enemy2_engineer_hp;  // 敌方2号工程血量
    uint16_t enemy3_infantry_hp;  // 敌方3号步兵血量
    uint16_t enemy4_infantry_hp;  // 敌方4号步兵血量
    uint16_t enemy7_sentry_hp;    // 敌方7号哨兵血量
} radar_enemy_hp_t;

typedef struct // 0x0301 机器人间通信 数据结构体（兼容旧代码映射）
{
    uint16_t data_cmd_id;            // 数据命令ID
    uint16_t sender_id;              // 发送者机器人ID
    uint16_t receiver_id;            // 接收者机器人ID
    radar_sentry_position_t position; // 雷达位置信息(0x0202)
    radar_enemy_hp_t enemy_hp;        // 雷达血量信息(0x0205)
} robot_interactive_data_t;

typedef struct // 0x0303 小地图下发信息标识
{
	float target_position_x;  // 目标位置X坐标
	float target_position_y;  // 目标位置Y坐标
	float target_position_z;  // 目标位置Z坐标
	uint8_t commd_keyboard;	  // 键盘按键命令
	uint16_t target_robot_ID; // 目标机器人ID
} ext_robot_command_t;

typedef struct // 0x0305 小地图接收信息标识
{
	uint16_t target_robot_ID; // 目标机器人ID
	float target_position_x;  // 目标位置X坐标
	float target_position_y;  // 目标位置Y坐标
} ext_client_map_command_t;

/* 自定义绘制UI结构体 -------------------------------------------------------*/
typedef struct // 绘制UI UI图形数据
{
	uint8_t graphic_name[3];   // 图形名称（客户端索引）
	uint32_t operate_tpye : 3; // 操作类型：0-空/1-增加/2-修改/3-删除
	uint32_t graphic_tpye : 3; // 图形类型：0-直线/1-矩形/2-圆/3-椭圆/4-圆弧/5-浮点/6-整形/7-字符
	uint32_t layer : 4;		   // 图层：0-9
	uint32_t color : 4;		   // 颜色：0-主色/1-黄/2-绿/3-橙/4-紫/5-粉/6-青/7-黑/8-白
	uint32_t start_angle : 9;  // 起始角度（0-360度）
	uint32_t end_angle : 9;	   // 结束角度（0-360度）
	uint32_t width : 10;	   // 线宽
	uint32_t start_x : 11;	   // 起始X坐标
	uint32_t start_y : 11;	   // 起始Y坐标
	uint32_t radius : 10;	   // 半径（圆/椭圆/圆弧用）
	uint32_t end_x : 11;	   // 结束X坐标（直线/矩形用）
	uint32_t end_y : 11;	   // 结束Y坐标（直线/矩形用）
} graphic_data_struct_t;

typedef struct // 绘制UI UI字符串数据
{
	uint8_t string_name[3];	   // 字符串名称（客户端索引）
	uint32_t operate_tpye : 3; // 操作类型：0-空/1-增加/2-修改/3-删除
	uint32_t graphic_tpye : 3; // 图形类型：固定7（字符型）
	uint32_t layer : 4;		   // 图层：0-9
	uint32_t color : 4;		   // 颜色：0-主色/1-黄/2-绿/3-橙/4-紫/5-粉/6-青/7-黑/8-白
	uint32_t start_angle : 9;  // 保留
	uint32_t end_angle : 9;	   // 保留
	uint32_t width : 10;	   // 线宽
	uint32_t start_x : 11;	   // 起始X坐标
	uint32_t start_y : 11;	   // 起始Y坐标
	uint32_t null;			   // 保留
	uint8_t stringdata[30];	   // 字符串内容（最大30字节）
} string_data_struct_t;

typedef struct // 绘制UI UI删除图形数据
{
	uint8_t operate_tpye; // 删除操作：0-空/1-删图层/2-删所有
	uint8_t layer;		  // 目标图层（删图层时有效）
} delete_data_struct_t;

typedef struct // 绘制UI 绘制1个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;			  // 协议帧头
	uint16_t CMD_ID;										  // 命令ID（0x0301）
	ext_student_interactive_header_data_t Interactive_Header; // 交互数据头
	graphic_data_struct_t Graphic[1];						  // 1个图形数据
	uint16_t CRC16;											  // CRC16校验
} UI_Graph1_t;

typedef struct // 绘制UI 绘制2个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;			  // 协议帧头
	uint16_t CMD_ID;										  // 命令ID（0x0301）
	ext_student_interactive_header_data_t Interactive_Header; // 交互数据头
	graphic_data_struct_t Graphic[2];						  // 2个图形数据
	uint16_t CRC16;											  // CRC16校验
} UI_Graph2_t;

typedef struct // 绘制UI 绘制5个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;			  // 协议帧头
	uint16_t CMD_ID;										  // 命令ID（0x0301）
	ext_student_interactive_header_data_t Interactive_Header; // 交互数据头
	graphic_data_struct_t Graphic[5];						  // 5个图形数据
	uint16_t CRC16;											  // CRC16校验
} UI_Graph5_t;

typedef struct // 绘制UI 绘制7个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;			  // 协议帧头
	uint16_t CMD_ID;										  // 命令ID（0x0301）
	ext_student_interactive_header_data_t Interactive_Header; // 交互数据头
	graphic_data_struct_t Graphic[7];						  // 7个图形数据
	uint16_t CRC16;											  // CRC16校验
} UI_Graph7_t;

typedef struct // 绘制UI 绘制1字符串完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;			  // 协议帧头
	uint16_t CMD_ID;										  // 命令ID（0x0301）
	ext_student_interactive_header_data_t Interactive_Header; // 交互数据头
	string_data_struct_t String;							  // 字符串数据
	uint16_t CRC16;											  // CRC16校验
} UI_String_t;

#define N sizeof(graphic_data_struct_t) // UI图形数据结构体大小（用于删除操作）

typedef struct // 绘制UI UI删除图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;			  // 协议帧头
	uint16_t CMD_ID;										  // 命令ID（0x0301）
	ext_student_interactive_header_data_t Interactive_Header; // 交互数据头
	delete_data_struct_t Delete;							  // 删除操作数据
	uint16_t CRC16;											  // CRC16校验
} UI_Delete_t;

#pragma pack(pop)

/* Functions -----------------------------------------------------------------*/
void Referee_StructInit(void);
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength);

void Referee_UnpackFifoData(void);
void Referee_SolveFifoData(uint8_t *frame);

void UI_Draw_Line(graphic_data_struct_t *Graph,		 // UI图形数据结构体指针
				  char GraphName[3],				 // 图形名 作为客户端的索引
				  uint8_t GraphOperate,				 // UI图形操作 对应UI_Graph_XXX的4种操作
				  uint8_t Layer,					 // UI图形图层 [0,9]
				  uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
				  uint16_t Width,					 // 线宽
				  uint16_t StartX,					 // 起始坐标X
				  uint16_t StartY,					 // 起始坐标Y
				  uint16_t EndX,					 // 截止坐标X
				  uint16_t EndY);					 // 截止坐标Y
void UI_Draw_Rectangle(graphic_data_struct_t *Graph, // UI图形数据结构体指针
					   char GraphName[3],			 // 图形名 作为客户端的索引
					   uint8_t GraphOperate,		 // UI图形操作 对应UI_Graph_XXX的4种操作
					   uint8_t Layer,				 // UI图形图层 [0,9]
					   uint8_t Color,				 // UI图形颜色 对应UI_Color_XXX的9种颜色
					   uint16_t Width,				 // 线宽
					   uint16_t StartX,				 // 起始坐标X
					   uint16_t StartY,				 // 起始坐标Y
					   uint16_t EndX,				 // 截止坐标X
					   uint16_t EndY);				 // 截止坐标Y
void UI_Draw_Circle(graphic_data_struct_t *Graph,	 // UI图形数据结构体指针
					char GraphName[3],				 // 图形名 作为客户端的索引
					uint8_t GraphOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
					uint8_t Layer,					 // UI图形图层 [0,9]
					uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
					uint16_t Width,					 // 线宽
					uint16_t CenterX,				 // 圆心坐标X
					uint16_t CenterY,				 // 圆心坐标Y
					uint16_t Radius);				 // 半径
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,	 // UI图形数据结构体指针
					 char GraphName[3],				 // 图形名 作为客户端的索引
					 uint8_t GraphOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
					 uint8_t Layer,					 // UI图形图层 [0,9]
					 uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
					 uint16_t Width,				 // 线宽
					 uint16_t CenterX,				 // 圆心坐标X
					 uint16_t CenterY,				 // 圆心坐标Y
					 uint16_t XHalfAxis,			 // X半轴长
					 uint16_t YHalfAxis);			 // Y半轴长
void UI_Draw_Arc(graphic_data_struct_t *Graph,		 // UI图形数据结构体指针
				 char GraphName[3],					 // 图形名 作为客户端的索引
				 uint8_t GraphOperate,				 // UI图形操作 对应UI_Graph_XXX的4种操作
				 uint8_t Layer,						 // UI图形图层 [0,9]
				 uint8_t Color,						 // UI图形颜色 对应UI_Color_XXX的9种颜色
				 uint16_t StartAngle,				 // 起始角度 [0,360]
				 uint16_t EndAngle,					 // 截止角度 [0,360]
				 uint16_t Width,					 // 线宽
				 uint16_t CenterX,					 // 圆心坐标X
				 uint16_t CenterY,					 // 圆心坐标Y
				 uint16_t XHalfAxis,				 // X半轴长
				 uint16_t YHalfAxis);				 // Y半轴长
void UI_Draw_Float(graphic_data_struct_t *Graph,	 // UI图形数据结构体指针
				   char GraphName[3],				 // 图形名 作为客户端的索引
				   uint8_t GraphOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
				   uint8_t Layer,					 // UI图形图层 [0,9]
				   uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
				   uint16_t NumberSize,				 // 字体大小
				   uint16_t Significant,			 // 有效位数
				   uint16_t Width,					 // 线宽
				   uint16_t StartX,					 // 起始坐标X
				   uint16_t StartY,					 // 起始坐标Y
				   float FloatData);				 // 数字内容
void UI_Draw_Int(graphic_data_struct_t *Graph,		 // UI图形数据结构体指针
				 char GraphName[3],					 // 图形名 作为客户端的索引
				 uint8_t GraphOperate,				 // UI图形操作 对应UI_Graph_XXX的4种操作
				 uint8_t Layer,						 // UI图形图层 [0,9]
				 uint8_t Color,						 // UI图形颜色 对应UI_Color_XXX的9种颜色
				 uint16_t NumberSize,				 // 字体大小
				 uint16_t Width,					 // 线宽
				 uint16_t StartX,					 // 起始坐标X
				 uint16_t StartY,					 // 起始坐标Y
				 int32_t IntData);					 // 数字内容
void UI_Draw_String(string_data_struct_t *String,	 // UI图形数据结构体指针
					char StringName[3],				 // 图形名 作为客户端的索引
					uint8_t StringOperate,			 // UI图形操作 对应UI_Graph_XXX的4种操作
					uint8_t Layer,					 // UI图形图层 [0,9]
					uint8_t Color,					 // UI图形颜色 对应UI_Color_XXX的9种颜色
					uint16_t CharSize,				 // 字体大小
					uint16_t StringLength,			 // 字符串长度
					uint16_t Width,					 // 线宽
					uint16_t StartX,				 // 起始坐标X
					uint16_t StartY,				 // 起始坐标Y
					char *StringData);				 // 字符串内容

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);

/* 裁判系统数据解码器 */
typedef struct Referee_Decoder
{
	uint16_t judgementFullCount; // FIFO缓冲区圈数
	uint64_t receive_data_len;	 // 接收数据总长度
	uint64_t decode_data_len;	 // 解码数据总长度

	uint8_t judgementStep; // 解码步骤：0-找帧头/1-读帧头/2-读数据/3-校验
	uint16_t index;		   // 当前解包位置索引
	uint16_t data_len;	   // 数据帧总长度

} Referee_Decoder;

typedef struct
{
	uint8_t alert_flag; // 哨兵告警标志：0-无告警/1-有告警
} Sentry_alert_t;

typedef struct Referee_t
{
	/* protocol包头结构体 */
	frame_header_struct_t Referee_Receive_Header; // 接收数据帧头

	/* 0x000X 比赛基础数据 */
	ext_game_status_t Game_Status;			// 0x0001 比赛状态
	ext_game_result_t Game_Result;			// 0x0002 比赛结果
	robot_HP_friend_t Game_Robot_friend_HP; // 0x0003 友方机器人血量

	/* 0x010X 场地事件数据 */
	ext_event_data_t Event_Data;							   // 0x0101 场地事件
	ext_supply_projectile_action_t Supply_Projectile_Action;   // 0x0102 补给站动作
	ext_supply_projectile_booking_t Supply_Projectile_Booking; // 0x0103 补弹请求
	ext_referee_warning_t Referee_Warning;					   // 0x0104 裁判警告
	ext_dart_remaining_time_t Dart_Remaining_Time;			   // 0x0105 飞镖倒计时

	/* 0x020X 机器人状态数据 */
	// ext_game_robot_state_t Game_Robot_State;
	// ext_power_heat_data_t Power_Heat_Data;
	// ext_game_robot_pos_t Game_Robot_Pos;
	// ext_buff_musk_t Buff_Musk;
	// aerial_robot_energy_t Aerial_Robot_Energy;
	// ext_robot_hurt_t Robot_Hurt;
	// ext_shoot_data_t Shoot_Data;
	// ext_bullet_remaining_t Bullet_Remaining;

	// ext_dart_client_cmd_t Dart_Client_Cmd;
	ext_game_robot_state_t Game_Robot_State;	   // 0x0201 机器人状态
	ext_power_heat_data_t Power_Heat_Data;		   // 0x0202 功率热量
	ext_game_robot_pos_t Game_Robot_Pos;		   // 0x0203 机器人位置
	ext_buff_musk_t Buff_Musk;					   // 0x0204 机器人增益
	ext_robot_hurt_t Robot_Hurt;				   // 0x0206 伤害状态
	ext_shoot_data_t Shoot_Data;				   // 0x0207 射击信息
	ext_bullet_remaining_t Bullet_Remaining;	   // 0x0208 剩余弹量
	ext_rfid_status_t rfid_status;				   // 0x0209 RFID状态
	ground_robot_position_t ground_robot_position; // 0x020B 地面机器人位置
	sentry_info_t Sentry_info;					   // 0x020D 哨兵信息

	radar_info_t Radar_Info; // 0x020E 雷达信息

	Sentry_alert_t Sentry_alert_info; // 哨兵告警信息

	/* 0x030X 机器人间通信数据 */
	ext_student_interactive_header_data_t Student_Interactive_Header_Data; // 通信头数据
	robot_interactive_data_t Robot_Interactive_Data;					   // 通信数据
	ext_robot_command_t Robot_Command;									   // 0x0303 小地图下发
	ext_client_map_command_t Client_Map_Command;						   // 0x0305 小地图接收

	/* 雷达站发送的数据 (通过0x0301机器人交互链路) */
	radar_sentinel_alert_t Radar_Alert_Info;     // 0x0201 哨兵预警信息
	radar_sentry_position_t Radar_Position_Info; // 0x0202 哨兵赛场坐标
	radar_enemy_hp_t Radar_Enemy_HP;             // 0x0205 敌方血量

	// /* 绘制UI专用结构体 */
	// UI_Graph1_t UI_Graph1;
	// UI_Graph2_t UI_Graph2;
	// UI_Graph5_t UI_Graph5;
	// UI_Graph7_t UI_Graph7;
	// UI_String_t UI_String;
	// UI_Delete_t UI_Delete;

	Referee_Decoder decoder; // 数据解码器

} Referee_t;

typedef struct RefereeDataUpdate
{
	int8_t is_max_power_data_update; // 最大功率数据更新标志：1-已更新/0-未更新
	int8_t is_power_data_update;	 // 功率数据更新标志：1-已更新/0-未更新
} RefereeDataUpdate;

enum
{
	NOT_RADAR_DATA = 0,	   // 0：不是雷达数据
	NEAREST_ENEMY_POS = 1, // 1：最近的敌人位置
	ALL_ENEMY_POS = 2,	   // 2：所有敌人位置
	ALL_ENEMY_HP = 3	   // 3：所有敌人血量
};

extern RefereeDataUpdate referee_data_updater; // 裁判系统数据更新标志
extern Referee_t referee_data;				   // 裁判系统总数据
extern uint8_t Radar_double_hurt_chance;	   // 雷达双倍伤害机会次数
// 哨兵决策数据结构体声明
extern Sentry_decision_referee_t sentry_decision_referee; // 哨兵自主决策数据

#define MAX_REFEREE_DATA_LEN 45

#endif /* __REFEREE_H__ */
