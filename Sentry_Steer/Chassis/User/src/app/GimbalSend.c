/**
 ******************************************************************************
 * @file    GimbalSend.c
 * @brief   底盘向云台发送数据：裁判系统数据、云台控制指令、底盘速度、位置信息等
 * @note    基于CAN2总线通信，按不同频率调度发送，数据来源于全局裁判系统结构体
 ******************************************************************************
 */

// 云台发送相关头文件
#include "GimbalSend.h"
#include "ChasisController.h" // 引入步兵机器人结构体

// 云台发送数据包1 全局实例
GimbalSendPack_1 gimbal_pack_send_1;
// 裁判系统发送数据结构体 全局实例
JudgeData_ForSend1_t JudgeData_ForSend1;
JudgeData_ForSend2_t JudgeData_ForSend2;
// 血量发送数据结构体 全局实例
JudgeBloodData_ForSend1_t JudgeBloodData_ForSend_Friend, JudgeBloodData_ForSend_Enemy;
// BUFF状态发送数据结构体 全局实例
JudgeData_Buff_t JudgeData_Buff;
// RFID状态发送数据结构体 全局实例
JudgeData_RFID_t JudgeData_RFID;
// 机器人位置发送数据结构体 全局实例
JudgeData_position_t JudgeData_position;
// 底盘速度数据包 全局实例
ChassisSpeedPack_t chassis_speed_pack_send;
// 外部声明：超级电容控制器
extern NingCapController cap_controller;
// 雷达消息更新标志位
uint8_t radar_msg_update_flag;

// 云台CAN发送数据缓冲区（8字节）
int8_t send_to_gimbal_data[8];

/**
 * @brief  云台数据打包并拷贝到发送缓冲区
 * @param  无
 * @retval 无
 */
void SendToGimbalPack()
{
  // 云台指令数据打包
  GimbalSendPack();
  // 将打包后的数据拷贝到CAN发送缓冲区
  memcpy(send_to_gimbal_data, &gimbal_pack_send_1, 8);
}

// 比赛开始标志位 1=比赛进行中 0=未开始
uint8_t If_Game_Start = 0;

/**
 * @brief  云台控制指令打包：射击权限、阵营、电容电压、弹速等
 * @param  无
 * @retval 无
 */
void GimbalSendPack()
{
  // 弹速放大100倍 提高传输精度
  float bullet_spd_100 = referee_data.Shoot_Data.bullet_speed * 100;
  // 射击允许标志
  gimbal_pack_send_1.is_shootable = heat_controller.shoot_flag;
  // 机器人阵营判断 <10为红方 否则蓝方
  gimbal_pack_send_1.robot_color = referee_data.Game_Robot_State.robot_id < 10 ? 1 : 0;
  // 超电容电压 换算后赋值
  gimbal_pack_send_1.half_CapVol = (uint8_t)cap_controller.cap_vol / 2.0f; // referee_data.Game_Robot_State.shooter_id1_17mm_speed_limit <= 15 ? 0 : (referee_data.Game_Robot_State.shooter_id1_17mm_speed_limit <= 22 ? 1 : 2);
  // BUFF状态 默认0
  gimbal_pack_send_1.buff_state = 0; // referee_data.Buff_Musk.power_rune_buff & 0x0F;

  // 弹丸速度赋值
  gimbal_pack_send_1.bullet_speed = (uint16_t)bullet_spd_100;
}

/**
 * @brief 底盘速度数据打包：通过舵电机角度和轮电机速度反解的底盘实际速度
 * @note 底盘坐标系：x向右，y向前，yaw逆时针为正
 *       infantry.x_v, y_v 单位为m/s，yaw_v单位为rad/s
 */
void ChassisSpeedPack(void)
{
  // 将底盘速度乘100发送，单位0.01 m/s / 0.01 rad/s
  chassis_speed_pack_send.chassis_x_v_100 = (int16_t)(infantry.x_v * 100.0f);
  chassis_speed_pack_send.chassis_y_v_100 = (int16_t)(infantry.y_v * 100.0f);
  chassis_speed_pack_send.chassis_yaw_v_100 = (int16_t)(infantry.yaw_v * 100.0f);
  // 保留位清零
  chassis_speed_pack_send.reserve[0] = 0;
  chassis_speed_pack_send.reserve[1] = 0;
}

/**
 * @brief 底盘速度数据发送（CAN2）
 * @param  无
 * @retval 无
 */
void Can2SendChassisSpeed(void)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  // 底盘速度发送CANID
  tx_message.StdId = SEND_TO_GIMBAL_CHASSIS_SPEED_CAN_ID;
  // 数据拷贝到CAN发送帧
  memcpy(tx_message.Data, &chassis_speed_pack_send, sizeof(ChassisSpeedPack_t));
  // CAN发送
  CAN_Transmit(CAN2, &tx_message);
}

/**
 * @brief  全场机器人/基地血量数据打包
 * @param  无
 * @retval 无
 * @note   友方血量来自裁判系统，敌方血量来自雷达 按比例压缩发送
 */
void JudgeDataBloodPack()
{
  JudgeBloodData_ForSend_Friend.blood_type = 0; // 友方为0
  JudgeBloodData_ForSend_Friend.ID1 = referee_data.Game_Robot_friend_HP.friend_1_robot_HP / 10;
  JudgeBloodData_ForSend_Friend.ID2 = referee_data.Game_Robot_friend_HP.friend_2_robot_HP / 10;
  JudgeBloodData_ForSend_Friend.ID3 = referee_data.Game_Robot_friend_HP.friend_3_robot_HP / 10;
  JudgeBloodData_ForSend_Friend.ID4 = referee_data.Game_Robot_friend_HP.friend_4_robot_HP / 10;
  JudgeBloodData_ForSend_Friend.ID_reserve = 0;
  JudgeBloodData_ForSend_Friend.ID7 = referee_data.Game_Robot_friend_HP.friend_7_robot_HP / 10;
  JudgeBloodData_ForSend_Friend.ID8 = referee_data.Game_Robot_friend_HP.friend_base_HP / 100;
  JudgeBloodData_ForSend_Friend.reserve = 0;

  JudgeBloodData_ForSend_Enemy.blood_type = 1;
  JudgeBloodData_ForSend_Enemy.ID1 = referee_data.Robot_Interactive_Data.enemy_hp.enemy1_hero_hp / 10;
  JudgeBloodData_ForSend_Enemy.ID2 = referee_data.Robot_Interactive_Data.enemy_hp.enemy2_engineer_hp / 10;
  JudgeBloodData_ForSend_Enemy.ID3 = referee_data.Robot_Interactive_Data.enemy_hp.enemy3_infantry_hp / 10;
  JudgeBloodData_ForSend_Enemy.ID4 = referee_data.Robot_Interactive_Data.enemy_hp.enemy4_infantry_hp / 10;
  JudgeBloodData_ForSend_Enemy.ID_reserve = 0;
  JudgeBloodData_ForSend_Enemy.ID7 = referee_data.Robot_Interactive_Data.enemy_hp.enemy7_sentry_hp / 10;
  //JudgeBloodData_ForSend_Enemy.ID8 = referee_data.Robot_Interactive_Data.enemy_hp.enemy_base_HP / 100;
  JudgeBloodData_ForSend_Enemy.ID8 = 0; // 雷达暂时无法获取敌方基地血量，先填0
  JudgeBloodData_ForSend_Enemy.reserve = 0;
}

/**
 * @brief  RFID状态与BUFF增益数据打包
 * @param  无
 * @retval 无
 */
void JudgeDataRFIDandBuffPack()
{
  JudgeData_RFID.data_type = 0x01;
  // RFID状态数据
  JudgeData_RFID.rfid_status = referee_data.rfid_status.rfid_status;
  // 场地事件数据
  JudgeData_RFID.event_data = referee_data.Event_Data;
  JudgeData_Buff.data_type = 0x0;
  // 回血BUFF
  JudgeData_Buff.recovery_buff = referee_data.Buff_Musk.recovery_buff;
  // 冷却BUFF
  JudgeData_Buff.cooling_buff = referee_data.Buff_Musk.cooling_buff;
  // 防御BUFF
  JudgeData_Buff.defence_buff = referee_data.Buff_Musk.defence_buff;
  // 易伤BUFF
  JudgeData_Buff.vulnerability_buff = referee_data.Buff_Musk.vulnerability_buff;
  // 攻击BUFF
  JudgeData_Buff.attack_buff = referee_data.Buff_Musk.attack_buff;
  // 剩余能量
  JudgeData_Buff.remaining_energy = referee_data.Buff_Musk.remaining_energy;
}

/**
 * @brief  全场机器人位置数据打包
 * @param  无
 * @retval 无
 * @note   友方坐标来自裁判系统 敌方坐标来自雷达交互数据
 */
void JudgeDataPositionPack()
{
  // 友方英雄机器人位置 类型1
  JudgeData_position.Friend[1].position_type = 1;
  JudgeData_position.Friend[1].ID_X_100 = (int16_t)(referee_data.ground_robot_position.hero_x * 100);
  JudgeData_position.Friend[1].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.hero_y * 100);
  JudgeData_position.Friend[1].reserve = 0;
  // 友方工程机器人位置 类型2
  JudgeData_position.Friend[2].position_type = 2;
  JudgeData_position.Friend[2].ID_X_100 = (int16_t)(referee_data.ground_robot_position.engineer_x * 100);
  JudgeData_position.Friend[2].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.engineer_y * 100);
  JudgeData_position.Friend[2].reserve = 0;
  // 友方3号步兵位置 类型3
  JudgeData_position.Friend[3].position_type = 3;
  JudgeData_position.Friend[3].ID_X_100 = (int16_t)(referee_data.ground_robot_position.standard_3_x * 100);
  JudgeData_position.Friend[3].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.standard_3_y * 100);
  JudgeData_position.Friend[3].reserve = 0;
  // 友方4号步兵位置 类型4
  JudgeData_position.Friend[4].position_type = 4;
  JudgeData_position.Friend[4].ID_X_100 = (int16_t)(referee_data.ground_robot_position.standard_4_x * 100);
  JudgeData_position.Friend[4].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.standard_4_y * 100);
  JudgeData_position.Friend[4].reserve = 0;
  // 友方5号步兵位置 类型5
  JudgeData_position.Friend[5].position_type = 5;
  JudgeData_position.Friend[5].ID_X_100 = (int16_t)(referee_data.ground_robot_position.standard_5_x * 100);
  JudgeData_position.Friend[5].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.standard_5_y * 100);
  JudgeData_position.Friend[5].reserve = 0;
  // 友方哨兵机器人位置 类型7
  JudgeData_position.Friend[7].position_type = 7;
  JudgeData_position.Friend[7].ID_X_100 = (int16_t)(referee_data.Game_Robot_Pos.x * 100);
  JudgeData_position.Friend[7].ID_Y_100 = (int16_t)(referee_data.Game_Robot_Pos.y * 100);
  JudgeData_position.Friend[7].reserve = 0;

  // 敌方机器人位置 来源于雷达交互数据
  JudgeData_position.Enemy[0].position_type = 100;
  JudgeData_position.Enemy[0].ID_X_100 = 0;//显式指定为0
  JudgeData_position.Enemy[0].ID_Y_100 = 0;
  JudgeData_position.Enemy[0].reserve = 0;
  JudgeData_position.Enemy[1].position_type = 101;
  JudgeData_position.Enemy[1].ID_X_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy1_hero_x * 100);
  JudgeData_position.Enemy[1].ID_Y_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy1_hero_y * 100);
  JudgeData_position.Enemy[1].reserve = 0;
  JudgeData_position.Enemy[2].position_type = 102;
  JudgeData_position.Enemy[2].ID_X_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy2_engineer_x * 100);
  JudgeData_position.Enemy[2].ID_Y_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy2_engineer_y * 100);
  JudgeData_position.Enemy[2].reserve = 0;
  JudgeData_position.Enemy[3].position_type = 103;
  JudgeData_position.Enemy[3].ID_X_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy3_infantry_x * 100);
  JudgeData_position.Enemy[3].ID_Y_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy3_infantry_y * 100);
  JudgeData_position.Enemy[3].reserve = 0;
  JudgeData_position.Enemy[4].position_type = 104;
  JudgeData_position.Enemy[4].ID_X_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy4_infantry_x * 100);
  JudgeData_position.Enemy[4].ID_Y_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy4_infantry_y * 100);
  JudgeData_position.Enemy[4].reserve = 0;
  JudgeData_position.Enemy[5].position_type = 105;
  JudgeData_position.Enemy[5].ID_X_100 = 0;//5号步兵不存在
  JudgeData_position.Enemy[5].ID_Y_100 = 0;
  JudgeData_position.Enemy[5].reserve = 0;
  JudgeData_position.Enemy[6].position_type = 106;
  JudgeData_position.Enemy[6].ID_X_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy6_drone_x * 100);
  JudgeData_position.Enemy[6].ID_Y_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy6_drone_y * 100);
  JudgeData_position.Enemy[6].reserve = 0;
  JudgeData_position.Enemy[7].position_type = 107;
  JudgeData_position.Enemy[7].ID_X_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy7_sentry_x * 100);
  JudgeData_position.Enemy[7].ID_Y_100 = (int16_t)(referee_data.Robot_Interactive_Data.position.enemy7_sentry_y * 100);
  JudgeData_position.Enemy[7].reserve = 0;

  // 雷达消息更新标志位 处理完成后清零
  if (radar_msg_update_flag == ALL_ENEMY_POS)
  {
    radar_msg_update_flag = 0;
  }
}

/**
 * @brief  裁判系统总数据打包：整合所有需要发送给云台的裁判数据
 * @param  无
 * @retval 无
 */
void JudgeDataPack()
{
  // 判断机器人阵营 红方<10 蓝方>=10
  if (referee_data.Game_Robot_State.robot_id < 10)
  {
    JudgeData_ForSend1.Robot_Red_Blue = 1;
  }
  else
  {
    JudgeData_ForSend1.Robot_Red_Blue = 0;
  }

  // 根据阵营赋值 己方/敌方前哨站血量
  JudgeData_ForSend1.self_outpost = (uint8_t)(0.04f * (referee_data.Game_Robot_friend_HP.friend_outpost_HP + 24));
  //JudgeData_ForSend1.Enemy_outpost = (uint8_t)(0.04f * (referee_data.Robot_Interactive_Data.enemy_HP.enemy_outpost_HP + 24));
  // 热量更新标志
  JudgeData_ForSend1.Heat_update = 0x01;
  // 17mm枪口热量
  JudgeData_ForSend1.shooter1_heat = referee_data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;
  // 17mm弹丸剩余数量
  JudgeData_ForSend1.bullet_remaining_num_17mm = referee_data.Bullet_Remaining.bullet_remaining_num_17mm;
  // 哨兵姿态 1=进攻 2=防御 3=移动 0=未知
  JudgeData_ForSend1.sentry_posture = referee_data.Sentry_info.sentry_posture;
  // 比赛进行状态 0x04=比赛中
  If_Game_Start = (referee_data.Game_Status.game_progress == 0x04) ? 1 : 0;
  JudgeData_ForSend1.is_game_start = If_Game_Start;
  // 阶段剩余时间 减半发送
  JudgeData_ForSend1.stage_remain_time = referee_data.Game_Status.stage_remain_time / 2;

  // 自身坐标X 放大100倍
  JudgeData_ForSend2.x = (uint16_t)(referee_data.Game_Robot_Pos.x * 100);
  // 自身坐标Y 放大100倍
  JudgeData_ForSend2.y = (uint16_t)(referee_data.Game_Robot_Pos.y * 100);
  // 自身Yaw角 放大10倍
  JudgeData_ForSend2.yaw_10 = (int16_t)(referee_data.Game_Robot_Pos.yaw * 10);
  // 自身剩余血量
  JudgeData_ForSend2.Self_blood = referee_data.Game_Robot_State.remain_HP;

  // 子数据包打包调用
  JudgeDataBloodPack();
  JudgeDataRFIDandBuffPack();
  JudgeDataPositionPack();
}

/**
 * @brief  CAN2发送裁判数据1
 * @param  Judge2Send: 待发送的数据指针
 * @retval 无
 */
void Can2Send1(JudgeData_ForSend1_t *Judge2Send)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_JUDGE_DATA_CAN_ID1;

  memcpy(tx_message.Data, Judge2Send, sizeof(JudgeData_ForSend1_t));

  CAN_Transmit(CAN2, &tx_message);
}

/**
 * @brief  CAN2发送裁判数据2
 * @param  Judge2Send: 待发送的数据指针
 * @retval 无
 */
void Can2Send2(JudgeData_ForSend2_t *Judge2Send)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_JUDGE_DATA_CAN_ID2;

  memcpy(tx_message.Data, Judge2Send, sizeof(JudgeData_ForSend2_t));

  CAN_Transmit(CAN2, &tx_message);
}

/**
 * @brief  CAN2发送血量数据1
 * @param  Blood2Send1: 待发送的血量数据指针
 * @retval 无
 */
void Can2Send3_blood1(JudgeBloodData_ForSend1_t *Blood2Send1)
{

  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1;
  memcpy(tx_message.Data, Blood2Send1, sizeof(JudgeBloodData_ForSend1_t));
  CAN_Transmit(CAN2, &tx_message);
}

/**
 * @brief  CAN2发送RFID数据
 * @param  judgeData_RFID: RFID数据指针
 * @retval 无
 */
void Can2Send4_RFID(JudgeData_RFID_t *judgeData_RFID)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID;
  memcpy(tx_message.Data, judgeData_RFID, sizeof(JudgeData_RFID_t));
  CAN_Transmit(CAN2, &tx_message);
}

/**
 * @brief  CAN2发送BUFF数据
 * @param  judgeData_buff: BUFF数据指针
 * @retval 无
 */
void Can2Send4_Buff(JudgeData_Buff_t *judgeData_buff)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID;
  memcpy(tx_message.Data, judgeData_buff, sizeof(JudgeData_Buff_t));
  CAN_Transmit(CAN2, &tx_message);
}

// 位置数据轮询发送计数器
uint16_t poscount = 0;

/**
 * @brief  CAN2轮询发送全场位置数据
 * @param  无
 * @retval 无
 * @note   14个机器人位置分14帧循环发送 避免单帧数据过长
 */
void Can2Send5()
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_POSITION_DATA_CAN_ID;

  // 轮询发送友方机器人位置
  if (poscount % 14 == 0)
    memcpy(tx_message.Data, &JudgeData_position.Friend[1], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 1)
    memcpy(tx_message.Data, &JudgeData_position.Friend[2], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 2)
    memcpy(tx_message.Data, &JudgeData_position.Friend[3], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 3)
    memcpy(tx_message.Data, &JudgeData_position.Friend[4], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 4)
    memcpy(tx_message.Data, &JudgeData_position.Friend[5], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 5)
    memcpy(tx_message.Data, &JudgeData_position.Friend[7], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 6)
    memcpy(tx_message.Data, &JudgeData_position.Friend[8], sizeof(Each_Robot_position_t));
  // 轮询发送敌方机器人位置
  if (poscount % 14 == 7)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[1], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 8)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[2], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 9)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[3], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 10)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[4], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 11)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[5], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 12)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[7], sizeof(Each_Robot_position_t));
  if (poscount % 14 == 13)
    memcpy(tx_message.Data, &JudgeData_position.Enemy[8], sizeof(Each_Robot_position_t));

  // 计数器自增
  poscount++;
  // 防止计数器溢出
  if (poscount > 10000)
    poscount = 0;

  // CAN发送
  CAN_Transmit(CAN2, &tx_message);
}

// 发送调度计数器 5ms自增1
int16_t send_count = 0;

/**
 * @brief  裁判数据CAN发送总调度函数
 * @param  无
 * @retval 无
 * @note   5ms执行一次 按不同频率调度所有CAN发送任务
 */
void JudgeDataCanSend(void)
{
  // 5ms执行一次

  // 云台指令打包
  SendToGimbalPack();
  // 裁判数据总打包
  JudgeDataPack();
  ChassisSpeedPack(); // 底盘速度数据打包

  // 20Hz 发送裁判数据1
  if (send_count % 10 == 0)
    Can2Send1(&JudgeData_ForSend1);
  // 20Hz 发送云台控制指令
  if (send_count % 10 == 6)
    CanSend(GIMBAL_CAN_COMM_CANx, send_to_gimbal_data, SEND_TO_GIMBAL_CAN_ID_1, 8);
  // 20Hz 发送裁判数据2
  if (send_count % 10 == 3)
    Can2Send2(&JudgeData_ForSend2);
  // 10Hz 发送友方血量数据
  if (send_count % 19 == 0)
    Can2Send3_blood1(&JudgeBloodData_ForSend_Friend);
  // 10Hz 发送敌方血量数据
  if (send_count % 19 == 9)
    Can2Send3_blood1(&JudgeBloodData_ForSend_Enemy);
  // 10Hz 发送BUFF数据
  if (send_count % 21 == 4)
    Can2Send4_Buff(&JudgeData_Buff);
  // 10Hz 发送RFID数据
  if (send_count % 21 == 8)
    Can2Send4_RFID(&JudgeData_RFID);
  // 10Hz 发送位置数据
  if (send_count % 21 == 16)
    Can2Send5();
  // 50Hz 发送底盘速度数据
  if (send_count % 10 == 2)
    Can2SendChassisSpeed();
  // 计数器溢出重置
  if (send_count == 3000)
  {
    send_count = -1;
  }
  // 计数器自增
  send_count++;
  // 热量更新标志清零
  JudgeData_ForSend1.Heat_update = 0x0;
}
