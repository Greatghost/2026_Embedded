#include "GimbalSend.h"
#include "ChasisController.h"  // 引入infantry结构体

GimbalSendPack_1 gimbal_pack_send_1;
JudgeData_ForSend1_t JudgeData_ForSend1;
JudgeData_ForSend2_t JudgeData_ForSend2;
JudgeBloodData_ForSend1_t JudgeBloodData_ForSend1,JudgeBloodData_ForSend2;
JudgeData_Buff_t JudgeData_Buff;
JudgeData_RFID_t JudgeData_RFID;
JudgeData_position_t JudgeData_position;
ChassisSpeedPack_t chassis_speed_pack_send;  // 底盘速度数据包
extern NingCapController cap_controller;
uint8_t radar_msg_update_flag;

int8_t send_to_gimbal_data[8];

void SendToGimbalPack()
{
    GimbalSendPack();
    memcpy(send_to_gimbal_data, &gimbal_pack_send_1, 8);
}

uint8_t If_Game_Start = 0;

void GimbalSendPack()
{
  float bullet_spd_100 = referee_data.Shoot_Data.bullet_speed * 100;
  gimbal_pack_send_1.is_shootable = heat_controller.shoot_flag;
  gimbal_pack_send_1.robot_color = referee_data.Game_Robot_State.robot_id < 10 ? 1 : 0;
  gimbal_pack_send_1.half_CapVol = (uint8_t) cap_controller.cap_vol/2.0f; // referee_data.Game_Robot_State.shooter_id1_17mm_speed_limit <= 15 ? 0 : (referee_data.Game_Robot_State.shooter_id1_17mm_speed_limit <= 22 ? 1 : 2);
  gimbal_pack_send_1.buff_state = 0;   // referee_data.Buff_Musk.power_rune_buff & 0x0F;

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
  chassis_speed_pack_send.reserve[0] = 0;
  chassis_speed_pack_send.reserve[1] = 0;
}

/**
 * @brief 底盘速度数据发送（CAN2）
 */
void Can2SendChassisSpeed(void)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SEND_TO_GIMBAL_CHASSIS_SPEED_CAN_ID;
  memcpy(tx_message.Data, &chassis_speed_pack_send, sizeof(ChassisSpeedPack_t));
  CAN_Transmit(CAN2, &tx_message);
}

void JudgeDataBloodPack(){
  if(JudgeData_ForSend1.Robot_Red_Blue == 1)
      {
        JudgeBloodData_ForSend1.blood_type = 0; // 友方为0
        JudgeBloodData_ForSend1.ID1 = referee_data.Game_Robot_HP.red_1_robot_HP / 10;
        JudgeBloodData_ForSend1.ID2 = referee_data.Game_Robot_HP.red_2_robot_HP / 10;
        JudgeBloodData_ForSend1.ID3 = referee_data.Game_Robot_HP.red_3_robot_HP / 10;
        JudgeBloodData_ForSend1.ID4 = referee_data.Game_Robot_HP.red_4_robot_HP / 10;
        JudgeBloodData_ForSend1.ID_reserve = 0;
        JudgeBloodData_ForSend1.ID7 = referee_data.Game_Robot_HP.red_7_robot_HP / 10;
        JudgeBloodData_ForSend1.ID8 = referee_data.Game_Robot_HP.red_base_HP /100;
        JudgeBloodData_ForSend1.reserve = 0;
        
        JudgeBloodData_ForSend2.blood_type = 1;
        JudgeBloodData_ForSend2.ID1 = referee_data.Game_Robot_HP.blue_1_robot_HP / 10;
        JudgeBloodData_ForSend2.ID2 = referee_data.Game_Robot_HP.blue_2_robot_HP / 10;
        JudgeBloodData_ForSend2.ID3 = referee_data.Game_Robot_HP.blue_3_robot_HP / 10;
        JudgeBloodData_ForSend2.ID4 = referee_data.Game_Robot_HP.blue_4_robot_HP / 10;
        JudgeBloodData_ForSend2.ID_reserve = 0;
        JudgeBloodData_ForSend2.ID7 = referee_data.Game_Robot_HP.blue_7_robot_HP / 10;
        JudgeBloodData_ForSend2.ID8 = referee_data.Game_Robot_HP.blue_base_HP /100;
        JudgeBloodData_ForSend2.reserve = 0;
      }
      else{
         JudgeBloodData_ForSend1.blood_type = 0;
         JudgeBloodData_ForSend1.ID1 = referee_data.Game_Robot_HP.blue_1_robot_HP / 10;
         JudgeBloodData_ForSend1.ID2 = referee_data.Game_Robot_HP.blue_2_robot_HP / 10;
         JudgeBloodData_ForSend1.ID3 = referee_data.Game_Robot_HP.blue_3_robot_HP / 10;
         JudgeBloodData_ForSend1.ID4 = referee_data.Game_Robot_HP.blue_4_robot_HP / 10;
         JudgeBloodData_ForSend1.ID7 = referee_data.Game_Robot_HP.blue_7_robot_HP / 10;
         JudgeBloodData_ForSend1.ID8 = referee_data.Game_Robot_HP.blue_base_HP /100;
         JudgeBloodData_ForSend1.reserve = 0;

         JudgeBloodData_ForSend2.blood_type = 1;
         JudgeBloodData_ForSend2.ID1 = referee_data.Game_Robot_HP.red_1_robot_HP / 10;
         JudgeBloodData_ForSend2.ID2 = referee_data.Game_Robot_HP.red_2_robot_HP / 10;
         JudgeBloodData_ForSend2.ID3 = referee_data.Game_Robot_HP.red_3_robot_HP / 10;
         JudgeBloodData_ForSend2.ID4 = referee_data.Game_Robot_HP.red_4_robot_HP / 10;
         JudgeBloodData_ForSend2.ID_reserve = 0;
         JudgeBloodData_ForSend2.ID7 = referee_data.Game_Robot_HP.red_7_robot_HP / 10;
         JudgeBloodData_ForSend2.ID8 = referee_data.Game_Robot_HP.red_base_HP /100;
         JudgeBloodData_ForSend2.reserve = 0;

      }
}
void JudgeDataRFIDandBuffPack()
{
  JudgeData_RFID.data_type = 0x01;
  JudgeData_RFID.rfid_status = referee_data.rfid_status.rfid_status;
  JudgeData_RFID.event_data = referee_data.Event_Data;
  JudgeData_Buff.data_type = 0x0;
  JudgeData_Buff.recovery_buff = referee_data.Buff_Musk.recovery_buff;
  JudgeData_Buff.cooling_buff = referee_data.Buff_Musk.cooling_buff;
  JudgeData_Buff.defence_buff = referee_data.Buff_Musk.defence_buff;
  JudgeData_Buff.vulnerability_buff = referee_data.Buff_Musk.vulnerability_buff;
  JudgeData_Buff.attack_buff = referee_data.Buff_Musk.attack_buff;
  JudgeData_Buff.remaining_energy = referee_data.Buff_Musk.remaining_energy;
}
void JudgeDataPositionPack()
{ 
  JudgeData_position.Friend[1].position_type = 1;
  JudgeData_position.Friend[1].ID_X_100 = (int16_t)(referee_data.ground_robot_position.hero_x * 100);
  JudgeData_position.Friend[1].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.hero_y * 100);
  JudgeData_position.Friend[1].reserve = 0;
  JudgeData_position.Friend[2].position_type = 2;
  JudgeData_position.Friend[2].ID_X_100 = (int16_t)(referee_data.ground_robot_position.engineer_x * 100);
  JudgeData_position.Friend[2].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.engineer_y * 100);
  JudgeData_position.Friend[2].reserve = 0;
  JudgeData_position.Friend[3].position_type = 3;
  JudgeData_position.Friend[3].ID_X_100 = (int16_t)(referee_data.ground_robot_position.standard_3_x * 100);
  JudgeData_position.Friend[3].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.standard_3_y * 100);
  JudgeData_position.Friend[3].reserve = 0;
  JudgeData_position.Friend[4].position_type = 4;
  JudgeData_position.Friend[4].ID_X_100 = (int16_t)(referee_data.ground_robot_position.standard_4_x * 100);
  JudgeData_position.Friend[4].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.standard_4_y * 100);
  JudgeData_position.Friend[4].reserve = 0;
  JudgeData_position.Friend[5].position_type = 5;
  JudgeData_position.Friend[5].ID_X_100 = (int16_t)(referee_data.ground_robot_position.standard_5_x * 100);
  JudgeData_position.Friend[5].ID_Y_100 = (int16_t)(referee_data.ground_robot_position.standard_5_y * 100);
  JudgeData_position.Friend[5].reserve = 0;
  JudgeData_position.Friend[7].position_type = 7;
  JudgeData_position.Friend[7].ID_X_100 = (int16_t)(referee_data.Game_Robot_Pos.x * 100);
  JudgeData_position.Friend[7].ID_Y_100 = (int16_t)(referee_data.Game_Robot_Pos.y * 100);
  JudgeData_position.Friend[7].reserve = 0;
  
	// 现在直接发送数据
		JudgeData_position.Enemy[1].position_type = 101;
    JudgeData_position.Enemy[1].ID_X_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.hero_position_x;
    JudgeData_position.Enemy[1].ID_Y_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.hero_position_y;
    JudgeData_position.Enemy[1].reserve = 0;
    JudgeData_position.Enemy[2].position_type = 102;
    JudgeData_position.Enemy[2].ID_X_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.engineer_position_x;
    JudgeData_position.Enemy[2].ID_Y_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.engineer_position_y;
    JudgeData_position.Enemy[2].reserve = 0;
    JudgeData_position.Enemy[3].position_type = 103;
    JudgeData_position.Enemy[3].ID_X_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.infantry_3_position_x;
    JudgeData_position.Enemy[3].ID_Y_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.infantry_3_position_y;
    JudgeData_position.Enemy[3].reserve = 0;
    JudgeData_position.Enemy[4].position_type = 104;
    JudgeData_position.Enemy[4].ID_X_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.infantry_4_position_x;
    JudgeData_position.Enemy[4].ID_Y_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.infantry_4_position_y;
    JudgeData_position.Enemy[4].reserve = 0;
    JudgeData_position.Enemy[5].position_type = 105;
    JudgeData_position.Enemy[5].ID_X_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.infantry_5_position_x;
    JudgeData_position.Enemy[5].ID_Y_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.infantry_5_position_y;
    JudgeData_position.Enemy[5].reserve = 0;
    JudgeData_position.Enemy[7].position_type = 107;
    JudgeData_position.Enemy[7].ID_X_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.sentry_position_x;
    JudgeData_position.Enemy[7].ID_Y_100 = (int16_t) referee_data.Robot_Interactive_Data.map_info.sentry_position_y;
    JudgeData_position.Enemy[7].reserve = 0;
  // 雷达站数据
  if(radar_msg_update_flag)
  {
    
    radar_msg_update_flag = 0;
  }
  

}
void JudgeDataPack()
{

  if(referee_data.Game_Robot_State.robot_id < 10)
  {
    //F105.Sendmessage.RobotRed = 1;
    JudgeData_ForSend1.Robot_Red_Blue = 1;
  }
  else
  {
    //F105.Sendmessage.RobotRed = 0;
    JudgeData_ForSend1.Robot_Red_Blue = 0;
  }

  if(JudgeData_ForSend1.Robot_Red_Blue == 1)
  {
    JudgeData_ForSend1.self_outpost = (uint8_t)(0.04f*(referee_data.Game_Robot_HP.red_outpost_HP+24));
    JudgeData_ForSend1.Enemy_outpost = (uint8_t)(0.04f* (referee_data.Game_Robot_HP.blue_outpost_HP+24));
  }
  else
  {
    JudgeData_ForSend1.self_outpost = (uint8_t)(0.04f*(referee_data.Game_Robot_HP.blue_outpost_HP+24));
    JudgeData_ForSend1.Enemy_outpost = (uint8_t)(0.04f*(referee_data.Game_Robot_HP.red_outpost_HP+24));
  }

    JudgeData_ForSend1.Heat_update = 0x01;
		JudgeData_ForSend1.shooter1_heat = referee_data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;
    JudgeData_ForSend1.bullet_remaining_num_17mm = referee_data.Bullet_Remaining.bullet_remaining_num_17mm;
    // 从裁判系统0x020D获取哨兵姿态: 1=进攻, 2=防御, 3=移动, 0=未知
    JudgeData_ForSend1.sentry_posture = referee_data.Sentry_info.sentry_posture;
    If_Game_Start = (referee_data.Game_Status.game_progress ==0x04)?1:0;
    JudgeData_ForSend1.is_game_start = If_Game_Start;
    JudgeData_ForSend1.stage_remain_time = referee_data.Game_Status.stage_remain_time/2 ;

    JudgeData_ForSend2.x = (uint16_t)(referee_data.Game_Robot_Pos.x * 100);
    JudgeData_ForSend2.y = (uint16_t)(referee_data.Game_Robot_Pos.y * 100);
    JudgeData_ForSend2.yaw_10 = (int16_t)(referee_data.Game_Robot_Pos.yaw * 10);
    //JudgeData_ForSend2.Base_Shield = referee_data.Event_Data.self_BaseShield;
    JudgeData_ForSend2.Self_blood = referee_data.Game_Robot_State.remain_HP;
    // if(referee_data.Sentry_alert_info.alert_flag == 0xff)
    // {
    //   JudgeData_ForSend2.commd_keyboard = 's';
    // }
    // else
    //   JudgeData_ForSend2.commd_keyboard = 0;

      JudgeDataBloodPack();
      JudgeDataRFIDandBuffPack();
      JudgeDataPositionPack();
    


   
		
}


void Can2Send1(JudgeData_ForSend1_t *Judge2Send)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = SEND_TO_GIMBAL_JUDGE_DATA_CAN_ID1;
	  
	 memcpy(tx_message.Data,Judge2Send,sizeof(JudgeData_ForSend1_t));
	
	  CAN_Transmit(CAN2,&tx_message);
}

void Can2Send2(JudgeData_ForSend2_t *Judge2Send)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = SEND_TO_GIMBAL_JUDGE_DATA_CAN_ID2;
	  
	 memcpy(tx_message.Data,Judge2Send,sizeof(JudgeData_ForSend2_t));
	
	  CAN_Transmit(CAN2,&tx_message);
}
void Can2Send3_blood1(JudgeBloodData_ForSend1_t *Blood2Send1){

	CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = SEND_TO_GIMBAL_BLOOD_DATA_CAN_ID1;
	memcpy(tx_message.Data,Blood2Send1,sizeof(JudgeBloodData_ForSend1_t));
	CAN_Transmit(CAN2,&tx_message);
}



// 直接使用全局变量
void Can2Send4_RFID(JudgeData_RFID_t *judgeData_RFID){


	CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID;
    memcpy(tx_message.Data,judgeData_RFID,sizeof(JudgeData_RFID_t));
	CAN_Transmit(CAN2,&tx_message);
}
void Can2Send4_Buff(JudgeData_Buff_t *judgeData_buff)
{
  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = SEND_TO_GIMBAL_RFID_AND_BUFF_DATA_CAN_ID;
  memcpy(tx_message.Data,judgeData_buff,sizeof(JudgeData_Buff_t));
	CAN_Transmit(CAN2,&tx_message);
}

uint16_t poscount = 0;
void Can2Send5(){
  

	CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = SEND_TO_GIMBAL_POSITION_DATA_CAN_ID;
    if(poscount % 14 == 0) memcpy(tx_message.Data,&JudgeData_position.Friend[1],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 1) memcpy(tx_message.Data,&JudgeData_position.Friend[2],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 2) memcpy(tx_message.Data,&JudgeData_position.Friend[3],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 3) memcpy(tx_message.Data,&JudgeData_position.Friend[4],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 4) memcpy(tx_message.Data,&JudgeData_position.Friend[5],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 5) memcpy(tx_message.Data,&JudgeData_position.Friend[7],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 6) memcpy(tx_message.Data,&JudgeData_position.Friend[8],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 7) memcpy(tx_message.Data,&JudgeData_position.Enemy[1],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 8) memcpy(tx_message.Data,&JudgeData_position.Enemy[2],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 9) memcpy(tx_message.Data,&JudgeData_position.Enemy[3],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 10) memcpy(tx_message.Data,&JudgeData_position.Enemy[4],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 11) memcpy(tx_message.Data,&JudgeData_position.Enemy[5],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 12) memcpy(tx_message.Data,&JudgeData_position.Enemy[7],sizeof(Each_Robot_position_t));
    if(poscount % 14 == 13) memcpy(tx_message.Data,&JudgeData_position.Enemy[8],sizeof(Each_Robot_position_t));
		poscount++;
	if(poscount > 10000) poscount = 0;

		
		CAN_Transmit(CAN2,&tx_message);
}
int16_t send_count = 0;
void JudgeDataCanSend(void)
{
  // 5ms执行一侧

  SendToGimbalPack();
  JudgeDataPack();
  ChassisSpeedPack();  // 底盘速度数据打包

  if(send_count %10 == 0) Can2Send1(&JudgeData_ForSend1);// 20hz
  if(send_count % 10 ==6) CanSend(GIMBAL_CAN_COMM_CANx, send_to_gimbal_data, SEND_TO_GIMBAL_CAN_ID_1, 8);
  if(send_count % 10 == 3) Can2Send2(&JudgeData_ForSend2);//20hz
  if(send_count %19 == 0) Can2Send3_blood1(&JudgeBloodData_ForSend1);//10hz
  if(send_count %19 == 9) Can2Send3_blood1(&JudgeBloodData_ForSend2);//10hz
  if(send_count % 21 == 4) Can2Send4_Buff(&JudgeData_Buff);//10hz,RFIDandBuff
  if(send_count % 21 == 8) Can2Send4_RFID(&JudgeData_RFID);//
  if(send_count % 21 == 16) Can2Send5();//10hz,POS
  if(send_count % 10 == 2) Can2SendChassisSpeed();  // 50hz发送底盘速度数据
  if(send_count == 3000) //3390是公倍数
  {
    send_count = -1;
  }
  send_count++;
  JudgeData_ForSend1.Heat_update = 0x0;
}


