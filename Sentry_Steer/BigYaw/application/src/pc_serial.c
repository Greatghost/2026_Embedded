/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "Gimbal.h"
#include "arm_atan2_f32.h"
#include "debug.h"

#include "SignalGenerator.h"

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_IFANTRY

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
PCSendData pc_send_data;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(unsigned char *PCbuffer)
{
    #if ROBOT == GOBLIN
    if (PCbuffer[0] == '!' && PCbuffer[1] == 0 && PCbuffer[8] == 0) //TODO: 判断方式可能错误
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }	
    #else
    if (PCbuffer[0] == '!'  && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }	
    #endif
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
void SendtoPCPack(unsigned char *buff)
{
    // sin函数用来测试发送延时
    // static SinFunction sin_function;
    // static int8_t is_init = 0;
    // if (!is_init)
    // {
    //     SinInit(&sin_function, 80, 10, 1000);
    //     is_init = 1;
    // }
	
		volatile unsigned char aim_press_r = remote_controller.dji_remote.mouse.press_r;
        #if ROBOT == QI_TIAN_DA_SHENG
        pc_send_data.start_flag = '!';
        pc_send_data.type_id = 0;
        pc_send_data.yaw = gimbal_controller.gyro_yaw_angle;
        pc_send_data.pitch = (short)gimbal_controller.gyro_pitch_angle * 100;
        pc_send_data.crc8 = 0;
        #else
		if(remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
			pc_send_data.mode_want = 1;
		else if(remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE)
			pc_send_data.mode_want = 2;
		else
			pc_send_data.mode_want = 0;
		
		pc_send_data.start_flag = '!';
		pc_send_data.pitch_now = gimbal_controller.gyro_pitch_angle;
		pc_send_data.yaw_now = gimbal_controller.gyro_yaw_angle;
		pc_send_data.roll_now = 0.0f;
		pc_send_data.actual_bullet_speed = 0.0f;
		pc_send_data.aim_request = aim_press_r;
		pc_send_data.number_want = 0;
		pc_send_data.enemy_color = !chassis_pack_get_1.robot_color;
		Append_CRC16_Check_Sum((uint8_t *)(&pc_send_data), PC_SENDBUF_SIZE);
        #endif
		
		memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    CDC_Transmit_FS(SendToPC_Buff, PC_SENDBUF_SIZE); // 通过USB_CDC发送
}

#endif

#if COMMUNICATION_CHOOSE == COMMUNICATION_OF_SENTRY
unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData_1 pc_recv_data_1;
PCRecvData pc_recv_data;//两版接受协议转换
PCSendData pc_send_data;
PCSendDataJudge PC_send_data_judge;

float pc_pitch,pc_yaw;
uint8_t PC_Shoot_flag;
uint8_t last_shoot_flag;
Nav_Cmd_t NAV_cmd;


//extern Shoot_Cmd_t Shoot_Cmd;
uint8_t ShootState = 0;
//JudgeData_1_t JudgeRecieveData;
//JudgeData_2_t JudgeRecieveData2;
//extern pc_offline_check_t pc_offline_check;
char PC_Receive_Flag_2_Armor = 0;

int shootflg_test = 0;

void PCReceive(unsigned char *PCbuffer)
{
    // 掉线检测改用步兵版本
	// switch(pc_offline_check.pc_offline_check_type)
	// {
	// 	case OFFLINE_START:
	// 		pc_offline_check.pc_offline_check_type = OFFLINE_CHECKING;
	// 		break;
	// 	case OFFLINE_CHECKING:
	// 		pc_offline_check.pc_offline_check_num = 0;
	// 		break;
	// 	case OFFLINE_PENDING:
	// 		pc_offline_check.pc_offline_check_type = OFFLINE_CHECKING;
	// 		pc_offline_check.pc_offline_check_num = 0;
	// 		break;
	// }
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
    if(PCbuffer[0] == '!' )
	{
		memcpy(&pc_recv_data_1,PCbuffer,PC_RECVBUF_SIZE);
		pc_recv_data.yaw = pc_recv_data_1.Aim_Yaw;		
		pc_recv_data.pitch = (pc_recv_data_1.Aim_Pitch/100.0f);
		NAV_cmd.Nav_Speed_x =  pc_recv_data_1.Aim_v_x*20.0f;
		NAV_cmd.Nav_Speed_y =  - pc_recv_data_1.Aim_v_y*20.0f;
		ShootState = pc_recv_data_1.FireState;
		shootflg_test = ShootState;
		//Shoot_Cmd.Friction_cmd = pc_recv_data_1.FrictionState; //暂时去掉
		//Shoot_Cmd.Shoot_Freq_cmd = pc_recv_data_1.ShootFreqMod;
		PC_Receive_Flag_2_Armor = 1;
        pc_recv_data.enemy_id = 1;//暂且写死

	}
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
//extern F105_Typedef F105;
//extern Gimbal_Typedef Gimbal;
extern int ShootCount_Number;//射击的总子弹数

void SendtoPCPack(unsigned char *buff)
{
    pc_send_data.start_flag = '!';
	pc_send_data.data_pack_type = USUAL_PC_DATA;
    pc_send_data.Shoot_State = ShootState;//反馈给上位机，已经收到设计状态
	pc_send_data.remain_bullet = ShootCount_Number;
	pc_send_data.vx = 0;
	pc_send_data.vy = 0;
    pc_send_data.pitch = (short)gimbal_controller.gyro_pitch_angle * 100;
    pc_send_data.yaw = gimbal_controller.gyro_yaw_angle;
	pc_send_data.crc8 = 0;
    //Append_CRC8_Check_Sum((unsigned char *)&pc_send_data, PC_SENDBUF_SIZE);
    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

//char buff_test[PC_SENDBUF_SIZE];

void Send2PCJudge(unsigned char* buff)
{
    // 发送给PC的裁判系统数据需要从下位机修改
	// PC_send_data_judge.start_flag = '!';
	// PC_send_data_judge.self_blood = JudgeRecieveData2.Self_blood;
	// PC_send_data_judge.data_pack_type = JUDGE_PC_DATA;
	// PC_send_data_judge.bullet_remaining_num_17mm = JudgeRecieveData.bullet_remaining_num_17mm;
	// PC_send_data_judge.Enemy_outpost = JudgeRecieveData.Enemy_outpost;
	// PC_send_data_judge.is_game_start = JudgeRecieveData.is_game_start;
	// PC_send_data_judge.Robot_Red_Blue = JudgeRecieveData.Robot_Red_Blue;
	// PC_send_data_judge.self_outpost = JudgeRecieveData.self_outpost;
	// PC_send_data_judge.Sentry_HomeReturned_flag = JudgeRecieveData.Sentry_HomeReturned_flag;
	// PC_send_data_judge.stage_remain_time = JudgeRecieveData.stage_remain_time;
	// PC_send_data_judge.UWB_x = JudgeRecieveData2.x;
	// PC_send_data_judge.UWB_y = JudgeRecieveData2.y;
	// PC_send_data_judge.Heat_update = (JudgeRecieveData2.commd_keyboard == 's' ? 0x1 : 0x0);
	// PC_send_data_judge.crc8 = 0;	
	// memcpy(buff, (void *)&PC_send_data_judge, PC_SENDBUF_SIZE);
	// //memcpy(buff_test, (void *)&PC_send_data_judge, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(uint8_t data_type)
{
	if(data_type == USUAL_PC_DATA)
	{
		SendtoPCPack(SendToPC_Buff);
	}
	else if(data_type == JUDGE_PC_DATA)
	{
		Send2PCJudge(SendToPC_Buff);
	}
	CDC_Transmit_FS(SendToPC_Buff,PC_SENDBUF_SIZE);
}
#endif
