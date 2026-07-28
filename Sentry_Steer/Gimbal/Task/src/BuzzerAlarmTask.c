/**
  ******************************************************************************
  * @file    BuzzerAlarmTask.c
  * @brief   蜂鸣器告警任务：通过蜂鸣器指示底盘哪个电机掉线
  *          监听 CAN 0x0A1 → motor_offline_recv.motor_offline_bitmap
  *          - bit0-3 : 轮电机1-4 掉线 → 分别响 1~4 声 (音符 C5/D5/E5/F5)
  *          - bit4-7 : 舵电机1-4 掉线 → 分别响 5~8 声 (音符 G5/A5/B5/C6)
  *          - 多个电机同时掉线 → 按编号顺序依次响
  *          - 无掉线或 CAN 失联 → 静音
  *          音符频率与时值沿用开机启动音效文件夹(诺基亚seeyouagain)封装风格:
  *          Set_Buzzer_Frequency(freq) + vTaskDelay(time) + Set_Buzzer_Frequency(0)
  ******************************************************************************
  */

#include "BuzzerAlarmTask.h"
#include "bsp_buzzer.h"
#include "bsp_PWM.h"
#include "tim.h"
#include "ChassisGet.h"

/* 告警参数配置 (采用开机音效文件夹封装的时值风格) */
#define BUZZER_BEEP_ON_MS            188U    /* 单声响时长 = 八分音符 (160BPM, 与 SEE_YOU_AGAIN_MAIN 一致) */
#define BUZZER_BEEP_OFF_MS           10U     /* 单声间隔 (文件夹风格短间隙) */
#define BUZZER_INTER_MOTOR_MS        400U    /* 不同电机告警之间的间隔 */
#define BUZZER_CYCLE_IDLE_MS         500U    /* 无掉线/失联时循环间隔 */
#define BUZZER_CAN_STALE_THRESHOLD   3000U   /* CAN 数据失联阈值 3 秒 */

/* 各电机对应的音符频率 (Hz), 取自开机音效文件夹音符频率对照表
 * 轮电机1-4 : C5 / D5 / E5 / F5  (523 / 587 / 659 / 698)
 * 舵电机1-4 : G5 / A5 / B5 / C6  (784 / 880 / 988 / 1047)
 * 音高随电机编号递增,便于通过音高二次区分掉线电机 */
static const uint32_t motor_note_freq[8] =
{
    523U, 587U, 659U, 698U,   /* 轮电机 1-4 */
    784U, 880U, 988U, 1047U   /* 舵电机 1-4 */
};

/* 任务首次启动时 last_rx_tick 为 0,需要给一个宽限期
 * 在此之前若未收到过 CAN 数据,则保持静音(避免开机误报) */

/**
  * @brief  蜂鸣器告警任务函数
  * @param  argument: 未使用
  * @retval 无
  */
void BuzzerAlarmTask(void const * argument)
{
    /* PWM 在开机音效 Initialization_Completed 后保持运行,无需重启 */
    buzzer_off();
    vTaskDelay(100);

    for (;;)
    {
          /* 测试用: 任务一启动就响 8 声音阶 (C5→C6),不依赖任何条件
     * 用于验证蜂鸣器任务能否正常发声 */
    #ifdef TestBuzzerAlarmTask
    for (uint8_t b = 0; b < 8; b++)
    {
        Set_Buzzer_Frequency(587);
        vTaskDelay(300);
        buzzer_off();
        vTaskDelay(50);
    }
    vTaskDelay(1000);
    continue;
    #endif
      
//        /* 读取电机掉线状态快照 */
//        uint8_t  bitmap = motor_offline_recv.motor_offline_bitmap;
//        uint32_t last_rx = motor_offline_recv.last_rx_tick;

//        /* 检查 CAN 数据新鲜度:从未接收(last_rx==0) 或 超过 3 秒未刷新 → 视为失联 */
//        if (last_rx == 0U || (HAL_GetTick() - last_rx) > BUZZER_CAN_STALE_THRESHOLD)
//        {
//            buzzer_off();
//            vTaskDelay(BUZZER_CYCLE_IDLE_MS);
//            continue;
//        }

//        /* 无电机掉线 → 静音 */
//        if (bitmap == 0U)
//        {
//            buzzer_off();
//            vTaskDelay(BUZZER_CYCLE_IDLE_MS);
//            continue;
//        }

//        /* 有电机掉线 → 按编号顺序依次响铃
//         * bit0(轮1)→1声 C5, bit1(轮2)→2声 D5, ..., bit3(轮4)→4声 F5
//         * bit4(舵1)→5声 G5, bit5(舵2)→6声 A5, ..., bit7(舵4)→8声 C6 */
//        for (uint8_t i = 0; i < 8U; i++)
//        {
//            if (bitmap & (1U << i))
//            {
//                uint8_t beep_count = i + 1U;  /* 响铃次数 = 编号 */
//                for (uint8_t b = 0; b < beep_count; b++)
//                {
//                    Set_Buzzer_Frequency(motor_note_freq[i]);
//                    vTaskDelay(BUZZER_BEEP_ON_MS);
//                    buzzer_off();
//                    vTaskDelay(BUZZER_BEEP_OFF_MS);
//                }
//                /* 不同电机告警之间增加间隔,便于区分 */
//                vTaskDelay(BUZZER_INTER_MOTOR_MS);
//            }
//        }

//        /* 一轮告警结束,稍作停顿再开始下一轮 */
//        vTaskDelay(BUZZER_CYCLE_IDLE_MS);
    }
}
