#include "bsp_buzzer.h"

// ========== 开机音乐选择宏定义 ==========
// 选择以下宏定义之一作为开机音乐：
// BUZZER_MUSIC_NOKIA               - 诺基亚来电铃声
// BUZZER_MUSIC_SEE_YOU_AGAIN_INTRO - See You Again 钢琴开头
// BUZZER_MUSIC_SEE_YOU_AGAIN_MAIN  - See You Again 主旋律(oh~How) 2倍速
// ========================================

#define BUZZER_MUSIC_SEE_YOU_AGAIN_MAIN 1  // 当前选择: See You Again 主旋律(oh~How) 2倍速

void Set_Buzzer_Frequency(uint32_t frequency)
{
    // frequency==0 时原本会触发整数除零 → HardFault,这里做保护:置 CCR3=0 实现静音
    if (frequency == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        return;
    }
    uint32_t period = (8400000 / frequency) - 1;
    // 修改自动重装载值（ARR）
    __HAL_TIM_SET_AUTORELOAD(&htim4, period);
    // 修改通道3比较值（CCR3），保持 50% 占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 2);
}

void Initialization_Completed(void)
{
    //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 启动PWM输出

#if defined(BUZZER_MUSIC_NOKIA)
    // ========== 诺基亚来电铃声 (180BPM) ==========

    // 第一段：E5 → D5 → F#4 → G#4
    Set_Buzzer_Frequency(659);  // E5
    HAL_Delay(167);
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);

    Set_Buzzer_Frequency(587);  // D5
    HAL_Delay(167);
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);

    Set_Buzzer_Frequency(370);  // F#4
    HAL_Delay(333);
    Set_Buzzer_Frequency(0);
    HAL_Delay(15);

    Set_Buzzer_Frequency(415);  // G#4
    HAL_Delay(333);
    Set_Buzzer_Frequency(0);
    HAL_Delay(20);

    // 第二段：C#5 → B4 → D4 → E4
    Set_Buzzer_Frequency(554);  // C#5
    HAL_Delay(167);
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);

    Set_Buzzer_Frequency(494);  // B4
    HAL_Delay(167);
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);

    Set_Buzzer_Frequency(294);  // D4
    HAL_Delay(333);
    Set_Buzzer_Frequency(0);
    HAL_Delay(15);

    Set_Buzzer_Frequency(330);  // E4
    HAL_Delay(333);
    Set_Buzzer_Frequency(0);
    HAL_Delay(20);

    // 第三段：B4 → A4 → C#4 → E4
    Set_Buzzer_Frequency(494);  // B4
    HAL_Delay(167);
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);

    Set_Buzzer_Frequency(440);  // A4
    HAL_Delay(167);
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);

    Set_Buzzer_Frequency(277);  // C#4
    HAL_Delay(333);
    Set_Buzzer_Frequency(0);
    HAL_Delay(15);

    Set_Buzzer_Frequency(330);  // E4
    HAL_Delay(333);
    Set_Buzzer_Frequency(0);
    HAL_Delay(20);

    // 结尾：A4
    Set_Buzzer_Frequency(440);  // A4
    HAL_Delay(667);
    Set_Buzzer_Frequency(0);
    HAL_Delay(100);
    // ==========================================================

#elif defined(BUZZER_MUSIC_SEE_YOU_AGAIN_INTRO)
    // ========== See You Again 钢琴开头 (120BPM) ==========

    for (int repeat = 0; repeat < 2; repeat++)
    {
        // 1̇2̇3̇2̇1̇2̇ (六个十六分音符)
        Set_Buzzer_Frequency(1319); HAL_Delay(125); Set_Buzzer_Frequency(0); HAL_Delay(5);
        Set_Buzzer_Frequency(1480); HAL_Delay(125); Set_Buzzer_Frequency(0); HAL_Delay(5);
        Set_Buzzer_Frequency(1661); HAL_Delay(125); Set_Buzzer_Frequency(0); HAL_Delay(5);
        Set_Buzzer_Frequency(1480); HAL_Delay(125); Set_Buzzer_Frequency(0); HAL_Delay(5);
        Set_Buzzer_Frequency(1319); HAL_Delay(125); Set_Buzzer_Frequency(0); HAL_Delay(5);
        Set_Buzzer_Frequency(1480); HAL_Delay(125); Set_Buzzer_Frequency(0); HAL_Delay(5);

        // 5 (八分音符)
        Set_Buzzer_Frequency(987); HAL_Delay(250); Set_Buzzer_Frequency(0); HAL_Delay(10);

        // 2̇1̇
        Set_Buzzer_Frequency(1480); HAL_Delay(250); Set_Buzzer_Frequency(0); HAL_Delay(10);
        Set_Buzzer_Frequency(1319); HAL_Delay(250); Set_Buzzer_Frequency(0); HAL_Delay(10);

        // 5
        Set_Buzzer_Frequency(987); HAL_Delay(250); Set_Buzzer_Frequency(0); HAL_Delay(50);

        HAL_Delay(500);
    }
    // ==========================================================

#elif defined(BUZZER_MUSIC_SEE_YOU_AGAIN_MAIN)
    // ========== See You Again 主旋律 (160BPM, 2倍速) ==========

    // | i̇ 7̇ |
    Set_Buzzer_Frequency(1319); HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(5);
    Set_Buzzer_Frequency(1245); HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(5);

    // | 6· 5 0 |
    Set_Buzzer_Frequency(1109); HAL_Delay(562); Set_Buzzer_Frequency(0); HAL_Delay(5);
    Set_Buzzer_Frequency(987);  HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(188);

    // | i̇ 7̇ |
    Set_Buzzer_Frequency(1319); HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(5);
    Set_Buzzer_Frequency(1245); HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(5);

    // | 6· 7̇6 5 3 |
    Set_Buzzer_Frequency(1109); HAL_Delay(281); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(1245); HAL_Delay(94);  Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(1109); HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(987);  HAL_Delay(188); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(831);  HAL_Delay(375); Set_Buzzer_Frequency(0); HAL_Delay(3);

    // | 5612 |
    Set_Buzzer_Frequency(494); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(554); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(659); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(740); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);

    // | 3· 2 3· 2 3· 2 3532 |
    for (int i = 0; i < 3; i++) {
        Set_Buzzer_Frequency(831); HAL_Delay(281); Set_Buzzer_Frequency(0); HAL_Delay(3);
        Set_Buzzer_Frequency(740); HAL_Delay(94);  Set_Buzzer_Frequency(0); HAL_Delay(3);
    }
    Set_Buzzer_Frequency(831); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(987); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(831); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(740); HAL_Delay(94); Set_Buzzer_Frequency(0); HAL_Delay(10);

    // | 1· 6 1· 21 - |
    Set_Buzzer_Frequency(659); HAL_Delay(281); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(554); HAL_Delay(94);  Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(659); HAL_Delay(281); Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(740); HAL_Delay(94);  Set_Buzzer_Frequency(0); HAL_Delay(3);
    Set_Buzzer_Frequency(659); HAL_Delay(375); Set_Buzzer_Frequency(0); HAL_Delay(10);
    // ==========================================================

#else
    // 默认：无音乐，短促提示音
    Set_Buzzer_Frequency(440);
    HAL_Delay(100);
    Set_Buzzer_Frequency(0);
    HAL_Delay(50);
#endif

    /* 不再 HAL_TIM_PWM_Stop,保持 PWM 运行,仅置 CCR3=0 静音,
     * 后续 BuzzerAlarmTask 可直接 Set_Buzzer_Frequency 发声 */

    LED_Off(GPIO_PIN_12);
    LED_On(GPIO_PIN_11);
}

void PC_On(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 启动 TIM4_CH3 PWM 输出

	Set_Buzzer_Frequency(260);
	HAL_Delay(100);
	Set_Buzzer_Frequency(440);
	HAL_Delay(100);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // 停止 PWM
}