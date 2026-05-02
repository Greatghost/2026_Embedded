#include "bsp_buzzer.h"

void Set_Buzzer_Frequency(uint32_t frequency)
{
    uint32_t period = (8400000 / frequency) - 1;
    // 修改自动重装载值（ARR）
    __HAL_TIM_SET_AUTORELOAD(&htim4, period);
    // 修改通道3比较值（CCR3），保持 50% 占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 2);
}

void Initialization_Completed(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 启动PWM输出
    
    // 原版节奏：180BPM，音符时长严格按照官方乐谱 诺基亚来电铃声
    // 音符格式：频率(Hz), 时长(ms)
    
    // 第一段：E5 → D5 → F#4(四分音符) → G#4(四分音符)
    Set_Buzzer_Frequency(659);  // E5 - 高音咪
    HAL_Delay(167);             // 八分音符 (167ms @180BPM)
    Set_Buzzer_Frequency(0);    // 短暂静音间隔
    HAL_Delay(10);
    
    Set_Buzzer_Frequency(587);  // D5 - 高音来
    HAL_Delay(167);             // 八分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);
    
    Set_Buzzer_Frequency(370);  // F#4 - 升发
    HAL_Delay(333);             // 四分音符 (333ms @180BPM)
    Set_Buzzer_Frequency(0);
    HAL_Delay(15);
    
    Set_Buzzer_Frequency(415);  // G#4 - 升嗦
    HAL_Delay(333);             // 四分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(20);
    
    // 第二段：C#5 → B4 → D4(四分音符) → E4(四分音符)
    Set_Buzzer_Frequency(554);  // C#5 - 升哆
    HAL_Delay(167);             // 八分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);
    
    Set_Buzzer_Frequency(494);  // B4 - 西
    HAL_Delay(167);             // 八分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);
    
    Set_Buzzer_Frequency(294);  // D4 - 来
    HAL_Delay(333);             // 四分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(15);
    
    Set_Buzzer_Frequency(330);  // E4 - 咪
    HAL_Delay(333);             // 四分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(20);
    
    // 第三段：B4 → A4 → C#4(四分音符) → E4(四分音符)
    Set_Buzzer_Frequency(494);  // B4 - 西
    HAL_Delay(167);             // 八分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);
    
    Set_Buzzer_Frequency(440);  // A4 - 啦
    HAL_Delay(167);             // 八分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(10);
    
    Set_Buzzer_Frequency(277);  // C#4 - 升哆
    HAL_Delay(333);             // 四分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(15);
    
    Set_Buzzer_Frequency(330);  // E4 - 咪
    HAL_Delay(333);             // 四分音符
    Set_Buzzer_Frequency(0);
    HAL_Delay(20);
    
    // 结尾：A4(二分音符) - 经典收尾
    Set_Buzzer_Frequency(440);  // A4 - 啦
    HAL_Delay(667);             // 二分音符 (667ms @180BPM)
    Set_Buzzer_Frequency(0);
    HAL_Delay(100);


	
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // 停止 PWM
//		__HAL_RCC_TIM4_CLK_DISABLE(); // 复位 TIM4，防止漏电

    LED_Off(GPIO_PIN_12);
    LED_On(GPIO_PIN_11);
}

void PC_On(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 启动 TIM4_CH3 PWM 输出
	
	
	Set_Buzzer_Frequency(260); // 设置 7kHz
	HAL_Delay(100); // 延时 100ms
	Set_Buzzer_Frequency(440); 
	HAL_Delay(100);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // 停止 PWM
}
