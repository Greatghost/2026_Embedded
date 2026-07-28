#ifndef _BSP_BUZZER_H
#define _BSP_BUZZER_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tim.h"
#include "tools.h"
#include "bsp_led.h"

void Buzzer_RCC_Configuration(void);
void Buzzer_GPIO_Configuration(void);
void TIM4_Configuration(void);
void Buzzer_Configuration(void);
void Set_Buzzer_Frequency(uint32_t frequency);
void Initialization_Completed(void);
void PC_On(void);

#endif
