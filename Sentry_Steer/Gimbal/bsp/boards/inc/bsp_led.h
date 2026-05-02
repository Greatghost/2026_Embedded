#ifndef _BSP_LED_H
#define _BSP_LED_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tools.h"

void LED_On(uint16_t GPIO_Pin);
void LED_Off(uint16_t GPIO_Pin);
void LED_Toggle(uint16_t GPIO_Pin);

#endif
