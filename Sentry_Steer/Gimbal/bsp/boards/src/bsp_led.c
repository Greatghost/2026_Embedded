#include "bsp_led.h"

void LED_On(uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_Pin,GPIO_PIN_SET);
}

void LED_Off(uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_Pin, GPIO_PIN_RESET);
}

void LED_Toggle(uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOH, GPIO_Pin);
}
