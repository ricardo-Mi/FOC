#include "Key.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Delay.h"



void Key_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStruct);
}

uint8_t Key_GetNum1(void)
{
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 0);
        Delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

uint8_t Key_GetNum2(void)
{
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2) == 0);
        Delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

