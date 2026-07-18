#include "Key.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "FreeRTOS.h"
#include "task.h"

void Key_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 1)
        {
            return 0;
        }
        while (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        KeyNum = 1;
    }

    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2) == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2) == 1)
        {
            return KeyNum;
        }
        while (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        KeyNum = 2;
    }

    return KeyNum;
}
