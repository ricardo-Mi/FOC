#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <USART.h>
#include "Key_device.h"

uint8_t previous;
uint8_t current;


static void Key_Device_EnableClock(GPIO_TypeDef *GPIO_PORT)
{
    if      (GPIO_PORT == GPIOA) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    else if (GPIO_PORT == GPIOB) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    else if (GPIO_PORT == GPIOC) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    else if (GPIO_PORT == GPIOD) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    else if (GPIO_PORT == GPIOE) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    else if (GPIO_PORT == GPIOF) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    else if (GPIO_PORT == GPIOG) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    else if (GPIO_PORT == GPIOH) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    else if (GPIO_PORT == GPIOI) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
}

void Key_Device_Init(KeyHandle_TypeDef *Handle)
{
    Key_Device_EnableClock(Handle->GPIO_PORT);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin   = Handle->GPIO_Pin;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Handle->GPIO_PORT, &GPIO_InitStruct);

    Handle->Previous = 1;
    Handle->Current  = 1;
}

void Key_Device_Scan(KeyHandle_TypeDef *Handle)
{
    Handle->Current = GPIO_ReadInputDataBit(Handle->GPIO_PORT, Handle->GPIO_Pin);

    if (Handle->Previous == 1 && Handle->Current == 0)
    {
        Handle->ClickedCallback();
    }

    Handle->Previous = Handle->Current;
}