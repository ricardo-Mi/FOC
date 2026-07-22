#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "Motor.h"
#include "PWM.h"


void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 |
                               GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOG, GPIO_Pin_10 | GPIO_Pin_11 |
                          GPIO_Pin_14 | GPIO_Pin_15);
}

void Motor_Speed(int16_t speed_left, int16_t speed_right)
{

    if (speed_left > 0)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_11);
        GPIO_ResetBits(GPIOG, GPIO_Pin_10);
    }
    else if (speed_left < 0)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_10);
        GPIO_ResetBits(GPIOG, GPIO_Pin_11);
        speed_left = (int16_t)-speed_left;
    }
    else
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_10 | GPIO_Pin_11);
    }

    if (speed_right > 0)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_14);
        GPIO_ResetBits(GPIOG, GPIO_Pin_15);
    }
    else if (speed_right < 0)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_15);
        GPIO_ResetBits(GPIOG, GPIO_Pin_14);
        speed_right = (int16_t)-speed_right;
    }
    else
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_14 | GPIO_Pin_15);
    }

    PWM_SetCompare3((uint16_t)speed_right);
    PWM_SetCompare4((uint16_t)speed_left);
}

void Motor_Stop(void)
{
    PWM_SetCompare3(0);
    PWM_SetCompare4(0);
}
