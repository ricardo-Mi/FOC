#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

void LED_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_ResetBits(GPIOA, GPIO_Pin_5);

}

void LED1_ON(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
}

void LED1_OFF(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}

void LED1_Turn()
{
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2) == 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
    }
    else 
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    }
}

void LED2_ON(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void LED2_OFF(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

void LED2_Turn()
{
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4) == 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
    }
    else 
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
}

void LED3_ON(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void LED3_OFF(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

void LED3_Turn()
{
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5) == 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
    }
    else 
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    }
}

void vLED1Task(void *pvParameters)
{
    (void)pvParameters;
    
    while (1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

 
void vLED3Task(void *pvParameters)
{
    (void)pvParameters;
    
    while (1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void Timer1Callback(TimerHandle_t xTimer)
{
    LED2_Turn();
}