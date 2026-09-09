#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "queue.h"
#include "semphr.h"


void LED_GPIO_Init(void)
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



void vLED1Task(void *pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vLED2Task(void *pvParameters)
{
    (void)pvParameters;
    
    while (1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


