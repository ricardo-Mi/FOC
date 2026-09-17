#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include <stddef.h>
#include <stdint.h>
#include "USART.h"
#include "FreeRTOS.h"
#include "task.h"


void USART1_Init(void)
{
    // USART1 初始化代码
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

}

void USART1_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

void USART1_SendString(const char *str)
{
    while (*str)
    {
        USART1_SendByte((uint8_t)(*str));
        str++;
    }
}

uint8_t USART1_ReadByte(void)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); //一直等收到数据
    return USART_ReceiveData(USART1);
}
/*
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);
        USART_SendData(USART1, data);
    }
}
*/
static void USART1_SendNumber(uint32_t num)
{
    char buf[10];
    int i = 0;

    if (num == 0)
    {
        USART1_SendByte('0');
        return;
    }

    while (num > 0)
    {
        buf[i++] = (char)('0' + (num % 10));
        num /= 10;
    }

    while (i > 0)
    {
        USART1_SendByte((uint8_t)buf[--i]);
    }
}

void vSystemInfoTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t current, minimum;

    while(1)
    {
        USART1_SendString("FreeHeap=");
        current = xPortGetFreeHeapSize();
        USART1_SendNumber(current);
        USART1_SendString("\n");

        USART1_SendString("MinFreeHeap=");
        minimum = xPortGetMinimumEverFreeHeapSize();
        USART1_SendNumber(minimum);
        USART1_SendString("\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}