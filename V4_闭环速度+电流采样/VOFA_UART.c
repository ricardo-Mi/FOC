#include "VOFA_UART.h"

#include <stddef.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#define VOFA_UART_BAUDRATE 921600u

/* STM32F407 AF7: PD8 = USART3_TX. PD9 is not used (TX-only). */
void VOFA_UART_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = VOFA_UART_BAUDRATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx;
    USART_Init(USART3, &usart);

    USART_Cmd(USART3, ENABLE);
}

/* Blocking TX. Telemetry is only 12 bytes at 100 Hz, so this is cheap and
 * avoids DMA pacing issues on a shared/loaded PD8 pin. */
void VOFA_UART_Send(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0u; i < length; ++i)
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART3, data[i]);
    }

    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
    }
}
