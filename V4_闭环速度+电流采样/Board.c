#include "Board.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/* PA2: DRV8323 fault output, open-drain/active-low on most boards.
 * Internal pull-up keeps it deasserted when nothing is connected. */
void Board_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
}

uint8_t Board_GetFaultInput(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}
