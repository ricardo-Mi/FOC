#include "Key.h"

#include "FOC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define KEY_SCAN_PERIOD_MS 10u

void Key_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = KEY_START_STOP_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(KEY_START_STOP_PORT, &gpio);
}

/* 10 ms scan period also acts as a simple debounce filter. */
void vKeyTask(void *pvParameters)
{
    uint8_t previous = 1u;
    uint8_t current;

    (void)pvParameters;

    for (;;)
    {
        current = (uint8_t)GPIO_ReadInputDataBit(KEY_START_STOP_PORT,
                                                 KEY_START_STOP_PIN);

        if ((previous != 0u) && (current == 0u))
        {
            FOC_ToggleRunning();
        }

        previous = current;
        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS));
    }
}
