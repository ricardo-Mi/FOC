#include "FreeRTOS.h"
#include "task.h"

#include "Board.h"
#include "Key.h"
#include "OLED.h"
#include "FOC.h"
#include "ADC.h"
#include "AS5600.h"
#include "VOFA.h"

#include "stm32f4xx.h"

static void App_CreateTasks(void);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    Board_Init();          /* PA2 fault input */
    VOFA_Init();           /* USART3 TX: PD8, 115200 8N1, FireWater */
    OLED_Init();           /* PF9/PF10 software I2C */
    FOC_Init();            /* TIM1 complementary PWM + control loop state */
    (void)ADC_Current_Init();
    AS5600_Init();         /* PB13/PB12 software I2C */
    Key_Init();            /* PC0 start/stop */

    App_CreateTasks();
    vTaskStartScheduler();

    for (;;)
    {
        /* The scheduler should never return. */
    }
}

static void App_CreateTasks(void)
{
    (void)xTaskCreate(vKeyTask, "Key", 128u, NULL, 1u, NULL);
    (void)xTaskCreate(vOLEDTask, "OLED", 256u, NULL, 1u, NULL);
    (void)xTaskCreate(vVOFATask, "VOFA", 256u, NULL, 1u, NULL);
    (void)xTaskCreate(vFOCSpeedTask, "FOC", 384u, NULL, 3u, NULL);
}

/* ==================== FreeRTOS hooks ==================== */

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    for (;;)
    {
        /* Heap exhausted: increase configTOTAL_HEAP_SIZE or shrink stacks. */
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();

    for (;;)
    {
        /* A task stack overflowed: increase its stack depth. */
    }
}
