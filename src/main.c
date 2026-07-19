#include "FreeRTOS.h"
#include "stm32f4xx_gpio.h"
#include "task.h"
#include "stm32f4xx.h"
#include "LED.h"
#include "Key.h"
#include <stdint.h>
#include "OLED.h"
#include "Timer.h"
#include "Exti.h"
#include "PWM.h"


uint8_t i;


 static void Breathe_Task(void *pvParam)
{
    (void)pvParam;
    uint8_t i;

    while (1)
    {
        for (i = 0; i <= 100; i++)
        {
            PWM_SetCompare1(i);                  // 0% → 100%
            vTaskDelay(pdMS_TO_TICKS(10));             // 10ms 步进
        }
        for (i = 0; i <= 100; i++)
        {
            PWM_SetCompare1(100 - i);            // 100% → 0%
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}


 int main(void)
 {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

	OLED_Init();		
    PWM_Init();

    xTaskCreate(Breathe_Task, "Breathe", 128, NULL, 1, NULL);

    vTaskStartScheduler();                             

    for (;;) { }    

 }




























 /* ==================== FreeRTOS 钩子 ==================== */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}


