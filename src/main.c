#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "user_task.h"



int main(void)
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_GPIO_Init();

    xTaskCreate(vLED1Task, "LED1", 128, NULL, 2, NULL);
    xTaskCreate(vLED2Task, "LED2", 128, NULL, 2, NULL);

    vTaskStartScheduler();


    while (1)
    {
    }
}







 /* ==================== FreeRTOS 钩子 ==================== */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    while (1)
    {
        /*
         * 内存申请失败会进入这里。
         * 一般是 configTOTAL_HEAP_SIZE 太小，或者任务栈分配太大。
         */
    }


}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();

    while (1)
    {
        /*
         * 任务栈溢出会进入这里。
         * 一般是某个任务栈给小了，比如 xTaskCreate 里的 128 不够。
         */
    }

}


