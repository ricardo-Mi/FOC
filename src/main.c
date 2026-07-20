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
#include "task.h"
#include "user_task.h"
#include "USART.h"
#include "queue.h"
#include "semphr.h"

QueueHandle_t g_usart1_rx_queue = NULL;

SemaphoreHandle_t g_key_semaphore = NULL;


int main(void)
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_GPIO_Init();

    g_usart1_rx_queue = xQueueCreate(64, sizeof(uint8_t));

    if(g_usart1_rx_queue == NULL)
    {
        // 创建队列失败，处理错误
        while(1);
    }

    g_key_semaphore = xSemaphoreCreateBinary();

    if(g_key_semaphore == NULL)
    {
        // 创建信号量失败，处理错误
        while(1);
    }

    USART1_Init();

    xTaskCreate(LED_Task, "LED_Task", 128, NULL, 2, NULL);
    xTaskCreate(USART_Command_Task, "USART_Command_Task", 256, NULL, 2, NULL);

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


