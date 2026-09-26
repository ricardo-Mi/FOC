#include "FreeRTOS.h"
#include "Key.h"
#include "USART.h"
#include "projdefs.h"
#include "stm32f4xx_gpio.h"
#include "task.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "user_task.h"
#include "LED.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "OLED.h"
#include "FOC.h"
#include "AS5600.h"



int main(void)
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_Init();

    //USART1_Init();  //PA9/PA10已改作FOC的PWM与使能脚, 串口调试停用

    FOC_Init();     //TIM1 PWM: PA9/PE13/PE14, 使能: PA10


    OLED_Init();

    AS5600_Init();  //AS5600编码器: SCL=PB13, SDA=PB12

    xTaskCreate(vKeyTask, "Key", 128, NULL, 1, NULL);
    xTaskCreate(vOLEDTask, "OLED", 128, NULL, 1, NULL);
    xTaskCreate(vFOCVelocityOpenloopTask, "FOC", 256, NULL, 3, NULL);


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


