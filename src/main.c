#include "FreeRTOS.h"
#include "Key.h"
#include "USART.h"
#include "projdefs.h"
#include "stm32f4xx_gpio.h"
#include "task.h"
#include "MPU6050.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "user_task.h"
#include "LED.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "OLED.h"

TaskHandle_t xLED1Taskhandle;
SemaphoreHandle_t xFireSemphr;
SemaphoreHandle_t xBuzzerSemphr;
SemaphoreHandle_t xSemphore1;
QueueHandle_t xQueue1;
QueueHandle_t xQueueForUSART1GateKeeper; //守门员任务
QueueHandle_t xMaixbox;  //邮箱
TimerHandle_t xTimer;    //定时器任务


const char *task1Str = "Task 1 is running.\n";
const char *task2Str = "Task 2 is running.\n";

const Car_TypeDef CarA = {.Key_GPIO_Port = GPIOF, .Key_GPIO_Pin = GPIO_Pin_1, .Name = "汽车A"};
const Car_TypeDef CarB = {.Key_GPIO_Port = GPIOF, .Key_GPIO_Pin = GPIO_Pin_2, .Name = "汽车B"};
const Car_TypeDef CarC = {.Key_GPIO_Port = GPIOF, .Key_GPIO_Pin = GPIO_Pin_3, .Name = "汽车C"};
const Car_TypeDef CarD = {.Key_GPIO_Port = GPIOF, .Key_GPIO_Pin = GPIO_Pin_4, .Name = "汽车D"};


int main(void)
{
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_Init();

    USART1_Init();
    Buzzer_Init();


    OLED_Init();

    MPU6050_Init();
    MPU6050_CalibrateGyro();		//开机校准，模块需保持静止

    //Key_Init();

    //xQueueForUSART1GateKeeper = xQueueCreate(5,sizeof(char*));
    xMaixbox = xQueueCreate(1, sizeof(MPUData_t));
    xTimer = xTimerCreate("vTimer", pdMS_TO_TICKS(200), pdTRUE, NULL, Timer1Callback);
    //xFireSemphr = xSemaphoreCreateBinary();
    //二进制信号量
    //xBuzzerSemphr = xSemaphoreCreateBinary();
    //xBuzzerSemphr = xSemaphoreCreateCounting(10, 0);
    //xSemaphoreTake(xFireSemphr, 0);
    //xSemaphoreTake(xBuzzerSemphr, 0);
    //xSemphore1 = xSemaphoreCreateCounting(3,3);
    //xQueue1 = xQueueCreate(10, sizeof(int));
    //xTaskCreate(prvLED1Task, "LED1", 128, NULL, 2, &xLED1Taskhandle);
    //xTaskCreate(prvTask1, "Task1", 128, NULL, 2, NULL);
    //xTaskCreate(vTaskFunction, "Task1", 128, (void*)task1Str, 1, NULL);
    //xTaskCreate(vTaskFunction, "Task2", 128, (void*)task2Str, 1, NULL);
    xTaskCreate(vKeyTask, "Key", 128, NULL, 1, NULL);
    //xTaskCreate(vTask1, "Task1", 128, NULL, 1, NULL);
    //xTaskCreate(vTask2, "Task2", 128, NULL, 2, NULL);
    xTaskCreate(vMPU6050task, "MPU6050Task", 512, NULL, 2, NULL);
    xTaskCreate(vOLEDTask, "OLED", 128, NULL, 1, NULL);

    //xTaskCreate(vLED1Task, "Task1", 128, NULL, 1, NULL);
    //xTaskCreate(vLED3Task, "Task3", 128, NULL, 1, NULL);
    //xTaskCreate(vSystemInfoTask, "Task4", 128, NULL, 1, NULL);
    //xTaskCreate(vRocketTask, "ROC", 128, NULL, 2, NULL);
    //xTaskCreate(vBuzzerTask, "Buzzer", 128, NULL, 2, NULL);
    //xTaskCreate(vCarTask, "A",64, (void *)&CarA, 1, NULL);
   //xTaskCreate(vCarTask, "B",64, (void *)&CarB, 1, NULL);
    //xTaskCreate(vCarTask, "C",64, (void *)&CarC, 1, NULL);
    //xTaskCreate(vCarTask, "D",64, (void *)&CarD, 1, NULL);
    //xTaskCreate(vMonkeyTask, "Monkey",64, NULL, 2, NULL);
    //xTaskCreate(vUSART1GateKeeperTask, "USART1", 128, NULL, 1, NULL);


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


