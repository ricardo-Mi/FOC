#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>
#include <USART.h>
#include "Key.h"
#include "string.h"

extern TaskHandle_t xLED1Taskhandle;
extern SemaphoreHandle_t xFireSemphr;
extern SemaphoreHandle_t xBuzzerSemphr;
extern SemaphoreHandle_t xSemphore1;
extern QueueHandle_t xQueue1;
extern QueueHandle_t xQueueForUSART1GateKeeper; //守门员任务


extern const char *task1Str;
extern const char *task2Str;

void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_ResetBits(GPIOA, GPIO_Pin_5);

}

void prvLED1Task(void *pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void prvTask1(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    vTaskSuspend(xLED1Taskhandle);
    vTaskDelay(pdMS_TO_TICKS(1000));

    vTaskResume(xLED1Taskhandle);
    vTaskDelay(pdMS_TO_TICKS(1000));

    vTaskDelete(xLED1Taskhandle);

    vTaskDelete(NULL);

}


void PrintString(const char *Str)
{
    char *pMemopy = pvPortMalloc(strlen(Str) +1);
    strcpy(pMemopy, Str);
    xQueueSend(xQueueForUSART1GateKeeper, &pMemopy, portMAX_DELAY);
}

void vTaskFunction(void *pvParameters)
{
    const char *str = (const char *)pvParameters;

    for(int i=0;i<100;i++)
    {
        PrintString(str);
    }

    vTaskDelete(NULL);
}

void vRocketTask(void *pvParameters)
{
    BaseType_t xReturn;

    USART1_SendString("火箭等待发射\n");
    xReturn =  xSemaphoreTake(xFireSemphr, pdMS_TO_TICKS(5000));
    if(xReturn == pdPASS)
    {
        USART1_SendString("火箭发射成功\n");
    }
    else
    {
        USART1_SendString("发射超时\n");
    }

    vTaskDelete(NULL);
}

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_SetBits(GPIOA, GPIO_Pin_2);

}

void vBuzzerTask(void *pvParameters)
{
    while(1)
    {
        xSemaphoreTake(xBuzzerSemphr, portMAX_DELAY);
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        vTaskDelay(pdMS_TO_TICKS(100));
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vCarTask(void *pvParameters)
{
    Car_TypeDef *Car = (Car_TypeDef *)pvParameters;

    while(1)
    {
        WaitForKeyClick(Car->Key_GPIO_Port, Car->Key_GPIO_Pin);
        if(xSemaphoreTake(xSemphore1, pdMS_TO_TICKS(10000)) == pdPASS)
        {
            USART1_SendString(Car->Name);
            USART1_SendString("入场");
        }
        else {
            USART1_SendString(Car->Name);
            USART1_SendString("入场失败");
            continue;
        }

        WaitForKeyClick(Car->Key_GPIO_Port, Car->Key_GPIO_Pin);
        xSemaphoreGive(xSemphore1);
        USART1_SendString(Car->Name);
        USART1_SendString("出场");
    }
}

void vMonkeyTask(void *pvParameters)
{
    int num;
    while(1)
    {
        xQueueReceive(xQueue1, &num, portMAX_DELAY);

        if(num == 1)        {USART1_SendString("骑自行车");}
        else if(num == 2)   {USART1_SendString("翻跟头");}
        else if(num == 3)   {USART1_SendString("滑滑板");}
        else if(num == 4)   {USART1_SendString("抛球");}

        for(int i = 0; i < 20; i ++)
        {
            USART1_SendString(".");
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void vUSART1GateKeeperTask(void *pvParameters)
{
    char *pMemory;
    while(1)
    {
        xQueueReceive(xQueueForUSART1GateKeeper, &pMemory, portMAX_DELAY);
        USART1_SendString(pMemory);
        vPortFree(pMemory);
    }

}

char *AppendString(char *dst, const char *src)
{
    while (*src)
    {
        *dst++ = *src++;
    }
    return dst;
}

char *AppendUInt(char *dst, uint32_t num)
{
    char tmp[10];
    int i = 0;

    if (num == 0)
    {
        *dst++ = '0';
        return dst;
    }

    while (num > 0)
    {
        tmp[i++] = (char)('0' + (num % 10));
        num /= 10;
    }

    while (i > 0)
    {
        *dst++ = tmp[--i];
    }
    return dst;
}

void vTask1(void *pvParameters)
{
    char buffer[20];
    char *p;

    for (int i = 1; i <= 100; i++)
    {
        p = AppendString(buffer, "Task1 ");
        p = AppendUInt(p, (uint32_t)i);
        p = AppendString(p, "\n");
        *p = '\0';
        PrintString(buffer);
    }

    vTaskDelete(NULL);
}

void vTask2(void *pvParameters)
{
    char buffer[20];
    char *p;

    for (char c = 'a'; c <= 'z'; c++)
    {
        p = AppendString(buffer, "Task2 ");
        *p++ = c;
        p = AppendString(p, "\n");
        *p = '\0';
        PrintString(buffer);
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    vTaskDelete(NULL);

}

