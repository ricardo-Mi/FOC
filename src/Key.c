#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <USART.h>
#include <sys/_intsup.h>
#include "Key_device.h"
#include "Key.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

static KeyHandle_TypeDef hKEY1;
static KeyHandle_TypeDef hKEY2;
static KeyHandle_TypeDef hKEY3;
static KeyHandle_TypeDef hKEY4;

extern SemaphoreHandle_t xFireSemphr;
extern SemaphoreHandle_t xBuzzerSemphr;
extern QueueHandle_t xQueue1;
extern TimerHandle_t xTimer; 


static void KEY1_ClickedCallback(void);
static void KEY2_ClickedCallback(void);
static void KEY3_ClickedCallback(void);
static void KEY4_ClickedCallback(void);

void Key_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOF, &GPIO_InitStruct);
}


void vKeyTask(void *pvParameters)
{
    (void)pvParameters;
    hKEY1.GPIO_PORT = GPIOF;
    hKEY1.GPIO_Pin  = GPIO_Pin_1;
    hKEY1.ClickedCallback = KEY1_ClickedCallback;
    Key_Device_Init(&hKEY1);

    hKEY2.GPIO_PORT = GPIOF;
    hKEY2.GPIO_Pin  = GPIO_Pin_2;
    hKEY2.ClickedCallback = KEY2_ClickedCallback;
    Key_Device_Init(&hKEY2);

    hKEY3.GPIO_PORT = GPIOF;
    hKEY3.GPIO_Pin  = GPIO_Pin_3;
    hKEY3.ClickedCallback = KEY3_ClickedCallback;
    Key_Device_Init(&hKEY3);

    hKEY4.GPIO_PORT = GPIOF;
    hKEY4.GPIO_Pin  = GPIO_Pin_4;
    hKEY4.ClickedCallback = KEY4_ClickedCallback;
    Key_Device_Init(&hKEY4);

    while(1)
    {
        Key_Device_Scan(&hKEY1);
        Key_Device_Scan(&hKEY2);
        Key_Device_Scan(&hKEY3);
        Key_Device_Scan(&hKEY4);

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void KEY1_ClickedCallback(void)
{   
    //xSemaphoreGive(xFireSemphr);
    //int num = 1;
    //xQueueSend(xQueue1, &num, portMAX_DELAY);
    if(xTimerIsTimerActive(xTimer) == pdFALSE)
    {
        xTimerStart(xTimer, portMAX_DELAY);
    }
    else {
        xTimerStop(xTimer, portMAX_DELAY);
    }
}

void KEY2_ClickedCallback(void)
{   
    //xSemaphoreGive(xBuzzerSemphr);
    int num = 2;
    xQueueSend(xQueue1, &num, portMAX_DELAY);

}

void KEY3_ClickedCallback(void)
{   
    //USART1_SendString("KEY3");
    int num = 3;
    xQueueSend(xQueue1, &num, portMAX_DELAY);

}

void KEY4_ClickedCallback(void)
{   
    //USART1_SendString("KEY4");
    int num = 4;
    xQueueSend(xQueue1, &num, portMAX_DELAY);

}

void WaitForKeyClick(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
    uint8_t currentState, lastState = 1;

    while(1)
    {   
        currentState = GPIO_ReadInputDataBit(GPIO_Port,GPIO_Pin);

        if(lastState != currentState && currentState == 1)
        {
            return;
        }

        lastState = currentState;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

}
