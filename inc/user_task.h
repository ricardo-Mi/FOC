#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *Key_GPIO_Port;
    uint16_t     Key_GPIO_Pin;
    const char   *Name;
}Car_TypeDef;


typedef struct
{
    float x;
    float y;
}Point_t;

void LED_GPIO_Init(void);
void Buzzer_Init(void);

void prvLED1Task(void *pvParameters);
void prvTask1(void *pvParameters);
void PrintString(const char *Str);
void vTaskFunction(void *pvParameters);
void vRocketTask(void *pvParameters);
void vBuzzerTask(void *pvParameters);
void vCarTask(void *pvParameters);
void vMonkeyTask(void *pvParameters);
void vUSART1GateKeeperTask(void *pvParameters);
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
char *AppendUInt(char *dst, uint32_t num);
char *AppendString(char *dst, const char *src);






#endif /* __USER_TASK_H */
