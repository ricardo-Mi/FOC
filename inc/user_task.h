#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "FreeRTOS.h"
#include "task.h"

void LED_GPIO_Init(void);
void LED_Task(void *pvParameters);
void Speed_Task(void *pvParameters);
void USART_Command_Task(void *pvParameters);



#endif /* __USER_TASK_H */
