#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "FreeRTOS.h"
#include "task.h"

void LED_GPIO_Init(void);
void vLED1Task(void *pvParameters);
void vLED2Task(void *pvParameters);



#endif /* __USER_TASK_H */
