#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"
#include <stdint.h>


void Key_Init(void);
uint8_t Key_GetNum(void);
void vKeyTask(void *pvParameters);
void WaitForKeyClick(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);



#endif /* __KEY_H */
