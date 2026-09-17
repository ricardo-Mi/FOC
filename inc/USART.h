#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"

#include <stdint.h>
void USART1_Init(void);
void USART1_SendByte(uint8_t byte);
void USART1_SendString(const char *str);
uint8_t USART1_ReadByte(void);
void vSystemInfoTask(void *pvParameters);


#endif // __USART_H