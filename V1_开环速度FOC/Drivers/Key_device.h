#ifndef __KEY_DEVICE_H
#define __KEY_DEVICE_H

#include "stm32f4xx.h"

typedef struct
{
    uint8_t Previous;
    uint8_t Current;
    GPIO_TypeDef *GPIO_PORT;
    uint16_t GPIO_Pin;

    void(*ClickedCallback)(void);

}KeyHandle_TypeDef;

void Key_Device_Init(KeyHandle_TypeDef *Handle);
void Key_Device_Scan(KeyHandle_TypeDef *Handle);


#endif 
