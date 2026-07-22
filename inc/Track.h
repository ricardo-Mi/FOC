#ifndef __TRACK_H
#define __TRACK_H

#define R2 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
#define R1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
#define M  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
#define L1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)
#define L2 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)

#include<stdint.h>

void Track_Init();
void Track_task();
void Speed_Set(int32_t left_speed, int32_t right_speed);

#endif
