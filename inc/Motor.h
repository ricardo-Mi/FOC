#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>


void Motor_Init(void);
void Motor_Speed(int16_t Speed_left,int16_t Speed_right);
void Motor_Stop(void);

#endif
