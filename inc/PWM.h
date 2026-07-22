#ifndef __PWM_H
#define __PWM_H
#include <stdint.h>
void PWM_Init(void);
void PWM_SetCompare4(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void Set_Speed(uint16_t Compare_left,uint16_t Compare_right);


#endif
