#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init_left(void);
void Encoder_Init_right(void);
int16_t Encoder_Get_left(void);
int16_t Encoder_Get_right(void);

#endif
