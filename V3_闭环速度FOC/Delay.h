#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

void DWT_Init(void);            /* 使能DWT周期计数器(可重复调用, 只初始化一次) */
uint32_t micros(void);          /* 开机以来的微秒数 */
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
