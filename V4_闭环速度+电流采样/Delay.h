#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

/* DWT cycle counter based timing (does not use SysTick). */
void DWT_Init(void);
uint32_t micros(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif /* __DELAY_H */
