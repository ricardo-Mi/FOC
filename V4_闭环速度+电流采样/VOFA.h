#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/* USART3 telemetry: PD8 TX, 115200 baud, FireWater protocol. */
/* FireWater frame: "value0,value1,...\n" (ASCII, '\n' terminated).
 * Channels: target velocity, measured velocity, phase A/B/C current,
 * phase A/B/C PWM duty in percent. */
void VOFA_Init(void);
void VOFA_SendFloat(float *arr, uint8_t n);
void vVOFATask(void *pvParameters);

#endif /* __VOFA_H */
