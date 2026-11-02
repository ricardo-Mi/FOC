#ifndef __ADC_CURRENT_H
#define __ADC_CURRENT_H

#include <stdint.h>

/* Synchronized three-phase current sampling:
 *   PA4 / ADC1_IN4 -> phase A
 *   PA5 / ADC2_IN5 -> phase B
 *   PA3 / ADC3_IN3 -> phase C
 * TIM1 TRGO triggers the triple injected conversion.
 *
 * The board has three 4 mOhm low-side shunts and a DRV8323 CSA gain of
 * 5 V/V. There is no bus-current shunt; the bus_current output is 0.
 *
 * Phase currents are signed.  On this board the DRV8323 output is
 * V_SOx = VREF/2 + I * Rshunt * GCSA, so a raw sample above the
 * zero-current offset corresponds to a positive phase current.
 *
 * A low-pass estimate of the three-phase common-mode component is
 * removed at the ADC layer before the currents are returned.  In a
 * three-wire motor the true phase currents sum to zero, so this
 * suppresses the CSA bias shift caused by PWM duty/common-mode changes.
 */
uint8_t ADC_Current_Init(void);
void ADC_Current_Reset(void);
void ADC_Current_BeginCalibration(void);
void ADC_Current_BeginOffsetSample(void);
void ADC_Current_SetRunning(uint8_t running);
uint8_t ADC_Current_IsValid(void);
uint8_t ADC_Current_IsReady(void);
uint8_t ADC_Current_Get(float *phase_a, float *phase_b,
                        float *phase_c, float *bus_current);
/* ISR 内使用：不判有效、不关中断 */
void ADC_Current_ReadIsr(float *phase_a, float *phase_b, float *phase_c);

#endif /* __ADC_CURRENT_H */
