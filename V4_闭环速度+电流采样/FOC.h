#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>

/*
 * Closed-loop speed FOC (voltage mode, encoder feedback).
 *
 * Power stage:
 *   PE9/PE8   -> TIM1_CH1/CH1N  phase A
 *   PE11/PE10 -> TIM1_CH2/CH2N  phase B
 *   PA10/PE12 -> TIM1_CH3/CH3N  phase C
 *   PE7       -> DRV8323 enable (active high)
 *
 * Encoder: AS5600 on PB13/PB12 (software I2C).
 * Current sensing: ADC1/ADC2/ADC3 injected group, triggered by TIM1 TRGO.
 *
 * First start performs a 3 s electrical-zero alignment; the motor must be
 * free to turn during this time. Later starts reuse the calibrated offset.
 */
typedef enum
{
    FOC_STATE_STOP = 0,
    FOC_STATE_WAKE,
    FOC_STATE_CALIBRATE,
    FOC_STATE_OFFSET,
    FOC_STATE_ALIGN,
    FOC_STATE_RUN,
    FOC_STATE_ERROR
} FOC_State;

void FOC_Init(void);
void FOC_SetEnable(uint8_t enable);

/* PC0 key interface. In ERROR state the first press clears the fault. */
void FOC_ToggleRunning(void);
uint8_t FOC_IsRunning(void);
uint8_t FOC_IsCalibrated(void);

/* Speed reference/feedback in mechanical rad/s. */
void FOC_SetTargetVelocity(float rad_s);
float FOC_GetTargetVelocity(void);
float FOC_GetVelocity(void);

/* Last PWM duty ratio, 0.0 .. 1.0 (for telemetry/debugging). */
float FOC_GetDutyA(void);
float FOC_GetDutyB(void);
float FOC_GetDutyC(void);

FOC_State FOC_GetState(void);
void vFOCSpeedTask(void *pvParameters);

#endif /* __FOC_H */
