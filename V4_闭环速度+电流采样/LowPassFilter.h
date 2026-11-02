#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H

#include <stdint.h>

/* First-order low-pass filter:
 *   y = alpha * y_prev + (1 - alpha) * x
 *   alpha = Tf / (Tf + dt)
 * dt is measured from the DWT microsecond counter, so the cutoff is
 * independent of the call rate.
 */
typedef struct
{
    float Tf;                /* filter time constant [s] */
    uint32_t timestamp_prev; /* last update time [us] */
    float y_prev;            /* last filtered value */
} LowPassFilter;

void LPF_Init(LowPassFilter *lpf, float Tf);
void LPF_Reset(LowPassFilter *lpf);
float LPF_Update(LowPassFilter *lpf, float x);

#endif /* __LOW_PASS_FILTER_H */
