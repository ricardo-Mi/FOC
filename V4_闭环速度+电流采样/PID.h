#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* PID controller with Tustin (trapezoidal) integral and output slew limit.
 * limit applies to both the integral term and the final output. */
typedef struct
{
    float P;
    float I;
    float D;
    float output_ramp;       /* output slew limit [units/s], 0 disables */
    float limit;             /* output/integral limit */
    float error_prev;
    float output_prev;
    float integral_prev;
    uint32_t timestamp_prev; /* last update time [us] */
} PIDController;

void PID_Init(PIDController *pid, float P, float I, float D,
              float ramp, float limit);
void PID_Reset(PIDController *pid);
float PID_Update(PIDController *pid, float error);

#endif /* __PID_H */
