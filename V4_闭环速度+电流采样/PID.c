#include "PID.h"

#include "Delay.h"

static float PID_Constrain(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

void PID_Init(PIDController *pid, float P, float I, float D,
              float ramp, float limit)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    PID_Reset(pid);
}

void PID_Reset(PIDController *pid)
{
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = micros();
}

float PID_Update(PIDController *pid, float error)
{
    uint32_t now = micros();
    float Ts = (now - pid->timestamp_prev) * 1e-6f;
    float proportional;
    float integral;
    float derivative;
    float output;

    if ((Ts <= 0.0f) || (Ts > 0.5f))
    {
        Ts = 1e-3f;
    }

    proportional = pid->P * error;

    integral = pid->integral_prev +
               (pid->I * Ts * 0.5f * (error + pid->error_prev));
    integral = PID_Constrain(integral, -pid->limit, pid->limit);

    derivative = pid->D * (error - pid->error_prev) / Ts;

    output = proportional + integral + derivative;
    output = PID_Constrain(output, -pid->limit, pid->limit);

    if (pid->output_ramp > 0.0f)
    {
        float output_rate = (output - pid->output_prev) / Ts;

        if (output_rate > pid->output_ramp)
        {
            output = pid->output_prev + (pid->output_ramp * Ts);
        }
        else if (output_rate < -pid->output_ramp)
        {
            output = pid->output_prev - (pid->output_ramp * Ts);
        }
    }

    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = now;

    return output;
}
