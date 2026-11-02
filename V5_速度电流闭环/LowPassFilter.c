#include "LowPassFilter.h"

#include "Delay.h"

void LPF_Init(LowPassFilter *lpf, float Tf)
{
    lpf->Tf = Tf;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = micros();
}

void LPF_Reset(LowPassFilter *lpf)
{
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = micros();
}

float LPF_Update(LowPassFilter *lpf, float x)
{
    uint32_t timestamp = micros();
    float dt = (timestamp - lpf->timestamp_prev) * 1e-6f;
    float alpha;
    float y;

    if (dt <= 0.0f)
    {
        dt = 1e-3f;
    }
    else if (dt > 0.3f)
    {
        /* Long gap (task stopped/debugger): restart from the new sample. */
        lpf->y_prev = x;
        lpf->timestamp_prev = timestamp;
        return x;
    }

    alpha = lpf->Tf / (lpf->Tf + dt);
    y = (alpha * lpf->y_prev) + ((1.0f - alpha) * x);

    lpf->y_prev = y;
    lpf->timestamp_prev = timestamp;

    return y;
}
