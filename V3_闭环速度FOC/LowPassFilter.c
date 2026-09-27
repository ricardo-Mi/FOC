// 一阶低通滤波器 (由 DengFOC/SimpleFOC 的 LowPassFilter 类移植)
// 公式: y = alpha*y_prev + (1-alpha)*x, alpha = Tf/(Tf+dt)
// dt由两次调用的时间戳差值计算, 与调用频率无关
#include "LowPassFilter.h"
#include "Delay.h"

void LPF_Init(LowPassFilter *lpf, float Tf)
{
    lpf->Tf = Tf;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = micros();
}

float LPF_Update(LowPassFilter *lpf, float x)
{
    uint32_t timestamp = micros();
    float dt = (timestamp - lpf->timestamp_prev) * 1e-6f;

    if (dt <= 0.0f)
    {
        dt = 1e-3f;              /* 首次调用/异常, 用默认1ms */
    }
    else if (dt > 0.3f)
    {
        lpf->y_prev = x;         /* 长时间未调用, 直接重置为当前值 */
        lpf->timestamp_prev = timestamp;
        return x;
    }

    float alpha = lpf->Tf / (lpf->Tf + dt);
    float y = alpha * lpf->y_prev + (1.0f - alpha) * x;

    lpf->y_prev = y;
    lpf->timestamp_prev = timestamp;

    return y;
}
