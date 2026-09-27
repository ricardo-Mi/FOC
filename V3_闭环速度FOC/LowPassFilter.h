#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

#include <stdint.h>

/*
 * 一阶低通滤波器 (由 DengFOC/SimpleFOC 的 LowPassFilter 类移植)
 *
 * 用于平滑编码器算出的速度, Tf越大越平滑但延迟越大
 *   公式: y = alpha*y_prev + (1-alpha)*x,  alpha = Tf/(Tf+dt)
 */
typedef struct
{
    float Tf;                /* 滤波时间常数(秒) */
    uint32_t timestamp_prev; /* 上次执行时间戳(us) */
    float y_prev;            /* 上次滤波输出 */
} LowPassFilter;

void LPF_Init(LowPassFilter *lpf, float Tf);
float LPF_Update(LowPassFilter *lpf, float x);

#endif /* __LOWPASS_FILTER_H */
