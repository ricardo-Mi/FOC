#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/*
 * PID控制器 (由 DengFOC/SimpleFOC 的 PIDController 类移植)
 *
 * I环使用Tustin(梯形)离散积分: integral += I*Ts*0.5*(error+error_prev)
 * output_ramp限制输出变化率(>0时生效), limit同时对输出和积分限幅(防饱和)
 */
typedef struct
{
    float P;                 /* 比例增益 */
    float I;                 /* 积分增益 */
    float D;                 /* 微分增益 */
    float output_ramp;       /* 输出变化率限幅 */
    float limit;             /* 输出/积分限幅 */
    float error_prev;        /* 上次误差 */
    float output_prev;       /* 上次输出 */
    float integral_prev;     /* 上次积分值 */
    uint32_t timestamp_prev; /* 上次执行时间戳(us) */
} PIDController;

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit);
void PID_Reset(PIDController *pid);
float PID_Update(PIDController *pid, float error);

#endif /* __PID_H */
