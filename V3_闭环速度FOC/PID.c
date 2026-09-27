
#include "PID.h"
#include "Delay.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    PID_Reset(pid);
}

/* 清除PID内部状态(停止/重启时调用, 避免残留积分和输出跳变) */
void PID_Reset(PIDController *pid)
{
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = micros();
}

float PID_Update(PIDController *pid, float error)
{
    /* 计算两次调用的时间间隔, 异常时用1ms */
    uint32_t timestamp_now = micros();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    /* P环 */
    float proportional = pid->P * error;

    /* I环(Tustin离散积分), 积分限幅防止饱和 */
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    integral = _constrain(integral, -pid->limit, pid->limit);

    /* D环 */
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    /* 输出 = P + I + D, 限幅 */
    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

    /* 输出变化率限幅(防止输出突变) */
    if (pid->output_ramp > 0.0f)
    {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
        {
            output = pid->output_prev + pid->output_ramp * Ts;
        }
        else if (output_rate < -pid->output_ramp)
        {
            output = pid->output_prev - pid->output_ramp * Ts;
        }
    }

    /* 保存本次值, 供下次循环使用 */
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;

    return output;
}
