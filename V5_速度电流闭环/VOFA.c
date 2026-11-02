#include "VOFA.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FOC.h"
#include "FOC_Current.h"
#include "ADC.h"
#include "VOFA_UART.h"

/* FireWater: ASCII decimal channels separated by ',', frame ends with '\n'.
 * 帧缓冲区只有本任务一个写者，因此不需要临界区；
 * taskENTER_CRITICAL 会屏蔽 ADC 电流环中断，必须避免。 */

#define VOFA_MAX_CHANNELS   16u
#define VOFA_CHANNEL_CHARS  16u  /* sign + 7 integer digits + '.' + 3 decimals */
#define VOFA_FRAME_SIZE     ((VOFA_MAX_CHANNELS * VOFA_CHANNEL_CHARS) + 2u)

static char frame[VOFA_FRAME_SIZE];

void VOFA_Init(void)
{
    VOFA_UART_Init();
}

static uint16_t VOFA_AppendUInt(char *buf, uint16_t pos, uint32_t value)
{
    char tmp[10];
    uint8_t len = 0u;

    if (value == 0u)
    {
        buf[pos++] = '0';
        return pos;
    }

    while (value != 0u)
    {
        tmp[len++] = (char)('0' + (value % 10u));
        value /= 10u;
    }

    while (len != 0u)
    {
        buf[pos++] = tmp[--len];
    }

    return pos;
}

/* Fixed 3-decimal formatting, avoids newlib-nano float printf support. */
static uint16_t VOFA_AppendFloat(char *buf, uint16_t pos, float value)
{
    const uint32_t scale = 1000u;
    uint32_t scaled;
    uint32_t whole;
    uint32_t frac;
    uint8_t negative = 0u;

    if (value != value) /* NaN */
    {
        buf[pos++] = '0';
        return pos;
    }

    if (value < 0.0f)
    {
        negative = 1u;
        value = -value;
    }

    if (value > 999999.0f)
    {
        value = 999999.0f;
    }

    scaled = (uint32_t)((value * (float)scale) + 0.5f);
    whole = scaled / scale;
    frac = scaled % scale;

    if (negative != 0u)
    {
        buf[pos++] = '-';
    }

    pos = VOFA_AppendUInt(buf, pos, whole);
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + ((frac / 100u) % 10u));
    buf[pos++] = (char)('0' + ((frac / 10u) % 10u));
    buf[pos++] = (char)('0' + (frac % 10u));

    return pos;
}

/* Format the channels as a FireWater frame and send it. */
void VOFA_SendFloat(float *arr, uint8_t n)
{
    uint16_t i;
    uint16_t frame_length = 0u;

    if ((arr == 0) || (n == 0u) || (n > VOFA_MAX_CHANNELS))
    {
        return;
    }

    for (i = 0u; i < n; ++i)
    {
        if (i != 0u)
        {
            frame[frame_length++] = ',';
        }

        frame_length = VOFA_AppendFloat(frame, frame_length, arr[i]);
    }

    frame[frame_length++] = '\n';

    VOFA_UART_Send((const uint8_t *)frame, frame_length);
}

/* FireWater 通道：
 *   0 目标速度 rad/s          1 实际速度 rad/s
 *   2/3/4  A/B/C 相电流 A
 *   5/6/7  A/B/C 相占空比 %
 *   8  iq_ref（速度环输出）A  9  iq_meas（实际 dq 电流）A        */
void vVOFATask(void *pvParameters)
{
    TickType_t last_wake_time;
    float phase_a;
    float phase_b;
    float phase_c;
    float bus_current;
    float id_meas;
    float iq_meas;

    (void)pvParameters;

    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        /* ADC_Current_Get() fills all outputs with 0 when no fresh sample
         * is available, so the channel count stays fixed during calibration. */
        (void)ADC_Current_Get(&phase_a, &phase_b, &phase_c, &bus_current);
        (void)bus_current;

        FOC_CurrentGetIdq(&id_meas, &iq_meas);
        (void)id_meas;

        VOFA_SendFloat((float[]){
                           FOC_GetTargetVelocity(),
                           FOC_GetVelocity(),
                           phase_a,
                           phase_b,
                           phase_c,
                           FOC_GetDutyA() * 100.0f,
                           FOC_GetDutyB() * 100.0f,
                           FOC_GetDutyC() * 100.0f,
                           FOC_CurrentGetIqRef(),
                           iq_meas
                       },
                       10u);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5)); /* fixed 100 Hz */
    }
}
