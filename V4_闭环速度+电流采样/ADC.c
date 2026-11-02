// PA4/ADC1 = phase A, PA5/ADC2 = phase B, PA3/ADC3 = phase C.
// Three low-side shunts, 4 mOhm each; DRV8323 CSA gain = 5 V/V.
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "ADC.h"
#include "Delay.h"
#include "misc.h"

#define ADC_REF_VOLTAGE   3.3f
#define ADC_FULL_SCALE    4095.0f
#define OFFSET_SAMPLES   1024u
#define DRV_CAL_PORT      GPIOE
#define DRV_CAL_PIN       GPIO_Pin_5

/* DRV8323 low-side current sense front end.
 * Three phase shunts, all 4 mOhm; DRV8323 CSA gain = 5 V/V.
 * VREF = 3.3 V, so the effective transconductance is
 *   4 mOhm * 5 V/V = 0.020 V/A
 * and one ADC count is about 0.0403 A. */
#define ADC_CSA_GAIN       5.0f
#define ADC_PHASE_SHUNT    0.004f   /* ohm, each phase low-side shunt */

#define ADC_PHASE_SCALE    (ADC_REF_VOLTAGE / \
                            (ADC_FULL_SCALE * ADC_PHASE_SHUNT * ADC_CSA_GAIN))

/* The DRV8323 low-side CSA common-mode output shifts when the PWM duty
 * changes.  Estimate the residual common-mode current at the ADC rate and
 * remove its low-frequency component.  The ADC runs at about 30 kHz, so
 * this alpha gives a time constant of roughly 20 ms. */
#define ADC_SAMPLE_RATE_HZ   30000.0f
#define ADC_COMMON_MODE_TF   0.020f
#define ADC_COMMON_MODE_ALPHA \
    (ADC_COMMON_MODE_TF / \
     (ADC_COMMON_MODE_TF + (1.0f / ADC_SAMPLE_RATE_HZ)))

static volatile uint16_t raw[3];
static uint16_t offsets[3];
static uint32_t sums[3];
static uint16_t sample_count;
static volatile uint8_t ready;
static volatile uint8_t calibrating;
static volatile uint8_t measuring;
static volatile uint8_t fresh_sample;
static volatile uint32_t last_sample_cycles;
static volatile float common_mode_current;
static volatile uint8_t common_mode_valid;

// PA4/ADC1 = phase A, PA5/ADC2 = phase B, PA3/ADC3 = phase C.

uint8_t ADC_Current_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    NVIC_InitTypeDef nvic;

    ready = 0;
    calibrating = 0;
    sample_count = 0;
    sums[0] = sums[1] = sums[2] = 0;
    raw[0] = raw[1] = raw[2] = 0;
    common_mode_current = 0.0f;
    common_mode_valid = 0;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
                           RCC_APB2Periph_ADC3, ENABLE);

    /* Analog inputs, no internal pull resistors. */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* DRV8323HR CAL input: inactive low, active high. */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = DRV_CAL_PIN;
    GPIO_ResetBits(DRV_CAL_PORT, DRV_CAL_PIN);
    GPIO_Init(DRV_CAL_PORT, &GPIO_InitStructure);

    /* ADC通用配置: 独立模式, ADCCLK = 84MHz/4 = 21MHz(上限36MHz) */
    ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_InjecSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC1: 12位, 软件触发, 单通道单次转换 */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Init(ADC3, &ADC_InitStructure);
    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);
    ADC_InjectedSequencerLengthConfig(ADC3, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_84Cycles);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_84Cycles);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_3, 1, ADC_SampleTime_84Cycles);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);

    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
    nvic.NVIC_IRQChannel = ADC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    return 1;
}

void ADC_Current_Reset(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    measuring = 0;
    fresh_sample = 0;
    calibrating = 0;
    ready = 0;
    common_mode_current = 0.0f;
    common_mode_valid = 0;
    raw[0] = offsets[0];
    raw[1] = offsets[1];
    raw[2] = offsets[2];
    __set_PRIMASK(mask);
    GPIO_ResetBits(DRV_CAL_PORT, DRV_CAL_PIN);
}

void ADC_Current_BeginCalibration(void)
{
    ADC_Current_Reset();
    /* Called after driver wake-up, with all PWM gate inputs inactive. */
    GPIO_SetBits(DRV_CAL_PORT, DRV_CAL_PIN);
    Delay_us(200);                 /* internal trim: min 100 us */
    GPIO_ResetBits(DRV_CAL_PORT, DRV_CAL_PIN);
    Delay_ms(5);                   /* let SOx settle before sampling */

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    sample_count = 0;
    sums[0] = sums[1] = sums[2] = 0;
    calibrating = 1;
    __set_PRIMASK(mask);
}

/* Re-sample the zero-current offset with the PWM outputs already enabled.
 * The DRV8323 offset can shift when the low-side common-mode starts
 * switching, so this must be done after MOE is enabled and the bridge is
 * at neutral duty. */
void ADC_Current_BeginOffsetSample(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ready = 0;
    measuring = 0;
    fresh_sample = 0;
    sample_count = 0;
    sums[0] = sums[1] = sums[2] = 0;
    calibrating = 1;
    __set_PRIMASK(mask);
}

void ADC_Current_SetRunning(uint8_t running)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    fresh_sample = 0;
    if ((running != 0u) && (ready != 0u))
    {
        /* Restart the common-mode estimate from the first new sample. */
        common_mode_current = 0.0f;
        common_mode_valid = 0;
    }
    measuring = running && ready;
    __set_PRIMASK(mask);
}

uint8_t ADC_Current_IsValid(void)
{
    uint32_t primask;
    uint32_t last_sample;
    uint32_t now;

    if ((measuring == 0u) || (ready == 0u) || (fresh_sample == 0u))
    {
        return 0u;
    }

    /* The ADC ISR updates last_sample_cycles at ~30 kHz.  Reading the DWT
     * counter and last_sample_cycles non-atomically can make the subtraction
     * underflow and produce a false "stale" result.  Snapshot both with
     * interrupts masked. */
    primask = __get_PRIMASK();
    __disable_irq();
    last_sample = last_sample_cycles;
    now = DWT->CYCCNT;
    __set_PRIMASK(primask);

    return ((uint32_t)(now - last_sample) < (SystemCoreClock / 100u)) ? 1u : 0u;
}

uint8_t ADC_Current_IsReady(void)
{
    return ready;
}

uint8_t ADC_Current_Get(float *phase_a, float *phase_b,
                     float *phase_c, float *bus_current)
{
    uint16_t values[3];
    float common_mode;
    uint32_t primask = __get_PRIMASK();
    uint8_t valid;
    __disable_irq();
    valid = ADC_Current_IsValid();
    values[0] = raw[0];
    values[1] = raw[1];
    values[2] = raw[2];
    common_mode = common_mode_current;
    __set_PRIMASK(primask);
    if (!valid)
    {
        *phase_a = *phase_b = *phase_c = *bus_current = 0.0f;
        return 0;
    }
    /* DRV8323 bidirectional output:
     *   V_SOx = VREF/2 - I * Rshunt * GCSA
     * so a sample below the zero-current offset represents positive
     * current.  Convert with I = (offset - raw) * ADC_PHASE_SCALE.
     *
     * Remove the low-frequency common-mode estimate as well.  In a
     * three-wire motor the true phase currents sum to zero, so the
     * average is an error term caused by CSA/common-mode shift. */
    *phase_a = (float)((int32_t)offsets[0] - (int32_t)values[0]) *
               ADC_PHASE_SCALE - common_mode;
    *phase_b = (float)((int32_t)offsets[1] - (int32_t)values[1]) *
               ADC_PHASE_SCALE - common_mode;
    *phase_c = (float)((int32_t)offsets[2] - (int32_t)values[2]) *
               ADC_PHASE_SCALE - common_mode;

    /* There is no bus-current shunt on this board. */
    *bus_current = 0.0f;
    return 1;
}

void ADC_IRQHandler(void)
{
    uint16_t values[3];
    int32_t counts[3];
    float common;
    unsigned i;
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) == RESET)
        return;
    values[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    values[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
    values[2] = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_1);
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    if (ready && measuring)
    {
        counts[0] = (int32_t)offsets[0] - (int32_t)values[0];
        counts[1] = (int32_t)offsets[1] - (int32_t)values[1];
        counts[2] = (int32_t)offsets[2] - (int32_t)values[2];

        common = (float)(counts[0] + counts[1] + counts[2]) *
                 (ADC_PHASE_SCALE / 3.0f);
        if (common_mode_valid == 0u)
        {
            common_mode_current = common;
            common_mode_valid = 1u;
        }
        else
        {
            common_mode_current +=
                ADC_COMMON_MODE_ALPHA * (common - common_mode_current);
        }

        for (i = 0; i < 3; ++i)
            raw[i] = values[i];
        last_sample_cycles = DWT->CYCCNT;
        fresh_sample = 1;
    }
    else if (calibrating)
    {
        for (i = 0; i < 3; ++i)
            sums[i] += values[i];
        if (++sample_count >= OFFSET_SAMPLES)
        {
            for (i = 0; i < 3; ++i)
            {
                offsets[i] = (uint16_t)(sums[i] / OFFSET_SAMPLES);
                raw[i] = offsets[i];
            }
            ready = 1;
            calibrating = 0;
        }
    }
}
