#include "FOC_Current.h"
#include "FOC_Params.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "FOC.h"
#include "AS5600.h"
#include "ADC.h"
#include "LowPassFilter.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define FOC_PI                    3.14159265358979323846f

/* ==================== power stage ==================== */
#define FOC_PWM_TIM               TIM1
#define FOC_PWM_ARR               255u
#define FOC_PWM_PSC               21u
#define FOC_DEADTIME              84u        /* 0.5 us at 168 MHz */
#define FOC_ENABLE_PORT           GPIOE
#define FOC_ENABLE_PIN            GPIO_Pin_7

/* ==================== motor/control ==================== */
/* 电机参数、带宽、增益都在 inc/FOC_Params.h 里推导 */
#define FOC_POLE_PAIRS            Pn
#define FOC_SUPPLY_VOLTAGE        VDC
#define FOC_DEFAULT_TARGET        15.0f      /* mechanical rad/s */
#define FOC_ALIGN_TIME_MS         3000u
#define FOC_ALIGN_ANGLE           ALIGN_Angle

/* Require several consecutive invalid current samples before latching ERROR.
 * A single sample can be lost to a transient/race; five 1 ms checks is 5 ms. */
#define FOC_CURRENT_FAULT_COUNT   5u

/* ==================== private state ==================== */
static volatile FOC_State motor_state = FOC_STATE_STOP;
static volatile uint8_t foc_running = 0u;
static volatile uint8_t foc_calibrated = 0u;

static float target_velocity = FOC_DEFAULT_TARGET;
static float measured_velocity = 0.0f;      /* filtered, mechanical rad/s */
static float zero_electric_angle = 0.0f;

static LowPassFilter velocity_lpf;
static float speed_integral = 0.0f;

static float duty_a;
static float duty_b;
static float duty_c;


/*输入限幅*/
static float FOC_Constrain(float value, float min, float max)
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

/*角度归一化*/
static float FOC_NormalizeAngle(float angle)
{
    float normalized = fmodf(angle, 2.0f * FOC_PI);
    return (normalized >= 0.0f) ? normalized : (normalized + (2.0f * FOC_PI));
}

/* 三路占空比写入。上限 0.86 保证 TIM1_CH4 触发 ADC 时三相下管仍导通。 */
void FOC_SetDuty(float da, float db, float dc)
{
    da = FOC_Constrain(da, Duty_Min, Duty_Max);
    db = FOC_Constrain(db, Duty_Min, Duty_Max);
    dc = FOC_Constrain(dc, Duty_Min, Duty_Max);

    duty_a = da;
    duty_b = db;
    duty_c = dc;

    TIM_SetCompare1(FOC_PWM_TIM, (uint16_t)(da * (float)FOC_PWM_ARR));
    TIM_SetCompare2(FOC_PWM_TIM, (uint16_t)(db * (float)FOC_PWM_ARR));
    TIM_SetCompare3(FOC_PWM_TIM, (uint16_t)(dc * (float)FOC_PWM_ARR));
}

/* 关 MOE + 关 DRV8323，用于保护和停机 */
void FOC_DisableOutputs(void)
{
    TIM_CtrlPWMOutputs(FOC_PWM_TIM, DISABLE);
    FOC_SetEnable(0u);
}

/* 电机关断*/
static void FOC_MotorDisable(FOC_State state)
{
    FOC_CurrentSetMode(FOC_Loop_Off);
    FOC_CurrentSetIqRef(0.0f);
    speed_integral = 0.0f;

    FOC_DisableOutputs();
    ADC_Current_Reset();

    FOC_SetDuty(0.5f, 0.5f, 0.5f);
    LPF_Reset(&velocity_lpf);

    motor_state = state;
}

/* 对齐结束：记录电角度零点，交给电流环模块 */
static void FOC_FinishAlignment(void)
{
    AS5600_Update();
    zero_electric_angle = FOC_NormalizeAngle(
        (float)FOC_POLE_PAIRS * AS5600_GetMechanicalAngle());

    FOC_CurrentSetZeroOffset(zero_electric_angle);
    foc_calibrated = 1u;
}

/* 速度环：有功阻尼 + 零点对消，输出 iq* */
static void FOC_SpeedLoopStep(void)
{
    float err = target_velocity - measured_velocity;
    float iq_raw;
    float iq_sat;
    float anti;

    speed_integral += (foc.Kiw * foc.Ts_spd) * err;
    iq_raw = (foc.Kpw * err) + speed_integral -
             (foc.Ba * measured_velocity);

    iq_sat = iq_raw;
    if (iq_sat > foc.iq_max)
    {
        iq_sat = foc.iq_max;
    }
    if (iq_sat < -foc.iq_max)
    {
        iq_sat = -foc.iq_max;
    }

    anti = iq_sat - iq_raw;
    speed_integral += foc.kaw_spd * anti;

    FOC_CurrentSetIqRef(iq_sat);
}

/* 进入 RUN：用电编码器当前角度对齐电流环的 theta_e */
static void FOC_EnterRun(void)
{
    AS5600_Update();

    FOC_CurrentSetWe(FOC_POLE_PAIRS * measured_velocity);
    FOC_CurrentSyncFromEncoder(AS5600_GetMechanicalAngle());
    FOC_CurrentSetIqRef(0.0f);
    speed_integral = 0.0f;
    FOC_CurrentSetMode(FOC_Loop_Run);

    motor_state = FOC_STATE_RUN;
}

/* ==================== public API ==================== */
void FOC_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef oc;
    TIM_BDTRInitTypeDef bdtr;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE,
                           ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                    GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOE, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio);

    /* DRV8323 enable pin, start disabled. */
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Pin = FOC_ENABLE_PIN;
    GPIO_ResetBits(FOC_ENABLE_PORT, FOC_ENABLE_PIN);
    GPIO_Init(FOC_ENABLE_PORT, &gpio);
    FOC_SetEnable(0u);

    /* 168 MHz / 22 / 256 = 29.8 kHz center-aligned-capable PWM. */
    TIM_TimeBaseStructInit(&time_base);
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Prescaler = FOC_PWM_PSC;
    time_base.TIM_Period = FOC_PWM_ARR;
    time_base.TIM_RepetitionCounter = 0u;
    TIM_TimeBaseInit(FOC_PWM_TIM, &time_base);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = FOC_PWM_ARR / 2u;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(FOC_PWM_TIM, &oc);
    TIM_OC2Init(FOC_PWM_TIM, &oc);
    TIM_OC3Init(FOC_PWM_TIM, &oc);

    /* CH4 is only the ADC trigger, no pin output.
     * Trigger near the end of the up-count period, when all three low-side
     * FETs are on and the 4 mOhm shunt currents are valid. */
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = FOC_PWM_ARR - 35u;
    TIM_OC4Init(FOC_PWM_TIM, &oc);
    TIM_SelectOutputTrigger(FOC_PWM_TIM, TIM_TRGOSource_OC4Ref);

    TIM_OC1PreloadConfig(FOC_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(FOC_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(FOC_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(FOC_PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(FOC_PWM_TIM, ENABLE);

    TIM_BDTRStructInit(&bdtr);
    bdtr.TIM_OSSRState = TIM_OSSRState_Enable;
    bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
    bdtr.TIM_DeadTime = FOC_DEADTIME;
    bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(FOC_PWM_TIM, &bdtr);

    TIM_CtrlPWMOutputs(FOC_PWM_TIM, DISABLE);
    TIM_GenerateEvent(FOC_PWM_TIM, TIM_EventSource_Update);
    TIM_Cmd(FOC_PWM_TIM, ENABLE);

    DWT_Init();

    FOC_CurrentInit();                          /* 参数 + 电流环状态 */
    LPF_Init(&velocity_lpf, VEL_LPF_Tf);
    FOC_SetDuty(0.5f, 0.5f, 0.5f);

    target_velocity = FOC_DEFAULT_TARGET;
    motor_state = FOC_STATE_STOP;
    foc_running = 0u;
    foc_calibrated = 0u;
    speed_integral = 0.0f;
}


void FOC_SetEnable(uint8_t enable)
{
    if (enable != 0u)
    {
        GPIO_SetBits(FOC_ENABLE_PORT, FOC_ENABLE_PIN);
    }
    else
    {
        GPIO_ResetBits(FOC_ENABLE_PORT, FOC_ENABLE_PIN);
    }
}

void FOC_ToggleRunning(void)
{
    if (motor_state == FOC_STATE_ERROR)
    {
        /* First press clears the fault; the state machine goes to STOP. */
        FOC_CurrentClearFault();
        foc_running = 0u;
        return;
    }

    foc_running = (foc_running == 0u) ? 1u : 0u;
}

uint8_t FOC_IsRunning(void)
{
    return foc_running;
}

uint8_t FOC_IsCalibrated(void)
{
    return foc_calibrated;
}

void FOC_SetTargetVelocity(float rad_s)
{
    target_velocity = FOC_Constrain(rad_s, -200.0f, 200.0f);
}

float FOC_GetTargetVelocity(void)
{
    return target_velocity;
}

float FOC_GetVelocity(void)
{
    return measured_velocity;
}

float FOC_GetDutyA(void)
{
    return duty_a;
}

float FOC_GetDutyB(void)
{
    return duty_b;
}

float FOC_GetDutyC(void)
{
    return duty_c;
}

FOC_State FOC_GetState(void)
{
    return motor_state;
}

/* ==================== FreeRTOS task ==================== */
void vFOCSpeedTask(void *pvParameters)
{
    TickType_t state_started = xTaskGetTickCount();
    uint8_t driver_awake = 0u;
    uint8_t offset_started = 0u;
    uint8_t align_started = 0u;
    uint8_t current_invalid_count = 0u;

    (void)pvParameters;

    FOC_MotorDisable(FOC_STATE_STOP);

    for (;;)
    {
        TickType_t now;

        AS5600_Update();
        measured_velocity = LPF_Update(&velocity_lpf, AS5600_GetVelocity());

        now = xTaskGetTickCount();

        if (foc_running == 0u)
        {
            if (motor_state != FOC_STATE_STOP)
            {
                FOC_MotorDisable(FOC_STATE_STOP);
            }
        }
        else
        {
            switch (motor_state)
            {
            case FOC_STATE_STOP:
                /* 每次重新启动都从干净的故障状态开始 */
                FOC_CurrentClearFault();
                state_started = now;
                driver_awake = 0u;
                offset_started = 0u;
                align_started = 0u;
                current_invalid_count = 0u;
                motor_state = FOC_STATE_WAKE;
                break;

            case FOC_STATE_WAKE:
                if (driver_awake == 0u)
                {
                    if ((TickType_t)(now - state_started) >=
                        pdMS_TO_TICKS(300))
                    {
                        FOC_SetEnable(1u);
                        driver_awake = 1u;
                        state_started = now;
                    }
                }
                else if ((TickType_t)(now - state_started) >=
                         pdMS_TO_TICKS(5))
                {
                    ADC_Current_BeginCalibration();
                    state_started = now;
                    motor_state = FOC_STATE_CALIBRATE;
                }
                break;

            case FOC_STATE_CALIBRATE:
                if (ADC_Current_IsReady())
                {
                    /* Neutral duty before enabling the complementary outputs. */
                    FOC_SetDuty(0.5f, 0.5f, 0.5f);
                    TIM_GenerateEvent(FOC_PWM_TIM, TIM_EventSource_Update);
                    TIM_CtrlPWMOutputs(FOC_PWM_TIM, ENABLE);
                    state_started = now;
                    offset_started = 0u;
                    motor_state = FOC_STATE_OFFSET;
                }
                else if ((TickType_t)(now - state_started) >=
                         pdMS_TO_TICKS(1000))
                {
                    FOC_MotorDisable(FOC_STATE_ERROR);
                }
                break;

            case FOC_STATE_OFFSET:
                if (offset_started == 0u)
                {
                    /* Let the bridge and CSA common-mode settle, then take a
                     * second zero-current average with PWM active. */
                    if ((TickType_t)(now - state_started) >=
                        pdMS_TO_TICKS(10))
                    {
                        ADC_Current_BeginOffsetSample();
                        offset_started = 1u;
                        state_started = now;
                    }
                }
                else if (ADC_Current_IsReady())
                {
                    ADC_Current_SetRunning(1u);
                    state_started = now;
                    current_invalid_count = 0u;

                    if (foc_calibrated != 0u)
                    {
                        FOC_EnterRun();
                    }
                    else
                    {
                        align_started = 0u;
                        motor_state = FOC_STATE_ALIGN;
                    }
                }
                else if ((TickType_t)(now - state_started) >=
                         pdMS_TO_TICKS(1000))
                {
                    FOC_MotorDisable(FOC_STATE_ERROR);
                }
                break;

            case FOC_STATE_ALIGN:
                /* 对齐期间也要检查锁存故障，避免带着故障空转 3 秒 */
                if (FOC_CurrentHasFault() != 0u)
                {
                    FOC_MotorDisable(FOC_STATE_ERROR);
                    break;
                }

                if (align_started == 0u)
                {
                    /* 固定电压矢量由电流环中断输出，任务只管计时 */
                    FOC_CurrentSetIqRef(0.0f);
                    FOC_CurrentSetMode(FOC_Loop_Align);
                    align_started = 1u;
                    state_started = now;
                }

                if ((TickType_t)(now - state_started) >= pdMS_TO_TICKS(20))
                {
                    if (ADC_Current_IsValid() == 0u)
                    {
                        if (current_invalid_count < 255u)
                        {
                            current_invalid_count++;
                        }

                        if (current_invalid_count >= FOC_CURRENT_FAULT_COUNT)
                        {
                            FOC_MotorDisable(FOC_STATE_ERROR);
                            break;
                        }
                    }
                    else
                    {
                        current_invalid_count = 0u;
                    }
                }

                if ((TickType_t)(now - state_started) >=
                    pdMS_TO_TICKS(FOC_ALIGN_TIME_MS))
                {
                    FOC_FinishAlignment();
                    current_invalid_count = 0u;
                    FOC_EnterRun();
                }
                break;

            case FOC_STATE_RUN:
                /* 中断里锁存的过流故障 */
                if (FOC_CurrentHasFault() != 0u)
                {
                    FOC_MotorDisable(FOC_STATE_ERROR);
                    break;
                }

                if ((TickType_t)(now - state_started) >= pdMS_TO_TICKS(20))
                {
                    if (ADC_Current_IsValid() == 0u)
                    {
                        if (current_invalid_count < 255u)
                        {
                            current_invalid_count++;
                        }

                        if (current_invalid_count >= FOC_CURRENT_FAULT_COUNT)
                        {
                            FOC_MotorDisable(FOC_STATE_ERROR);
                            break;
                        }
                    }
                    else
                    {
                        current_invalid_count = 0u;
                    }
                }
                else
                {
                    current_invalid_count = 0u;
                }

#if OPENLOOP_TEST
                /* 【临时诊断】固定 iq* + 电角度开环匀速旋转，完全绕过编码器
                 * 和速度环，用来验证功率级、编码器方向和实际极对数。 */
                FOC_CurrentSetIqRef(OPENLOOP_IQ);
                FOC_CurrentSetWe(OPENLOOP_WE);
#elif FIXED_FRAME_TEST
                /* 【临时诊断】固定 iq*，配合中断里的固定 theta_e=0，
                 * 完全绕过编码器与速度环，只验证"电流→力矩"通路。 */
                FOC_CurrentSetIqRef(FIXED_FRAME_IQ);
#else
                /* 速度环输出 iq*，电流环在中断里已经按它执行 */
                FOC_CurrentSetWe(FOC_POLE_PAIRS * measured_velocity);
                FOC_CurrentTrackAngle(AS5600_GetMechanicalAngle());
                FOC_SpeedLoopStep();
#endif
                break;

            case FOC_STATE_ERROR:
                /* Press PC0 to clear, then press again to restart. */
                break;

            default:
                FOC_MotorDisable(FOC_STATE_STOP);
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}