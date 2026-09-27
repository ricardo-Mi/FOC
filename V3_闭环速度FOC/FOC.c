// 移植自灯哥开源 DengFOC 闭环速度代码
// 平台: STM32F407ZGT6 + 标准外设库 + FreeRTOS
// PWM: PA9(TIM1_CH2) PE13(TIM1_CH3) PE14(TIM1_CH4)  使能: PA10
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "FOC.h"
#include "AS5600.h"
#include "PID.h"
#include "LowPassFilter.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ==================== 硬件参数 ==================== */
#define FOC_PWM_TIM       TIM1
#define FOC_PWM_ARR       255       /* 8位精度, 对应 Arduino ledcSetup 的 8bit */
#define FOC_PWM_PSC       21        /* 168MHz/(21+1)/(255+1) ≈ 29.83kHz, 对应 30kHz */
#define FOC_ENABLE_PORT   GPIOA
#define FOC_ENABLE_PIN    GPIO_Pin_10
#define FOC_POLE_PAIRS    7         /* 电机极对数, 原代码为 7 */
#define _3PI_2            4.71238898038f  /* 3PI/2, 校准用的电压矢量方向 */
#define FOC_KP_POS        0.133f          /* 位置环P增益, 来自DengFOC例程 */
/* 速度环PID增益(误差单位rad/s, 输出单位V)
   调参: 震荡大->减小KP或增大滤波Tf; 响应慢/带载掉速->适当加KP; 静差消不掉->加KI */
#define FOC_VEL_KP        0.08f
#define FOC_VEL_KI        0.30f

/* ==================== 初始变量及函数定义 ==================== */
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/* 宏定义实现的一个约束函数, 用于限制一个值的范围:
   如果 amt 小于 low 则返回 low, 大于 high 则返回 high, 否则返回 amt */

float voltage_power_supply = 12.6f;
float target_velocity = 10.0f;      /* 速度闭环目标 rad/s */
float shaft_angle = 0;
volatile uint8_t foc_running = 0;     /* 电机启停标志, 由PC0按键切换 */
volatile uint8_t foc_calibrated = 0;  /* 校准完成标志: 0=F(校准中) 1=T(完成), OLED显示用 */
float zero_electric_angle = 0;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

static uint32_t open_loop_timestamp = 0;      /* 开环上次运行的时间戳(us) */
static float measured_velocity = 0.0f;        /* 最近一次滤波后的实测速度(rad/s) */
static PIDController vel_loop;                /* 速度环PID */
static LowPassFilter vel_filter;              /* 速度反馈低通滤波 */

/* ==================== 初始化 ==================== */
void FOC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* 开启时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);   /* TIM1在APB2, 时钟168MHz */

    /* PWM引脚初始化: PA9/PE13/PE14 复用推挽 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_TIM1);   /* PA9  -> TIM1_CH2 */
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);   /* PE13 -> TIM1_CH3 */
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);   /* PE14 -> TIM1_CH4 */

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* 使能引脚 PA10 普通推挽输出, 先关闭使能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = FOC_ENABLE_PIN;
    GPIO_Init(FOC_ENABLE_PORT, &GPIO_InitStructure);
    FOC_SetEnable(0);

    /* 时基单元初始化: 向上计数, 约30kHz */
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = FOC_PWM_PSC;
    TIM_TimeBaseInitStructure.TIM_Period = FOC_PWM_ARR;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /* 输出比较初始化: CH2/CH3/CH4, PWM1模式, 初始占空比50%(相电压为0) */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = FOC_PWM_ARR / 2;

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);   /* PA9  A相 */
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);   /* PE13 B相 */
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);   /* PE14 C相 */

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* TIM1是高级定时器, 必须使能主输出(MOE), 否则引脚无PWM */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* 微秒计时初始化 */
    DWT_Init();

    /* 速度环PID: 输出斜率100000, 限幅Vbus/2 (增益整定见FOC_VEL_KP定义处) */
    PID_Init(&vel_loop, FOC_VEL_KP, FOC_VEL_KI, 0.0f, 100000.0f, voltage_power_supply / 2.0f);

    /* 速度反馈低通滤波: Tf=20ms, 更强滤波减小震荡(太大会迟钝) */
    LPF_Init(&vel_filter, 0.025f);
}

void FOC_SetEnable(uint8_t enable)
{
    if (enable)
    {
        GPIO_SetBits(FOC_ENABLE_PORT, FOC_ENABLE_PIN);
    }
    else
    {
        GPIO_ResetBits(FOC_ENABLE_PORT, FOC_ENABLE_PIN);
    }
}

/* ==================== FOC 算法 ==================== */

/* 电角度求解 */
float _electricalAngle(float shaft_angle, int pole_pairs)
{
    return (shaft_angle * pole_pairs);
}

/* 归一化角度到 [0,2PI] */
float _normalizeAngle(float angle)
{
    float a = fmodf(angle, 2 * PI);   /* 取余运算可以用于归一化 */
    return a >= 0 ? a : (a + 2 * PI);
}

/* 设置PWM到控制器输出 */
void setPwm(float Ua, float Ub, float Uc)
{
    /* 计算占空比, 限制占空比从0到1 */
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    /* 写入PWM到TIM1的CH2/CH3/CH4 */
    TIM_SetCompare2(TIM1, (uint16_t)(dc_a * FOC_PWM_ARR));
    TIM_SetCompare3(TIM1, (uint16_t)(dc_b * FOC_PWM_ARR)); 
    TIM_SetCompare4(TIM1, (uint16_t)(dc_c * FOC_PWM_ARR));
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);

    /* 帕克逆变换 */
    Ualpha = -Uq * sinf(angle_el);
    Ubeta  =  Uq * cosf(angle_el);

    /* 克拉克逆变换 */
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (sqrtf(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPwm(Ua, Ub, Uc);
}

/* 输出力矩电压矢量:
   Uq限幅到±Vbus/2, 反Park/反Clarke后写入PWM */
void setTorque(float Uq, float angle_el)
{
    Uq = _constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);
    angle_el = _normalizeAngle(angle_el);

    /* 帕克逆变换 */
    Ualpha = -Uq * sinf(angle_el);
    Ubeta  =  Uq * cosf(angle_el);

    /* 克拉克逆变换 */
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (sqrtf(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPwm(Ua, Ub, Uc);
}

/* 由编码器实测角度计算电角度: 
   注: 传感器方向已在AS5600驱动里翻转(AS5600_DIRECTION=-1), 这里不再乘DIR */
float FOC_ElectricalAngle(void)
{
    return _normalizeAngle((float)FOC_POLE_PAIRS * AS5600_GetMechanicalAngle() - zero_electric_angle);
}

/* 电角度零点校准: 
   输出固定电压矢量把转子吸到已知位置, 3秒后读取电角度作为零点
   校准期间电机必须空载、可自由转动
   注意: 需在FreeRTOS任务中调用(用vTaskDelay让出CPU, 便于OLED刷新显示) */
void FOC_AlignSensor(void)
{
    FOC_SetEnable(1);                 /* 校准需要使能驱动器 */
    setTorque(3.0f, _3PI_2);          /* 输出3V矢量, 方向3PI/2 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    AS5600_Update();                  /* 读取对齐后的角度 */
    zero_electric_angle = FOC_ElectricalAngle();
    foc_calibrated = 1;               /* 校准完成, OLED显示T */

    setTorque(0, _3PI_2);             /* 撤销电压矢量 */
    FOC_SetEnable(0);
}

/* 位置闭环(纯P) */
float positionClosedLoop(float target)
{
    float error = (target - AS5600_GetAngle()) * 180.0f / PI;   /* 误差转成度 */
    float Uq = FOC_KP_POS * error;

    setTorque(Uq, FOC_ElectricalAngle());

    return Uq;
}

/* 最近一次滤波后的实测速度(rad/s), 供OLED显示 */
float FOC_GetVelocity(void)
{
    return measured_velocity;
}

/* 速度闭环(PID+低通滤波):
   注1: 原代码忽略了传入的Target参数(笔误, 实际用的串口值), 这里修正为使用target
   注2: 原代码把误差乘180/PI转成度/秒, 这里直接用rad/s, 增益按rad/s整定(见FOC_VEL_KP) */
float velocityClosedLoop(float target)
{
    float error = target - measured_velocity;   /* 误差(rad/s) */
    float Uq = PID_Update(&vel_loop, error);

    setTorque(Uq, FOC_ElectricalAngle());

    return Uq;
}

/* 开环速度函数 */
float velocityOpenloop(float target_velocity)
{
    uint32_t now_us = micros();   /* 获取当前微秒数 */

    /* 计算当前每个Loop的运行时间间隔, 无符号相减可自动处理计数回绕 */
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;

    /* 时间间隔异常(<=0或>0.5s)时, 使用默认值 1e-3f */
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    /* 目标速度乘以时间间隔得到转动的机械角度, 并归一化到 [0,2PI] */
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

    /* 使用voltage_power_supply的1/3作为Uq值, 直接影响输出力矩
       最大只能设置为Uq = voltage_power_supply/2, 否则Ua/Ub/Uc会超出供电电压限幅 */
    float Uq = voltage_power_supply / 3;

    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, FOC_POLE_PAIRS));

    open_loop_timestamp = now_us;   /* 用于计算下一个时间间隔 */

    return Uq;
}

/* ==================== FreeRTOS 任务 ==================== */
/* 闭环速度: 上电先校准电角度零点, 之后目标速度target_velocity, PC0按键控制启停 */
void vFOCVelocityTask(void *pvParameters)
{
    (void)pvParameters;

    FOC_AlignSensor();                 /* 开机校准(约3秒), 期间OLED显示 CAL:F */

    for (;;)
    {
        AS5600_Update();               /* 采样编码器角度 */
        /* 速度反馈: 编码器速度 + 低通滤波(25ms) */
        measured_velocity = LPF_Update(&vel_filter, AS5600_GetVelocity());

        if (foc_running)
        {
            FOC_SetEnable(1);          /* 使能驱动器 */
            velocityClosedLoop(target_velocity);
        }
        else
        {
            FOC_SetEnable(0);          /* 关闭驱动器 */
            setTorque(0, FOC_ElectricalAngle());   /* 零电压 */
            PID_Reset(&vel_loop);                   /* 清除PID状态, 避免重启时跳变 */
            open_loop_timestamp = micros();        /* 保留给开环使用 */
        }

        vTaskDelay(pdMS_TO_TICKS(1));  /* 1ms周期, 约1kHz更新 */
    }
}
