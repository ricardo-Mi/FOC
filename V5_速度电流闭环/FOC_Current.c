#include "stm32f4xx.h"
#include "FOC_Current.h"
#include "FOC_Params.h"
#include "FOC.h"
#include "ADC.h"
#include <math.h>

#define TWO_PI   6.283185307179586f
#define SQRT3    1.7320508075688772f
#define PI_F     3.141592653589793f

FOC_Params_t foc;

/* ---------------- 中断与任务共享 ---------------- */
static volatile float         theta_e;       /* 电角度，ISR 积分维护      */
static volatile float         we_est;        /* 电角速度 rad/s，任务写    */
static volatile float         theta_corr;    /* 一次性相位修正量          */
static volatile uint8_t       corr_pending;
static volatile float         iq_ref;
static volatile float         id_meas;
static volatile float         iq_meas;
static volatile FOC_LoopMode  loop_mode;
static volatile uint8_t       oc_fault;
static volatile float         zero_offset;   /* 电角度零点                */

/* ---------------- 仅 ISR 访问 ---------------- */
static float   int_d;
static float   int_q;
static uint8_t oc_count;
static uint16_t oc_blank;      /* 模式切换后的过流屏蔽窗口（拍数） */
static float   align_cos;
static float   align_sin;

/* 切换模式后屏蔽约 20 ms 的过流判断：覆盖占空比跳变、MOE 使能瞬间的
 * 电流建立过程，避免把瞬态当成故障。 */
#define OC_BLANK_SAMPLES   600u

/*归一角度到[0, 2π) */
static float wrap_angle(float a)
{
    a = fmodf(a, TWO_PI);
    if (a < 0.0f)
    {
        a += TWO_PI;
    }
    return a;
}

/*参数初始化*/
void FOC_ParamsInit(void)
{
    foc.pn     = Pn;
    foc.r      = R;
    foc.ld     = Ld;
    foc.lq     = Lq;
    foc.psi_f  = PSI_F;
    foc.j      = J;
    foc.b      = B;
    foc.Kt     = 1.5f * Pn * PSI_F;

    foc.Vdc    = VDC;
    foc.Ts_cur = Ts_Cur;
    foc.Ts_spd = Ts_Spd;
    foc.alpha  = Alpha;
    foc.beta   = Beta;

    /* 电流环：内模整定 / 零极点对消 */
    foc.Kpd   = Alpha * Ld;
    foc.Kid   = Alpha * R;
    foc.Kpq   = Alpha * Lq;
    foc.Kiq   = Alpha * R;

    /* 速度环：有功阻尼 + 零点对消 */
    foc.Kpw = Beta * J / foc.Kt;
    foc.Kiw = Beta * foc.Kpw;
    foc.Ba  = (Beta * J - B) / foc.Kt;
    if (foc.Ba < 0.0f)
    {
        foc.Ba = 0.0f;
    }

    foc.iq_max        = Iq_Max;
    foc.vmax          = V_Max;
    foc.kaw_cur       = Kaw_Cur;
    foc.kaw_spd       = Kaw_Spd;
    foc.align_voltage = ALIGN_Vol;
    foc.align_angle   = ALIGN_Angle;
    foc.oc_limit      = OC_Limit;
}

/*重置状态变量*/
void FOC_CurrentReset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    theta_e      = 0.0f;
    we_est       = 0.0f;
    theta_corr   = 0.0f;
    corr_pending = 0u;
    iq_ref       = 0.0f;
    int_d        = 0.0f;
    int_q        = 0.0f;
    oc_count     = 0u;
    oc_blank     = 0u;
    oc_fault     = 0u;
    id_meas      = 0.0f;
    iq_meas      = 0.0f;
    loop_mode    = FOC_Loop_Off;
    __set_PRIMASK(primask);
}

/*对齐矢量*/
void FOC_CurrentInit(void)
{
    FOC_ParamsInit();

    /* 对齐矢量在 alpha-beta 中的方向固定，先算好三角函数 */
    align_cos = cosf(foc.align_angle);
    align_sin = sinf(foc.align_angle);

    FOC_CurrentReset();
}

/*切换电流环模式*/
void FOC_CurrentSetMode(FOC_LoopMode mode)
{
    uint32_t primask = __get_PRIMASK();

    /* 兜底：若 FOC_CurrentInit() 被漏调用，vmax/oc_limit 会是 0，
     * 那会变成"一有噪声就过流"。这里补齐参数，宁可慢一拍也不能用 0 限幅。 */
    if ((foc.vmax <= 0.0f) || (foc.oc_limit <= 0.0f))
    {
        FOC_ParamsInit();
    }

    __disable_irq();
    if (mode != loop_mode)
    {
        int_d    = 0.0f;
        int_q    = 0.0f;
        oc_count = 0u;
        /* 模式切换后旧测量值已经无效，先清零，避免 VOFA 显示上一轮 RUN
         * 的 iq 让人误以为电流环还在跟踪。 */
        id_meas  = 0.0f;
        iq_meas  = 0.0f;
        /* 切换模式时开一段屏蔽窗，避开电流建立过程的瞬态 */
        oc_blank = (mode == FOC_Loop_Off) ? 0u : OC_BLANK_SAMPLES;
    }
    loop_mode = mode;
    __set_PRIMASK(primask);
}

/*返回当前电流环模式*/
FOC_LoopMode FOC_CurrentGetMode(void)
{
    return loop_mode;
}
/*设置 q 轴电流指令 iq_ref*/
void FOC_CurrentSetIqRef(float iq)
{
    iq_ref = iq;
}

/*返回当前 q 轴电流指令*/
float FOC_CurrentGetIqRef(void)
{
    return iq_ref;
}

/*读取id iq 测量值*/
void FOC_CurrentGetIdq(float *id, float *iq)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *id = id_meas;
    *iq = iq_meas;
    __set_PRIMASK(primask);
}

/*设置电流采样的零点偏置值*/
void FOC_CurrentSetZeroOffset(float zero)
{
    zero_offset = zero;
}

/*得到电角度*/
void FOC_CurrentSyncFromEncoder(float mech_angle)
{
    theta_e      = wrap_angle((foc.pn * mech_angle) - zero_offset);
    theta_corr   = 0.0f;
    corr_pending = 0u;
}

/*补偿电角度*/
void FOC_CurrentTrackAngle(float mech_angle)
{
    float target = wrap_angle((foc.pn * mech_angle) - zero_offset);
    float err    = target - theta_e;

    /* 归一化到 [-pi, pi] */
    while (err > PI_F)
    {
        err -= TWO_PI;
    }
    while (err < -PI_F)
    {
        err += TWO_PI;
    }

    /* 每个 1 ms 只补 1/4，避免编码器量化跳变直接灌进电流环 */
    theta_corr   = 0.25f * err;
    corr_pending = 1u;
}

/*设置电角速度估计值*/
void FOC_CurrentSetWe(float we)
{
    we_est = we;
}

/*过流故障检测*/
uint8_t FOC_CurrentHasFault(void)
{
    return oc_fault;
}


/*清除过流故障*/
void FOC_CurrentClearFault(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    oc_fault = 0u;
    oc_count = 0u;
    __set_PRIMASK(primask);
}

/* 反 Park + 最小最大零序注入 */
static void current_modulate(float ud, float uq, float c, float s)
{
    float ua = (c * ud) - (s * uq);
    float ub = (s * ud) + (c * uq);

    /* 幅值不变反 Clarke */
    float pa = ua;
    float pb = ((SQRT3 * ub) - ua) * 0.5f;
    float pc = ((-ua) - (SQRT3 * ub)) * 0.5f;

    float mx = fmaxf(fmaxf(pa, pb), pc);
    float mn = fminf(fminf(pa, pb), pc);
    float offset = -0.5f * (mx + mn);
    float inv = 1.0f / foc.Vdc;

    float da = ((pa + offset) * inv) + 0.5f;
    float db = ((pb + offset) * inv) + 0.5f;
    float dc = ((pc + offset) * inv) + 0.5f;

    FOC_SetDuty(da, db, dc);
}

/*  电流环主体：由 ADC 中断在每个采样点调用（29.8 kHz） */

void FOC_CurrentIsr(void)
{
    float ia, ib, ic;
    float ialpha, ibeta, id, iq;
    float ed, eq;
    float ud, uq, ud_raw, uq_raw;
    float mag, scale;                   // 矢量幅值与限幅比例
    float c, s;
    FOC_LoopMode mode = loop_mode;

    if (mode == FOC_Loop_Off)
    {
        return;
    }

    ADC_Current_ReadIsr(&ia, &ib, &ic);

    /*过流保护：先于控制动作,过流时关断PWM输出*/
    if (oc_blank > 0u)
    {
        /* 刚切换模式：先让电流建立起来，这段时间不判过流 */
        oc_blank--;
        oc_count = 0u;
    }
    else if ((fabsf(ia) > foc.oc_limit) ||
             (fabsf(ib) > foc.oc_limit) ||
             (fabsf(ic) > foc.oc_limit))
    {
        if (oc_count < OC_Count)
        {
            oc_count++;
        }
        if (oc_count >= OC_Count)
        {
            oc_fault  = 1u;
            loop_mode = FOC_Loop_Off;
            FOC_DisableOutputs();
            return;
        }
    }
    else
    {
        oc_count = 0u;
    }

    /* ---- 对齐：固定电压矢量，不用编码器角度 ---- */
    if (mode == FOC_Loop_Align)
    {
        current_modulate(0.0f, foc.align_voltage, align_cos, align_sin);
        return;
    }

    /* ---- 电角度：积分外推 + 1 kHz 相位修正 ---- */
#if FIXED_FRAME_TEST
    theta_e = 0.0f;                 /* 【临时】固定坐标系，排除编码器影响 */
#else
    theta_e = wrap_angle(theta_e + (we_est * foc.Ts_cur));
    if (corr_pending != 0u)
    {
        theta_e      = wrap_angle(theta_e + theta_corr);
        corr_pending = 0u;
    }
#endif
    c = cosf(theta_e);
    s = sinf(theta_e);

    /* ---- Clarke（幅值不变） ---- */
    ialpha = 0.6666667f * (ia - (0.5f * ib) - (0.5f * ic));
    ibeta  = 0.5773503f * (ib - ic);

    /* ---- Park，与 FOC_SetPhaseVoltage 的约定配套 ---- */
    id = (c * ialpha) + (s * ibeta);
    iq = (-s * ialpha) + (c * ibeta);

    id_meas = id;
    iq_meas = iq;

    /* ---- d/q 电流 PI ---- */
    ed = 0.0f - id;              /* id_ref 恒为 0 */
    eq = iq_ref - iq;

    int_d += (foc.Kid * foc.Ts_cur) * ed;
    int_q += (foc.Kiq * foc.Ts_cur) * eq;

    /* ---- 解耦（bring-up 阶段可关，见 FOC_Params.h 的 DECOUPLE_ENABLE） ---- */
    ud_raw = (foc.Kpd * ed) + int_d;
    uq_raw = (foc.Kpq * eq) + int_q;
#if DECOUPLE_ENABLE
    ud_raw -= we_est * foc.lq * iq;
    uq_raw += we_est * ((foc.ld * id) + foc.psi_f);
#endif

#if FIXED_FRAME_TEST
    /* 【临时诊断】强制 ud=0，只闭 q 轴电流环。
     * 此时矢量必定落在 beta 轴 → A 相占空比必然 50%，
     * 用来判断问题是不是出在 d 轴环（d 轴积分器被偏置顶起）。 */
    ud_raw = 0.0f;
    int_d  = 0.0f;
#endif

    /* ---- 合成矢量限幅 ---- */
    ud = ud_raw;
    uq = uq_raw;
    mag = sqrtf((ud * ud) + (uq * uq));
    if (mag > foc.vmax)
    {
        scale = foc.vmax / mag;
        ud *= scale;
        uq *= scale;
    }

    /* ---- 抗饱和回算：把积分拉回可实现范围 ---- */
    int_d += foc.kaw_cur * (ud - ud_raw);
    int_q += foc.kaw_cur * (uq - uq_raw);

    current_modulate(ud, uq, c, s);
}