#ifndef __FOC_PARAMS_H
#define __FOC_PARAMS_H

#include <stdint.h>


/* ---- 电机本体参数 ---- */
#define Pn               7.0f         /* 极对数 12N14P                  */
#define R                5.1f         /* 相电阻 ohm，实测 3V/0.6A ≈ 5 Ω      */
#define Ld               1.0e-3f      /* d 轴电感 H（待实测）            */
#define Lq               1.0e-3f      /* q 轴电感 H（表贴式 ≈ Ld）       */
#define PSI_F            0.00358f     /* 永磁磁链 Wb（由 KV=220 推）     */
#define J                2.0e-5f      /* 转动惯量 kg*m^2
                                       * 原 1e-6 会让速度环 Kp 小到 0.0004 A/(rad/s)，
                                       * 15 rad/s 误差只给 0.006 A，连静摩擦都克服不了。
                                       * 5e-5 又会在本电机上产生约 10 Hz 极限环；
                                       * 由 iq≈0.1 A 时 α≈200 rad/s^2 反推 J≈2e-5。 */
#define B                0.0f         /* 粘性阻尼 N*m*s/rad              */

/* ---- 电气与采样 ---- */
#define VDC              12.6f        /* 母线电压 V                      */
#define Ts_Cur           33.5e-6f     /* 电流环周期 s（29.8 kHz）        */
#define Ts_Spd           1.0e-3f      /* 速度环周期 s（1 kHz）           */

/* ---- 带宽 rad/s ----
 * alpha：电流环闭环极点，上界受采样率/电流纹波限制，下界 5*beta
 * beta ：速度环闭环极点，上界受速度反馈低通 1/(2*Tf) 限制
 */
#define Alpha            200.0f
#define Beta             12.0f
#define VEL_LPF_Tf       0.030f       /* 速度反馈一阶低通，s             */

/* 反电势/交叉解耦开关。
 * bring-up 阶段先关掉：低速(<20 rad/s)时解耦项只有 0.1~0.4 V，贡献很小，
 * 但它是 we_est 异常时唯一能把电压指令顶到限幅的通路。
 * 速度上到 50 rad/s 以上再打开。 */
#define DECOUPLE_ENABLE  0

/* ---- 限幅与保护 ---- */
#define Iq_Max           1.0f         /* q 轴电流限幅 A。额定 0.5 A，最大 2 A；
                                       * 起动要克服齿槽，给到 1 A（短时）      */
#define V_Max            (0.72f * VDC / 1.7320508f)  /* 5.24 V
                                       * 占空比上限 0.86，与触发点 ARR-35 重合：
                                       * 下管窗口 86%~100%，ADC 84 周期(4us) 在
                                       * 98% 前完成，采样成立。 */
#define OC_Limit         2.0f         /* 相电流过流阈值 A。额定 0.5 A、短时 1 A，
                                       * 留一倍余量；原诊断值 6 A 太大，已收回。 */
#define OC_Count         6u           /* 连续超限次数（约 0.2 ms）       */
#define ALIGN_Vol        3.0f         /* 对齐电压 V（3.0/5.1 ≈ 0.59 A），与 V4 一致：
                                       * 电流小了对齐力矩不足，零点会被齿槽带偏 */
/* 对齐时 ud=0、uq=ALIGN_Vol，所以实际合成电压矢量在
 * ALIGN_Angle + 90° = 0°（alpha 轴）。这里保持 1.5π 是为了让
 * RUN 的 theta_e=0 对应 q 轴，不要随意改成 0。 */
#define ALIGN_Angle      (1.5f * 3.14159265f)

/* ---- 调制 ---- */
#define Duty_Min         0.02f
#define Duty_Max         0.86f        /* 对应 ARR-35 的 ADC 采样窗口     */

/* ---- 抗饱和回算系数（每周期） ---- */
#define Kaw_Cur          (Ts_Cur * Alpha)
#define Kaw_Spd          (Ts_Spd * Beta)

/* ==========================================================================
 * 【临时诊断】固定电角度 + 固定 iq 测试
 *   1 = RUN 状态下强制 theta_e = 0、iq* = FIXED_FRAME_IQ，
 *       完全绕过编码器/速度环，只验证"电流 → 力矩"通路。
 *   预期：转子跳约 90° 电角（12.9° 机械）后停住，并保持较强锁紧力；
 *         占空比应接近 A≈50%、B/C 对称（约 68/32）。
 *   验证完改回 0。
 * ========================================================================== */
#define FIXED_FRAME_TEST  0
#define FIXED_FRAME_IQ    0.5f

/* ==========================================================================
 * 【临时诊断】开环旋转测试
 *   1 = RUN 状态下不再使用速度环/编码器角度跟踪，改为固定 iq* 并让电角度
 *       以 OPENLOOP_WE 匀速旋转。用来单独验证：
 *         1) 功率级能否让电机连续转动；
 *         2) 编码器读数方向与相序是否一致；
 *         3) 实际极对数是否与 Pn 一致（机械转速 = OPENLOOP_WE / Pn）。
 *   验证完改回 0。
 * ========================================================================== */
#define OPENLOOP_TEST     0
#define OPENLOOP_WE       2.0f          /* 电角速度 rad/s，慢速准静态测试 */
#define OPENLOOP_IQ       0.5f

#if (FIXED_FRAME_TEST != 0) && (OPENLOOP_TEST != 0)
#error "FIXED_FRAME_TEST and OPENLOOP_TEST cannot be enabled together"
#endif

typedef struct
{
    float pn, r, ld, lq, psi_f, Kt, j, b;
    float Vdc, Ts_cur, Ts_spd;
    float alpha, beta;
    float Kpd, Kid, Kpq, Kiq;
    float Kpw, Kiw, Ba;
    float iq_max, vmax, kaw_cur, kaw_spd;
    float align_voltage, align_angle, oc_limit;
}FOC_Params_t;

extern FOC_Params_t foc;

void FOC_ParamsInit(void);

#endif