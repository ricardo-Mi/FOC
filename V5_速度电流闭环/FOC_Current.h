#ifndef __FOC_CURRENT_H
#define __FOC_CURRENT_H

#include <stdint.h>

/* 电流环运行模式，由 FOC 状态机设置 */
typedef enum
{
    FOC_Loop_Off = 0,   /* 不输出，占空比保持中性（STOP/CAL/OFST/ERROR） */
    FOC_Loop_Align,     /* 固定电压矢量（电角度对齐）                    */
    FOC_Loop_Run        /* 完整 d/q 电流闭环                             */
} FOC_LoopMode;

void FOC_CurrentInit(void);
void FOC_CurrentReset(void);

void FOC_CurrentSetMode(FOC_LoopMode mode);
FOC_LoopMode FOC_CurrentGetMode(void);

/* q 轴电流给定；任务→中断安全 */
void FOC_CurrentSetIqRef(float iq);
float FOC_CurrentGetIqRef(void);
void FOC_CurrentGetIdq(float *id, float *iq);

/* 电角度 = pn * 机械角 - 零点 */
void FOC_CurrentSetZeroOffset(float zero);
void FOC_CurrentSyncFromEncoder(float mech_angle);   /* 进 RUN 时一次性对齐 */
void FOC_CurrentTrackAngle(float mech_angle);        /* 1 kHz 相位修正      */
void FOC_CurrentSetWe(float we);                      /* 电角速度，任务写    */

/* 在 ADC 中断末尾调用一次 */
void FOC_CurrentIsr(void);

uint8_t FOC_CurrentHasFault(void);
void FOC_CurrentClearFault(void);



#endif