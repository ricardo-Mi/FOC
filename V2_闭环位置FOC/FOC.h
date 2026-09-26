#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>

/*
 * FOC 闭环位置控制 (由 Arduino/DengFOC 代码移植)
 *
 * PWM输出引脚: PA9  -> TIM1_CH2 (A相)
 *              PE13 -> TIM1_CH3 (B相)
 *              PE14 -> TIM1_CH4 (C相)
 * 使能引脚:    PA10 (高电平使能)
 * 编码器:      AS5600 (SCL=PB13, SDA=PB12)
 */

extern float voltage_power_supply;      /* 电源电压 */
extern float target_angle;              /* 位置闭环目标角度 rad (先写死验证) */
extern float shaft_angle;               /* 机械角度(开环用) */
extern volatile uint8_t foc_running;    /* 电机启停标志: 1运行 0停止(PC0按键切换) */
extern volatile uint8_t foc_calibrated; /* 校准完成标志: 0=F(校准中) 1=T(完成) */
extern float zero_electric_angle;       /* 电角度零点 */
extern float Ualpha, Ubeta, Ua, Ub, Uc; /* 变换中间量 */
extern float dc_a, dc_b, dc_c;          /* 三相占空比 0~1 */

void FOC_Init(void);                    /* PWM、使能脚、DWT计时初始化 */
void FOC_SetEnable(uint8_t enable);     /* 1: PA10输出高(使能) 0: 输出低 */
void FOC_AlignSensor(void);             /* 电角度零点校准(电机需空载, 约3秒) */

float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
float FOC_ElectricalAngle(void);        /* 由编码器实测角度算电角度 */
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
void setTorque(float Uq, float angle_el);
float velocityOpenloop(float target_velocity);
float positionClosedLoop(float target);

void vFOCPositionTask(void *pvParameters);

#endif /* __FOC_H */
