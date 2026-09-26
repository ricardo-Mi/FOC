#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>

/*
 * AS5600 磁编码器驱动 (由 SimpleFOC/DengFOC 的 Sensor_AS5600 类移植)
 *
 * SCL: PB13
 * SDA: PB12
 * 注: F407的PB12/PB13没有硬件I2C复用, 这里使用软件I2C
 */

void AS5600_Init(void);                 /* GPIO、软件I2C初始化, 并采样初始角度 */
void AS5600_Update(void);               /* 读取一次角度, 更新圈数与时间戳(需周期调用) */

float AS5600_GetSensorAngle(void);      /* 直接读寄存器, 返回机械角度 [0,2PI) */
float AS5600_GetMechanicalAngle(void);  /* 最近一次采样角度 [0,2PI) */
float AS5600_GetAngle(void);            /* 带圈数的总角度 (可累积多圈) */
float AS5600_GetVelocity(void);         /* 由两次采样差计算速度 rad/s */

#endif /* __AS5600_H */
