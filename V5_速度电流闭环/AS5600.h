#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>

/*
 * AS5600 magnetic encoder (software I2C).
 *   SCL: PB13
 *   SDA: PB12
 *
 * PB12/PB13 are not connected to a hardware I2C peripheral on the F407,
 * so the driver bit-bangs the bus.
 */

void AS5600_Init(void);
void AS5600_Update(void);

float AS5600_GetSensorAngle(void);      /* direct register read, [0, 2PI) */
float AS5600_GetMechanicalAngle(void);  /* latest sampled angle, [0, 2PI) */
float AS5600_GetAngle(void);            /* multi-turn angle */
float AS5600_GetVelocity(void);         /* mechanical rad/s */

#endif /* __AS5600_H */
