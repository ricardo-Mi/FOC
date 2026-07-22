#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

#define MPU6050_ADDRESS 0xD0

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init(void);
void MPU6050_GetData(int16_t* AccelX, int16_t* AccelY, int16_t* AccelZ, int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ);
uint8_t MPU6050_GetID(void);

#endif // __MPU6050_H