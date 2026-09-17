#ifndef __MPU6050_H
#define __MPU6050_H

#include "stdint.h"


typedef struct
{
    float AccX;
    float AccY;
    float AccZ;
    float GyroX;
    float GyroY;
    float GyroZ;
}MPUData_t;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
void MPU6050_CalibrateGyro(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(float *AccX, float *AccY, float *AccZ, 
						float *GyroX, float *GyroY, float *GyroZ,float *raw,float *pitch,float *roll);
void MPU6050_W_out(void);
void vMPU6050task(void *pvParameters);
void SysTick_Handler(void);
void SysTick_Init(void);

extern uint32_t A;
extern float debug_dt;

#endif
