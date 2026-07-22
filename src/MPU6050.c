#include "stm32f4xx.h"                  // Device header
#include <stdint.h>
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "MyI2C.h"

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);       // 发送器件地址+写
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);            // 发送寄存器地址
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);                  // 发送数据
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}   

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);       // 发送器件地址+写
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);            // 发送寄存器地址
    MyI2C_ReceiveAck();

    MyI2C_Start();                         // 重复起始条件
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);// 发送器件地址+读
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();            // 接收数据
    MyI2C_SendAck(1);                      // 发送NACK
    MyI2C_Stop();
    
    return Data;
}


void MPU6050_Init(void)
{
    MyI2C_Init();

    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x01); // 唤醒MPU6050  电源管理寄存器1
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_2, 0x00); // 电源管理寄存器2
    MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x09); // 设置采样率分频器 10分频
    MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x06); // 配置寄存器 低通滤波器
    MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18); // 陀螺仪配置寄存器
    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x18); // 加速度计配置寄存器  
}


uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_RA_WHO_AM_I);
}

void MPU6050_GetData(int16_t* AccelX, int16_t* AccelY, int16_t* AccelZ, int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ)
{
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_RA_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_RA_ACCEL_XOUT_L);
    *AccelX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_RA_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_RA_ACCEL_YOUT_L);
    *AccelY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_RA_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_RA_ACCEL_ZOUT_L);
    *AccelZ = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_RA_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_RA_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_RA_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_RA_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_RA_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_RA_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;

}
