#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

/******************** 设备地址 ********************/
#define MPU6050_ADDR_WRITE        0xD0
#define MPU6050_ADDR_READ         0xD1

/******************** 采样率分频 ********************/
#define MPU6050_RA_SMPLRT_DIV     0x19

/******************** 配置寄存器 ********************/
#define MPU6050_RA_CONFIG         0x1A
#define MPU6050_RA_GYRO_CONFIG    0x1B
#define MPU6050_RA_ACCEL_CONFIG   0x1C

/******************** 加速度计测量值 ********************/
#define MPU6050_RA_ACCEL_XOUT_H   0x3B
#define MPU6050_RA_ACCEL_XOUT_L   0x3C
#define MPU6050_RA_ACCEL_YOUT_H   0x3D
#define MPU6050_RA_ACCEL_YOUT_L   0x3E
#define MPU6050_RA_ACCEL_ZOUT_H   0x3F
#define MPU6050_RA_ACCEL_ZOUT_L   0x40

/******************** 温度测量值 ********************/
#define MPU6050_RA_TEMP_OUT_H     0x41
#define MPU6050_RA_TEMP_OUT_L     0x42

/******************** 陀螺仪测量值 ********************/
#define MPU6050_RA_GYRO_XOUT_H    0x43
#define MPU6050_RA_GYRO_XOUT_L    0x44
#define MPU6050_RA_GYRO_YOUT_H    0x45
#define MPU6050_RA_GYRO_YOUT_L    0x46
#define MPU6050_RA_GYRO_ZOUT_H    0x47
#define MPU6050_RA_GYRO_ZOUT_L    0x48

/******************** 电源管理 ********************/
#define MPU6050_RA_PWR_MGMT_1     0x6B
#define MPU6050_RA_PWR_MGMT_2     0x6C

/******************** WHO_AM_I ********************/
#define MPU6050_RA_WHO_AM_I       0x75

/******************** 中断使能 ********************/
#define MPU6050_RA_INT_ENABLE     0x38
#define MPU6050_RA_INT_STATUS     0x3A

/******************** 用户控制 ********************/
#define MPU6050_RA_USER_CTRL      0x6A

/******************** FIFO ********************/
#define MPU6050_RA_FIFO_EN        0x23
#define MPU6050_RA_FIFO_COUNT_H   0x72
#define MPU6050_RA_FIFO_R_W       0x74

#endif /* __MPU6050_REG_H */
