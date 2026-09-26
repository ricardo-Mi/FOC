// AS5600 磁编码器移植 (来自 SimpleFOC/DengFOC 的 Sensor_AS5600 类)
// 平台: STM32F407 + 标准外设库
// SCL: PB13  SDA: PB12 (软件I2C, F407的PB12/PB13没有硬件I2C复用)
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "AS5600.h"
#include "Delay.h"
#include <math.h>
#include <stdint.h>

#define _2PI 6.28318530718f

/* AS5600 I2C地址与寄存器 */
#define AS5600_ADDR        0x36      /* 7位从机地址 */
#define AS5600_ANGLE_REG   0x0C      /* 角度寄存器: 0x0C高4位 + 0x0D低8位, 共12位 */
#define AS5600_CPR         4096.0f   /* 12位分辨率 */

/* 编码器方向: 读数方向与电机正转方向一致填1, 相反填-1 */
#define AS5600_DIRECTION   (-1)

/* 引脚配置: 开漏输出 + 上拉 */
#define AS5600_W_SCL(x)    GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)(x))
#define AS5600_W_SDA(x)    GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)(x))
#define AS5600_R_SDA()     GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)

/* ==================== 角度处理状态 ==================== */
static float angle_prev = 0;            /* 最近一次机械角度 [0,2PI) */
static uint32_t angle_prev_ts = 0;      /* 最近一次采样时间戳(us) */
static float vel_angle_prev = 0;        /* 上次计算速度用的角度 */
static uint32_t vel_angle_prev_ts = 0;  /* 上次计算速度用的时间戳(us) */
static int32_t full_rotations = 0;      /* 累计圈数 */
static int32_t vel_full_rotations = 0;  /* 上次计算速度用的圈数 */

/* DWT 周期计数器实现 micros(), 与FOC.c中一致 */
static void AS5600_DWTInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能DWT */
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;          /* 使能周期计数器 */
    }
}

static uint32_t AS5600_micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* ==================== 软件I2C ==================== */

/* I2C开始 */
static void AS5600_I2C_Start(void)
{
    AS5600_W_SDA(1); Delay_us(2);
    AS5600_W_SCL(1); Delay_us(2);
    AS5600_W_SDA(0); Delay_us(2);
    AS5600_W_SCL(0); Delay_us(2);
}

/* I2C停止 */
static void AS5600_I2C_Stop(void)
{
    AS5600_W_SDA(0); Delay_us(2);
    AS5600_W_SCL(1); Delay_us(2);
    AS5600_W_SDA(1); Delay_us(2);
}

/* I2C发送一个字节(忽略从机应答) */
static void AS5600_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        AS5600_W_SDA(!!(Byte & (0x80 >> i))); Delay_us(2);
        AS5600_W_SCL(1); Delay_us(2);
        AS5600_W_SCL(0); Delay_us(2);
    }
    /* 第9个时钟, 读取从机应答 */
    AS5600_W_SDA(1); Delay_us(2);
    AS5600_W_SCL(1); Delay_us(2);
    AS5600_W_SCL(0); Delay_us(2);
}

/* I2C读取一个字节, ack=1时发送应答, ack=0时发送非应答 */
static uint8_t AS5600_I2C_ReadByte(uint8_t ack)
{
    uint8_t i, Byte = 0;

    AS5600_W_SDA(1);                  /* 释放SDA, 由从机驱动 */
    for (i = 0; i < 8; i++)
    {
        AS5600_W_SCL(1); Delay_us(2);
        Byte <<= 1;
        if (AS5600_R_SDA()) Byte |= 0x01;
        AS5600_W_SCL(0); Delay_us(2);
    }
    /* 第9个时钟发送应答 */
    AS5600_W_SDA(ack ? 0 : 1); Delay_us(2);
    AS5600_W_SCL(1); Delay_us(2);
    AS5600_W_SCL(0); Delay_us(2);
    AS5600_W_SDA(1);

    return Byte;
}

/* ==================== 传感器读取 ==================== */

/* 读取AS5600原始角度, 返回机械角度 [0,2PI) */
float AS5600_GetSensorAngle(void)
{
    uint8_t readArray[2];
    uint16_t readValue;

    AS5600_I2C_Start();
    AS5600_I2C_SendByte(AS5600_ADDR << 1);           /* 写地址 0x6C */
    AS5600_I2C_SendByte(AS5600_ANGLE_REG);           /* 角度寄存器 0x0C */
    AS5600_I2C_Start();                               /* 重复起始 */
    AS5600_I2C_SendByte((AS5600_ADDR << 1) | 0x01);  /* 读地址 0x6D */
    readArray[0] = AS5600_I2C_ReadByte(1);           /* 高字节, 应答 */
    readArray[1] = AS5600_I2C_ReadByte(0);           /* 低字节, 非应答 */
    AS5600_I2C_Stop();

    /* 12位原始角度: 高4位在0x0C低4位, 低8位在0x0D */
    readValue = (uint16_t)(((readArray[0] & 0x0F) << 8) | readArray[1]);

    float angle = (readValue / AS5600_CPR) * _2PI;

    /* 方向反转: 使编码器读数方向与电机正转方向一致, 同时保持[0,2PI) */
    if (AS5600_DIRECTION < 0)
    {
        angle = (angle > 0.0f) ? (_2PI - angle) : 0.0f;
    }

    return angle;
}

/* ==================== 角度处理 ==================== */

/* 初始化: 对应原代码 Sensor_init() */
void AS5600_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    AS5600_W_SCL(1);
    AS5600_W_SDA(1);

    AS5600_DWTInit();

    Delay_ms(500);                     /* 对应原代码 wire->begin() 后的 delay(500) */

    AS5600_GetSensorAngle();
    vel_angle_prev = AS5600_GetSensorAngle();
    vel_angle_prev_ts = AS5600_micros();
    Delay_ms(1);
    AS5600_GetSensorAngle();
    angle_prev = AS5600_GetSensorAngle();
    angle_prev_ts = AS5600_micros();
}

/* 周期调用: 对应原代码 Sensor_update() */
void AS5600_Update(void)
{
    float val = AS5600_GetSensorAngle();
    angle_prev_ts = AS5600_micros();

    float d_angle = val - angle_prev;

    /* 圈数检测: 角度跳变超过0.8圈说明跨过了0/2PI边界 */
    if (fabsf(d_angle) > (0.8f * _2PI))
    {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }

    angle_prev = val;
}

/* 最近一次采样角度 [0,2PI) */
float AS5600_GetMechanicalAngle(void)
{
    return angle_prev;
}

/* 带圈数的总角度 */
float AS5600_GetAngle(void)
{
    return (float)full_rotations * _2PI + angle_prev;
}

/* 由两次采样差计算速度 rad/s: 对应原代码 getVelocity() */
float AS5600_GetVelocity(void)
{
    /* 计算采样时间 */
    float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6f;

    /* 快速修复奇怪的情况(微溢出) */
    if (Ts <= 0) Ts = 1e-3f;

    /* 速度计算 */
    float vel = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;

    /* 保存变量以待将来使用 */
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;

    return vel;
}
