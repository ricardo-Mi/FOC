#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MPU6050_Reg.h"
#include "Delay.h"
#include "math.h"
#include "MPU6050.h"

extern QueueHandle_t xMaixbox;	//MPU6050数据邮箱

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

/*软件I2C引脚定义 PB15=SCL, PB14=SDA*/
#define MPU_W_SCL(x)	GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)(x))
#define MPU_W_SDA(x)	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)(x))
#define MPU_R_SDA()		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)

float gx, gy, gz;

static float GyroOffsetX = 0, GyroOffsetY = 0, GyroOffsetZ = 0;	// 陀螺仪零偏校准值

float debug_dt = 0;		// 调试用：最近一次积分的时间间隔（秒）
float dt_debug = 0;		// 调试用：同上，供OLED显示

/**
  * 函    数：MPU6050 I2C引脚初始化
  * 参    数：无
  * 返 回 值：无
  */
static void MPU6050_I2C_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		//开启GPIOB的时钟（F4中GPIO在AHB1总线上）

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;				//开漏输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;				//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;					//PB15 SCL
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;					//PB14 SDA
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	MPU_W_SCL(1);
	MPU_W_SDA(1);
}

/**
  * 函    数：I2C起始条件
  * 参    数：无
  * 返 回 值：无
  */
static void MPU6050_I2C_Start(void)
{
	MPU_W_SDA(1);
	MPU_W_SCL(1);
	MPU_W_SDA(0);
	MPU_W_SCL(0);
}

/**
  * 函    数：I2C停止条件
  * 参    数：无
  * 返 回 值：无
  */
static void MPU6050_I2C_Stop(void)
{
	MPU_W_SDA(0);
	MPU_W_SCL(1);
	MPU_W_SDA(1);
}

/**
  * 函    数：I2C发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
static void MPU6050_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		MPU_W_SDA(!!(Byte & (0x80 >> i)));
		Delay_us(5);
		MPU_W_SCL(1);
		Delay_us(5);
		MPU_W_SCL(0);
		Delay_us(5);
	}
	MPU_W_SCL(1);	//额外一个时钟，用于从机应答
	Delay_us(5);
	MPU_W_SCL(0);
	Delay_us(5);
}

/**
  * 函    数：I2C接收一个字节
  * 参    数：Ack 0=发送NACK(最后一个字节), 1=发送ACK
  * 返 回 值：接收到的字节
  */
static uint8_t MPU6050_I2C_ReceiveByte(uint8_t Ack)
{
	uint8_t i, Byte = 0;
	MPU_W_SDA(1);				//释放SDA，让从机控制
	Delay_us(5);
	for (i = 0; i < 8; i++)
	{
		MPU_W_SCL(1);
		Delay_us(5);
		if (MPU_R_SDA())
			Byte |= (0x80 >> i);
		MPU_W_SCL(0);
		Delay_us(5);
	}
	MPU_W_SDA(Ack ? 0 : 1);	//发送ACK或NACK
	Delay_us(5);
	MPU_W_SCL(1);
	Delay_us(5);
	MPU_W_SCL(0);
	Delay_us(5);
	return Byte;
}

/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址
  * 参    数：Data 要写入寄存器的数据
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MPU6050_I2C_Start();
	MPU6050_I2C_SendByte(MPU6050_ADDRESS);		//从机地址，方向为写
	MPU6050_I2C_SendByte(RegAddress);			//寄存器地址
	MPU6050_I2C_SendByte(Data);				//数据
	MPU6050_I2C_Stop();
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址
  * 返 回 值：读取寄存器的数据
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;

	MPU6050_I2C_Start();
	MPU6050_I2C_SendByte(MPU6050_ADDRESS);		//从机地址，方向为写
	MPU6050_I2C_SendByte(RegAddress);			//寄存器地址
	MPU6050_I2C_Start();						//重复起始条件
	MPU6050_I2C_SendByte(MPU6050_ADDRESS | 0x01);	//从机地址，方向为读
	Data = MPU6050_I2C_ReceiveByte(0);			//接收数据，发送NACK（单字节读取）
	MPU6050_I2C_Stop();

	return Data;
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	MPU6050_I2C_Init();							//软件I2C引脚初始化

	/*MPU6050寄存器初始化*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);	//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);	//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);	// 200Hz
	MPU6050_WriteReg(MPU6050_CONFIG, 0x03);		// DLPF=3, BW=42Hz
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);	// ±250°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050陀螺仪校准
  * 参    数：无
  * 返 回 值：无
  * 说    明：开机时调用，模块必须保持静止，采样500次取平均作为零偏
  */
void MPU6050_CalibrateGyro(void)
{
	uint16_t i;
	int16_t rawData;
	uint8_t DataH, DataL;
	float sumX = 0, sumY = 0, sumZ = 0;

	for (i = 0; i < 500; i++)
	{
		DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
		DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
		rawData = (int16_t)((DataH << 8) | DataL);
		sumX += rawData;

		DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
		DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
		rawData = (int16_t)((DataH << 8) | DataL);
		sumY += rawData;

		DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
		DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
		rawData = (int16_t)((DataH << 8) | DataL);
		sumZ += rawData;
	}

	/* 取平均值并转换为°/s */
	GyroOffsetX = (sumX / 500.0f) / 32768.0f * 250.0f;
	GyroOffsetY = (sumY / 500.0f) / 32768.0f * 250.0f;
	GyroOffsetZ = (sumZ / 500.0f) / 32768.0f * 250.0f;
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，输出参数，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，输出参数，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(float *AccX, float *AccY, float *AccZ,
						float *GyroX, float *GyroY, float *GyroZ,float *raw,float *pitch,float *roll)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	int16_t rawData;									//定义16位有符号原始数据

	/* 读取加速度计X轴原始数据 */
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	rawData = (int16_t)((DataH << 8) | DataL);			//拼接为有符号16位，范围-32768~32767
	*AccX = (rawData / 32768.0f) * 16.0f;				//转换为g单位，满量程±16g

	/* 读取加速度计Y轴原始数据 */
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	rawData = (int16_t)((DataH << 8) | DataL);
	*AccY = (rawData / 32768.0f) * 16.0f;

	/* 读取加速度计Z轴原始数据 */
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	rawData = (int16_t)((DataH << 8) | DataL);
	*AccZ = (rawData / 32768.0f) * 16.0f;

	/* 读取陀螺仪X轴原始数据 */
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	rawData = (int16_t)((DataH << 8) | DataL);
	*GyroX = (rawData / 32768.0f) * 250.0f - GyroOffsetX;	//转换为°/s并减去零偏

	/* 读取陀螺仪Y轴原始数据 */
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	rawData = (int16_t)((DataH << 8) | DataL);
	*GyroY = (rawData / 32768.0f) * 250.0f - GyroOffsetY;

	/* 读取陀螺仪Z轴原始数据 */
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	rawData = (int16_t)((DataH << 8) | DataL);
	*GyroZ = (rawData / 32768.0f) * 250.0f - GyroOffsetZ;


		/* 使用DWT硬件周期计数器计算dt，不依赖SysTick（SysTick会被Delay_us反复重置） */
		static uint32_t lastCycles = 0;
		static uint8_t dwtInit = 0;
		if (!dwtInit)
		{
			CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	// 使能DWT
			DWT->CYCCNT = 0;
			DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;				// 使能周期计数器
			dwtInit = 1;
		}
		uint32_t now_cycles = DWT->CYCCNT;
		if (lastCycles == 0) { lastCycles = now_cycles; }
		else {
			float dt = (now_cycles - lastCycles) / (float)SystemCoreClock;	// dt单位：秒
			lastCycles = now_cycles;
			dt_debug = dt;		// 输出到全局变量，供OLED显示
			debug_dt = dt;
			/* 死区滤波：小于0.5°/s的陀螺仪读数视为噪声，不积分 */
			if (*GyroZ > 0.5f || *GyroZ < -0.5f)
			{
				*roll = *roll + *GyroZ * dt*3;	// 偏航角 = 陀螺仪Z轴积分
			}
		}

		/*重力G解算pitch,roll*/
		*raw = atan2(*AccX,*AccZ)/3.1415917*180;
		*pitch  = atan2(*AccY,*AccZ)/3.1415917*180;

	//	/*互补滤波*/
	//	*roll   = *raw   + *GyroZ * 0.005;
	//	*pitch = (*pitch + *GyroY * 0.005 )* 0.95238 + (1-0.95238) * atan2(*AccX,*AccZ)/3.1415917*180;
	//	*raw  = (*roll  + *GyroX * 0.005 )* 0.95238 + (1-0.95238) * atan2(*AccY,*AccZ)/3.1415917*180;
}

void vMPU6050task(void *pvParameters)
{
    MPUData_t data;
	float raw = 0.0f, pitch = 0.0f, roll = 0.0f;
    (void)pvParameters;

    /* 初始化与校准已放到 main() 里（启动调度器之前）执行 */

    while(1)
    {
        MPU6050_GetData(&data.AccX, &data.AccY, &data.AccZ,
                        &data.GyroX, &data.GyroY, &data.GyroZ,
                        &raw, &pitch, &roll);

        xQueueOverwrite(xMaixbox, &data);	//写入邮箱（长度1，永远覆盖最新值）

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
