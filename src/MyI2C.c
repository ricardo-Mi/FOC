#include "stm32f4xx.h"                  // Device header
#include "Delay.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>

#define MYI2C_PORT     GPIOB
#define MYI2C_SDA_PIN  GPIO_Pin_14
#define MYI2C_SCL_PIN  GPIO_Pin_15

void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(MYI2C_PORT, MYI2C_SCL_PIN, (BitAction)BitValue);
    Delay_us(10);    
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(MYI2C_PORT, MYI2C_SDA_PIN, (BitAction)BitValue);
    Delay_us(10);    
}

uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue = GPIO_ReadInputDataBit(MYI2C_PORT, MYI2C_SDA_PIN);
    Delay_us(10);    
    return BitValue;
}

void MyI2C_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;     
    GPIO_InitStruct.GPIO_Pin = MYI2C_SDA_PIN | MYI2C_SCL_PIN;  // PB14:SDA PB15:SCL
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(MYI2C_PORT, &GPIO_InitStruct);

    GPIO_SetBits(MYI2C_PORT, MYI2C_SDA_PIN | MYI2C_SCL_PIN);  //高电平释放总线
}

void MyI2C_Start(void) //起始条件：SCL高电平期间，SDA由高电平变为低电平
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
} 

void MyI2C_Stop(void) //停止条件：SCL高电平期间，SDA由低电平变为高电平
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
    for (int i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> i)); //发送数据的最高位
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i,Byte = 0x00;
    MyI2C_W_SDA(1); //释放总线，准备接收数据
    for(i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);
        if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);} //接收数据的最高位
        MyI2C_W_SCL(0);
    }
    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit) //发送应答位
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void) //接收应答位
{
    uint8_t AckBit;
    MyI2C_W_SDA(1); //释放总线，准备接收
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    return AckBit;
}

