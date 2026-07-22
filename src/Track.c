#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "Track.h"
#include "Delay.h"
#include "Motor.h"

extern volatile uint8_t track_enable;
extern volatile int32_t Target_Speed_left;
extern volatile int32_t Target_Speed_right;

void Track_Init()
{
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_3;

    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Speed_Set(int32_t left_speed, int32_t right_speed)
{
    Target_Speed_left  = left_speed;
    Target_Speed_right = right_speed;
}

void Track_task(void)
{


    if((L2==0 && L1==0 && M==1 && R1==0 && R2==0))  Speed_Set(2000, 2000);  //中间

    else if((L2==0 && L1==0 && M==1 && R1==1 && R2==0) ||
       (L2==0 && L1==0 && M==0 && R1==1 && R2==0))  Speed_Set(2300, 2000);  //小右偏

    else if((L2==0 && L1==1 && M==1 && R1==0 && R2==0) ||
       (L2==0 && L1==1 && M==0 && R1==1 && R2==0))  Speed_Set(2000, 2300);   //小左偏

    else if((L2==0 && L1==0 && M==0 && R1==0 && R2==1) ||
       (L2==0 && L1==0 && M==0 && R1==1 && R2==1))  Speed_Set(2600, 2000);  //大右偏

    else if((L2==1 && L1==0 && M==0 && R1==0 && R2==1) ||
       (L2==1 && L1==1 && M==0 && R1==1 && R2==1))  Speed_Set(2000, 2600);  //大左偏

    else if((L2==0 && L1==0 && M==1 && R1==1 && R2==1))      //右转
    {
        Speed_Set(2600, -2600);  
        Delay_ms(100);
    }

    else if((L2==1 && L1==1 && M==1 && R1==0 && R2==0))      //左转
    {
        Speed_Set(-2600, 2600);  
        Delay_ms(100);
    }

    else if ((L2==1 && L1==1 && M==1 && R1==1 && R2==1))     //停止
    {  
        track_enable = 0;
        Speed_Set(0,0);
        Motor_Stop();
        return;
    }

    else   Speed_Set(1000, 1000); 

}