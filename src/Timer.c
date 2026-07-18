#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // 输入滤波，不关心
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Prescaler = 83;     // 84M / (83+1) = 1MHz    
    TIM_TimeBaseStruct.TIM_Period = 999;    // 1M / (999+1) = 1kHz = 1ms
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;      // 基本定时器无此功能，写0
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;  // TIM6 和 DAC 共用中断线
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM6, ENABLE);

}

// void TIM6_DAC_IRQHandler(void)
// {
//     if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
//     {
//         TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//         // 1ms 定时操作
//     }
// }