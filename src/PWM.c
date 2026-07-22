#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include <stdint.h>

void PWM_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);			
		
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);		
							
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);                                         //受外设控制的引脚，均需要配置为复用模式		
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);                                         //受外设控制的引脚，均需要配置为复用模式		

    
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;				//定义结构体变量
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStruct.TIM_Period = 10000;					//计数周期，即ARR的值
	TIM_TimeBaseInitStruct.TIM_Prescaler = 83;				//预分频器，即PSC的值
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);           //将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单元
	
	/*输出比较初始化*/
	TIM_OCInitTypeDef TIM_OCInitStruct;							//定义结构体变量
	TIM_OCStructInit(&TIM_OCInitStruct);
																	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;	//输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;								//初始的CCR值

	TIM_OC3Init(TIM1, &TIM_OCInitStruct);  // PE13 CH3
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);  // PE14 CH4

	/* TIM1 高级定时器需使能主输出 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);		

}

void PWM_SetCompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM1, Compare);
}

void PWM_SetCompare4(uint16_t Compare)
{
    TIM_SetCompare4(TIM1, Compare);
}

void Set_Speed(uint16_t Compare_left,uint16_t Compare_right)  //设置左右轮速度
{
	TIM_SetCompare3(TIM1, Compare_left);
	TIM_SetCompare4(TIM1, Compare_right);
}
