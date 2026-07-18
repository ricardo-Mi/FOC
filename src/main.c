#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "LED.h"
#include "Key.h"
#include <stdint.h>
#include "OLED.h"
#include "Timer.h"

uint16_t Num;			//定义在定时器中断里自增的变量

 int main(void)
 {
	OLED_Init();		
	Timer_Init();
	OLED_ShowString(1, 1, "Num:");			//1行1列显示字符串Num:
	
     while (1)
     {

		OLED_ShowNum(1, 5, Num, 5);			//不断刷新显示Num变量


     }
 }

 void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
		Num++;
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}