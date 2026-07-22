#include "FreeRTOS.h"
#include "stm32f4xx_gpio.h"
#include "task.h"
#include "stm32f4xx.h"
#include "LED.h"
#include "Key.h"
#include <stdint.h>
#include "OLED.h"
#include "Timer.h"
#include "Exti.h"
#include "PWM.h"
#include "task.h"
#include "user_task.h"
#include "USART.h"
#include "queue.h"
#include "semphr.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include <stdbool.h>
#include "Track.h"

uint8_t Key_Num1;
uint8_t Key_Num2;
volatile uint8_t track_enable = 0;

volatile int32_t Speed_left;
volatile int32_t Speed_right;
volatile int32_t Target_Speed_left;
volatile int32_t Target_Speed_right;
volatile int32_t Motor_Output_left;
volatile int32_t Motor_Output_right;




int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    Key_Init();
    OLED_Init();
    PWM_Init();
    Motor_Init();
    Encoder_Init_left();
    Encoder_Init_right();
    Timer_Init();
    Track_Init();

    OLED_ShowString(1, 1, "LS:");		//1行1列显示字符串Speed左轮速度:
	OLED_ShowString(2, 1, "RS:");		//2行2列显示字符串Speed右轮速度:

    while (1)
    {
        OLED_ShowSignedNum(1, 4, Target_Speed_left, 5);
        OLED_ShowSignedNum(2, 4, Target_Speed_right, 5);
        OLED_ShowSignedNum(1, 10, Motor_Output_left, 5);
        OLED_ShowSignedNum(2, 10, Motor_Output_right, 5);
    
        OLED_ShowNum(3, 1, L2, 1);
        OLED_ShowNum(3, 3, L1, 1);
        OLED_ShowNum(3, 5, M, 1);
        OLED_ShowNum(3, 7, R1, 1);
        OLED_ShowNum(3, 9, R2, 1);


        Key_Num1 = Key_GetNum1();
        Key_Num2 = Key_GetNum2();

        if(Key_Num1 == 1)
        {
            track_enable = !track_enable;
        }

        if(track_enable == 1)
        {
            Track_task();
        }
        else
        {
            Motor_Stop();
            Speed_Set(0,0);

        }


    }
}


 void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        int16_t motor_left;
        int16_t motor_right;

        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        if(track_enable == 0)
        {
            Motor_Output_left = 0;
            Motor_Output_right = 0;
            Motor_Stop();
            return;
        }

		Speed_left = Encoder_Get_left();
        Speed_right = Encoder_Get_right();


        motor_left = Left_PID_Calculate(Speed_left, Target_Speed_left);
        motor_right = Right_PID_Calculate(Speed_right,Target_Speed_right);
        Motor_Output_left = motor_left;
        Motor_Output_right = motor_right;

        Motor_Speed(motor_left, motor_right);

    }
}





























 /* ==================== FreeRTOS 钩子 ==================== */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    while (1)
    {
        /*
         * 内存申请失败会进入这里。
         * 一般是 configTOTAL_HEAP_SIZE 太小，或者任务栈分配太大。
         */
    }


}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();

    while (1)
    {
        /*
         * 任务栈溢出会进入这里。
         * 一般是某个任务栈给小了，比如 xTaskCreate 里的 128 不够。
         */
    }

}
