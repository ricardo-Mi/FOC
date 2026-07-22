#ifndef __TIMER_H
#define __TIMER_H

#define TIMER_CONTROL_FREQUENCY_HZ   100U
#define TIMER_CONTROL_PERIOD_SECONDS (1.0f / (float)TIMER_CONTROL_FREQUENCY_HZ)

void Timer_Init(void);

#endif



/*
 	       基本定时器	 通用定时器	           高级定时器
型号	   TIM6,TIM7	TIM25,TIM914	      TIM1, TIM8
位数	    16 位	  16/32 位(TIM2/5是32位)	 16 位
向上计数	   有	      有	                    有
向下计数	   无	      有	                    有
中央对齐	   无	      有 (TIM2~5)	            有
输出比较/ PWM  无	      有	                    有
输入捕获	   无	      有	                    有
编码器接口	   无	      有 (TIM2~5)	            有
刹车输入	   无	      无	                    有
死区插入	   无	      无	                    有（互补输出）
DMA           有	     有	                       有
时钟源	   内部 CK_INT	内部/外部	                 内部/外部
典型场景  简单定时中断、DAC触发	  PWM输出、输入捕获、编码器	   电机控制（6路互补PWM+刹车）

定时器类型	              所挂总线	  定时器时钟
TIM1, TIM8~11	            APB2	  168MHz
TIM27, TIM1214	            APB1	  84MHz


*/
