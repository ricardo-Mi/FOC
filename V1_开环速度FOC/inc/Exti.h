#ifndef __EXTI_H
#define __EXTI_H

#include <stdint.h>
void  Exti_Init();
uint16_t Count_Get();


#endif

/*
F407 有 23 个 EXTI 线（EXTI0 ~ EXTI22），每个线可以选一个 GPIO 口作为中断源：
GPIO Pin x ──→ EXTI 线 x ──→ 边沿检测 ──→ 挂起标志 ──→ NVIC
   │              (0~15)       ↑↓/↑/↓         │
   └── 通过 SYSCFG 选择           └── 可选屏蔽

EXTI 线	  可选的引脚	                                           说明
EXTI0	  PA0 / PB0 / PC0 / PD0 / PE0 / PF0 / PG0 / PH0 / PI0	  所有 Px0
EXTI1	  PA1 / PB1 / PC1 / ... / PI1	所有 Px1
...	...	 
EXTI15	  PA15 / PB15 / ... / PI15	                              所有 Px15

23 个 EXTI 线只占 7 条 NVIC 中断线：

NVIC 中断线	        包含的 EXTI 线
EXTI0_IRQn	       EXTI0
EXTI1_IRQn	       EXTI1
EXTI2_IRQn	       EXTI2
EXTI3_IRQn	       EXTI3
EXTI4_IRQn	       EXTI4
EXTI9_5_IRQn	   EXTI5 ~ EXTI9 共享
EXTI15_10_IRQn	   EXTI10 ~ EXTI15 共享


*/

