#include "stm32f4xx.h"

static void Delay_DWT_Init(void)
{
	static uint8_t initialized = 0;
	if (!initialized)
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	/* 使能DWT */
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;			/* 使能周期计数器 */
		initialized = 1;
	}
}

/* 使用DWT周期计数器延时，不占用SysTick（FreeRTOS的节拍靠SysTick） */
void Delay_us(uint32_t xus)
{
	uint32_t start, ticks, guard;

	Delay_DWT_Init();

	start = DWT->CYCCNT;
	ticks = xus * (SystemCoreClock / 1000000U);
	guard = ticks * 2U;					/* 防止DWT不计数时死循环 */
	while ((DWT->CYCCNT - start) < ticks)
	{
		if (guard-- == 0U) break;
	}
}

void Delay_ms(uint32_t xms)
{
	while (xms--)
	{
		Delay_us(1000);
	}
}

void Delay_s(uint32_t xs)
{
	while (xs--)
	{
		Delay_ms(1000);
	}
}