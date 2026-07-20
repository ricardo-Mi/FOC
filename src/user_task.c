#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "USART.h"
#include "queue.h"
#include "semphr.h"

static volatile uint32_t led_delay = 100;

extern QueueHandle_t g_usart1_rx_queue;
extern SemaphoreHandle_t g_key_semaphore;

typedef enum
{
    LED_MODE_BLINK = 0,
    LED_MODE_ON,
    LED_MODE_OFF
} LED_Mode_t;

static volatile LED_Mode_t current_led_mode = LED_MODE_BLINK;




void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}


static void LED_ON(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

static void LED_OFF(void)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

static void LED_turn(void)
{
    GPIOC->ODR ^= GPIO_Pin_13;
}


void LED_Task(void *pvParameters)
{
    (void)pvParameters;

    static uint8_t mode =0;

    while (1)
    {
        if(xSemaphoreTake(g_key_semaphore, portMAX_DELAY) == pdTRUE)
        {
            mode++;
            if(mode>3)
            {
                mode = 0;
            }
            switch (mode)
            {
                case 0:
                    LED_turn();
                    break;
                case 1:
                    LED_ON();
                    break;
                case 2:
                    LED_OFF();
                    break;
                default:
                    break;
            }
        }
    }
}


void Speed_Task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
      led_delay = 500;
      vTaskDelay(pdMS_TO_TICKS(3000));
      
      led_delay = 100;
      vTaskDelay(pdMS_TO_TICKS(3000));
    
    }
}

void USART_Command_Task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t received_byte;

    while (1)
    {
        if (xQueueReceive(g_usart1_rx_queue, &received_byte, portMAX_DELAY) == pdPASS)
        {
            if (received_byte == '\r' || received_byte == '\n')
            {
                continue; // 忽略回车和换行
            }
            
            USART1_SendString("Received: ");
            USART1_SendByte(received_byte);
            USART1_SendString("\r\n");

            switch (received_byte)
            {
                case 'f':
                    current_led_mode = LED_MODE_BLINK;
                    led_delay = 100;
                    USART1_SendString("LED Fast\r\n");
                    break;
                case 's':
                    current_led_mode = LED_MODE_BLINK;
                    led_delay = 500;
                    USART1_SendString("LED Slow\r\n");
                    break;
                case 'o':
                    current_led_mode = LED_MODE_ON;
                    USART1_SendString("LED ON\r\n");
                    break;
                case 'x':
                    current_led_mode = LED_MODE_OFF;
                    USART1_SendString("LED OFF\r\n");
                    break;
                default:
                    break;
            }
        }
    }
}
