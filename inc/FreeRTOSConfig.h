#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* System clock: 168MHz, tick: 1ms = 1000Hz */
#define configCPU_CLOCK_HZ                       ( 168000000UL )
#define configTICK_RATE_HZ                       ( ( TickType_t ) 1000 )

/* Max priorities and minimal stack */
#define configMAX_PRIORITIES                     ( 16 )
#define configMINIMAL_STACK_SIZE                 ( ( unsigned short ) 128 )

/* Heap size: 15KB */
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 15 * 1024 ) )

/* Task name max length */
#define configMAX_TASK_NAME_LEN                  ( 16 )

/* Scheduler: 1 = preemptive, 0 = cooperative */
#define configUSE_PREEMPTION                     1

/* Use 32-bit tick count */
#define configUSE_16_BIT_TICKS                   0

/* Idle task yields to other tasks at same priority */
#define configIDLE_SHOULD_YIELD                  1

/* Enable mutexes, recursive mutexes, counting semaphores */
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1

/* Enable task notifications */
#define configUSE_TASK_NOTIFICATIONS             1

/* FPU: 0=none, 1=manual save/restore, 2=lazy stacking (Cortex-M4F default) */
#define configUSE_TASK_FPU_SUPPORT               2

/* Enable queues, timers, event groups */
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( 128 )

#define configUSE_EVENT_GROUPS                   1
#define configUSE_STREAM_BUFFERS                 1
#define configUSE_MESSAGE_BUFFERS                1

/* Hook functions */
#define configCHECK_FOR_STACK_OVERFLOW           0
#define configUSE_MALLOC_FAILED_HOOK             0
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0

/* Optional functions - enable the ones you need */
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_xTaskAbortDelay                  1
#define INCLUDE_xTaskGetHandle                   1

/* Run time stats (off by default for smaller code) */
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

/* ARM Cortex-M4 interrupt priority (STM32F4: 4 bits, range 0..15) */
#define configPRIO_BITS                          4

/* Lowest priority: 15 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY        0x0F

/* Max syscall priority: 5 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5

/* NVIC shifted values */
#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* Map FreeRTOS ISR names to CMSIS names in startup file */
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler
#define vPortSVCHandler                          SVC_Handler

/* Assert */
#define configASSERT( x )                        if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

#endif /* FREERTOS_CONFIG_H */
