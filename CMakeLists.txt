cmake_minimum_required(VERSION 3.20)

project(STM32F407VGT6_Project C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TARGET_NAME stm32f407vgt6_app)

# MCU 参数 (Cortex-M4 + FPU)
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

add_compile_definitions(
    STM32F40_41xxx
    USE_STDPERIPH_DRIVER
)

add_compile_options(
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-braces
    -ffunction-sections
    -fdata-sections
    -fno-common
    -g
    -O0
)

include_directories(
    inc
    Drivers
    Drivers/CMSIS/CM4/CoreSupport
    Drivers/CMSIS/CM4/DeviceSupport/ST/STM32F4xx
    Drivers/STM32F4xx_StdPeriph_Driver/inc
    FreeRTOS/include
    FreeRTOS/portable/GCC/ARM_CM4F
    FreeRTOS/portable/MemMang
)

file(GLOB USER_SOURCES
    src/*.c
)

file(GLOB STD_PERIPH_SOURCES
    Drivers/STM32F4xx_StdPeriph_Driver/src/*.c
)

set(FREERTOS_SOURCES
    FreeRTOS/Source/tasks.c
    FreeRTOS/Source/list.c
    FreeRTOS/Source/queue.c
    FreeRTOS/Source/timers.c
    FreeRTOS/Source/event_groups.c
    FreeRTOS/Source/stream_buffer.c
    FreeRTOS/portable/GCC/ARM_CM4F/port.c
    FreeRTOS/portable/MemMang/heap_4.c
)

set(STARTUP_FILE
    src/startup_stm32f40_41xxx.s
)

add_executable(${TARGET_NAME}.elf
    ${USER_SOURCES}
    ${STD_PERIPH_SOURCES}
    ${FREERTOS_SOURCES}
    ${CMSIS_SOURCES}
    ${STARTUP_FILE}
    inc/EXTI.h
    inc/PWM.h
    inc/FOC.h
    inc/AS5600.h
    inc/USART.h
    inc/user_task.h
    inc/Delay.h
    Drivers/Key_device.h
    Drivers/Key_device.c
    inc/MPU6050.h
    inc/MPU6050_Reg.h
    src/AS5600.c
    inc/AS5600.h
)

set(LINKER_SCRIPT
    ${CMAKE_SOURCE_DIR}/linker/STM32F407VGTx_FLASH.ld
)

target_link_options(${TARGET_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,-Map=${TARGET_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -specs=nano.specs
)

target_link_libraries(${TARGET_NAME}.elf PRIVATE
    c
    m
    gcc
)

add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}.elf> ${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}.elf> ${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}.elf>
    COMMENT "Generating HEX and BIN files"
)
