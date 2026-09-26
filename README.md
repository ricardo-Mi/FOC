# FOC

基于 STM32F407ZGT6 + 标准外设库 + FreeRTOS 的 FOC 学习工程。

仓库只保存每个版本的 `.c/.h` 源码快照(完整工程保留在本地,不上传)。

## 版本

- `V1_开环速度FOC`: TIM1三路PWM开环速度控制 + AS5600编码器 + OLED显示 + PC0按键启停
  - 含 `FOC.c/h`、`AS5600.c/h`、`main.c`、`OLED.c`、`Key.c`
