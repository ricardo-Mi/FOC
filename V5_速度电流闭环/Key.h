#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

/* PC0: motor start/stop key, active-low with internal pull-up. */
#define KEY_START_STOP_PORT GPIOC
#define KEY_START_STOP_PIN  GPIO_Pin_0

void Key_Init(void);
void vKeyTask(void *pvParameters);

#endif /* __KEY_H */
