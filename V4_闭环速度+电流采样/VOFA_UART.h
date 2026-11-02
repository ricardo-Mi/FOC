#ifndef __VOFA_UART_H
#define __VOFA_UART_H

#include <stdint.h>

void VOFA_UART_Init(void);
void VOFA_UART_Send(const uint8_t *data, uint16_t length);

#endif /* __VOFA_UART_H */
