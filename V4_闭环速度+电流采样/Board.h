#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>

/* Board-level inputs that are not owned by a specific driver. */
void Board_Init(void);
uint8_t Board_GetFaultInput(void);

#endif /* __BOARD_H */
