#ifndef UASRT1_H_
#define UASRT1_H_

#include "board.h"

#define USART1_MAX_SEND_LEN 120U
extern uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN];

/* Bounded UART1 formatting. Call from one context: the TX buffer is shared. */
void u1_printf(char *fmt, ...);

#endif
