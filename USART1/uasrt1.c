#include "uasrt1.h"
#include <stdarg.h>
#include "stdio.h"
#include "string.h"

uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN];

void u1_printf(char* fmt,...)
{
	uint16_t i,j;
	va_list ap;
	if (fmt == NULL) return;
	va_start(ap,fmt);
	int written = vsnprintf((char*)USART1_TX_BUF, sizeof(USART1_TX_BUF), fmt, ap);
	va_end(ap);
	if (written < 0) return;
	i=strlen((const char*)USART1_TX_BUF);
	for(j=0;j<i;j++)
	{
        while(DL_UART_isBusy(UART_1_pid_INST) == true);
        DL_UART_Main_transmitData(UART_1_pid_INST, USART1_TX_BUF[j]);
	}
}
