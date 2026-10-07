#include "maxicam.h"

// 接收缓冲区
volatile uint8_t maxicam_rx_buf[64] = {0};
volatile uint8_t maxicam_rx_len = 0;
volatile uint8_t maxicam_rx_flag = 0;

void maxicam_send_byte(uint8_t data)
{
    while(DL_UART_isBusy(maxicam_INST) == true);
    DL_UART_Main_transmitData(maxicam_INST, data);
}

void maxicam_send_buf(const uint8_t *buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) {
        maxicam_send_byte(buf[i]);
    }
}

void maxicam_send_str(const char *s)
{
    while(*s) {
        maxicam_send_byte((uint8_t)*s++);
    }
}
