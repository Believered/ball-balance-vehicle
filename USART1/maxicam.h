#ifndef MAXICAM_H_
#define MAXICAM_H_

#include "ti_msp_dl_config.h"

// maxicam UART2 通信函数
void maxicam_send_byte(uint8_t data);
void maxicam_send_buf(const uint8_t *buf, uint16_t len);
void maxicam_send_str(const char *s);

// maxicam 接收缓冲区 (中断中填充)
extern volatile uint8_t maxicam_rx_buf[64];
extern volatile uint8_t maxicam_rx_len;
extern volatile uint8_t maxicam_rx_flag;  // 1=收到新数据

#endif
