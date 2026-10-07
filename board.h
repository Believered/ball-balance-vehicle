/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-26     LCKFB     first version
 */
#ifndef	__BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"

void board_init(void);

void delay_us(unsigned long __us);
void delay_ms(unsigned long ms);
void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);

void uart0_send_char(char ch);
void uart0_send_string(char* str);

#define USART_REC_LEN		200					//最大接收缓存字节数
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define UART0_MAX_SEND_LEN		600					//最大发送缓存字节数


extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 		//接收缓冲,最大UART4_MAX_RECV_LEN字节

extern volatile uint16_t USART_RX_STA;  


extern uint8_t  UART0_TX_BUF[UART0_MAX_SEND_LEN]; 		//发送缓冲,最大UART4_MAX_SEND_LEN字节

#endif
