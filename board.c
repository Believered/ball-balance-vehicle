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
#include "board.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"

#define RE_0_BUFF_LEN_MAX	128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;

volatile unsigned char uart_data = 0;

 





void board_init(void)
{

	SYSCFG_DL_init();

	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);

	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	
	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}


void delay_us(unsigned long __us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;

    ticks = __us * (CPUCLK_FREQ / 1000000);  // 动态获取主频

    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            if (tcnt >= ticks)
                break;
        }
    }
}

void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }


void uart0_send_char(char ch)
{

	while( DL_UART_isBusy(UART_0_INST) == true );

	DL_UART_Main_transmitData(UART_0_INST, ch);

}

void uart0_send_string(char* str)
{

	while(str != NULL && *str != 0)
	{

		uart0_send_char(*str++);
	}
}


/* Legacy ARMCC stream definitions do not apply to TI Clang. */
#if defined(__CC_ARM) && !defined(__MICROLIB)

#if (__ARMCLIB_VERSION <= 6000000)

struct __FILE
{
	int handle;
};
#endif

FILE __stdout;


void _sys_exit(int x)
{
	x = x;
}
#endif



int fputc(int ch, FILE *stream)
{

	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}


void UART_0_INST_IRQHandler2(void)
{
	uint8_t receivedData = 0;
   
   uint8_t Res;
   
   
   
	

	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX:
         
   
			

			Res = DL_UART_Main_receiveData(UART_0_INST);
      		if((USART_RX_STA&0x8000)==0)
			{
			if(USART_RX_STA&0x4000)
				{
                 
				if(Res!=0x0a)USART_RX_STA=0;
               
				else USART_RX_STA|=0x8000;
				}
			else
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;
					}		 
				}
			} 


//			if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
//			{
//				recv0_buff[recv0_length++] = receivedData;


//				uart0_send_char(receivedData);
//			}
//			else
//			{
//				recv0_length = 0;
//			}


//			recv0_flag = 1;
		
			break;
		
		default:
			break;
	}
   
   

//    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
//    {


//            uart_data = DL_UART_Main_receiveData(UART_0_INST);

//            uart0_send_char(uart_data);
//            break;


//            break;
//    }
//   
   
   
}
