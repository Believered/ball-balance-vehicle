#ifndef __KEY_H__
#define __KEY_H__


#include "ti_msp_dl_config.h"
#define key0 DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key1_PIN)
#define key1 DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key2_PIN)
#define key2 DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key3_PIN)

#define BEEP_ON  DL_GPIO_setPins(LED1_PORT,LED1_PIN_14_PIN);
#define BEEP_OFF DL_GPIO_clearPins(LED1_PORT,LED1_PIN_14_PIN);


void Key_Tick(void);
uint8_t Key_GetNum(void);
void Key_Init(void);

#endif
