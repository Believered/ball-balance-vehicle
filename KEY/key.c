#include "key.h"

/****************************
      非阻塞式按键
        引脚说明：
        Key1:PB11
        Key2:PB0
        Key3:PB7
        Key4:PB10
****************************/


volatile uint8_t Key_Num;

void Key_Init(void)
{
    DL_TimerG_enableInterrupt(TIMER_Count_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(TIMER_Count_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_Count_INST);
}

uint8_t Key_GetState(void)
{
    if(DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key1_PIN) == 0)
    {
        return 1;
    }
    if(DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key2_PIN) == 0)
    {
        return 2;
    }
    if(DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key3_PIN) == 0)
    {
        return 3;
    }
    if(DL_GPIO_readPins(KEY_PORT, KEY_PIN_Key4_PIN) == 0)
    {
        return 4;
    }
    return 0;
}

void Key_Tick(void)
{
    static uint8_t Count;
    static uint8_t PrevState, CurrState;

    Count++;
    if(Count >= 20)
    {
        Count = 0;

        PrevState = CurrState;
        CurrState = Key_GetState();

        if (CurrState == 0 && PrevState != 0)
        {
            Key_Num = PrevState;
        }
    }
}

uint8_t Key_GetNum(void)
{
    uint8_t Temp;

    if (Key_Num) 
    {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }
    return 0;
}
