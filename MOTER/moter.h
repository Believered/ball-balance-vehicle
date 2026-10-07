#ifndef __MOTER_H__
#define __MOTER_H__

#include "ti_msp_dl_config.h"

/* 与 empty.syscfg 中 TIMA0 PWM period=1000 一致。 */
#define MOTOR_PWM_MAX 1000

/* 不再额外截断速度 PI，使用 TIMA0 的完整 0..1000 PWM 范围。 */
#define MOTOR_PWM_SAFE_LIMIT MOTOR_PWM_MAX

#define Zuo_Qian_Hou_1_ON   DL_GPIO_setPins(TB6612_PORT,TB6612_ZUO_IN1_PIN) 
#define Zuo_Qian_Hou_1_OFF  DL_GPIO_clearPins(TB6612_PORT, TB6612_ZUO_IN1_PIN)
#define Zuo_Qian_Hou_2_ON   DL_GPIO_setPins(TB6612_PORT,TB6612_ZUO_IN2_PIN)
#define Zuo_Qian_Hou_2_OFF  DL_GPIO_clearPins(TB6612_PORT, TB6612_ZUO_IN2_PIN)

#define Yuo_Qian_Hou_1_ON   DL_GPIO_setPins(TB6612_PORT,TB6612_YOU_IN1_PIN) 
#define Yuo_Qian_Hou_1_OFF  DL_GPIO_clearPins(TB6612_PORT, TB6612_YOU_IN1_PIN)
#define Yuo_Qian_Hou_2_ON   DL_GPIO_setPins(TB6612_PORT,TB6612_YOU_IN2_PIN)
#define Yuo_Qian_Hou_2_OFF  DL_GPIO_clearPins(TB6612_PORT, TB6612_YOU_IN2_PIN)

//DL_GPIO_setPins(LED1_PORT,LED1_PIN_22_PIN);
//DL_GPIO_clearPins(LED1_PORT,LED1_PIN_22_PIN);


void Set_Pwm(int a,int b,int c,int d);
/** 正常停车使用TB6612短路制动；故障停车使用高阻滑行。 */
void Motor_Stop(void);
void Motor_BrakeLeft(void);
void Motor_BrakeRight(void);
void Motor_Brake(void);
void Motor_Coast(void);
/** 10ms调用一次：急停短刹到时后自动切换为高阻，避免持续制动发热。 */
void Motor_Service10ms(void);
#endif
