/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include <stdio.h>
#include "oled.h"
#include "key.h"
#include "menu.h"
#include "uasrt1.h"
#include "encoder.h"
#include "moter.h"
#include "pid.h"
#include "imu601.h"
#include "maxicam.h"
#include "line_parser.h"
#include "xunji.h"
#include "h_route.h"
#include <string.h>

int menu; 

//int beep = 0;

/*
 * 模式二独立速度目标。第四、第五问的基础速度同样为40，但模式二调参不再
 * 借用purpose，避免调速度环时改动其他循迹/旧测试模式的目标。
 */
volatile float speed_mode_setpoint = 40.0f;

/* 其他旧循迹/测试模式保留自己的通用目标。 */
volatile float purpose = 50;

uint8_t t = 0;
uint8_t len = 0; 
    
volatile float radians_new = 0;
    
volatile int r, p, y = 0; 
volatile int differ_Yaw  = 0;

__IO uint32_t uwTick = 0;
__IO uint32_t oled_Tick = 0;
__IO uint32_t speed_Tick = 0;
__IO uint32_t kongzhi_Tick = 0;
__IO uint32_t differ_Yaw_Tick =0 ;
__IO uint32_t time_Tick =0 ;
    

uint8_t	USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
volatile uint16_t USART_RX_STA=0; //接收状态标记	 中断里面的变量要加这个声明
volatile uint16_t USART_RX_LEN=0; 


volatile unsigned char recvBuff[128] = {0};
volatile int recvLen = 0;

extern volatile int _encoder_l_count;
extern volatile int _encoder_r_count;

uint32_t key_Tick = 0;



int velocity_r  = 0;
int velocity_l  = 0;
volatile int raw_velocity_l = 0;      // 最近10ms计数×5，保持原“50ms等效速度”尺度
volatile int raw_velocity_r = 0;
volatile int filt_velocity_r = 0;     //滤波后的速度
volatile int last_filt_velocitya_r = 0;

volatile int filt_velocity_l = 0;     //滤波后的速度
volatile int last_filt_velocitya_l = 0;

float Kp=0,Ki=0,Kd=0;  //调完速度环后精调

 static volatile uint32_t cnt = 0;
// static volatile uint32_t cnt_1 = 0;
  volatile int jb = 0;//0=空闲,1=速度环,2=循迹,3=旧H路线,4=位置环,5=里程,6/7/8=第二问,9=第四问,10=第五问
volatile int last_pwm_l = 0, last_pwm_r = 0;
volatile uint32_t tima1_count = 0;
volatile uint8_t pid_send_flag = 0;
volatile uint8_t speed_mode_fault = 0; /* 1/2堵转，3/4超速，5/6左/右物理异常反馈 */
/* 故障停车后实时窗口会被清空，单独锁存触发时诊断位供OLED/VOFA复盘。 */
volatile uint8_t speed_fault_encoder_warning = 0U;
volatile int encoder_l_invalid_window = 0;
volatile int encoder_r_invalid_window = 0;
/* 0=普通双轮速度模式，1=仅左轮调参，2=仅右轮调参，3=双轮调参。 */
volatile uint8_t speed_tune_selection = 0U;

/* 速度采样和首次启动状态。测速、滤波、速度PID与PWM现在统一固定为10ms。 */
static volatile uint8_t speed_filter_initialized = 0U;
static HBallLaunchState speed_launch_state;
static int speed_mode_applied_target_l = 0;
static int speed_mode_applied_target_r = 0;
volatile int speed_mode_target_l = 0;
volatile int speed_mode_target_r = 0;
volatile uint8_t speed_mode_launch_phase = H_BALL_LAUNCH_SYNC;
static volatile uint8_t speed_mode_velocity_reprime_pending = 0U;
static volatile uint8_t speed_stall_left_windows = 0U;
static volatile uint8_t speed_stall_right_windows = 0U;
static volatile uint8_t speed_overspeed_left_windows = 0U;
static volatile uint8_t speed_overspeed_right_windows = 0U;
static volatile uint8_t speed_encoder_left_bad_windows = 0U;
static volatile uint8_t speed_encoder_right_bad_windows = 0U;
static volatile uint32_t speed_last_invalid_left = 0U;
static volatile uint32_t speed_last_invalid_right = 0U;

#define SPEED_STALL_PWM H_BALL_STALL_PWM_THRESHOLD
#define SPEED_STALL_CONFIRM_WINDOWS       15U /* 10ms×15=150ms */
#define SPEED_OVERSPEED_CONFIRM_WINDOWS   15U /* 10ms×15=150ms */
#define SPEED_ENCODER_BAD_CONFIRM_WINDOWS  5U /* 10ms×5=50ms，拒绝坏反馈继续加力 */
//  volatile int mp = 0;//1表示测量值是y，2表示测量值是differ_Yaw

int PWM = 0;

int main2(void)
{
	//开发板初始化
	board_init();  
    OLED_Init();     //初始化OLED
    OLED_Clear();

    while(1) 
    {
       printf("%d %d %d\r\n",r,p,y);
      OLED_ShowAngle(r,y);
      OLED_Refresh();
    }//实时处理方式使用这个
}


int main3(void)
{
	//开发板初始化
	board_init();
        
    OLED_Init();     //初始化OLED
    OLED_Clear();
   
   //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    while(1) 
    {
         if (recvLen){
            printf("原始数据:%s", recvBuff); 
            int result = UartLineParser_ParseAttitude((const char *)recvBuff, &r, &p, &y);
             if (result != 0) {
                 printf("解析成功:\n");
                 printf("R = %d\n", r);
                 printf("P = %d\n", p);
                 printf("Y = %d\n", y);
             } else {
                 printf("解析失败！成功匹配字段数: %d\r\n", result);
             }
             recvLen = 0;
          }//非实时处理
         OLED_ShowAngle(r,y);
           OLED_Refresh();
          
          
   }
} 
 

int main4(void)
{
	board_init();
        
    OLED_Init();     //初始化OLED
    OLED_Clear();
   int n = 0;
    while(1) 
    {
       
       for( int i=0; i<1000; ++i ){
          //volatile int r, p, y = 0; 
          //sscanf("111,211,1113\r\n", "%d,%d,%d\r\n", &r, &p, &y);、
          volatile float r, p, y = 0; 
          sscanf("111,211,1113\r\n", "%f,%f,%f\r\n", &r, &p, &y);   
          printf("%f %f %f\r\n",r,p,y);
       }
       printf("%d\r\n",n++);
       continue;
    }
 }

// 直接通过UART0发送字符串，不使用printf（避免堆溢出）
void uart0_send_str(const char *s)
{
    while(*s) {
        while(DL_UART_isBusy(UART_0_INST) == true);
        DL_UART_Main_transmitData(UART_0_INST, *s++);
    }
}

// 直接通过UART0发送整数
void uart0_send_int(int val)
{
    char buf[12];
    unsigned int magnitude = (val < 0) ? 0U - (unsigned int)val : (unsigned int)val;
    int i = 0;
    if (val < 0) uart0_send_str("-");
    do {
        buf[i++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude != 0U);
    while (i > 0) {
        while (DL_UART_isBusy(UART_0_INST) == true) { }
        DL_UART_Main_transmitData(UART_0_INST, buf[--i]);
    }
}

int main(void)
{
	/*
	 * Use the generated hardware initialization directly. board_init() enables
	 * and prints through UART0 before the application can mask noisy RX input;
	 * the production boot path must reach motor-safe state and OLED first.
	 */
	SYSCFG_DL_init();
	/*
	 * Mask every asynchronous input that is not required to draw the startup
	 * screen. A noisy debug RX or encoder pin must not starve initialization.
	 */
	NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
	NVIC_DisableIRQ(IMU601_INST_INT_IRQN);
	NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
	NVIC_DisableIRQ(TIMER_A1_KONG_ZHI_INST_INT_IRQN);
	NVIC_SetPriority(SysTick_IRQn, 3U);
	Motor_Stop();
	OLED_Init();
	OLED_Clear();

	// OLED上电自检: 显示启动画面确认硬件正常
	OLED_ShowString(0,0,(uint8_t*)"MSPM0G3507",16,1);
	OLED_ShowString(0,16,(uint8_t*)"Motor Ctrl v1",16,1);
	OLED_ShowString(0,32,(uint8_t*)"Initializing...",16,1);
	OLED_Refresh();

	// Step 0: 启用编码器输入引脚迟滞(Schmitt trigger)，拒绝PWM开关噪声
	// HYSTEN位: 0=启用迟滞, 1=禁用迟滞 (MSPM0低电平有效)
	IOMUX->SECCFG.PINCM[ENCODER_Left_A_IOMUX]  &= ~IOMUX_PINCM_HYSTEN_MASK;
	IOMUX->SECCFG.PINCM[ENCODER_Left_B_IOMUX]  &= ~IOMUX_PINCM_HYSTEN_MASK;
	IOMUX->SECCFG.PINCM[ENCODER_Right_A_IOMUX] &= ~IOMUX_PINCM_HYSTEN_MASK;
	IOMUX->SECCFG.PINCM[ENCODER_Right_B_IOMUX] &= ~IOMUX_PINCM_HYSTEN_MASK;

	// Step 1: GPIO setup
	// 禁用Key1(PB11)的GPIO中断，按键由TIMG6轮询
	DL_GPIO_disableInterrupt(GPIOB, KEY_PIN_Key1_PIN);
	DL_GPIO_clearInterruptStatus(GPIOB, KEY_PIN_Key1_PIN);
	NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
	/* 必须在启用四相边沿前先设高优先级，避免初始化窗口沿用默认优先级。 */
	NVIC_SetPriority(GPIO_MULTIPLE_GPIOB_INT_IRQN, 1);
	/* 两路A/B四线全部启用双边沿，由encoder模块进行合法Gray跃迁解码。 */
	Encoder_InitQuadrature();
	NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

	

	// Step 2: Key_Init (starts TIMG6)
	NVIC_SetPriority(TIMER_Count_INST_INT_IRQN, 3U);
	Key_Init();

	

	// Step 3: initialize all control state while motor and control IRQ stay off.
	Motor_Stop();
	Encoder_Clear();
	Xunji_ControlInit();
	HRoute_Init();

	/*
	 * The IMU is intentionally not initialized here. Modes 3/4 start it
	 * asynchronously when needed, so an absent IMU can never block the menu.
	 */
	Encoder_EnableInterrupts();

	// Step 4: start the 10 ms control base only after all shared state is valid.
	NVIC_SetPriority(TIMER_A1_KONG_ZHI_INST_INT_IRQN, 2U);
	NVIC_ClearPendingIRQ(TIMER_A1_KONG_ZHI_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_A1_KONG_ZHI_INST_INT_IRQN);
	DL_TimerA_startCounter(TIMER_A1_KONG_ZHI_INST);

	/*
	 * UART0仅在模式二用轮询TX向VOFA发送速度环数据；当前没有串口调参命令，
	 * 因此保持RX中断关闭，避免其他模式被无关串口字节抢占。轮询TX不依赖NVIC。
	 */
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);

	OLED_ShowString(0,32,(uint8_t*)"Ready           ",16,1);
	OLED_Refresh();

	while(1)
	{
        // // 串口发送速度环数据给VOFA上位机
		// if(pid_send_flag && jb == 1) {
		// 		pid_send_flag = 0;
		// 		uart0_send_int((int)purpose);
		// 		uart0_send_str(",");
		// 		uart0_send_int(filt_velocity_l);
		// 		uart0_send_str(",");
		// 		uart0_send_int(filt_velocity_r);
		// 		uart0_send_str(",");
		// 		uart0_send_int(last_pwm_l);
		// 		uart0_send_str(",");
		// 		uart0_send_int(last_pwm_r);
		// 		uart0_send_str("\n");
		// 	}

		// // 简化操作: Key2启停速度环
		// uint8_t key = Key_GetNum();
		// if(key == 2) {
		// 	if(jb == 0) {
		// 		jb = 1;
		// 		velocity_sum_l = 0;
		// 		velocity_sum_r = 0;
		// 	} else {
		// 		jb = 0;
		// 		Motor_Stop();
		// 		velocity_sum_l = 0;
		// 		velocity_sum_r = 0;
		// 	}
		// 	delay_ms(200);
		// }

		// // OLED显示
		// OLED_ShowString(0, 0, (uint8_t *)"Speed PID", 16, 1);
		// OLED_ShowString(96, 0, (uint8_t *)((jb == 1) ? "ON " : "OFF"), 16, 1);

		// OLED_ShowString(0, 16, (uint8_t *)"L", 16, 1);
		// OLED_ShowNum(8, 16, filt_velocity_l, 4, 16, 1);
		// OLED_ShowString(64, 16, (uint8_t *)"R", 16, 1);
		// OLED_ShowNum(72, 16, filt_velocity_r, 4, 16, 1);

		// OLED_ShowString(0, 32, (uint8_t *)"Tgt", 16, 1);
		// OLED_ShowNum(24, 32, (int)purpose, 3, 16, 1);
		// OLED_ShowString(64, 32, (uint8_t *)"K2:ON/OFF", 16, 1);

		// OLED_Refresh();
		// delay_ms(50);
		int mode = menu1();
		switch(mode) {
			case 1: menu2(); break;
			case 2: menu3(); break;
			case 3: menu4(); break;
			case 4: menu5(); break;
			case 5: menu6(); break;
			case 6: menu7(); break;
			case 7: menu8(); break;
			case 8: menu9(); break;
		}
	}
}


//实时处理
/* UART0 remains disabled in the default control firmware. These legacy
 * CSV receivers are bounded even if an application explicitly enables RX. */
void ParserData_Real(unsigned char data)
{
    static UartLineParser parser;
    if (UartLineParser_Push(&parser, data) != 0U) {
        int parsed_r, parsed_p, parsed_y;
        if (UartLineParser_ParseAttitude(parser.data, &parsed_r,
                                       &parsed_p, &parsed_y) != 0U) {
            r = parsed_r;
            p = parsed_p;
            y = parsed_y;
        }
        UartLineParser_Reset(&parser);
    }
}

void ParserData(unsigned char data)
{
    static UartLineParser parser;
    if (UartLineParser_Push(&parser, data) != 0U) {
        if (recvLen == 0) {
            size_t i;
            for (i = 0U; i <= parser.length; i++) {
                recvBuff[i] = (unsigned char)parser.data[i];
            }
            recvLen = (int)parser.length;
        }
        UartLineParser_Reset(&parser);
    }
}

void UART_0_INST_IRQHandler(void){
	//如果产生了串口中断
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断
         cnt++;
         //ParserData_Real( DL_UART_receiveData(UART_0_INST) );
         #define UART_DATA ((uint8_t)(UART_0_INST->RXDATA & UART_RXDATA_DATA_MASK)) 
         ParserData_Real( UART_DATA );
			break;
		
		default://其他的串口中断
			break;
	}
}


//滴答定时器中断服务函数

void SysTick_Handler(void)
{
    uwTick++;
}

/**
 * @brief 清除并重新对齐10ms测速窗口。
 *
 * 模式二和模式三每次启动前都调用。滤波器首个完整样本直接装载实测值，
 * 不再从0按0.3系数缓慢爬升，从根源消除显示和反馈的假启动延迟。
 */
void Control_ResetSpeedFeedback(void)
{
    __disable_irq();
    Encoder_Clear();
    raw_velocity_l = 0;
    raw_velocity_r = 0;
    filt_velocity_l = 0;
    filt_velocity_r = 0;
    last_filt_velocitya_l = 0;
    last_filt_velocitya_r = 0;
    speed_filter_initialized = 0U;
    encoder_l_invalid_window = 0;
    encoder_r_invalid_window = 0;
    speed_last_invalid_left = encoder_l_invalid_transitions;
    speed_last_invalid_right = encoder_r_invalid_transitions;
    __enable_irq();
}

static uint8_t speed_left_is_active(void)
{
    return (uint8_t)((speed_tune_selection == 0U) ||
                     (speed_tune_selection == 1U) ||
                     (speed_tune_selection == 3U));
}

static uint8_t speed_right_is_active(void)
{
    return (uint8_t)((speed_tune_selection == 0U) ||
                     (speed_tune_selection == 2U) ||
                     (speed_tune_selection == 3U));
}

static uint8_t speed_active_mask(void)
{
    return (uint8_t)((speed_left_is_active() ? 1U : 0U) |
                     (speed_right_is_active() ? 2U : 0U));
}

static int speed_approach_step(int current, int target, int step)
{
    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

/**
 * @brief 启动速度调试/速度测试的公共流程。
 *
 * 不再瞬间给两轮800 PWM。输出先保持为0，再由10ms控制中断按28/拍平滑
 * 爬升至目标相关前馈；调试模式下未选中的轮在全部阶段都强制为0。
 */
static void speed_control_start(uint8_t selection)
{
    jb = 0;
    /* 调试启动前直接高阻并清历史量，避免反复启停产生额外制动电流。 */
    Motor_Coast();
    Control_ResetSpeedFeedback();
    PID_ResetVelocity();

    speed_tune_selection = selection;
    speed_mode_fault = 0U;
    speed_fault_encoder_warning = 0U;
    speed_stall_left_windows = 0U;
    speed_stall_right_windows = 0U;
    speed_overspeed_left_windows = 0U;
    speed_overspeed_right_windows = 0U;
    speed_encoder_left_bad_windows = 0U;
    speed_encoder_right_bad_windows = 0U;
    HBallLaunch_Reset(&speed_launch_state);
    speed_mode_applied_target_l = 0;
    speed_mode_applied_target_r = 0;
    speed_mode_target_l = 0;
    speed_mode_target_r = 0;
    speed_mode_launch_phase = H_BALL_LAUNCH_SYNC;
    speed_mode_velocity_reprime_pending = 0U;
    pid_send_flag = 0U;
    last_pwm_l = 0;
    last_pwm_r = 0;
    jb = 1;
    Set_Pwm(0, 0, 0, 0);
}

/** 模式二普通双轮速度测试。 */
void SpeedMode_Start(void)
{
    speed_control_start(0U);
}

/** 模式一专用调试启动：1=左，2=右，3=双轮。 */
void SpeedTune_Start(uint8_t selection)
{
    if ((selection < 1U) || (selection > 3U)) {
        selection = 1U;
    }
    speed_control_start(selection);
}

/** 模式二停车：先撤销运行状态，再清空所有可能影响下一次启动的历史量。 */
void SpeedMode_Stop(void)
{
    jb = 0;
    Motor_Stop();
    Control_ResetSpeedFeedback();
    PID_ResetVelocity();
    HBallLaunch_Reset(&speed_launch_state);
    speed_mode_applied_target_l = 0;
    speed_mode_applied_target_r = 0;
    speed_mode_target_l = 0;
    speed_mode_target_r = 0;
    speed_mode_launch_phase = H_BALL_LAUNCH_SYNC;
    speed_mode_velocity_reprime_pending = 0U;
    speed_stall_left_windows = 0U;
    speed_stall_right_windows = 0U;
    speed_overspeed_left_windows = 0U;
    speed_overspeed_right_windows = 0U;
    speed_encoder_left_bad_windows = 0U;
    speed_encoder_right_bad_windows = 0U;
    speed_mode_fault = 0U;
    speed_fault_encoder_warning = 0U;
    last_pwm_l = 0;
    last_pwm_r = 0;
    pid_send_flag = 0U;
    speed_tune_selection = 0U;
}

/** 返回0正常；返回1/2为左/右堵转，3/4为左/右反馈异常超速。 */
static uint8_t speed_mode_check_wheel(int target,
                                      int measured,
                                      int pwm,
                                      volatile uint8_t *stall_windows,
                                      volatile uint8_t *overspeed_windows,
                                      uint8_t stall_fault,
                                      uint8_t overspeed_fault)
{
    int measured_abs = (measured < 0) ? -measured : measured;
    int pwm_abs = (pwm < 0) ? -pwm : pwm;
    int low_speed_threshold = target / 4;

    if (low_speed_threshold < 3) low_speed_threshold = 3;

    /* 半堵转或驱动通道异常时仍可能有少量编码器边沿，不能只检测完全零速。 */
    if ((target >= 15) && (measured_abs < low_speed_threshold) &&
        (pwm_abs >= SPEED_STALL_PWM)) {
        if (*stall_windows < 255U) (*stall_windows)++;
    } else {
        *stall_windows = 0U;
    }
    if (*stall_windows >= SPEED_STALL_CONFIRM_WINDOWS) return stall_fault;

    if ((target >= 15) && (measured_abs > (target * 2 + 20))) {
        if (*overspeed_windows < 255U) (*overspeed_windows)++;
    } else {
        *overspeed_windows = 0U;
    }
    if (*overspeed_windows >= SPEED_OVERSPEED_CONFIRM_WINDOWS) {
        return overspeed_fault;
    }
    return 0U;
}

// TIMG6中断: 按键扫描 (1ms周期)
void TIMER_Count_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_Count_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            Key_Tick();
            break;
        default:
            break;
    }
}






void TIMA1_IRQHandler(void)
{
    int raw_l;
    int raw_r;

    DL_TimerA_getPendingInterrupt(TIMER_A1_KONG_ZHI_INST);
    tima1_count++;
    /* 急停短刹必须按固定时基释放，不能依赖主循环和OLED刷新速度。 */
    Motor_Service10ms();

    /* 灰度采样和赛道状态识别需要更快响应，固定每10ms执行。 */
    if (jb == 2) {
        Xunji_Tick10ms();
    } else if (jb == 3) {
        HRoute_Tick10ms();
    } else if ((jb == 6) || (jb == 7) || (jb == 8)) {
        H2Motion_Tick10ms();
    }

    /* 编码器、滤波、速度PI与PWM每10ms执行。原目标60表示50ms边沿数，
     * 所以10ms原始计数乘5后送入速度环；里程仍累计未放大的真实计数。 */
    Encoder_ReadAndClear(&raw_l, &raw_r);
    /*
     * 单个坏窗口不允许把速度瞬间改成0，否则PI会在故障确认前反而加大PWM。
     * 健康窗口才发布新速度；坏窗口最多保持上次有效值50ms，随后fault 5/6
     * 撤销驱动。里程接口raw_l/raw_r仍为0，不会伪造行驶距离。
     */
    if ((encoder_feedback_valid_mask & 1U) != 0U) {
        raw_velocity_l = raw_l * 5;
    }
    if ((encoder_feedback_valid_mask & 2U) != 0U) {
        raw_velocity_r = raw_r * 5;
    }
    if ((jb == 9) || (jb == 10)) {
        HBall_Tick10ms(raw_l, raw_r);
    }
    encoder_l_invalid_window = (int)(encoder_l_invalid_transitions -
                                     speed_last_invalid_left);
    encoder_r_invalid_window = (int)(encoder_r_invalid_transitions -
                                     speed_last_invalid_right);
    speed_last_invalid_left = encoder_l_invalid_transitions;
    speed_last_invalid_right = encoder_r_invalid_transitions;
    if (encoder_mapping_swapped != 0U) {
        int temporary_invalid = encoder_l_invalid_window;
        encoder_l_invalid_window = encoder_r_invalid_window;
        encoder_r_invalid_window = temporary_invalid;
    }

    if ((jb == 1) || (jb == 2) || (jb == 3) || (jb == 4) ||
        (jb == 5) || (jb == 6) || (jb == 7) || (jb == 8) ||
        (jb == 9) || (jb == 10)) {
        if (speed_filter_initialized == 0U) {
            /* 首个完整10ms样本直接装载，避免从0低通造成假慢启动。 */
            filt_velocity_l = raw_velocity_l;
            filt_velocity_r = raw_velocity_r;
            speed_filter_initialized = 1U;
        } else {
            /* 10ms下20%新样本兼顾±5计数量化与快速反馈，时间常数约45ms。 */
            filt_velocity_l = (raw_velocity_l * 2 +
                               last_filt_velocitya_l * 8) / 10;
            filt_velocity_r = (raw_velocity_r * 2 +
                               last_filt_velocitya_r * 8) / 10;
        }
        last_filt_velocitya_l = filt_velocity_l;
        last_filt_velocitya_r = filt_velocity_r;
    }

    if (jb == 1) {
        int pwm_l;
        int pwm_r;
        int requested_pwm_l;
        int requested_pwm_r;
        int final_target =
            (speed_mode_setpoint < 0.0f) ?
                (int)(-speed_mode_setpoint) :
                (int)speed_mode_setpoint;
        uint8_t left_active = speed_left_is_active();
        uint8_t right_active = speed_right_is_active();
        uint8_t active_mask = speed_active_mask();
        uint8_t launch_timeout_mask;

        /*
         * 物理不可能的编码器窗口内冻结整个双轮启动链：
         * - 不推进同步/加速状态机；
         * - 不更新PI积分；
         * - 保持上一拍PWM，最多50ms后F5/F6撤销驱动。
         * 这样错误反馈不会让某一轮继续加力，也不会破坏双轮同步判定。
         */
        if ((left_active != 0U) &&
            ((encoder_feedback_valid_mask & 1U) == 0U)) {
            if (speed_encoder_left_bad_windows < 255U) {
                speed_encoder_left_bad_windows++;
            }
        } else {
            speed_encoder_left_bad_windows = 0U;
        }
        if ((right_active != 0U) &&
            ((encoder_feedback_valid_mask & 2U) == 0U)) {
            if (speed_encoder_right_bad_windows < 255U) {
                speed_encoder_right_bad_windows++;
            }
        } else {
            speed_encoder_right_bad_windows = 0U;
        }
        if (speed_encoder_left_bad_windows >=
            SPEED_ENCODER_BAD_CONFIRM_WINDOWS) {
            speed_mode_fault = 5U;
        } else if (speed_encoder_right_bad_windows >=
                   SPEED_ENCODER_BAD_CONFIRM_WINDOWS) {
            speed_mode_fault = 6U;
        } else {
            speed_mode_fault = 0U;
        }
        if (speed_mode_fault != 0U) {
            speed_fault_encoder_warning = encoder_phase_warning_mask;
            jb = 0;
            Motor_Coast();
            PID_ResetVelocity();
            pid_send_flag = 1U;
            return;
        }
        if ((encoder_feedback_valid_mask & active_mask) != active_mask) {
            speed_stall_left_windows = 0U;
            speed_stall_right_windows = 0U;
            speed_overspeed_left_windows = 0U;
            speed_overspeed_right_windows = 0U;
            speed_mode_velocity_reprime_pending = 1U;
            Set_Pwm(-last_pwm_l, last_pwm_r, 0, 0);
            pid_send_flag = 1U;
            return;
        }

        (void)HBallLaunch_Update(&speed_launch_state,
                                 final_target,
                                 filt_velocity_l,
                                 filt_velocity_r,
                                 active_mask);
        speed_mode_applied_target_l = speed_approach_step(
            speed_mode_applied_target_l,
            speed_launch_state.target_left,
            H_BALL_TARGET_SLEW_STEP);
        speed_mode_applied_target_r = speed_approach_step(
            speed_mode_applied_target_r,
            speed_launch_state.target_right,
            H_BALL_TARGET_SLEW_STEP);
        speed_mode_target_l = speed_mode_applied_target_l;
        speed_mode_target_r = speed_mode_applied_target_r;
        speed_mode_launch_phase =
            (uint8_t)speed_launch_state.phase;
        launch_timeout_mask =
            HBallLaunch_GetTimeoutMask(&speed_launch_state);

        if (speed_mode_velocity_reprime_pending != 0U) {
            PID_PrimeVelocityNoFeedforward(
                (float)speed_mode_target_l,
                (float)speed_mode_target_r,
                (float)filt_velocity_l,
                (float)filt_velocity_r,
                (float)last_pwm_l,
                (float)last_pwm_r);
            speed_mode_velocity_reprime_pending = 0U;
        }

        /*
         * 模式二复现模式三/四的真实启动链：无前馈速度PI、目标同步门槛、
         * PWM斜率限制和独立800上限。VOFA看到的曲线与滚球任务一致。
         */
        if (speed_mode_target_l == 0) {
            requested_pwm_l = 0;
            PID_ResetVelocityLeft();
        } else {
            requested_pwm_l = (int)velocity_PID_value_l_no_ff(
                (float)speed_mode_target_l,
                (float)filt_velocity_l);
        }
        if (speed_mode_target_r == 0) {
            requested_pwm_r = 0;
            PID_ResetVelocityRight();
        } else {
            requested_pwm_r = (int)velocity_PID_value_r_no_ff(
                (float)speed_mode_target_r,
                (float)filt_velocity_r);
        }
        if (requested_pwm_l < 0) requested_pwm_l = 0;
        if (requested_pwm_r < 0) requested_pwm_r = 0;
        if (requested_pwm_l > H_BALL_PWM_LIMIT) {
            requested_pwm_l = H_BALL_PWM_LIMIT;
        }
        if (requested_pwm_r > H_BALL_PWM_LIMIT) {
            requested_pwm_r = H_BALL_PWM_LIMIT;
        }

        pwm_l = speed_approach_step(
            last_pwm_l, requested_pwm_l, H_BALL_PWM_SLEW_STEP);
        pwm_r = speed_approach_step(
            last_pwm_r, requested_pwm_r, H_BALL_PWM_SLEW_STEP);
        /*
         * 左右PWM斜率限制必须分别处理。原逻辑只要任意一轮受限就同时播种
         * 两轮PI，左轮的测速量化波动会反复覆盖右轮积分，造成右轮目标50、
         * 实际40但PWM长期停在约290。现在只修正真正受限的对应轮。
         */
        if (pwm_l != requested_pwm_l) {
            PID_PrimeVelocityLeftNoFeedforward(
                (float)speed_mode_target_l,
                (float)filt_velocity_l,
                (float)pwm_l);
        }
        if (pwm_r != requested_pwm_r) {
            PID_PrimeVelocityRightNoFeedforward(
                (float)speed_mode_target_r,
                (float)filt_velocity_r,
                (float)pwm_r);
        }

        /* 正向速度测试时，负PI输出只表示应减小驱动力，不能直接反接电机。 */
        if (speed_mode_setpoint >= 0.0f) {
            if (pwm_l < 0) pwm_l = 0;
            if (pwm_r < 0) pwm_r = 0;
        }
        last_pwm_l = pwm_l;
        last_pwm_r = pwm_r;
        Set_Pwm(-pwm_l, pwm_r, 0, 0);

        speed_mode_fault = 0U;
        if ((launch_timeout_mask & 1U) != 0U) {
            speed_mode_fault = 1U;
        } else if ((launch_timeout_mask & 2U) != 0U) {
            speed_mode_fault = 2U;
        }
        if ((speed_mode_fault == 0U) && (left_active != 0U)) {
            speed_mode_fault = speed_mode_check_wheel(
                speed_mode_target_l, filt_velocity_l, pwm_l,
                &speed_stall_left_windows,
                &speed_overspeed_left_windows, 1U, 3U);
        } else if (left_active == 0U) {
            speed_stall_left_windows = 0U;
            speed_overspeed_left_windows = 0U;
        }
        if ((speed_mode_fault == 0U) && (right_active != 0U)) {
            speed_mode_fault = speed_mode_check_wheel(
                speed_mode_target_r, filt_velocity_r, pwm_r,
                &speed_stall_right_windows,
                &speed_overspeed_right_windows, 2U, 4U);
        } else if (right_active == 0U) {
            speed_stall_right_windows = 0U;
            speed_overspeed_right_windows = 0U;
        }

        if (speed_mode_fault != 0U) {
            /* 不限制健康反馈下的正常PWM；确认故障后撤销驱动并清PID。 */
            speed_fault_encoder_warning = encoder_phase_warning_mask;
            jb = 0;
            Motor_Coast();
            PID_ResetVelocity();
        }
        pid_send_flag = 1;
    } else if (jb == 2) {
        /* 里程与速度使用同一份经过合法正交跃迁校验、并完成通道映射的计数。 */
        Xunji_UpdateEncoder(raw_l, raw_r);
        if (jb == 2) {
            Xunji_SpeedControl(filt_velocity_l, filt_velocity_r);
        }
    } else if (jb == 3) {
        /* 模式四的里程、航向/寻迹规划与轮速PI共用同一份10ms编码器快照。 */
        HRoute_Control10ms(raw_l, raw_r, filt_velocity_l, filt_velocity_r);
    } else if (jb == 4) {
        /* Mode 5 shares mode 4's 20 ms position loop and the project's 10 ms
         * wheel-speed loop, but does not execute any route transition. */
        HPositionTune_Control10ms(filt_velocity_l, filt_velocity_r);
    } else if (jb == 5) {
        /* 模式六只累计手推里程，绝不向电机输出。 */
        H2Mileage_Control10ms(raw_l, raw_r);
    } else if ((jb == 6) || (jb == 7) || (jb == 8)) {
        H2Motion_Control10ms(raw_l, raw_r,
                            filt_velocity_l, filt_velocity_r);
    } else if ((jb == 9) || (jb == 10)) {
        HBall_Control10ms(filt_velocity_l, filt_velocity_r);
    }
}

// maxicam UART2 中断处理函数
void maxicam_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(maxicam_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t recv = DL_UART_receiveData(maxicam_INST);
            if(maxicam_rx_len < 64) {
                maxicam_rx_buf[maxicam_rx_len++] = recv;
            }
            // 简单帧结束检测: 收到换行符置标志
            if(recv == '\n') {
                maxicam_rx_flag = 1;
            }
            break;
        }
        default:
            break;
    }
}

// UART1 中断处理函数 - 防止未实现IRQHandler进入Default_Handler死循环
void UART_1_pid_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_1_pid_INST))
    {
        case DL_UART_IIDX_RX:
        {
            volatile uint8_t recv = DL_UART_receiveData(UART_1_pid_INST);
            (void)recv;
            break;
        }
        default:
            break;
    }
}






