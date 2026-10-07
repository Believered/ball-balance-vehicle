#include "board.h"
#include "moter.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 记录上次实际施加的方向。方向反转必须先经过一个PWM为0的控制周期。 */
static int8_t s_left_direction = 0;
static int8_t s_right_direction = 0;

/* 急停只短刹约50ms，随后自动高阻释放。持续短刹的内轮会被外轮拖动并产生大电流。 */
#define MOTOR_BRAKE_PULSE_TICKS 5U
static volatile uint8_t s_brake_release_ticks = 0U;


static uint16_t motor_abs_limit(int value)
{
    /* 先用 int 范围限幅再取绝对值，避免窄化到 int16_t 或 INT_MIN 取反溢出。 */
    if (value > MOTOR_PWM_SAFE_LIMIT) {
        value = MOTOR_PWM_SAFE_LIMIT;
    } else if (value < -MOTOR_PWM_SAFE_LIMIT) {
        value = -MOTOR_PWM_SAFE_LIMIT;
    }

    return (uint16_t)((value < 0) ? -value : value);
}

/** TB6612短路制动：IN1=IN2=1。保持到下一条驱动命令，禁止机械惯性滑行。 */
void Motor_BrakeLeft(void)
{
    Zuo_Qian_Hou_1_ON;
    Zuo_Qian_Hou_2_ON;
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST,
                                     MOTOR_PWM_MAX,
                                     GPIO_PWM_A0_C0_IDX);
    s_left_direction = 0;
}

void Motor_BrakeRight(void)
{
    Yuo_Qian_Hou_1_ON;
    Yuo_Qian_Hou_2_ON;
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST,
                                     MOTOR_PWM_MAX,
                                     GPIO_PWM_A0_C1_IDX);
    s_right_direction = 0;
}

void Motor_Brake(void)
{
    Motor_BrakeLeft();
    Motor_BrakeRight();
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0, GPIO_PWM_A0_C2_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0, GPIO_PWM_A0_C3_IDX);
}

/** 单独释放左H桥；目标速度为0时不能把该轮无限期短刹。 */
static void motor_coast_left(void)
{
    Zuo_Qian_Hou_1_OFF;
    Zuo_Qian_Hou_2_OFF;
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0,
                                     GPIO_PWM_A0_C0_IDX);
    s_left_direction = 0;
}

/** 单独释放右H桥，逻辑与左轮一致。 */
static void motor_coast_right(void)
{
    Yuo_Qian_Hou_1_OFF;
    Yuo_Qian_Hou_2_OFF;
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0,
                                     GPIO_PWM_A0_C1_IDX);
    s_right_direction = 0;
}


/**
 * @brief 安全施加左轮命令。
 *
 * 上层保持“左轮负命令=前进”的接口；实车确认该负命令对应
 * IN1=0、IN2=1。如果命令要求从前进直接切为后退（或反向），本次
 * 调用先进入短路制动，至少等待下一个10ms控制周期才允许施加新方向，
 * 避免电机惯性下的反接制动峰值电流。
 */
static void set_left_safe(int command)
{
    int8_t new_direction = (command < 0) ? -1 : ((command > 0) ? 1 : 0);

    if ((s_left_direction != 0) && (new_direction != 0) &&
        (new_direction != s_left_direction)) {
        Motor_BrakeLeft();
        return;
    }

    if (new_direction == 0) {
        motor_coast_left();
        return;
    }

    if(command < 0) {
        Zuo_Qian_Hou_1_OFF; Zuo_Qian_Hou_2_ON;  // IN1=0,IN2=1 实车前进
    } else {
        Zuo_Qian_Hou_1_ON; Zuo_Qian_Hou_2_OFF;  // IN1=1,IN2=0 实车后退
    }
    s_left_direction = new_direction;
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST,
                                     motor_abs_limit(command),
                                     GPIO_PWM_A0_C0_IDX);
}

/**
 * @brief 安全施加右轮命令。
 *
 * 左右电机面对面安装，因此本车“共同前进”对应：
 *   左轮：Set_Pwm 的负命令，IN1=0、IN2=1；
 *   右轮：Set_Pwm 的正命令，IN1=0、IN2=1。
 * 编码器插座只决定反馈属于哪一轮，不能据此翻转H桥的物理正反转极性。
 */
static void set_right_safe(int command)
{
    int8_t new_direction = (command < 0) ? -1 : ((command > 0) ? 1 : 0);

    if ((s_right_direction != 0) && (new_direction != 0) &&
        (new_direction != s_right_direction)) {
        Motor_BrakeRight();
        return;
    }

    if (new_direction == 0) {
        motor_coast_right();
        return;
    }

    if(command < 0) {
        Yuo_Qian_Hou_1_ON; Yuo_Qian_Hou_2_OFF;  // IN1=1,IN2=0 实车后退
    } else {
        Yuo_Qian_Hou_1_OFF; Yuo_Qian_Hou_2_ON;  // IN1=0,IN2=1 实车前进
    }
    s_right_direction = new_direction;
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST,
                                     motor_abs_limit(command),
                                     GPIO_PWM_A0_C1_IDX);
}

// Set_Pwm: a=左轮PWM, b=右轮PWM, c/d保留不用
// 硬件: CC0(PA21)和CC1(PB9)接电机, CC2/CC3未接线
void Set_Pwm(int a,int b,int c,int d)
{
    (void)c;
    (void)d;

    /* A new motion command supersedes an earlier short-brake request. Without
     * this cancellation Motor_Service10ms could release the old brake later
     * and coast a motor that has already restarted. */
    if ((a != 0) || (b != 0)) {
        s_brake_release_ticks = 0U;
    }

    set_left_safe(a);
    set_right_safe(b);

    // CC2/CC3未接线
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0, GPIO_PWM_A0_C2_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0, GPIO_PWM_A0_C3_IDX);
}

void Motor_Coast(void)
{
    /* 仅供疑似过流/驱动异常故障使用：高阻断开，避免追加制动电流。 */
    motor_coast_left();
    motor_coast_right();
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0, GPIO_PWM_A0_C2_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_A0_INST, 0, GPIO_PWM_A0_C3_IDX);
    s_brake_release_ticks = 0U;
}

void Motor_Stop(void)
{
    /* 正常任务停止必须短路制动，不能依靠轮胎和地面摩擦自然停下。 */
    Motor_Brake();
    s_brake_release_ticks = MOTOR_BRAKE_PULSE_TICKS;
}

void Motor_Service10ms(void)
{
    if (s_brake_release_ticks == 0U) {
        return;
    }

    s_brake_release_ticks--;
    if (s_brake_release_ticks == 0U) {
        Motor_Coast();
    }
}
