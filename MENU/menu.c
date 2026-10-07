#include "menu.h"
#include "key.h"
#include "oled.h"
#include "board.h"
#include "moter.h"
#include "encoder.h"
#include "pid.h"
#include "xunji.h"
#include "imu601.h"
#include "h_route.h"
#include <stdio.h>
#include <string.h>

extern __IO uint32_t uwTick;
extern __IO uint32_t oled_Tick;
extern volatile int jb;
extern volatile float purpose;
extern volatile float speed_mode_setpoint;
extern volatile int filt_velocity_l, filt_velocity_r;
extern volatile int raw_velocity_l, raw_velocity_r;
extern volatile int last_filt_velocitya_l, last_filt_velocitya_r;
extern volatile int last_pwm_l, last_pwm_r;
extern volatile uint32_t tima1_count;
extern volatile uint8_t pid_send_flag;
extern volatile uint8_t speed_mode_fault;
extern volatile uint8_t speed_fault_encoder_warning;
extern volatile uint8_t speed_tune_selection;
extern volatile int speed_mode_target_l, speed_mode_target_r;
extern volatile uint8_t speed_mode_launch_phase;
extern volatile int encoder_l_invalid_window, encoder_r_invalid_window;

void uart0_send_str(const char *s);
void uart0_send_int(int val);
void Control_ResetSpeedFeedback(void);
void SpeedMode_Start(void);
void SpeedTune_Start(uint8_t selection);
void SpeedMode_Stop(void);
static const char *menu_h2_stage_name(H2CalStage stage);
static const char *menu_h2_segment_name(uint8_t index);

// Key number mapping (from Key_GetNum)
// Key1 = 1 (next), Key2 = 2 (confirm/toggle), Key3 = 3 (back)

#define KEY_NUM_NEXT    1
#define KEY_NUM_CONFIRM 2
#define KEY_NUM_BACK    3
#define KEY_NUM_PLUS    4

#define SPEED_PID_PARAM_MAX (999.0f)
#define POSITION_PID_PARAM_MAX (99.999f)

/**
 * 速度PID分段步进：低值区保留精调分辨率，高值区快速跨越，避免数百次按键。
 * P与I/D分开设步进，是因为积分和微分通常需要比比例项更细的低值调整。
 */
static float speed_pid_step(float value, uint8_t is_proportional)
{
    if (is_proportional != 0U) {
        if (value < 20.0f) return 0.5f;
        if (value < 100.0f) return 2.0f;
        return 10.0f;
    }
    if (value < 10.0f) return 0.1f;
    if (value < 50.0f) return 0.5f;
    return 2.0f;
}

static float adjust_speed_pid(float value, int direction, uint8_t is_proportional)
{
    value += (float)direction * speed_pid_step(value, is_proportional);
    if (value < 0.0f) value = 0.0f;
    if (value > SPEED_PID_PARAM_MAX) value = SPEED_PID_PARAM_MAX;
    return value;
}

/* Position gains are much smaller than wheel-speed gains. Keep fine steps
 * around the current 0.0x values, while still leaving ample range for tests. */
static float position_pid_step(float value)
{
    if (value < 0.10f) return 0.005f;
    if (value < 1.00f) return 0.020f;
    if (value < 10.0f) return 0.100f;
    return 1.000f;
}

static float adjust_position_pid(float value, int direction)
{
    value += (float)direction * position_pid_step(value);
    if (value < 0.0f) value = 0.0f;
    if (value > POSITION_PID_PARAM_MAX) value = POSITION_PID_PARAM_MAX;
    return value;
}

static int position_pid_scaled(float value)
{
    return (int)(value * 1000.0f + 0.5f);
}

/** 模式二30列VOFA诊断帧：保留原字段，并追加双轮实时目标及启动阶段。 */
static void menu_send_speed_csv(void)
{
    int snapshot[30];

    /* 串口发送期间控制中断仍会更新数据。先复制完整快照，确保一行全部字段
     * 对应同一个10ms采样时刻；临界区内不做任何UART发送。 */
    __disable_irq();
    snapshot[0] = (speed_mode_target_l > speed_mode_target_r) ?
        speed_mode_target_l : speed_mode_target_r;
    snapshot[1] = filt_velocity_l;
    snapshot[2] = filt_velocity_r;
    snapshot[3] = raw_velocity_l;
    snapshot[4] = raw_velocity_r;
    snapshot[5] = last_pwm_l;
    snapshot[6] = last_pwm_r;
    snapshot[7] = speed_mode_fault;
    snapshot[8] = encoder_mapping_swapped;
    snapshot[9] = encoder_mapping_valid;
    snapshot[10] = encoder_l_invalid_window;
    snapshot[11] = encoder_r_invalid_window;
    snapshot[12] = encoder_l_a_edges_window;
    snapshot[13] = encoder_l_b_edges_window;
    snapshot[14] = encoder_r_a_edges_window;
    snapshot[15] = encoder_r_b_edges_window;
    snapshot[16] = encoder_l_quadrature_window;
    snapshot[17] = encoder_r_quadrature_window;
    snapshot[18] = (speed_mode_fault != 0U) ?
        (int)speed_fault_encoder_warning : (int)encoder_phase_warning_mask;
    snapshot[19] = (int)(VKp_l * 10.0f + 0.5f);
    snapshot[20] = (int)(VKi_l * 10.0f + 0.5f);
    snapshot[21] = (int)(VKd_l * 10.0f + 0.5f);
    snapshot[22] = (int)(VKp_r * 10.0f + 0.5f);
    snapshot[23] = (int)(VKi_r * 10.0f + 0.5f);
    snapshot[24] = (int)(VKd_r * 10.0f + 0.5f);
    snapshot[25] = speed_tune_selection;
    snapshot[26] = speed_mode_target_l;
    snapshot[27] = speed_mode_target_r;
    snapshot[28] = (int)speed_mode_setpoint;
    snapshot[29] = (int)speed_mode_launch_phase;
    __enable_irq();

    for (int i = 0; i < 30; i++) {
        if (i != 0) uart0_send_str(",");
        uart0_send_int(snapshot[i]);
    }
    uart0_send_str("\n");
}

/** 将角度格式化成固定两位小数；不用 %f，避免嵌入式 printf 浮点配置差异。 */
/**********************************************************
              主菜单 - 选择模式
**********************************************************/
int menu1(void)
{
    int flag = 1;

    while (1) {
        uint8_t key = Key_GetNum();

        if (key == KEY_NUM_NEXT) {
            flag++;
            if (flag > 8) flag = 1;
        }

        if (key == KEY_NUM_CONFIRM) {
            OLED_Clear();
            return flag;
        }

        /* Four text rows per page. Overwrite every row
         * instead of OLED_Clear() to avoid visible flashing. */
        if (flag <= 4) {
            OLED_ShowString(0, 0,  (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 0,  (uint8_t *)((flag == 1) ? ">1.H2 Final" : " 1.H2 Final"), 16, 1);
            OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 16, (uint8_t *)((flag == 2) ? ">2.Speed PID" : " 2.Speed PID"), 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)((flag == 3) ? ">3.H4 Ball AB" : " 3.H4 Ball AB"), 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)((flag == 4) ? ">4.H5 Ball Lap" : " 4.H5 Ball Lap"), 16, 1);
        } else {
            OLED_ShowString(0, 0,  (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 0,  (uint8_t *)((flag == 5) ? ">5.H5 PID Lap" : " 5.H5 PID Lap"), 16, 1);
            OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 16, (uint8_t *)((flag == 6) ? ">6.Straight" : " 6.Straight"), 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)((flag == 7) ? ">7.Timed Track" : " 7.Timed Track"), 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)((flag == 8) ? ">8.H2 Curve" : " 8.H2 Curve"), 16, 1);
        }
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode1 - 第二问最终综合控制
 **********************************************************/
int menu2(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* AB,BC,CD,DA,WGT,RUN,EXIT */
    char line[17];

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    IMU601_init();
    pid_send_flag = 0U;
    OLED_Clear();

    while (1) {
        H2MotionStatus status;
        uint8_t key;
        uint32_t selected_time = 0U;

        IMU601_Service();
        key = Key_GetNum();
        H2Motion_GetStatus(&status);

        if ((key == KEY_NUM_NEXT) && (running == 0U)) {
            selection++;
            if (selection > 6U) selection = 0U;
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (selection < 4U) && (running == 0U)) {
            int32_t updated =
                (int32_t)H2Final_GetStageTime(selection);
            updated += (key == KEY_NUM_PLUS) ?
                (int32_t)H2_FINAL_TIME_STEP_MS :
                -(int32_t)H2_FINAL_TIME_STEP_MS;
            H2Final_SetStageTime(selection, (uint32_t)
                ((updated < 500) ? 500 : updated));
        }

        /*
         * Stopped: K3/K4 edit WGT only when WGT is selected.
         * Running: K3/K4 always edit WGT for immediate curve tuning.
         */
        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (((selection == 4U) && (running == 0U)) ||
             (running != 0U))) {
            int updated_weight = H2Final_GetLineWeight();
            updated_weight += (key == KEY_NUM_PLUS) ?
                H2_FINAL_LINE_WEIGHT_STEP :
                -H2_FINAL_LINE_WEIGHT_STEP;
            H2Final_SetLineWeight(updated_weight);
        }

        if (key == KEY_NUM_CONFIRM) {
            if (running != 0U) {
                H2Motion_Stop();
                running = 0U;
            } else if (selection == 5U) {
                Control_ResetSpeedFeedback();
                running = H2Final_Start();
            } else if (selection == 6U) {
                H2Motion_Stop();
                OLED_Clear();
                return 0;
            }
        }

        if ((running != 0U) && (jb != 8)) {
            running = 0U;
        }
        H2Motion_GetStatus(&status);
        if (selection < 4U) {
            selected_time = H2Final_GetStageTime(selection);
        }

        /* 逐行覆盖而不清屏，避免OLED刷新闪烁干扰IMU串口服务。 */
        (void)snprintf(line, 17U, "1:%-4s %-3sF%02u",
                       menu_h2_stage_name(status.stage),
                       (running != 0U) ? "ON" :
                       (status.stage == H2_CAL_STAGE_DONE) ? "DON" : "OFF",
                       (unsigned)status.fault);
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        if (running != 0U) {
            (void)snprintf(line, 17U, ">WGT %02d K3/K4",
                           H2Final_GetLineWeight());
        } else if (selection < 4U) {
            (void)snprintf(line, 17U, ">%s %05lums",
                           menu_h2_segment_name(selection),
                           (unsigned long)selected_time);
        } else if (selection == 4U) {
            (void)snprintf(line, 17U, ">WGT %02d K3/K4",
                           H2Final_GetLineWeight());
        } else {
            (void)snprintf(line, 17U, ">%-4s K2",
                           (selection == 5U) ? "RUN" : "EXIT");
        }
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "T%02lu.%02lus D%02d",
                       (unsigned long)(status.total_elapsed_ms / 1000U),
                       (unsigned long)((status.total_elapsed_ms % 1000U) /
                                       10U),
                       (int)status.curve_delta);
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        if ((status.stage == H2_CAL_STAGE_BC) ||
            (status.stage == H2_CAL_STAGE_DA)) {
            /*
             * 模式一的IMU只作为可选辅助修正，通信暂态不得作为用户可见故障。
             * 这里不再显示I0/I1，避免把陀螺仪可用状态误认为路线故障；
             * 真正的电机、编码器、超时和丢线故障仍由Fxx显示。
             */
            (void)snprintf(line, 17U, "A%03d C%+03d",
                           (int)status.yaw_progress,
                           (int)status.line_correction);
        } else {
            (void)snprintf(line, 17U, "L%02d/%02d R%02d/%02d",
                           status.target_left, filt_velocity_l,
                           status.target_right, filt_velocity_r);
        }
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode2 - 速度环PID
**********************************************************/
int menu3(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* PL,IL,DL,PR,IR,DR,SPD,RUN,EXIT */
    uint8_t run_wheels = 3U; /* 1=左，2=右，3=双轮；默认双轮。 */
    OLED_Clear();
    SpeedMode_Stop();

    while (1) {
        uint8_t key = Key_GetNum();

        if (key == KEY_NUM_NEXT) {
            selection++;
            if (selection > 8U) selection = 0U;
        }

        if (key == KEY_NUM_CONFIRM) {
            if ((running == 0U) && (selection == 8U)) {
                SpeedMode_Stop();
                OLED_Clear();
                return 0;
            }
            running = (running == 0U) ? 1U : 0U;
            if (running != 0U) {
                SpeedTune_Start(run_wheels);
            } else {
                SpeedMode_Stop();
            }
        }

        /* K3减、K4加；运行中修改对应轮参数后只复位该轮PID历史。 */
        if ((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) {
            int direction = (key == KEY_NUM_PLUS) ? 1 : -1;
            if (selection <= 5U) __disable_irq();
            if (selection == 0U) {
                VKp_l = adjust_speed_pid(VKp_l, direction, 1U);
                PID_ResetVelocityLeft();
            } else if (selection == 1U) {
                VKi_l = adjust_speed_pid(VKi_l, direction, 0U);
                PID_ResetVelocityLeft();
            } else if (selection == 2U) {
                VKd_l = adjust_speed_pid(VKd_l, direction, 0U);
                PID_ResetVelocityLeft();
            } else if (selection == 3U) {
                VKp_r = adjust_speed_pid(VKp_r, direction, 1U);
                PID_ResetVelocityRight();
            } else if (selection == 4U) {
                VKi_r = adjust_speed_pid(VKi_r, direction, 0U);
                PID_ResetVelocityRight();
            } else if (selection == 5U) {
                VKd_r = adjust_speed_pid(VKd_r, direction, 0U);
                PID_ResetVelocityRight();
            } else if ((selection == 6U) && (running == 0U)) {
                speed_mode_setpoint += (float)(direction * 5);
                if (speed_mode_setpoint < 20.0f) {
                    speed_mode_setpoint = 20.0f;
                }
                if (speed_mode_setpoint > 100.0f) {
                    speed_mode_setpoint = 100.0f;
                }
            } else if ((selection == 7U) && (running == 0U)) {
                if (direction > 0) {
                    run_wheels++;
                    if (run_wheels > 3U) run_wheels = 1U;
                } else {
                    if (run_wheels <= 1U) run_wheels = 3U;
                    else run_wheels--;
                }
            }
            if (selection <= 5U) __enable_irq();
        }

        /* 安全监控可能在控制中断中主动停车，同步本地显示/按键状态。 */
        if ((running != 0U) && (jb != 1)) {
            running = 0U;
        }

        /* VOFA CSV，20Hz：保留编码器/PID诊断，并追加同步启动目标和阶段。
         * 故障停车后仍发送最后一帧。 */
        if ((pid_send_flag != 0U) && ((jb == 1) || (speed_mode_fault != 0U))) {
            pid_send_flag = 0U;
            menu_send_speed_csv();
        }

        OLED_ShowString(0, 0, (uint8_t *)"2:SPD ", 16, 1);
        OLED_ShowString(48, 0,
            (uint8_t *)((speed_mode_fault != 0U) ? "FLT" :
                       (running != 0U) ? "ON " : "OFF"), 16, 1);
        OLED_ShowString(80, 0,
            (uint8_t *)((selection == 0U) ? "PL  " :
                       (selection == 1U) ? "IL  " :
                       (selection == 2U) ? "DL  " :
                       (selection == 3U) ? "PR  " :
                       (selection == 4U) ? "IR  " :
                       (selection == 5U) ? "DR  " :
                       (selection == 6U) ? "SPD " :
                       (selection == 7U) ? "RUN " : "EXIT"), 16, 1);

        OLED_ShowString(0, 16, (uint8_t *)"V", 16, 1);
        if (selection == 0U) OLED_ShowNum(8, 16, (int)(VKp_l * 10.0f + 0.5f), 4, 16, 1);
        else if (selection == 1U) OLED_ShowNum(8, 16, (int)(VKi_l * 10.0f + 0.5f), 4, 16, 1);
        else if (selection == 2U) OLED_ShowNum(8, 16, (int)(VKd_l * 10.0f + 0.5f), 4, 16, 1);
        else if (selection == 3U) OLED_ShowNum(8, 16, (int)(VKp_r * 10.0f + 0.5f), 4, 16, 1);
        else if (selection == 4U) OLED_ShowNum(8, 16, (int)(VKi_r * 10.0f + 0.5f), 4, 16, 1);
        else if (selection == 5U) OLED_ShowNum(8, 16, (int)(VKd_r * 10.0f + 0.5f), 4, 16, 1);
        else if (selection == 6U) OLED_ShowNum(
            8, 16, (int)speed_mode_setpoint, 4, 16, 1);
        else if (selection == 7U) OLED_ShowNum(8, 16, run_wheels, 4, 16, 1);
        else OLED_ShowNum(8, 16, 0, 4, 16, 1);
        OLED_ShowString(56, 16, (uint8_t *)"T", 16, 1);
        OLED_ShowNum(64, 16, (int)speed_mode_setpoint, 3, 16, 1);
        OLED_ShowString(104, 16,
            (uint8_t *)((run_wheels == 1U) ? "L " :
                       (run_wheels == 2U) ? "R " : "B "), 16, 1);

        OLED_ShowString(0, 32, (uint8_t *)"L", 16, 1);
        OLED_ShowNum(8, 32, filt_velocity_l, 3, 16, 1);
        OLED_ShowString(56, 32, (uint8_t *)"R", 16, 1);
        OLED_ShowNum(64, 32, filt_velocity_r, 3, 16, 1);

        OLED_ShowString(0, 48, (uint8_t *)"P", 16, 1);
        OLED_ShowNum(8, 48, last_pwm_l, 3, 16, 1);
        OLED_ShowString(56, 48, (uint8_t *)"P", 16, 1);
        OLED_ShowNum(64, 48, last_pwm_r, 3, 16, 1);
        OLED_ShowString(96, 48, (uint8_t *)"F", 16, 1);
        OLED_ShowNum(104, 48, speed_mode_fault, 1, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode3 - 循迹PID
**********************************************************/
#if 0
/* 旧模式三巡线界面停用；模式一已改为2026 H题第二问。 */
int menu4(void)
{
    uint8_t target_laps = 1U;
    OLED_Clear();

    while (1) {
        XunjiStatus status;
        uint8_t key = Key_GetNum();

        Xunji_GetStatus(&status);

        if ((key == KEY_NUM_NEXT) &&
            ((status.state == XUNJI_STATE_IDLE) ||
             (status.state == XUNJI_STATE_FINISHED))) {
            target_laps++;
            if (target_laps > 5U) {
                target_laps = 1U;
            }
        }

        if (key == KEY_NUM_CONFIRM) {
            if (status.state == XUNJI_STATE_FAULT) {
                /* 故障后第一次按K2只确认并清除故障，禁止自动重新启动。 */
                Xunji_Stop();
            } else if ((status.state == XUNJI_STATE_TRACK) ||
                (status.state == XUNJI_STATE_CORNER_LEFT) ||
                (status.state == XUNJI_STATE_CORNER_RIGHT) ||
                (status.state == XUNJI_STATE_CORNER_ADVANCE) ||
                (status.state == XUNJI_STATE_LOST)) {
                Xunji_Stop();
            } else {
                Control_ResetSpeedFeedback();
                Xunji_Start(target_laps);
            }
        }

        if (key == KEY_NUM_BACK) {
            Xunji_Stop();
            OLED_Clear();
            return 0;
        }

        Xunji_GetStatus(&status);

        /* OLED_Clear会立即向屏幕发送全黑帧；循环中调用会形成肉眼可见闪烁。 */
        OLED_ShowString(0, 0, (uint8_t *)"3:", 16, 1);
        OLED_ShowString(16, 0,
            (uint8_t *)((status.state == XUNJI_STATE_TRACK) ? "TRACK" :
                       (status.state == XUNJI_STATE_CORNER_LEFT) ? "LTURN" :
                       (status.state == XUNJI_STATE_CORNER_RIGHT) ? "RTURN" :
                       (status.state == XUNJI_STATE_CORNER_ADVANCE) ? "FWD  " :
                       (status.state == XUNJI_STATE_LOST) ? "LOST " :
                       (status.state == XUNJI_STATE_FINISHED) ? "DONE " :
                       (status.state == XUNJI_STATE_FAULT) ? "FAULT" : "IDLE "),
            16, 1);
        OLED_ShowString(88, 0, (uint8_t *)"N", 16, 1);
        OLED_ShowNum(100, 0, target_laps, 1, 16, 1);

        /* N=设定圈数，L=已完成圈数，C=本圈已确认真实角点数。 */
        OLED_ShowString(0, 16, (uint8_t *)"L", 16, 1);
        OLED_ShowNum(12, 16, status.completed_laps, 1, 16, 1);
        OLED_ShowChar(24, 16, '/', 16, 1);
        OLED_ShowNum(36, 16, target_laps, 1, 16, 1);
        OLED_ShowString(64, 16, (uint8_t *)"C", 16, 1);
        OLED_ShowNum(76, 16, status.corners_in_lap, 1, 16, 1);
        OLED_ShowChar(88, 16, '/', 16, 1);
        OLED_ShowNum(100, 16, 4, 1, 16, 1);

        if (status.state == XUNJI_STATE_FAULT) {
            OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)"FAULT", 16, 1);
            OLED_ShowNum(48, 32, status.fault, 1, 16, 1);
        } else {
            OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
            /* X=位置环滤波位置，A=当前触发探头数量，二者足够判断是否正常见线。 */
            OLED_ShowString(0, 32, (uint8_t *)"X", 16, 1);
            OLED_ShowNum(12, 32, status.control_position, 3, 16, 1);
            OLED_ShowString(64, 32, (uint8_t *)"A", 16, 1);
            OLED_ShowNum(76, 32, status.sensor.active_count, 1, 16, 1);
        }

        /* 最后一行只显示左右轮实际下发的目标速度。 */
        OLED_ShowString(0, 48, (uint8_t *)"L", 16, 1);
        OLED_ShowNum(12, 48, status.target_left, 3, 16, 1);
        OLED_ShowString(64, 48, (uint8_t *)"R", 16, 1);
        OLED_ShowNum(76, 48, status.target_right, 3, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}
#endif

static const char *menu_h2_stage_name(H2CalStage stage)
{
    switch (stage) {
        case H2_CAL_STAGE_AB:    return "AB";
        case H2_CAL_STAGE_BC:    return "BC";
        case H2_CAL_STAGE_CD:    return "CD";
        case H2_CAL_STAGE_DA:    return "DA";
        case H2_CAL_STAGE_DONE:  return "DONE";
        case H2_CAL_STAGE_FAULT: return "FLT";
        default:                 return "IDLE";
    }
}

static const char *menu_h2_segment_name(uint8_t index)
{
    static const char *names[4] = {"AB", "BC", "CD", "DA"};
    return (index < 4U) ? names[index] : "--";
}

/**********************************************************
              mode6 - finite straight task
**********************************************************/
int menu7(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* RUN, EXIT */
    char line[17];

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    pid_send_flag = 0U;
    OLED_Clear();

    while (1) {
        uint8_t key = Key_GetNum();
        XunjiStatus status;

        Xunji_GetStatus(&status);

        if ((key == KEY_NUM_NEXT) && (running == 0U)) {
            selection = (selection == 0U) ? 1U : 0U;
        } else if (key == KEY_NUM_CONFIRM) {
            if (status.fault != XUNJI_FAULT_NONE) {
                Xunji_Stop();
                running = 0U;
            } else if (running != 0U) {
                Xunji_Stop();
                running = 0U;
            } else if (selection != 0U) {
                Xunji_Stop();
                OLED_Clear();
                return 0;
            } else {
                Control_ResetSpeedFeedback();
                Xunji_StartStraightTask(
                    XUNJI_STRAIGHT_DISTANCE_COUNTS,
                    XUNJI_MODE6_POST_CRUISE_MS);
                running = 1U;
            }
        } else if (key == KEY_NUM_BACK) {
            Xunji_Stop();
            OLED_Clear();
            return 0;
        }

        if ((running != 0U) && (jb != 2)) running = 0U;
        Xunji_GetStatus(&status);

        (void)snprintf(line, 17U, "6:LINE %-4s %s",
                       (status.fault != XUNJI_FAULT_NONE) ? "FLT" :
                       (running != 0U) ? "ON" : "OFF",
                       (selection == 0U) ? "RUN" : "EXIT");
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "D%05ld/%05ld",
                       (long)status.total_distance,
                       (long)status.task_distance_target);
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "T%lu.%02lus P%u",
                       (unsigned long)(status.elapsed_ms / 1000U),
                       (unsigned long)((status.elapsed_ms / 10U) % 100U),
                       (unsigned)status.straight_phase);
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "L%02d/%02d R%02d/%02d",
                       status.target_left, filt_velocity_l,
                       status.target_right, filt_velocity_r);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode7 - AB/BC/CD/DA four-stage timing calibration
**********************************************************/
#if 0
int menu8(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* AB,BC,CD,DA,WGT,RUN,EXIT */
    char line[17];

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    IMU601_init();
    pid_send_flag = 0U;
    OLED_Clear();

    while (1) {
        H2MotionStatus status;
        uint8_t key;
        uint32_t selected_time = 0U;

        IMU601_Service();
        key = Key_GetNum();
        H2Motion_GetStatus(&status);

        if ((key == KEY_NUM_NEXT) && (running == 0U)) {
            selection++;
            if (selection > 6U) selection = 0U;
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (selection < 4U) && (running == 0U)) {
            int32_t updated = (int32_t)H2Timed_GetStageTime(selection);
            updated += (key == KEY_NUM_PLUS) ?
                (int32_t)H2_CAL_TIME_STEP_MS :
                -(int32_t)H2_CAL_TIME_STEP_MS;
            H2Timed_SetStageTime(selection, (uint32_t)
                ((updated < 500) ? 500 : updated));
        }

        /*
         * 停车时选中WGT可调；运行中K3/K4直接调整权重，便于实时观察循迹效果。
         * 权重采用x10定点值，按键每变化1对应实际权重变化0.1。
         */
        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (((selection == 4U) && (running == 0U)) ||
             (running != 0U))) {
            int updated_weight = H2Timed_GetLineWeightX10();
            updated_weight += (key == KEY_NUM_PLUS) ?
                H2_CAL_LINE_WEIGHT_STEP_X10 :
                -H2_CAL_LINE_WEIGHT_STEP_X10;
            H2Timed_SetLineWeightX10(updated_weight);
        }

        if (key == KEY_NUM_CONFIRM) {
            if (running != 0U) {
                H2Motion_Stop();
                running = 0U;
            } else if (selection == 5U) {
                Control_ResetSpeedFeedback();
                running = H2Timed_Start();
            } else if (selection == 6U) {
                H2Motion_Stop();
                OLED_Clear();
                return 0;
            }
        }

        if ((running != 0U) && (jb != 6)) running = 0U;
        H2Motion_GetStatus(&status);
        if (selection < 4U) {
            selected_time = H2Timed_GetStageTime(selection);
        }

        (void)snprintf(line, 17U, "7:%-4s %-3s F%02u",
                       menu_h2_stage_name(status.stage),
                       (status.waiting_imu != 0U) ? "IMU" :
                       (running != 0U) ? "ON" : "OFF",
                       (unsigned)status.fault);
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        if (running != 0U) {
            int weight_x10 = H2Timed_GetLineWeightX10();
            (void)snprintf(line, 17U, ">WGT %d.%d K3/K4",
                           weight_x10 / 10,
                           weight_x10 % 10);
        } else if (selection < 4U) {
            (void)snprintf(line, 17U, ">%s %05lums",
                           menu_h2_segment_name(selection),
                           (unsigned long)selected_time);
        } else if (selection == 4U) {
            int weight_x10 = H2Timed_GetLineWeightX10();
            (void)snprintf(line, 17U, ">WGT %d.%d K3/K4",
                           weight_x10 / 10,
                           weight_x10 % 10);
        } else {
            (void)snprintf(line, 17U, ">%-4s K2",
                           (selection == 5U) ? "RUN" : "EXIT");
        }
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        {
            uint8_t index = (status.stage >= H2_CAL_STAGE_AB &&
                             status.stage <= H2_CAL_STAGE_DA) ?
                (uint8_t)(status.stage - H2_CAL_STAGE_AB) : 0U;
            if ((status.stage == H2_CAL_STAGE_BC) ||
                (status.stage == H2_CAL_STAGE_DA)) {
                /*
                 * 弯道优先显示真正决定是否补转的累计角度和实时差速。
                 * 例如 A172 D31 T3700 表示已转172度、当前差速31、本段3700ms。
                 */
                (void)snprintf(line, 17U, "A%03d D%02d T%04lu",
                               (int)status.yaw_progress,
                               (int)status.curve_delta,
                               (unsigned long)status.stage_elapsed_ms);
            } else {
                (void)snprintf(line, 17U, "T%04lu/%04lu Y%03d",
                               (unsigned long)status.stage_elapsed_ms,
                               (unsigned long)status.stage_time_ms[index],
                               (int)status.yaw);
            }
        }
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "L%02d/%02d R%02d/%02d",
                       status.target_left, filt_velocity_l,
                       status.target_right, filt_velocity_r);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}
#endif

int menu8(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* TIME, WGT, RUN, EXIT */
    uint32_t run_time_ms = 15000U;
    char line[17];

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    pid_send_flag = 0U;
    OLED_Clear();

    while (1) {
        XunjiStatus status;
        uint8_t key = Key_GetNum();

        Xunji_GetStatus(&status);
        if ((key == KEY_NUM_NEXT) && (running == 0U)) {
            selection++;
            if (selection > 3U) selection = 0U;
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (selection == 0U) && (running == 0U)) {
            int32_t updated = (int32_t)run_time_ms +
                ((key == KEY_NUM_PLUS) ? 100 : -100);
            if (updated < 500) updated = 500;
            if (updated > 60000) updated = 60000;
            run_time_ms = (uint32_t)updated;
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (((selection == 1U) && (running == 0U)) ||
             (running != 0U))) {
            int weight_x10 = Xunji_GetTimedLineWeightX10();
            weight_x10 += (key == KEY_NUM_PLUS) ? 1 : -1;
            Xunji_SetTimedLineWeightX10(weight_x10);
        }

        if (key == KEY_NUM_CONFIRM) {
            if (running != 0U) {
                /* K2 is also the immediate stop while the timed run is active. */
                Xunji_Stop();
                running = 0U;
            } else if (status.fault != XUNJI_FAULT_NONE) {
                Xunji_Stop();
            } else if (selection == 2U) {
                Control_ResetSpeedFeedback();
                Xunji_StartTimedTask(run_time_ms);
                running = 1U;
            } else if (selection == 3U) {
                Xunji_Stop();
                OLED_Clear();
                return 0;
            }
        }

        if ((running != 0U) && (jb != 2)) running = 0U;
        Xunji_GetStatus(&status);

        (void)snprintf(line, 17U, "7:V80 %-3s F%02u",
                       (running != 0U) ? "ON" :
                       (status.state == XUNJI_STATE_FINISHED) ? "DON" : "OFF",
                       (unsigned)status.fault);
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        if (running != 0U) {
            int weight_x10 = Xunji_GetTimedLineWeightX10();
            (void)snprintf(line, 17U, ">WGT %d.%d K3/K4",
                           weight_x10 / 10, weight_x10 % 10);
        } else if (selection == 0U) {
            (void)snprintf(line, 17U, ">SET %lu.%02lus",
                           (unsigned long)(run_time_ms / 1000U),
                           (unsigned long)((run_time_ms / 10U) % 100U));
        } else if (selection == 1U) {
            int weight_x10 = Xunji_GetTimedLineWeightX10();
            (void)snprintf(line, 17U, ">WGT %d.%d K3/K4",
                           weight_x10 / 10, weight_x10 % 10);
        } else {
            (void)snprintf(line, 17U, ">%-4s K2",
                           (selection == 2U) ? "RUN" : "EXIT");
        }
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "T%lu.%02lu/%lu.%02lu",
                       (unsigned long)(status.elapsed_ms / 1000U),
                       (unsigned long)((status.elapsed_ms / 10U) % 100U),
                       (unsigned long)(run_time_ms / 1000U),
                       (unsigned long)((run_time_ms / 10U) % 100U));
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "L%02d/%02d R%02d/%02d",
                       status.target_left, filt_velocity_l,
                       status.target_right, filt_velocity_r);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode8 - fixed 180-degree clockwise curve calibration
**********************************************************/
int menu9(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* DELTA,RUN,EXIT */
    char line[17];

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    IMU601_init();
    pid_send_flag = 0U;
    OLED_Clear();

    while (1) {
        H2MotionStatus status;
        uint8_t key;

        IMU601_Service();
        key = Key_GetNum();
        H2Motion_GetStatus(&status);

        if ((key == KEY_NUM_NEXT) && (running == 0U)) {
            selection++;
            if (selection > 2U) selection = 0U;
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (selection == 0U) && (running == 0U)) {
            int delta = H2Curve_GetDelta();
            delta += (key == KEY_NUM_PLUS) ?
                H2_CAL_CURVE_DELTA_STEP : -H2_CAL_CURVE_DELTA_STEP;
            H2Curve_SetDelta(delta);
        }

        if (key == KEY_NUM_CONFIRM) {
            if (running != 0U) {
                H2Motion_Stop();
                running = 0U;
            } else if (selection == 1U) {
                Control_ResetSpeedFeedback();
                running = H2Curve_Start();
            } else if (selection == 2U) {
                H2Motion_Stop();
                OLED_Clear();
                return 0;
            }
        }

        if ((running != 0U) && (jb != 7)) running = 0U;
        H2Motion_GetStatus(&status);

        (void)snprintf(line, 17U, "8:CUR %-3s F%02u",
                       (status.waiting_imu != 0U) ? "IMU" :
                       (running != 0U) ? "ON" :
                       (status.stage == H2_CAL_STAGE_DONE) ? "DON" : "OFF",
                       (unsigned)status.fault);
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, ">%-5s D%02d A180",
                       (selection == 0U) ? "DELTA" :
                       (selection == 1U) ? "RUN" : "EXIT",
                       H2Curve_GetDelta());
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "Y%03d R%03d T%04lu",
                       (int)status.yaw_progress,
                       (int)((status.yaw_rate < 0.0f) ?
                             -status.yaw_rate : status.yaw_rate),
                       (unsigned long)status.total_elapsed_ms);
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "L%02d/%02d R%02d/%02d",
                       status.target_left, filt_velocity_l,
                       status.target_right, filt_velocity_r);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

#if 0
static const char *menu_h_state_name(HRouteState state)
{
    switch (state) {
        case H_ROUTE_STATE_STRAIGHT_AC: return "A-C";
        case H_ROUTE_STATE_ARC_CB:      return "C-B";
        case H_ROUTE_STATE_STRAIGHT_BD: return "B-D";
        case H_ROUTE_STATE_ARC_DA:      return "D-A";
        case H_ROUTE_STATE_ALIGN_A:     return "A-ADJ";
        case H_ROUTE_STATE_FINISHED:    return "DONE";
        case H_ROUTE_STATE_FAULT:       return "FAULT";
        default:                        return "IDLE";
    }
}

/**********************************************************
              mode4 - 2024 H题第三问
**********************************************************/
int menu5(void)
{
    uint8_t running = 0U;

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    IMU601_init();
    OLED_Clear();

    while (1) {
        HRouteStatus status;
        IMU601_Snapshot_t imu;
        char line[17];
        uint8_t key = Key_GetNum();

        IMU601_Service();
        HRoute_GetStatus(&status);
        IMU601_GetSnapshot(&imu);

        /* 停车时K1减圈、K4加圈；运行中锁定目标，避免误按改变终点。 */
        if (running == 0U) {
            if ((key == KEY_NUM_NEXT) && (status.target_laps > 1U)) {
                HRoute_SetLapTarget((uint8_t)(status.target_laps - 1U));
            } else if ((key == KEY_NUM_PLUS) && (status.target_laps < 5U)) {
                HRoute_SetLapTarget((uint8_t)(status.target_laps + 1U));
            }
        }

        if (key == KEY_NUM_CONFIRM) {
            if (running != 0U) {
                HRoute_Stop();
                running = 0U;
            } else if (status.state == H_ROUTE_STATE_FAULT) {
                /* 第一次K2只确认故障；摆正车头或恢复IMU后再按一次才启动。 */
                HRoute_Stop();
            } else if (IMU601_IsReady() == 0U) {
                /*
                 * Never calibrate yaw after the car has started moving.
                 * Wait for a post-calibration frame before allowing mode 4.
                 */
                running = 0U;
            } else {
                Control_ResetSpeedFeedback();
                running = HRoute_Start();
            }
        }

        if (key == KEY_NUM_BACK) {
            HRoute_Stop();
            OLED_Clear();
            return 0;
        }

        if ((running != 0U) && (jb != 3)) running = 0U;
        HRoute_GetStatus(&status);

        /* 不清屏，只逐行覆盖，避免运行时OLED闪烁干扰控制和观察。 */
        if (IMU601_GetInitFault() != 0U) {
            (void)snprintf(line, 17U, "4:IMU TX ERROR  ");
        } else if (IMU601_IsReady() == 0U) {
            (void)snprintf(line, 17U, "4:IMU STARTING  ");
        } else {
            (void)snprintf(line, 17U, "4:%-5s N%u F%02u",
                           menu_h_state_name(status.state),
                           (unsigned)status.target_laps,
                           (unsigned)status.fault);
        }
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        /* 空闲时直接显示实时IMU，便于把车头预先摆到A->C的38度。 */
        if ((status.state == H_ROUTE_STATE_IDLE) ||
            (status.state == H_ROUTE_STATE_FAULT)) {
            status.yaw = imu.attitude.yaw;
        }
        (void)snprintf(line, 17U, "Y%03d T%03d E%+03d",
                       (int)status.yaw,
                       (int)status.target_yaw,
                       (int)status.heading_error);
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "P%u L%u/%u A%uE%u B%02X",
                       (unsigned)status.point_index,
                       (unsigned)status.completed_laps,
                       (unsigned)status.target_laps,
                       (unsigned)status.arc_line_acquired,
                       (unsigned)status.arc_exit_armed,
                       (unsigned)status.sensor.bits);
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "L%02d R%02d V%02d/%02d",
                       status.target_left, status.target_right,
                       filt_velocity_l, filt_velocity_r);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode5 - live position-loop PID tuning
**********************************************************/
#endif

/**
 * 模式三/四共用滚球调参界面。
 *
 * K1切换参数；K3减小；K4增大；K2在RUN/EXIT项确认。运行中K2立即执行
 * 安全中止，时间、差速和权重仅在停车时修改，避免误按造成瞬时轨迹变化。
 */
static const char *menu_hball_phase_name(HBallRunPhase phase)
{
    switch (phase) {
        case H_BALL_PHASE_ROUTE:             return "RUN";
        case H_BALL_PHASE_POST_POINT_CRUISE: return "PASS";
        case H_BALL_PHASE_POST_POINT_DECEL:  return "DEC";
        case H_BALL_PHASE_STOPPING:          return "STOP";
        default:                             return "OFF";
    }
}

static int menu_ball_route(HBallRouteMode mode, uint8_t menu_number)
{
    uint8_t running = 0U;
    uint8_t selection = 0U;
    const uint8_t has_curve_tuning =
        (uint8_t)(mode == H_BALL_ROUTE_Q5);
    const uint8_t weight_selection = has_curve_tuning ? 2U : 1U;
    const uint8_t run_selection = has_curve_tuning ? 3U : 2U;
    const uint8_t exit_selection = has_curve_tuning ? 4U : 3U;
    int expected_jb = (mode == H_BALL_ROUTE_Q5) ? 10 : 9;
    char line[17];

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    HBall_Abort();
    IMU601_init();
    pid_send_flag = 0U;
    OLED_Clear();

    while (1) {
        HBallRouteStatus status;
        uint8_t key;
        uint32_t selected_time = 0U;

        IMU601_Service();
        key = Key_GetNum();
        HBall_GetStatus(&status);

        if ((key == KEY_NUM_NEXT) && (running == 0U)) {
            selection++;
            if (selection > exit_selection) selection = 0U;
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (running == 0U)) {
            int direction = (key == KEY_NUM_PLUS) ? 1 : -1;

            if (selection == 0U) {
                int32_t updated =
                    (int32_t)HBall_GetStageTime(mode, 0U);
                updated += direction * (int32_t)H_BALL_TIME_STEP_MS;
                HBall_SetStageTime(mode, 0U, (uint32_t)
                    ((updated < 500) ? 500 : updated));
            } else if ((has_curve_tuning != 0U) &&
                       (selection == 1U)) {
                HBall_SetCurveDelta(
                    mode,
                    HBall_GetCurveDelta(mode) +
                    direction * H_BALL_CURVE_DELTA_STEP);
            } else if (selection == weight_selection) {
                HBall_SetLineWeightX10(
                    mode,
                    HBall_GetLineWeightX10(mode) +
                    direction * H_BALL_WEIGHT_STEP_X10);
            }
        }

        if (key == KEY_NUM_CONFIRM) {
            if (running != 0U) {
                HBall_Abort();
                running = 0U;
            } else if (selection == run_selection) {
                Control_ResetSpeedFeedback();
                running = HBall_Start(mode);
            } else if (selection == exit_selection) {
                HBall_Abort();
                OLED_Clear();
                return 0;
            }
        }

        if ((running != 0U) && (jb != expected_jb)) {
            running = 0U;
        }
        HBall_GetStatus(&status);
        if (selection == 0U) {
            selected_time = HBall_GetStageTime(mode, 0U);
        }

        (void)snprintf(line, 17U, "%u:Q%u %-4s F%02u",
                       (unsigned)menu_number,
                       (unsigned)mode,
                       (running != 0U) ?
                           menu_h2_stage_name(status.stage) :
                       (status.stage == H2_CAL_STAGE_DONE) ? "DONE" :
                       (status.stage == H2_CAL_STAGE_FAULT) ? "FLT" :
                           "OFF",
                       (unsigned)status.fault);
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        if (selection == 0U) {
            (void)snprintf(line, 17U, ">AB %05lums",
                           (unsigned long)selected_time);
        } else if ((has_curve_tuning != 0U) &&
                   (selection == 1U)) {
            (void)snprintf(line, 17U, ">DEL %02d K3/K4",
                           HBall_GetCurveDelta(mode));
        } else if (selection == weight_selection) {
            int weight_x10 = HBall_GetLineWeightX10(mode);
            (void)snprintf(line, 17U, ">WGT %d.%d K3/K4",
                           weight_x10 / 10,
                           weight_x10 % 10);
        } else {
            (void)snprintf(line, 17U, ">%-4s K2",
                           (selection == run_selection) ? "RUN" : "EXIT");
        }
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(
            line, 17U, "T%02lu.%02lus %-4s",
            (unsigned long)(status.total_elapsed_ms / 1000U),
            (unsigned long)((status.total_elapsed_ms % 1000U) / 10U),
            menu_hball_phase_name(status.phase));
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "L%02d/%02d R%02d/%02d",
                       status.target_left, filt_velocity_l,
                       status.target_right, filt_velocity_r);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

/**********************************************************
              mode4 - H题第五问：平稳完整一圈
**********************************************************/
int menu5(void)
{
    return menu_ball_route(H_BALL_ROUTE_Q5, 4U);
}

/**********************************************************
              mode5 - live position-loop PID tuning
**********************************************************/
int menu6(void)
{
    uint8_t running = 0U;
    uint8_t selection = 0U; /* KP, KI, KD, RUN, EXIT */

    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    HPositionTune_Stop();
    OLED_Clear();

    while (1) {
        XunjiStatus status;
        char line[17];
        uint8_t key = Key_GetNum();

        Xunji_GetStatus(&status);

        if (key == KEY_NUM_NEXT) {
            selection++;
            if (selection > 4U) selection = 0U;
        }

        if (key == KEY_NUM_CONFIRM) {
            if (status.fault != XUNJI_FAULT_NONE) {
                /* First confirmation after an electrical fault only clears
                 * the latch. A second deliberate press is required to run. */
                Xunji_Stop();
                running = 0U;
            } else if ((running == 0U) && (selection == 4U)) {
                Xunji_Stop();
                OLED_Clear();
                return 0;
            } else if (running != 0U) {
                Xunji_Stop();
                running = 0U;
            } else {
                Control_ResetSpeedFeedback();
                pid_send_flag = 0U;
                Xunji_StartStraightTask(
                    H2_FINAL_AB_DISTANCE_COUNTS +
                    H2_FINAL_BC_DISTANCE_COUNTS +
                    H2_FINAL_CD_DISTANCE_COUNTS +
                    H2_FINAL_DA_DISTANCE_COUNTS,
                    XUNJI_MODE5_POST_CRUISE_MS);
                running = 1U;
            }
        }

        if (((key == KEY_NUM_BACK) || (key == KEY_NUM_PLUS)) &&
            (selection <= 2U)) {
            int direction = (key == KEY_NUM_PLUS) ? 1 : -1;
            uint32_t interrupt_state = __get_PRIMASK();
            __disable_irq();
            if (selection == 0U) {
                place_Kp = adjust_position_pid(place_Kp, direction);
            } else if (selection == 1U) {
                place_Ki = adjust_position_pid(place_Ki, direction);
            } else {
                place_Kd = adjust_position_pid(place_Kd, direction);
            }
            /* Live gain changes must not inherit the old I/D history. */
            PID_ResetPlace();
            if (interrupt_state == 0U) __enable_irq();
        }

        if ((running != 0U) && (jb != 2)) running = 0U;
        Xunji_GetStatus(&status);

        (void)snprintf(line, 17U, "5:POS %-4s %-4s",
                       (status.fault != XUNJI_FAULT_NONE) ? "FLT" :
                       (status.state == XUNJI_STATE_LOST) ? "LOST" :
                       (running != 0U) ? "ON" : "OFF",
                       (selection == 0U) ? "KP" :
                       (selection == 1U) ? "KI" :
                       (selection == 2U) ? "KD" :
                       (selection == 3U) ? "RUN" : "EXIT");
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "V%05d     F%02u",
                       (selection == 0U) ? position_pid_scaled(place_Kp) :
                       (selection == 1U) ? position_pid_scaled(place_Ki) :
                       (selection == 2U) ? position_pid_scaled(place_Kd) : 0,
                       (unsigned)status.fault);
        OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "T%lu.%02lus D%04ld",
                       (unsigned long)(status.elapsed_ms / 1000U),
                       (unsigned long)((status.elapsed_ms / 10U) % 100U),
                       (long)status.total_distance);
        OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        (void)snprintf(line, 17U, "X%03d C%+03d P%u",
                       status.control_position,
                       status.control_correction,
                       (unsigned)status.straight_phase);
        OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        delay_ms(50);
    }
}

#if 0
static const char *menu_imu_state_name(const IMU601_Snapshot_t *imu,
                                       uint8_t stale)
{
    if (imu->state == IMU601_STATE_TX_FAULT) return "TXERR";
    if (imu->state == IMU601_STATE_NO_FRAME) return "TIMEO";
    if (imu->raw_bytes == 0U) return "NORX ";
    if (imu->valid_frames == 0U) {
        if (imu->header_hits == 0U) return "NOHDR";
        if (imu->checksum_errors != 0U) return "CSERR";
        return "FRAME";
    }
    if (stale != 0U) return "STALE";
    if (imu->state != IMU601_STATE_READY) return "INIT ";
    return "OK   ";
}

/**
 * @brief 模式三：显示IMU姿态和UART诊断，不启动巡线或速度环。
 *
 * K1切换姿态/诊断页，K2重新初始化IMU，K3返回。没有有效帧时自动显示
 * 诊断页，从而区分无RX字节、无AA55帧头、校验失败和UART硬件错误。
 */
int menu4(void)
{
    uint32_t previous_frames = 0U;
    uint8_t stale_ticks = 0U;
    uint8_t diagnostic_page = 0U;

    /* 进入模式三前统一急停，确保旧模式不会遗留电机控制输出。 */
    SpeedMode_Stop();
    Xunji_Stop();
    HRoute_Stop();
    H2Mileage_Stop();
    H2Motion_Stop();
    IMU601_init();
    OLED_Clear();

    while (1) {
        IMU601_Snapshot_t imu;
        char line[17];
        uint8_t key = Key_GetNum();
        uint8_t stale;

        IMU601_Service();

        if (key == KEY_NUM_NEXT) {
            diagnostic_page = (diagnostic_page == 0U) ? 1U : 0U;
        } else if (key == KEY_NUM_CONFIRM) {
            IMU601_Restart();
            previous_frames = 0U;
            stale_ticks = 0U;
            diagnostic_page = 1U;
        }

        if (key == KEY_NUM_BACK) {
            /* 离开显示页也急停，给未来的模式三扩展保留安全边界。 */
            Xunji_Stop();
            OLED_Clear();
            return 0;
        }

        IMU601_GetSnapshot(&imu);
        if (imu.valid_frames != previous_frames) {
            previous_frames = imu.valid_frames;
            stale_ticks = 0U;
        } else if (stale_ticks < 255U) {
            stale_ticks++;
        }
        stale = (uint8_t)((imu.valid_frames != 0U) &&
                          (stale_ticks >= 20U));

        /* 循环内不清屏，逐行覆盖，避免OLED闪烁。 */
        OLED_ShowString(0, 0, (uint8_t *)"                ", 16, 1);
        (void)snprintf(line, 17U, "3:IMU %-5s R%u",
                       menu_imu_state_name(&imu, stale),
                       (unsigned)imu.retry_count);
        OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);

        /*
         * Before the first valid frame force diagnostics onto the screen.
         * After data is valid, K1 can switch between diagnostics and angles.
         */
        if ((diagnostic_page != 0U) || (imu.valid_frames == 0U)) {
            (void)snprintf(line, 17U, "RX%05lu H%04lu",
                           (unsigned long)(imu.raw_bytes % 100000U),
                           (unsigned long)(imu.header_hits % 10000U));
            OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

            (void)snprintf(line, 17U, "OK%04lu C%04lu U%02lu",
                           (unsigned long)(imu.valid_frames % 10000U),
                           (unsigned long)(imu.checksum_errors % 10000U),
                           (unsigned long)(imu.uart_errors % 100U));
            OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

            (void)snprintf(line, 17U, "B%02X %02X %02X %02X",
                           (unsigned)imu.last_bytes[0],
                           (unsigned)imu.last_bytes[1],
                           (unsigned)imu.last_bytes[2],
                           (unsigned)imu.last_bytes[3]);
            OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        } else {
            menu_format_imu_angle('Y', imu.attitude.yaw, line);
            OLED_ShowString(0, 16, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);
            menu_format_imu_angle('P', imu.attitude.pitch, line);
            OLED_ShowString(0, 32, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);
            menu_format_imu_angle('R', imu.attitude.roll, line);
            OLED_ShowString(0, 48, (uint8_t *)"                ", 16, 1);
            OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        }

        OLED_Refresh();
        delay_ms(50);
    }
}
#endif

/**********************************************************
              mode3 - H题第四问：平稳通过AB直线
**********************************************************/
int menu4(void)
{
    return menu_ball_route(H_BALL_ROUTE_Q4, 3U);
}
