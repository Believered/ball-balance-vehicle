#include "h_route.h"
#include "imu601.h"
#include "moter.h"
#include "pid.h"
#include "encoder.h"
#include <string.h>

extern volatile int jb;
extern volatile float purpose;
extern volatile int last_pwm_l;
extern volatile int last_pwm_r;

/* ===================== 模式四实车可调关键参数 =====================
 * 所有时间参数先以ms给出，再统一换算成10ms控制拍，后期无需手算ticks。
 * 修改顺序建议：实际角度 -> 出圈确认时间 -> 速度 -> 容差/超时。 */
#define H_CONTROL_PERIOD_MS                10U
#define H_MS_TO_TICKS(ms)                  (((ms) + H_CONTROL_PERIOD_MS - 1U) / H_CONTROL_PERIOD_MS)

/* 速度60已经由模式二实车验证；弧线降到48，为40cm半径保留横向修正余量。 */
#define H_STRAIGHT_SPEED                  60
#define H_ARC_SPEED                       48
#define H_FINAL_ALIGN_SPEED               36
#define H_MIN_WHEEL_SPEED                 14
#define H_HEADING_CORRECTION_LIMIT        22
#define H_FINAL_ALIGN_CORRECTION_LIMIT     28
#define H_ARC_CORRECTION_LIMIT            28
#define H_ARC_SEARCH_OUTER_SPEED           60
#define H_ARC_SEARCH_INNER_SPEED           22
#define H_ARC_SEARCH_CONFIRM_MS             20U
#define H_ARC_SEARCH_CONFIRM_TICKS          H_MS_TO_TICKS(H_ARC_SEARCH_CONFIRM_MS)
#define H_ARC_CENTER_DEADBAND               60
#define H_POSITION_PID_PERIOD_MS             20U
#define H_POSITION_PID_DIVIDER               H_MS_TO_TICKS(H_POSITION_PID_PERIOD_MS)
#define H_HEADING_D_TIME_SCALE              5.0f /* 原50ms航向Kd换算到10ms */
#define H_TARGET_RISE_STEP                  1  /* 原6/50ms换算为约1/10ms */
#define H_TARGET_FALL_STEP                  4  /* 原18/50ms换算为约4/10ms */

/*
 * 模式五必须和模式一使用同一套低冲击工况，否则调出的方向环不能直接迁移。
 * 模式四仍保留原路线速度，避免本轮修改扩大到尚未要求重构的任务。
 */
#define H_POSITION_TUNE_CORRECTION_PERCENT  35
#define H_POSITION_TUNE_TARGET_SLEW_STEP     1
#define H_POSITION_TUNE_CORRECTION_SLEW_STEP 1
#define H_POSITION_TUNE_PWM_SLEW_STEP       12

/*
 * 当前没有可靠的“计数/厘米”实车标定，因此不把参考工程的1042照抄过来。
 * 里程只负责屏蔽起点旧黑线和提供极宽的超程保护，终点必须由灰度连续确认；
 * 弧线出口还必须同时满足IMU切线角门控。
 */
#define H_STRAIGHT_LINE_ENABLE_COUNTS    600
#define H_STRAIGHT_MAX_COUNTS          12000
#define H_START_LINE_RELEASE_MS             50U
#define H_POINT_LINE_CONFIRM_MS             30U
#define H_ARC_MIN_TRACK_MS                 1500U /* 进弯后至少循迹1.5s才允许全白判终点 */
#define H_ARC_EXIT_CONFIRM_MS               30U /* 用户要求：出圈全白确认30ms */
#define H_ARC_ACQUIRE_CONFIRM_MS            50U
#define H_ARC_ACQUIRE_GUARD_MS             200U
#define H_START_LINE_RELEASE_TICKS          H_MS_TO_TICKS(H_START_LINE_RELEASE_MS)
#define H_POINT_LINE_CONFIRM_TICKS          H_MS_TO_TICKS(H_POINT_LINE_CONFIRM_MS)
#define H_ARC_MIN_TRACK_TICKS               H_MS_TO_TICKS(H_ARC_MIN_TRACK_MS)
#define H_ARC_EXIT_WHITE_TICKS              H_MS_TO_TICKS(H_ARC_EXIT_CONFIRM_MS)
#define H_ARC_ACQUIRE_CONFIRM_TICKS          H_MS_TO_TICKS(H_ARC_ACQUIRE_CONFIRM_MS)
#define H_ARC_ACQUIRE_GUARD_TICKS            H_MS_TO_TICKS(H_ARC_ACQUIRE_GUARD_MS)
#define H_ARC_TRACK_MAX_ACTIVE               4U
#define H_START_YAW_TOLERANCE_DEG          20.0f
#define H_FINAL_ALIGN_YAW_TOLERANCE_DEG     4.0f
#define H_FINAL_ALIGN_CONFIRM_MS             50U
#define H_FINAL_ALIGN_TIMEOUT_MS           3000U
#define H_IMU_STALE_MS                     1000U
#define H_ROUTE_TIMEOUT_MS_PER_LAP        39000U
#define H_SEGMENT_TIMEOUT_MS               9000U
#define H_FINAL_ALIGN_CONFIRM_TICKS          H_MS_TO_TICKS(H_FINAL_ALIGN_CONFIRM_MS)
#define H_FINAL_ALIGN_TIMEOUT_TICKS          H_MS_TO_TICKS(H_FINAL_ALIGN_TIMEOUT_MS)
#define H_IMU_STALE_TICKS                    H_MS_TO_TICKS(H_IMU_STALE_MS)
#define H_ROUTE_TIMEOUT_TICKS_PER_LAP        H_MS_TO_TICKS(H_ROUTE_TIMEOUT_MS_PER_LAP)
#define H_SEGMENT_TIMEOUT_TICKS              H_MS_TO_TICKS(H_SEGMENT_TIMEOUT_MS)

/* 无电流传感器时，以“高PWM+低反馈”作为驱动异常的最后一道保护。 */
#define H_STALL_PWM                        800
#define H_STALL_CONFIRM_MS                  300U
#define H_OVERSPEED_CONFIRM_MS              150U
#define H_STALL_CONFIRM_WINDOWS              H_MS_TO_TICKS(H_STALL_CONFIRM_MS)
#define H_OVERSPEED_CONFIRM_WINDOWS          H_MS_TO_TICKS(H_OVERSPEED_CONFIRM_MS)

float h_heading_kp = 1.00f;
float h_heading_kd = 1.20f;

typedef struct {
    HRouteStatus status;
    uint32_t last_imu_frames;
    uint8_t imu_live_confirmed;
    uint16_t total_ticks;
    uint8_t line_release_ticks;
    uint8_t line_released;
    uint8_t line_confirm_ticks;
    uint8_t white_ticks;
    uint8_t arc_lost_ticks;
    uint8_t arc_endpoint_candidate;
    uint8_t ignore_distance_window;
    uint8_t heading_initialized;
    float last_heading_error;
    int16_t filtered_position;
    uint8_t position_initialized;
    uint8_t position_pid_divider;
    int16_t arc_correction_request;
    int16_t last_arc_correction;
    int8_t search_direction;
    uint8_t arc_acquire_ticks;
    uint8_t arc_line_acquired;
    uint8_t arc_exit_armed;
    uint8_t final_align_confirm_ticks;
    int16_t applied_left;
    int16_t applied_right;
    uint8_t stall_left_windows;
    uint8_t stall_right_windows;
    uint8_t overspeed_left_windows;
    uint8_t overspeed_right_windows;
} HRouteController;

static HRouteController s_hroute;
static uint8_t s_hroute_target_laps = 1U;

#define H_TUNE_LINE_REACQUIRE_MS          50U
#define H_TUNE_LINE_REACQUIRE_TICKS       H_MS_TO_TICKS(H_TUNE_LINE_REACQUIRE_MS)

typedef struct {
    HPositionTuneStatus status;
    int32_t filtered_position_x4;
    uint8_t position_initialized;
    uint8_t position_pid_divider;
    uint8_t line_valid_ticks;
    int16_t correction_request;
    int16_t applied_correction;
    int16_t applied_left;
    int16_t applied_right;
    int16_t pwm_left;
    int16_t pwm_right;
    uint8_t stall_left_windows;
    uint8_t stall_right_windows;
    uint8_t overspeed_left_windows;
    uint8_t overspeed_right_windows;
} HPositionTuneController;

static HPositionTuneController s_hposition_tune;

static int h_abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static int h_clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float h_wrap_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float h_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int h_approach(int current, int target)
{
    if (current < target) {
        current += H_TARGET_RISE_STEP;
        if (current > target) current = target;
    } else if (current > target) {
        current -= H_TARGET_FALL_STEP;
        if (current < target) current = target;
    }
    return current;
}

static int h_approach_step(int current, int target, int step)
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

static uint8_t h_state_is_running(HRouteState state)
{
    return (uint8_t)((state == H_ROUTE_STATE_STRAIGHT_AC) ||
                     (state == H_ROUTE_STATE_ARC_CB) ||
                     (state == H_ROUTE_STATE_STRAIGHT_BD) ||
                     (state == H_ROUTE_STATE_ARC_DA) ||
                     (state == H_ROUTE_STATE_ALIGN_A));
}

static void h_clear_transition_evidence(void)
{
    s_hroute.status.segment_distance = 0;
    s_hroute.status.state_ticks = 0U;
    s_hroute.line_release_ticks = 0U;
    s_hroute.line_released = 0U;
    s_hroute.line_confirm_ticks = 0U;
    s_hroute.white_ticks = 0U;
    s_hroute.arc_lost_ticks = 0U;
    s_hroute.arc_endpoint_candidate = 0U;
    s_hroute.ignore_distance_window = 1U;
    s_hroute.heading_initialized = 0U;
    s_hroute.position_initialized = 0U;
    s_hroute.position_pid_divider = 0U;
    s_hroute.arc_correction_request = 0;
    s_hroute.last_arc_correction = 0;
    s_hroute.search_direction = 0;
    s_hroute.arc_acquire_ticks = 0U;
    s_hroute.arc_line_acquired = 0U;
    s_hroute.arc_exit_armed = 0U;
    s_hroute.final_align_confirm_ticks = 0U;
    s_hroute.status.arc_lost_ticks = 0U;
    s_hroute.status.search_direction = 0;
    s_hroute.status.arc_line_acquired = 0U;
    s_hroute.status.arc_exit_armed = 0U;
    PID_ResetPlace();
}

static void h_enter_state(HRouteState state)
{
    s_hroute.status.state = state;
    h_clear_transition_evidence();

    if (state == H_ROUTE_STATE_STRAIGHT_AC) {
        s_hroute.status.target_yaw = H_ROUTE_HEADING_AC_DEG;
        s_hroute.status.point_index = 0U;
    } else if (state == H_ROUTE_STATE_ARC_CB) {
        /* C->B沿圆弧到B时切线为180°；全白出圈后才切换到B->D的140°。 */
        s_hroute.status.target_yaw = H_ROUTE_ARC_CB_EXIT_YAW_DEG;
        s_hroute.status.point_index = 1U;
        /* 从38度沿右半圆转向180度；探头一旦给出明确偏差会覆盖该兜底值。 */
        s_hroute.search_direction = 1;
        s_hroute.status.search_direction = 1;
    } else if (state == H_ROUTE_STATE_STRAIGHT_BD) {
        s_hroute.status.target_yaw = H_ROUTE_HEADING_BD_DEG;
        s_hroute.status.point_index = 2U;
    } else if (state == H_ROUTE_STATE_ARC_DA) {
        /* D->A沿圆弧到A时切线为0°；全白出圈后才切换到38°。 */
        s_hroute.status.target_yaw = H_ROUTE_ARC_DA_EXIT_YAW_DEG;
        s_hroute.status.point_index = 3U;
        s_hroute.search_direction = -1;
        s_hroute.status.search_direction = -1;
    } else if (state == H_ROUTE_STATE_ALIGN_A) {
        /* 最后一圈已经以0°离开A点黑线，独立调整到38°后才能DONE。 */
        s_hroute.status.target_yaw = H_ROUTE_HEADING_AC_DEG;
        s_hroute.status.point_index = 4U;
    }
}

static void h_fault(HRouteFault fault)
{
    s_hroute.status.fault = fault;
    s_hroute.status.state = H_ROUTE_STATE_FAULT;
    s_hroute.status.target_left = 0;
    s_hroute.status.target_right = 0;
    s_hroute.applied_left = 0;
    s_hroute.applied_right = 0;
    jb = 0;
    PID_ResetAll();
    if ((fault == H_ROUTE_FAULT_STALL_LEFT) ||
        (fault == H_ROUTE_FAULT_STALL_RIGHT) ||
        (fault == H_ROUTE_FAULT_OVERSPEED_LEFT) ||
        (fault == H_ROUTE_FAULT_OVERSPEED_RIGHT)) {
        /* 高PWM低反馈可能是堵转/接触不良，此类电气故障高阻断开更安全。 */
        Motor_Coast();
    } else {
        /* 路线/传感器故障属于正常运动急停，使用50ms短路制动。 */
        Motor_Stop();
    }
}

static void h_finish(void)
{
    s_hroute.status.state = H_ROUTE_STATE_FINISHED;
    s_hroute.status.fault = H_ROUTE_FAULT_NONE;
    s_hroute.status.point_index = 4U;
    s_hroute.status.target_left = 0;
    s_hroute.status.target_right = 0;
    s_hroute.applied_left = 0;
    s_hroute.applied_right = 0;
    jb = 0;
    PID_ResetAll();
    /* 到A点属于正常必要停车，使用统一短路急停而非自然滑停。 */
    Motor_Stop();
}

void HRoute_Init(void)
{
    memset(&s_hroute, 0, sizeof(s_hroute));
    s_hroute.status.state = H_ROUTE_STATE_IDLE;
    s_hroute.status.target_yaw = H_ROUTE_HEADING_AC_DEG;
    s_hroute.status.target_laps = s_hroute_target_laps;
}

void HRoute_SetLapTarget(uint8_t laps)
{
    if (laps < 1U) laps = 1U;
    if (laps > 5U) laps = 5U;
    s_hroute_target_laps = laps;
    s_hroute.status.target_laps = laps;
}

uint8_t HRoute_Start(void)
{
    IMU601_Snapshot_t imu;
    float start_error;

    jb = 0;
    IMU601_GetSnapshot(&imu);
    HRoute_Init();
    s_hroute.status.yaw = imu.attitude.yaw;
    s_hroute.last_imu_frames = imu.valid_frames;

    if (imu.valid_frames == 0U) {
        s_hroute.status.state = H_ROUTE_STATE_FAULT;
        s_hroute.status.fault = H_ROUTE_FAULT_IMU_NOT_READY;
        Motor_Stop();
        return 0U;
    }

    start_error = h_wrap_180(imu.attitude.yaw - H_ROUTE_HEADING_AC_DEG);
    s_hroute.status.heading_error = start_error;
    if (h_abs_float(start_error) > H_START_YAW_TOLERANCE_DEG) {
        /* WHY: 起点直接转38度会偏离A-C直线；要求摆车时先把车头对准38度。 */
        s_hroute.status.state = H_ROUTE_STATE_FAULT;
        s_hroute.status.fault = H_ROUTE_FAULT_START_HEADING;
        Motor_Stop();
        return 0U;
    }

    PID_ResetAll();
    h_enter_state(H_ROUTE_STATE_STRAIGHT_AC);
    jb = 3;
    Set_Pwm(0, 0, 0, 0);
    return 1U;
}

void HRoute_Stop(void)
{
    jb = 0;
    PID_ResetAll();
    Motor_Stop();
    HRoute_Init();
}

void HRoute_Tick10ms(void)
{
    IMU601_Snapshot_t imu;
    XunjiSensor sensor;

    if ((jb != 3) || (h_state_is_running(s_hroute.status.state) == 0U)) {
        return;
    }

    IMU601_GetSnapshot(&imu);
    Xunji_ReadSensorSnapshot(&sensor);
    s_hroute.status.sensor = sensor;
    s_hroute.status.yaw = imu.attitude.yaw;
    s_hroute.status.state_ticks++;
    s_hroute.total_ticks++;

    if (imu.valid_frames != s_hroute.last_imu_frames) {
        s_hroute.last_imu_frames = imu.valid_frames;
        s_hroute.status.imu_stale_ticks = 0U;
        if (s_hroute.imu_live_confirmed == 0U) {
            /* 必须看到启动后的新帧，禁止拿历史快照驱动车轮。 */
            if (h_abs_float(h_wrap_180(imu.attitude.yaw -
                                       H_ROUTE_HEADING_AC_DEG)) >
                H_START_YAW_TOLERANCE_DEG) {
                h_fault(H_ROUTE_FAULT_START_HEADING);
                return;
            }
            s_hroute.imu_live_confirmed = 1U;
        }
    } else if (s_hroute.status.imu_stale_ticks < 255U) {
        s_hroute.status.imu_stale_ticks++;
    }
    if (s_hroute.status.imu_stale_ticks >= H_IMU_STALE_TICKS) {
        h_fault(H_ROUTE_FAULT_IMU_STALE);
        return;
    }
    if ((uint32_t)s_hroute.total_ticks >=
        H_ROUTE_TIMEOUT_TICKS_PER_LAP * (uint32_t)s_hroute.status.target_laps) {
        h_fault(H_ROUTE_FAULT_ROUTE_TIMEOUT);
        return;
    }

    /* 最后一圈A点已经完成“0°出圈+全白50ms”，此状态只负责把姿态调整到38°。
     * 它不是巡线状态，因此无论灰度是否全白都绝不能累计F05。 */
    if (s_hroute.status.state == H_ROUTE_STATE_ALIGN_A) {
        float align_error = h_wrap_180(s_hroute.status.yaw -
                                       H_ROUTE_HEADING_AC_DEG);

        s_hroute.status.heading_error = align_error;
        if (h_abs_float(align_error) <= H_FINAL_ALIGN_YAW_TOLERANCE_DEG) {
            if (s_hroute.final_align_confirm_ticks < 255U) {
                s_hroute.final_align_confirm_ticks++;
            }
        } else {
            s_hroute.final_align_confirm_ticks = 0U;
        }

        if (s_hroute.final_align_confirm_ticks >=
            H_FINAL_ALIGN_CONFIRM_TICKS) {
            h_finish();
        } else if (s_hroute.status.state_ticks >=
                   H_FINAL_ALIGN_TIMEOUT_TICKS) {
            h_fault(H_ROUTE_FAULT_FINAL_ALIGN_TIMEOUT);
        }
        return;
    }

    if ((s_hroute.status.state == H_ROUTE_STATE_STRAIGHT_AC) ||
        (s_hroute.status.state == H_ROUTE_STATE_STRAIGHT_BD)) {
        if (sensor.valid == 0U) {
            if (s_hroute.line_release_ticks < 255U) s_hroute.line_release_ticks++;
            if (s_hroute.line_release_ticks >= H_START_LINE_RELEASE_TICKS) {
                s_hroute.line_released = 1U;
            }
        } else {
            s_hroute.line_release_ticks = 0U;
        }

        /* 起点所在旧黑线必须先连续释放；然后里程达到门槛才允许识别终点线。 */
        if ((s_hroute.status.segment_distance >= H_STRAIGHT_LINE_ENABLE_COUNTS) &&
            (s_hroute.line_released != 0U) &&
            (sensor.valid != 0U)) {
            if (s_hroute.line_confirm_ticks < 255U) s_hroute.line_confirm_ticks++;
        } else {
            s_hroute.line_confirm_ticks = 0U;
        }

        if (s_hroute.line_confirm_ticks >= H_POINT_LINE_CONFIRM_TICKS) {
            if (s_hroute.status.state == H_ROUTE_STATE_STRAIGHT_AC) {
                h_enter_state(H_ROUTE_STATE_ARC_CB);
            } else {
                h_enter_state(H_ROUTE_STATE_ARC_DA);
            }
        } else if ((s_hroute.status.segment_distance >= H_STRAIGHT_MAX_COUNTS) ||
                   (s_hroute.status.state_ticks >= H_SEGMENT_TIMEOUT_TICKS)) {
            h_fault(H_ROUTE_FAULT_STRAIGHT_TIMEOUT);
        }
        return;
    }

    /*
     * 入弧捕获与巡线后丢失必须分开：A-C/B-D方向环刚压到C/D黑线时，
     * 探头可能先离开节点再看到弧线。此阶段尚未稳定捕获，不能累计丢线故障。
     */
    /* 入弧前200ms即使节点黑线曾连续有效，也可能只是把节点误当成弧线；
     * 若很快全白则退回A0继续捕获，不启动F05计时。 */
    if ((s_hroute.arc_line_acquired != 0U) &&
        (sensor.valid == 0U) &&
        (s_hroute.status.state_ticks < H_ARC_ACQUIRE_GUARD_TICKS)) {
        s_hroute.arc_line_acquired = 0U;
        s_hroute.status.arc_line_acquired = 0U;
        s_hroute.arc_acquire_ticks = 0U;
        s_hroute.arc_lost_ticks = 0U;
        s_hroute.arc_exit_armed = 0U;
        s_hroute.status.arc_exit_armed = 0U;
        s_hroute.position_initialized = 0U;
        s_hroute.arc_correction_request = 0;
        PID_ResetPlace();
    }

    if (s_hroute.arc_line_acquired == 0U) {
        if ((sensor.valid != 0U) &&
            (sensor.active_count <= H_ARC_TRACK_MAX_ACTIVE)) {
            if (s_hroute.arc_acquire_ticks < 255U) {
                s_hroute.arc_acquire_ticks++;
            }
            if (s_hroute.arc_acquire_ticks >= H_ARC_ACQUIRE_CONFIRM_TICKS) {
                s_hroute.arc_line_acquired = 1U;
                s_hroute.status.arc_line_acquired = 1U;
                /* 稳定捕获时丢弃C/D节点的旧位置历史，防止旧修正残留。 */
                s_hroute.position_initialized = 0U;
                s_hroute.position_pid_divider = 0U;
                s_hroute.arc_correction_request = 0;
                PID_ResetPlace();
            }
        } else {
            /* 全白或大于4路同时触发更像节点宽黑区，不能据此确认弧线。 */
            s_hroute.arc_acquire_ticks = 0U;
        }
    }

    /* 至少稳定进入过位置PID巡线，并且圆弧过程已满1.5s，才开放“全白=终点”。
     * E1仅表示时间门已经开放，不再依赖角度或里程。 */
    if ((s_hroute.arc_line_acquired != 0U) &&
        (s_hroute.status.state_ticks >= H_ARC_MIN_TRACK_TICKS)) {
        s_hroute.arc_exit_armed = 1U;
        s_hroute.status.arc_exit_armed = 1U;
    }

    /* 全白始终是可恢复状态，不再触发F05/F11；见线时继续实时位置PID。 */
    if (sensor.valid != 0U) {
        uint8_t was_lost = s_hroute.arc_lost_ticks;

        s_hroute.white_ticks = 0U;
        s_hroute.arc_lost_ticks = 0U;
        s_hroute.arc_endpoint_candidate = 0U;
        s_hroute.status.arc_lost_ticks = 0U;
        /* A0看到的可能仍是C/D节点宽黑区。只有稳定进入A1后，
         * 传感器位置才代表弧线，才允许更新找线记忆和位置PID。 */
        if (s_hroute.arc_line_acquired != 0U) {
        /* 只在最后位置明确偏向一侧时更新搜索方向；中心量化抖动不能覆盖记忆。 */
        if (sensor.error <= -25) {
            s_hroute.search_direction = -1;
        } else if (sensor.error >= 25) {
            s_hroute.search_direction = 1;
        }
        s_hroute.status.search_direction = s_hroute.search_direction;

        if (was_lost != 0U) {
            /* WHY: 重新见线后重置位置控制历史并用当前位置重播种，
             * 防止“全白保持值 -> 新线位置”的跳变继承旧修正。 */
            s_hroute.filtered_position = sensor.position;
            s_hroute.position_initialized = 1U;
            s_hroute.position_pid_divider = 0U;
            PID_ResetPlace();
        }
        if (s_hroute.position_initialized == 0U) {
            s_hroute.filtered_position = sensor.position;
            s_hroute.position_initialized = 1U;
        } else {
            /* 10ms位置环只保留一半旧值；比原1/4新值更快，中心死区负责抑制量化抖动。 */
            s_hroute.filtered_position = (int16_t)(
                (s_hroute.filtered_position + sensor.position + 1) / 2);
        }

        /* WHY: 见线期间位置PID固定20ms运行；速度环10ms执行最新目标。
         * 400/500是中心量化值，统一送450以避免左右轮目标来回翻转。 */
        s_hroute.position_pid_divider++;
        if (s_hroute.position_pid_divider >= H_POSITION_PID_DIVIDER) {
            s_hroute.position_pid_divider = 0U;
            if (h_abs_int(s_hroute.filtered_position - 450) <=
                H_ARC_CENTER_DEADBAND) {
                s_hroute.arc_correction_request = (int16_t)
                    place_PID_value(450.0f, 450.0f);
            } else {
                s_hroute.arc_correction_request = (int16_t)
                    place_PID_value(450.0f, (float)s_hroute.filtered_position);
            }
            s_hroute.arc_correction_request = (int16_t)h_clamp_int(
                s_hroute.arc_correction_request,
                -H_ARC_CORRECTION_LIMIT,
                H_ARC_CORRECTION_LIMIT);
            if (s_hroute.arc_correction_request != 0) {
                s_hroute.last_arc_correction = s_hroute.arc_correction_request;
            }
        }

        }
    } else {
        /* 1.5s时间门开放之后才开始独立累计终点全白50ms；此前的全白只找线，
         * 不能借用先前累计值提前跳转。 */
        if (s_hroute.arc_exit_armed != 0U) {
            if (s_hroute.white_ticks < 255U) s_hroute.white_ticks++;
        } else {
            s_hroute.white_ticks = 0U;
        }
        if ((s_hroute.arc_line_acquired != 0U) &&
            (s_hroute.arc_lost_ticks < 255U)) {
            s_hroute.arc_lost_ticks++;
        }
        s_hroute.status.arc_lost_ticks = s_hroute.arc_lost_ticks;
        s_hroute.status.search_direction = s_hroute.search_direction;
        /* 时间门开放后的全白直接作为终点候选；50ms确认期间保持出圈切线角。 */
        s_hroute.arc_endpoint_candidate = (uint8_t)(
            (s_hroute.arc_line_acquired != 0U) &&
            (s_hroute.arc_exit_armed != 0U));
    }

    if ((s_hroute.arc_line_acquired != 0U) &&
        (s_hroute.arc_exit_armed != 0U) &&
        (s_hroute.white_ticks >= H_ARC_EXIT_WHITE_TICKS)) {
        if (s_hroute.status.state == H_ROUTE_STATE_ARC_CB) {
            /* C->B过程满1.5s后连续全白50ms，直接切换140°航向环前往D。 */
            h_enter_state(H_ROUTE_STATE_STRAIGHT_BD);
        } else {
            if (s_hroute.status.completed_laps < 255U) {
                s_hroute.status.completed_laps++;
            }
            if (s_hroute.status.completed_laps >= s_hroute.status.target_laps) {
                /* 最后一圈A点全白后进入独立38°姿态调整。 */
                h_enter_state(H_ROUTE_STATE_ALIGN_A);
            } else {
                /* 非最后一圈A点全白后直接切换38°航向环进入下一圈A->C。 */
                h_enter_state(H_ROUTE_STATE_STRAIGHT_AC);
            }
        }
    }
}

/** 通用航向PD规划器：直线、出圈切线保持和最终A点姿态调整共用同一算法。 */
static void h_plan_heading(int base_speed,
                           int correction_limit,
                           int *target_left,
                           int *target_right)
{
    float error = h_wrap_180(s_hroute.status.yaw - s_hroute.status.target_yaw);
    float derivative = 0.0f;
    int correction;

    if (s_hroute.heading_initialized != 0U) {
        derivative = h_wrap_180(error - s_hroute.last_heading_error) *
                     H_HEADING_D_TIME_SCALE;
    } else {
        s_hroute.heading_initialized = 1U;
    }
    s_hroute.last_heading_error = error;
    s_hroute.status.heading_error = error;
    correction = (int)(h_heading_kp * error + h_heading_kd * derivative);
    correction = h_clamp_int(correction,
                             -correction_limit,
                             correction_limit);

    /* 正修正=车头偏顺时针，减左轮使车辆逆时针回正；外轮不超过基准速度。 */
    if (correction >= 0) {
        *target_left = base_speed - correction;
        *target_right = base_speed;
    } else {
        *target_left = base_speed;
        *target_right = base_speed + correction;
    }
    s_hroute.status.correction = (int16_t)correction;
}

static void h_plan_straight(int *target_left, int *target_right)
{
    h_plan_heading(H_STRAIGHT_SPEED,
                   H_HEADING_CORRECTION_LIMIT,
                   target_left,
                   target_right);
}

/** 最终A点使用较低速度和较大差速完成0°->38°调整，减少离开原点的距离。 */
static void h_plan_final_align(int *target_left, int *target_right)
{
    h_plan_heading(H_FINAL_ALIGN_SPEED,
                   H_FINAL_ALIGN_CORRECTION_LIMIT,
                   target_left,
                   target_right);
}

static void h_plan_arc(int *target_left,
                       int *target_right,
                       int measured_left,
                       int measured_right)
{
    int correction;

    s_hroute.status.heading_error =
        h_wrap_180(s_hroute.status.yaw - s_hroute.status.target_yaw);
    if (s_hroute.arc_line_acquired == 0U) {
        /* A0阶段即使探头仍压在C/D宽黑节点上，也不提前启用位置PID；
         * 只按该圆弧已知几何方向柔和捕线。 */
        if (s_hroute.search_direction < 0) {
            *target_left = H_ARC_SEARCH_INNER_SPEED;
            *target_right = H_ARC_SEARCH_OUTER_SPEED;
            correction = H_ARC_SEARCH_OUTER_SPEED - H_ARC_SEARCH_INNER_SPEED;
        } else {
            *target_left = H_ARC_SEARCH_OUTER_SPEED;
            *target_right = H_ARC_SEARCH_INNER_SPEED;
            correction = -(H_ARC_SEARCH_OUTER_SPEED - H_ARC_SEARCH_INNER_SPEED);
        }
        s_hroute.status.correction = (int16_t)correction;
        return;
    } else if ((s_hroute.status.sensor.valid != 0U) &&
               (s_hroute.position_initialized != 0U)) {
        /* 位置PID每20ms更新；10ms速度环只消费最新修正量。 */
        correction = (int)PID_ApplyPlaceRateDamping(
            (float)s_hroute.arc_correction_request,
            (float)measured_left,
            (float)measured_right,
            (float)H_ARC_CORRECTION_LIMIT);
    } else if (s_hroute.arc_endpoint_candidate != 0U) {
        /* B/A末端没有黑线是正常现象。50ms确认期间只保持当前出圈切线角：
         * C->B保持180°、D->A保持0°，绝不提前切到140°或38°。 */
        h_plan_straight(target_left, target_right);
        return;
    } else if (s_hroute.arc_lost_ticks > H_ARC_SEARCH_CONFIRM_TICKS) {
        /* 借鉴参考工程last_xun思想：沿最后看见黑线的方向扩大差速找线。
         * 外轮只提高到已在模式二验证的60，内轮仍保持正向22，绝不反转。 */
        if (s_hroute.search_direction < 0) {
            *target_left = H_ARC_SEARCH_INNER_SPEED;
            *target_right = H_ARC_SEARCH_OUTER_SPEED;
            correction = H_ARC_SEARCH_OUTER_SPEED - H_ARC_SEARCH_INNER_SPEED;
        } else {
            *target_left = H_ARC_SEARCH_OUTER_SPEED;
            *target_right = H_ARC_SEARCH_INNER_SPEED;
            correction = -(H_ARC_SEARCH_OUTER_SPEED - H_ARC_SEARCH_INNER_SPEED);
        }
        s_hroute.status.correction = (int16_t)correction;
        return;
    } else {
        /* 前20ms可能只是数字探头缝隙，先保持最后差速，不立即放大搜索。 */
        correction = s_hroute.last_arc_correction;
    }

    if (correction >= 0) {
        *target_left = H_ARC_SPEED - correction;
        *target_right = H_ARC_SPEED;
    } else {
        *target_left = H_ARC_SPEED;
        *target_right = H_ARC_SPEED + correction;
    }
    *target_left = h_clamp_int(*target_left, H_MIN_WHEEL_SPEED, H_ARC_SPEED);
    *target_right = h_clamp_int(*target_right, H_MIN_WHEEL_SPEED, H_ARC_SPEED);
    s_hroute.status.correction = (int16_t)correction;
}

static HRouteFault h_check_wheel(int target,
                                 int measured,
                                 int pwm,
                                 uint8_t *stall_windows,
                                 uint8_t *overspeed_windows,
                                 HRouteFault stall_fault,
                                 HRouteFault overspeed_fault)
{
    if ((target >= 20) && (h_abs_int(measured) < target / 4) &&
        (h_abs_int(pwm) >= H_STALL_PWM)) {
        if (*stall_windows < 255U) (*stall_windows)++;
    } else {
        *stall_windows = 0U;
    }
    if (*stall_windows >= H_STALL_CONFIRM_WINDOWS) return stall_fault;

    if ((target >= 20) && (h_abs_int(measured) > target * 2 + 20)) {
        if (*overspeed_windows < 255U) (*overspeed_windows)++;
    } else {
        *overspeed_windows = 0U;
    }
    if (*overspeed_windows >= H_OVERSPEED_CONFIRM_WINDOWS) return overspeed_fault;
    return H_ROUTE_FAULT_NONE;
}

void HRoute_Control10ms(int encoder_left,
                        int encoder_right,
                        int measured_left,
                        int measured_right)
{
    int target_left;
    int target_right;
    int pwm_left;
    int pwm_right;
    int distance_delta;
    HRouteFault fault;

    if ((jb != 3) || (h_state_is_running(s_hroute.status.state) == 0U)) {
        return;
    }

    if (s_hroute.imu_live_confirmed == 0U) {
        s_hroute.status.target_left = 0;
        s_hroute.status.target_right = 0;
        last_pwm_l = 0;
        last_pwm_r = 0;
        Set_Pwm(0, 0, 0, 0);
        return;
    }

    distance_delta = (h_abs_int(encoder_left) + h_abs_int(encoder_right)) / 2;
    s_hroute.status.total_distance += distance_delta;
    if (s_hroute.ignore_distance_window != 0U) {
        /* 状态可能在10ms窗口中途切换，首窗混有前一段计数，不能算入新路段。 */
        s_hroute.ignore_distance_window = 0U;
    } else {
        s_hroute.status.segment_distance += distance_delta;
    }

    if (s_hroute.status.state == H_ROUTE_STATE_ALIGN_A) {
        h_plan_final_align(&target_left, &target_right);
    } else if ((s_hroute.status.state == H_ROUTE_STATE_STRAIGHT_AC) ||
        (s_hroute.status.state == H_ROUTE_STATE_STRAIGHT_BD)) {
        h_plan_straight(&target_left, &target_right);
    } else {
        h_plan_arc(&target_left, &target_right,
                   measured_left, measured_right);
    }

    s_hroute.applied_left = (int16_t)h_approach(s_hroute.applied_left,
                                                target_left);
    s_hroute.applied_right = (int16_t)h_approach(s_hroute.applied_right,
                                                 target_right);
    s_hroute.status.target_left = s_hroute.applied_left;
    s_hroute.status.target_right = s_hroute.applied_right;

    pwm_left = (int)velocity_PID_value_l((float)s_hroute.applied_left,
                                         (float)measured_left);
    pwm_right = (int)velocity_PID_value_r((float)s_hroute.applied_right,
                                          (float)measured_right);
    pwm_left = h_clamp_int(pwm_left, 0, MOTOR_PWM_SAFE_LIMIT);
    pwm_right = h_clamp_int(pwm_right, 0, MOTOR_PWM_SAFE_LIMIT);
    last_pwm_l = pwm_left;
    last_pwm_r = pwm_right;

    fault = h_check_wheel(s_hroute.applied_left, measured_left, pwm_left,
                          &s_hroute.stall_left_windows,
                          &s_hroute.overspeed_left_windows,
                          H_ROUTE_FAULT_STALL_LEFT,
                          H_ROUTE_FAULT_OVERSPEED_LEFT);
    if (fault == H_ROUTE_FAULT_NONE) {
        fault = h_check_wheel(s_hroute.applied_right, measured_right, pwm_right,
                              &s_hroute.stall_right_windows,
                              &s_hroute.overspeed_right_windows,
                              H_ROUTE_FAULT_STALL_RIGHT,
                              H_ROUTE_FAULT_OVERSPEED_RIGHT);
    }
    if (fault != H_ROUTE_FAULT_NONE) {
        h_fault(fault);
        return;
    }

    /* 本车实测方向：左轮负命令、右轮正命令才是共同前进。 */
    Set_Pwm(-pwm_left, pwm_right, 0, 0);
}

void HRoute_GetStatus(HRouteStatus *status)
{
    uint32_t interrupt_state;

    if (status == 0) return;
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *status = s_hroute.status;
    if (interrupt_state == 0U) __enable_irq();
}

void HPositionTune_Start(void)
{
    memset(&s_hposition_tune, 0, sizeof(s_hposition_tune));
    s_hposition_tune.status.running = 1U;
    s_hposition_tune.status.waiting_line = 1U;
    PID_ResetAll();
    last_pwm_l = 0;
    last_pwm_r = 0;
    Motor_Stop();
    jb = 4;
}

void HPositionTune_Stop(void)
{
    if (jb == 4) jb = 0;
    memset(&s_hposition_tune, 0, sizeof(s_hposition_tune));
    PID_ResetAll();
    last_pwm_l = 0;
    last_pwm_r = 0;
    Motor_Stop();
}

void HPositionTune_Control10ms(int measured_left, int measured_right)
{
    XunjiSensor sensor;
    int target_left = 0;
    int target_right = 0;
    int correction = 0;
    int requested_pwm_left;
    int requested_pwm_right;
    int tune_speed = (purpose > 0.0f) ? (int)purpose : 0;
    int correction_limit =
        (tune_speed * H_POSITION_TUNE_CORRECTION_PERCENT) / 100;
    uint8_t line_ready = 0U;
    HRouteFault fault;

    if ((jb != 4) || (s_hposition_tune.status.running == 0U)) return;

    Xunji_ReadSensorSnapshot(&sensor);
    s_hposition_tune.status.sensor = sensor;

    if (sensor.valid == 0U) {
        /*
         * 灰度瞬时全白不是功率故障。模式五不再短路急停，而是把轮速目标
         * 平滑降到零并等待重新见线，防止调参时一次闪断把钢珠甩动。
         */
        if (s_hposition_tune.status.waiting_line == 0U) {
            PID_ResetPlace();
        }
        s_hposition_tune.status.waiting_line = 1U;
        s_hposition_tune.line_valid_ticks = 0U;
        s_hposition_tune.position_initialized = 0U;
        s_hposition_tune.position_pid_divider = 0U;
        s_hposition_tune.correction_request = 0;
    } else if (s_hposition_tune.status.waiting_line != 0U) {
        if (s_hposition_tune.line_valid_ticks < 255U) {
            s_hposition_tune.line_valid_ticks++;
        }
        if (s_hposition_tune.line_valid_ticks >= H_TUNE_LINE_REACQUIRE_TICKS) {
            /*
             * 此历史调试接口保留独立低通；菜单模式五实际调用Xunji公共控制链，
             * 否则同一组Kp/Ki/Kd在任务中会表现成另一套控制器。
             */
            s_hposition_tune.status.waiting_line = 0U;
            s_hposition_tune.filtered_position_x4 =
                (int32_t)sensor.position * 4;
            s_hposition_tune.status.filtered_position = sensor.position;
            s_hposition_tune.position_initialized = 1U;
            s_hposition_tune.position_pid_divider = 0U;
            s_hposition_tune.correction_request = 0;
            s_hposition_tune.applied_correction = 0;
            PID_ResetPlace();
        }
    }

    if ((sensor.valid != 0U) &&
        (s_hposition_tune.status.waiting_line == 0U)) {
        line_ready = 1U;
        if (s_hposition_tune.position_initialized == 0U) {
            s_hposition_tune.filtered_position_x4 =
                (int32_t)sensor.position * 4;
            s_hposition_tune.position_initialized = 1U;
        } else {
            s_hposition_tune.filtered_position_x4 =
                (s_hposition_tune.filtered_position_x4 * 3 +
                 (int32_t)sensor.position * 4) / 4;
        }
        s_hposition_tune.status.filtered_position = (int16_t)(
            (s_hposition_tune.filtered_position_x4 + 2) / 4);

        s_hposition_tune.position_pid_divider++;
        if (s_hposition_tune.position_pid_divider >= H_POSITION_PID_DIVIDER) {
            s_hposition_tune.position_pid_divider = 0U;
            correction = (int)place_PID_value_limited(
                450.0f,
                (float)s_hposition_tune.status.filtered_position,
                (float)correction_limit);
            s_hposition_tune.correction_request = (int16_t)h_clamp_int(
                correction, -correction_limit, correction_limit);
        }
    }

    if (line_ready == 0U) {
        s_hposition_tune.correction_request = 0;
    }
    s_hposition_tune.applied_correction = (int16_t)h_approach_step(
        s_hposition_tune.applied_correction,
        (int)PID_ApplyPlaceRateDamping(
            (float)s_hposition_tune.correction_request,
            (float)measured_left,
            (float)measured_right,
            (float)correction_limit),
        H_POSITION_TUNE_CORRECTION_SLEW_STEP);
    correction = s_hposition_tune.applied_correction;

    if (line_ready != 0U) {
        if (correction >= 0) {
            target_left = tune_speed - correction;
            target_right = tune_speed;
        } else {
            target_left = tune_speed;
            target_right = tune_speed + correction;
        }
        target_left = h_clamp_int(target_left, 0, tune_speed);
        target_right = h_clamp_int(target_right, 0, tune_speed);
    }

    s_hposition_tune.applied_left = (int16_t)h_approach_step(
        s_hposition_tune.applied_left, target_left,
        H_POSITION_TUNE_TARGET_SLEW_STEP);
    s_hposition_tune.applied_right = (int16_t)h_approach_step(
        s_hposition_tune.applied_right, target_right,
        H_POSITION_TUNE_TARGET_SLEW_STEP);
    s_hposition_tune.status.target_left = s_hposition_tune.applied_left;
    s_hposition_tune.status.target_right = s_hposition_tune.applied_right;
    s_hposition_tune.status.correction = (int16_t)correction;

    if (s_hposition_tune.applied_left == 0) {
        requested_pwm_left = 0;
        PID_ResetVelocityLeft();
    } else {
        requested_pwm_left = (int)velocity_PID_value_l_no_ff(
            (float)s_hposition_tune.applied_left, (float)measured_left);
    }
    if (s_hposition_tune.applied_right == 0) {
        requested_pwm_right = 0;
        PID_ResetVelocityRight();
    } else {
        requested_pwm_right = (int)velocity_PID_value_r_no_ff(
            (float)s_hposition_tune.applied_right, (float)measured_right);
    }
    requested_pwm_left = h_clamp_int(requested_pwm_left,
                                     0, MOTOR_PWM_SAFE_LIMIT);
    requested_pwm_right = h_clamp_int(requested_pwm_right,
                                      0, MOTOR_PWM_SAFE_LIMIT);
    s_hposition_tune.pwm_left = (int16_t)h_approach_step(
        s_hposition_tune.pwm_left, requested_pwm_left,
        H_POSITION_TUNE_PWM_SLEW_STEP);
    s_hposition_tune.pwm_right = (int16_t)h_approach_step(
        s_hposition_tune.pwm_right, requested_pwm_right,
        H_POSITION_TUNE_PWM_SLEW_STEP);

    /*
     * PWM斜率限制是单轮执行器约束。只回算真正受限的一侧，
     * 否则另一轮的积分会被无关轮子的测速量化波动反复覆盖。
     */
    if (s_hposition_tune.pwm_left != requested_pwm_left) {
        PID_PrimeVelocityLeftNoFeedforward(
            (float)s_hposition_tune.applied_left,
            (float)measured_left,
            (float)s_hposition_tune.pwm_left);
    }
    if (s_hposition_tune.pwm_right != requested_pwm_right) {
        PID_PrimeVelocityRightNoFeedforward(
            (float)s_hposition_tune.applied_right,
            (float)measured_right,
            (float)s_hposition_tune.pwm_right);
    }
    last_pwm_l = s_hposition_tune.pwm_left;
    last_pwm_r = s_hposition_tune.pwm_right;

    fault = h_check_wheel(s_hposition_tune.applied_left, measured_left,
                          s_hposition_tune.pwm_left,
                          &s_hposition_tune.stall_left_windows,
                          &s_hposition_tune.overspeed_left_windows,
                          H_ROUTE_FAULT_STALL_LEFT,
                          H_ROUTE_FAULT_OVERSPEED_LEFT);
    if (fault == H_ROUTE_FAULT_NONE) {
        fault = h_check_wheel(s_hposition_tune.applied_right, measured_right,
                              s_hposition_tune.pwm_right,
                              &s_hposition_tune.stall_right_windows,
                              &s_hposition_tune.overspeed_right_windows,
                              H_ROUTE_FAULT_STALL_RIGHT,
                              H_ROUTE_FAULT_OVERSPEED_RIGHT);
    }
    if (fault != H_ROUTE_FAULT_NONE) {
        s_hposition_tune.status.fault = fault;
        s_hposition_tune.status.running = 0U;
        s_hposition_tune.status.target_left = 0;
        s_hposition_tune.status.target_right = 0;
        jb = 0;
        last_pwm_l = 0;
        last_pwm_r = 0;
        PID_ResetAll();
        Motor_Coast();
        return;
    }

    Set_Pwm(-s_hposition_tune.pwm_left,
            s_hposition_tune.pwm_right, 0, 0);
}

void HPositionTune_GetStatus(HPositionTuneStatus *status)
{
    uint32_t interrupt_state;

    if (status == 0) return;
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *status = s_hposition_tune.status;
    if (interrupt_state == 0U) __enable_irq();
}

/* ========================================================================
 * 2026 H题第二问：模式六/七/八标定控制器
 *
 * 模式六只累计编码器，不驱动电机。
 * 模式七按AB、BC、CD、DA四段时间运行：
 *   - 直线：60基础速度 + 小幅灰度位置修正 + IMU航向PD；
 *   - 半圆：左外轮60、右内轮(60-delta) + 小幅灰度位置修正 +
 *           IMU角速度微调，保持顺时针旋转均匀。
 * 模式八关闭位置/IMU修正，只验证固定差速，并在累计转角达到180度时急停。
 * ======================================================================== */

#define H2_JB_MILEAGE                         5
#define H2_JB_TIMED_ROUTE                     6
#define H2_JB_CURVE_TEST                      7
#define H2_JB_FINAL_ROUTE                     8
#define H2_PERIOD_MS                         10U
#define H2_MS_TO_TICKS(ms)                   (((ms) + H2_PERIOD_MS - 1U) / H2_PERIOD_MS)

/* 关键实车参数区：后续标定只需要优先修改本区。 */
#define H2_DEFAULT_AB_TIME_MS               H2_FINAL_AB_TIME_MS
#define H2_DEFAULT_BC_TIME_MS               H2_FINAL_BC_TIME_MS
#define H2_DEFAULT_CD_TIME_MS               H2_FINAL_CD_TIME_MS
#define H2_DEFAULT_DA_TIME_MS               H2_FINAL_DA_TIME_MS
#define H2_DEFAULT_CURVE_DELTA                21
#define H2_CURVE_DELTA_MIN                     4
#define H2_CURVE_DELTA_MAX                    50
#define H2_MIN_WHEEL_SPEED                    20

/* 外环仅作辅助，避免它覆盖已经整定好的60速度内环。 */
#define H2_POSITION_PERIOD_TICKS               2U /* 20ms */
#define H2_STRAIGHT_LINE_LIMIT                 5
#define H2_CURVE_LINE_LIMIT                    2
#define H2_FINAL_CURVE_LINE_LIMIT              5
#define H2_DEFAULT_LINE_WEIGHT_X10 H2_FINAL_LINE_WEIGHT_X10 /* 默认5.0；模式七可按0.1实时调整 */
#define H2_LINE_WEIGHT_MIN_X10                  0 /* 0.0，仅用于对比测试 */
#define H2_LINE_WEIGHT_MAX_X10                 40 /* 4.0，仍受轮速修正限幅约束 */
#define H2_TOTAL_CORRECTION_LIMIT              8
#define H2_CORRECTION_SLEW_STEP                1
#define H2_LINE_HOLD_TICKS                    10U /* 短丢线100ms保持最后修正 */
#define H2_LINE_FAULT_TICKS                  100U /* 连续1s全白才视为脱轨 */

/*
 * IMU参数：
 * - 直线只消除慢性偏航；
 * - 弯道使用“21基础差速 + 累计角度/角速度小修正 + 灰度小修正”。
 *   21来自原80速度、差速29按60/80等比例缩放，保持左右轮速度比和基础曲率；
 *   所有弯道外环合计只允许改变3个速度单位，避免修正覆盖实车标定结果。
 */
#define H2_HEADING_KP                       (0.35f)
#define H2_HEADING_KD                       (0.06f)
#define H2_HEADING_LIMIT                       4
#define H2_CURVE_ANGLE_KP                   (0.10f)
#define H2_CURVE_RATE_KP                    (0.04f)
#define H2_CURVE_IMU_TRIM_LIMIT                3
#define H2_CURVE_TOTAL_TRIM_LIMIT              3
#define H2_FINAL_CURVE_TRIM_DECREASE_LIMIT      1
#define H2_FINAL_CURVE_TRIM_INCREASE_LIMIT      3
#define H2_FINAL_CURVE_TOTAL_TRIM_LIMIT        19
#define H2_CURVE_TRIM_SLEW_STEP                1
#define H2_CURVE_SIGN_LEARN_DEG              (5.0f)
#define H2_CURVE_EXIT_MIN_DEG               (176.0f)
#define H2_CURVE_EXTENSION_TICKS              80U /* 标称时间后最多再补转800ms */
#define H2_FINAL_STOP_LEAD_MS                50U /* DA段满足转角后提前50ms短路制动 */
#define H2_IMU_STALE_TICKS                   50U
#define H2_IMU_YAW_MIN_DEG                  (0.0f)
#define H2_IMU_YAW_MAX_DEG                (360.0f)
#define H2_IMU_MAX_RATE_DPS                (720.0f)
#define H2_CURVE_MIN_RUN_TICKS               50U
#define H2_CURVE_TEST_TIMEOUT_TICKS         800U
#define H2_ROUTE_TIMEOUT_MARGIN_TICKS       200U /* 四段总设定外再留2s */

/* 无电流采样时保留高PWM低转速、超速和编码器失效保护。 */
#define H2_STALL_PWM                         800
#define H2_STALL_CONFIRM_TICKS                30U
#define H2_OVERSPEED_CONFIRM_TICKS            15U
#define H2_ENCODER_BAD_CONFIRM_TICKS          20U

typedef enum {
    H2_RUN_NONE = 0,
    H2_RUN_TIMED,
    H2_RUN_CURVE,
    H2_RUN_FINAL
} H2RunMode;

typedef struct {
    H2MotionStatus status;
    H2RunMode mode;
    uint32_t last_imu_frames;
    uint16_t imu_gap_ticks;
    uint16_t total_ticks;
    uint16_t stage_ticks;
    float last_frame_yaw;
    float accumulated_yaw;
    float heading_reference;
    int8_t curve_turn_sign;
    uint8_t imu_available;
    int32_t filtered_position_x4;
    uint8_t position_initialized;
    uint8_t position_divider;
    uint16_t line_lost_ticks;
    int16_t line_request;
    int16_t applied_correction;
    int16_t rate_trim;
    uint8_t stall_left_ticks;
    uint8_t stall_right_ticks;
    uint8_t overspeed_left_ticks;
    uint8_t overspeed_right_ticks;
    uint8_t encoder_left_bad_ticks;
    uint8_t encoder_right_bad_ticks;
    uint8_t velocity_reprime_pending;
    uint8_t ignore_distance_window;
} H2MotionController;

static H2MileageStatus s_h2_mileage;
static H2MotionController s_h2_motion;
static uint32_t s_h2_stage_time_ms[4] = {
    H2_DEFAULT_AB_TIME_MS,
    H2_DEFAULT_BC_TIME_MS,
    H2_DEFAULT_CD_TIME_MS,
    H2_DEFAULT_DA_TIME_MS
};
static volatile uint32_t s_h2_final_stage_time_ms[4] = {
    H2_FINAL_AB_TIME_MS,
    H2_FINAL_BC_TIME_MS,
    H2_FINAL_CD_TIME_MS,
    H2_FINAL_DA_TIME_MS
};
static const int32_t s_h2_final_stage_distance[4] = {
    H2_FINAL_AB_DISTANCE_COUNTS,
    H2_FINAL_BC_DISTANCE_COUNTS,
    H2_FINAL_CD_DISTANCE_COUNTS,
    H2_FINAL_DA_DISTANCE_COUNTS
};
static int s_h2_curve_delta = H2_DEFAULT_CURVE_DELTA;
/* 主循环菜单写入、10ms控制中断读取，因此必须声明为volatile。 */
static volatile int16_t s_h2_line_weight_x10 =
    H2_DEFAULT_LINE_WEIGHT_X10;
/* Mode 1 uses an integer weight; the control loop converts it to x10. */
static volatile int16_t s_h2_final_line_weight =
    (int16_t)(H2_FINAL_LINE_WEIGHT_X10 / 10);

/*
 * IMU601 的 yaw 正常范围为 [0, 360]。使用成对比较还能同时拒绝 NaN：
 * NaN 与上下界的比较均为 false，不能进入后续 wrap/浮点转整数路径。
 */
static uint8_t h2_imu_yaw_is_valid(float yaw)
{
    return (uint8_t)((yaw >= H2_IMU_YAW_MIN_DEG) &&
                     (yaw <= H2_IMU_YAW_MAX_DEG));
}

static uint8_t h2_stage_is_curve(H2CalStage stage)
{
    return (uint8_t)((stage == H2_CAL_STAGE_BC) ||
                     (stage == H2_CAL_STAGE_DA));
}

/**
 * 返回当前弯道沿实际转向方向的累计角度。
 *
 * 首个弯道开始后的前5度用于自动学习IMU正负方向；学习前仅用于显示的绝对值，
 * 学习后若车辆反向，返回值会变小而不是被绝对值误判为正常转弯。
 */
static float h2_curve_progress(void)
{
    if (s_h2_motion.curve_turn_sign > 0) {
        return s_h2_motion.accumulated_yaw;
    }
    if (s_h2_motion.curve_turn_sign < 0) {
        return -s_h2_motion.accumulated_yaw;
    }
    return h_abs_float(s_h2_motion.accumulated_yaw);
}

/** 返回与正常弯道方向同号的角速度；尚未学习方向时使用绝对值。 */
static float h2_curve_rate(void)
{
    if (s_h2_motion.curve_turn_sign > 0) {
        return s_h2_motion.status.yaw_rate;
    }
    if (s_h2_motion.curve_turn_sign < 0) {
        return -s_h2_motion.status.yaw_rate;
    }
    return h_abs_float(s_h2_motion.status.yaw_rate);
}

static uint8_t h2_stage_index(H2CalStage stage)
{
    if ((stage < H2_CAL_STAGE_AB) || (stage > H2_CAL_STAGE_DA)) {
        return 0U;
    }
    return (uint8_t)(stage - H2_CAL_STAGE_AB);
}

/** 返回当前运行模式使用的阶段时间，模式一与可调模式七互不覆盖。 */
static uint32_t h2_active_stage_time_ms(uint8_t stage_index)
{
    if (stage_index >= 4U) return 0U;
    if (s_h2_motion.mode == H2_RUN_FINAL) {
        return s_h2_final_stage_time_ms[stage_index];
    }
    return s_h2_stage_time_ms[stage_index];
}

/** 模式一固定使用4.0，模式七继续使用菜单中的0.1步进可调值。 */
static int h2_active_line_weight_x10(void)
{
    if (s_h2_motion.mode == H2_RUN_FINAL) {
        return (int)s_h2_final_line_weight * 10;
    }
    return (int)s_h2_line_weight_x10;
}

/** 模式一固定使用实测成功的差速，模式八调节不会污染最终比赛参数。 */
static int h2_active_curve_delta(void)
{
    if (s_h2_motion.mode == H2_RUN_FINAL) {
        return H2_FINAL_CURVE_DELTA;
    }
    return s_h2_curve_delta;
}

static void h2_reset_outer_loops(void)
{
    s_h2_motion.filtered_position_x4 = 450 * 4;
    s_h2_motion.position_initialized = 0U;
    s_h2_motion.position_divider = 0U;
    s_h2_motion.line_lost_ticks = 0U;
    s_h2_motion.line_request = 0;
    s_h2_motion.applied_correction = 0;
    s_h2_motion.rate_trim = 0;
    s_h2_motion.status.line_correction = 0;
    s_h2_motion.status.imu_correction = 0;
    s_h2_motion.status.filtered_position = 450;
    PID_ResetPlace();
}

static void h2_enter_stage(H2CalStage stage, float current_yaw)
{
    uint8_t stage_index = h2_stage_index(stage);

    s_h2_motion.status.stage = stage;
    s_h2_motion.status.segment_index = stage_index;
    s_h2_motion.stage_ticks = 0U;
    s_h2_motion.status.stage_elapsed_ms = 0U;
    s_h2_motion.status.stage_distance = 0;
    s_h2_motion.status.time_ready = 0U;
    s_h2_motion.status.distance_ready = 0U;
    s_h2_motion.status.angle_ready =
        (uint8_t)(h2_stage_is_curve(stage) == 0U);
    if (s_h2_motion.mode == H2_RUN_FINAL) {
        s_h2_motion.status.stage_distance_target[stage_index] =
            s_h2_final_stage_distance[stage_index];
    }
    /* 新路段先显示基础差速；进入弯道后会被实时生效差速覆盖。 */
    s_h2_motion.status.curve_delta =
        (int16_t)h2_active_curve_delta();
    s_h2_motion.heading_reference = current_yaw;
    s_h2_motion.accumulated_yaw = 0.0f;
    s_h2_motion.status.yaw_progress = 0.0f;
    h2_reset_outer_loops();
    s_h2_motion.velocity_reprime_pending = 1U;
    /*
     * 状态切换发生在本次10ms编码器快照之前，因此紧随其后的窗口仍主要属于
     * 上一段。丢弃这一窗，避免把AB末端计数记入BC等下一路段。
     */
    s_h2_motion.ignore_distance_window = 1U;
}

static void h2_clear_output_state(void)
{
    s_h2_motion.status.target_left = 0;
    s_h2_motion.status.target_right = 0;
    s_h2_motion.status.running = 0U;
    last_pwm_l = 0;
    last_pwm_r = 0;
    PID_ResetAll();
}

static void h2_stop_outputs(void)
{
    h2_clear_output_state();
    Motor_Stop();
}

static void h2_freeze_final_timer(void)
{
    if ((s_h2_motion.mode != H2_RUN_FINAL) ||
        (s_h2_motion.status.timing_complete != 0U)) {
        return;
    }
    s_h2_motion.status.timing_complete = 1U;
    s_h2_motion.status.total_elapsed_ms =
        (uint32_t)s_h2_motion.total_ticks * H2_PERIOD_MS;
}

static void h2_fault(H2CalFault fault)
{
    h2_freeze_final_timer();
    s_h2_motion.status.fault = fault;
    s_h2_motion.status.stage = H2_CAL_STAGE_FAULT;
    s_h2_motion.status.waiting_imu = 0U;
    jb = 0;
    h2_clear_output_state();
    if ((fault == H2_CAL_FAULT_STALL_LEFT) ||
        (fault == H2_CAL_FAULT_STALL_RIGHT) ||
        (fault == H2_CAL_FAULT_OVERSPEED_LEFT) ||
        (fault == H2_CAL_FAULT_OVERSPEED_RIGHT) ||
        (fault == H2_CAL_FAULT_ENCODER_LEFT) ||
        (fault == H2_CAL_FAULT_ENCODER_RIGHT)) {
        /*
         * 高PWM低反馈、异常超速或编码器失效都可能是驱动/接线故障。
         * 此时继续短路制动会增加TB6612和电源回路电流，因此撤销驱动并高阻滑停。
         */
        Motor_Coast();
    } else {
        /* 正常路线类异常仍使用50ms短路急停，防止小车继续冲出赛道。 */
        Motor_Stop();
    }
}

static void h2_finish(void)
{
    h2_freeze_final_timer();
    s_h2_motion.status.fault = H2_CAL_FAULT_NONE;
    s_h2_motion.status.stage = H2_CAL_STAGE_DONE;
    s_h2_motion.status.waiting_imu = 0U;
    jb = 0;
    h2_stop_outputs();
}

void H2Mileage_Reset(void)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    memset(&s_h2_mileage, 0, sizeof(s_h2_mileage));
    s_h2_mileage.active = (uint8_t)(jb == H2_JB_MILEAGE);
    if (interrupt_state == 0U) __enable_irq();
}

void H2Mileage_Start(void)
{
    jb = 0;
    PID_ResetAll();
    Motor_Coast();
    H2Mileage_Reset();
    s_h2_mileage.active = 1U;
    jb = H2_JB_MILEAGE;
}

void H2Mileage_Stop(void)
{
    if (jb == H2_JB_MILEAGE) jb = 0;
    s_h2_mileage.active = 0U;
    Motor_Stop();
}

void H2Mileage_MarkPoint(void)
{
    uint32_t interrupt_state;

    if ((s_h2_mileage.active == 0U) ||
        (s_h2_mileage.completed != 0U)) {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    s_h2_mileage.segment_counts[s_h2_mileage.segment_index] =
        s_h2_mileage.current_counts;
    s_h2_mileage.current_counts = 0;
    s_h2_mileage.segment_index++;
    if (s_h2_mileage.segment_index >= 4U) {
        s_h2_mileage.segment_index = 3U;
        s_h2_mileage.completed = 1U;
        s_h2_mileage.active = 0U;
        if (jb == H2_JB_MILEAGE) jb = 0;
    }
    if (interrupt_state == 0U) __enable_irq();
}

void H2Mileage_Control10ms(int encoder_left, int encoder_right)
{
    int distance_delta;

    if ((jb != H2_JB_MILEAGE) || (s_h2_mileage.active == 0U)) return;
    distance_delta = (h_abs_int(encoder_left) +
                      h_abs_int(encoder_right)) / 2;
    s_h2_mileage.current_counts += distance_delta;
}

void H2Mileage_GetStatus(H2MileageStatus *status)
{
    uint32_t interrupt_state;

    if (status == 0) return;
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *status = s_h2_mileage;
    if (interrupt_state == 0U) __enable_irq();
}

void H2Timed_SetStageTime(uint8_t stage_index, uint32_t time_ms)
{
    uint32_t interrupt_state;

    if (stage_index >= 4U) return;
    if (time_ms < 500U) time_ms = 500U;
    if (time_ms > 10000U) time_ms = 10000U;
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    s_h2_stage_time_ms[stage_index] =
        (time_ms / H2_CAL_TIME_STEP_MS) * H2_CAL_TIME_STEP_MS;
    if (interrupt_state == 0U) __enable_irq();
}

uint32_t H2Timed_GetStageTime(uint8_t stage_index)
{
    if (stage_index >= 4U) return 0U;
    return s_h2_stage_time_ms[stage_index];
}

void H2Timed_SetLineWeightX10(int weight_x10)
{
    uint32_t interrupt_state;

    if (weight_x10 < H2_LINE_WEIGHT_MIN_X10) {
        weight_x10 = H2_LINE_WEIGHT_MIN_X10;
    } else if (weight_x10 > H2_LINE_WEIGHT_MAX_X10) {
        weight_x10 = H2_LINE_WEIGHT_MAX_X10;
    }

    /*
     * 运行中允许微调，临界区保证10ms控制中断不会读到更新过程中的中间值。
     * 参数使用x10定点格式：20代表2.0，21代表2.1。
     */
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    s_h2_line_weight_x10 = (int16_t)weight_x10;
    if (interrupt_state == 0U) __enable_irq();
}

int H2Timed_GetLineWeightX10(void)
{
    return (int)s_h2_line_weight_x10;
}

void H2Final_SetStageTime(uint8_t stage_index, uint32_t time_ms)
{
    uint32_t interrupt_state;

    if (stage_index >= 4U) return;
    if (time_ms < 500U) time_ms = 500U;
    if (time_ms > 10000U) time_ms = 10000U;
    time_ms = (time_ms / H2_FINAL_TIME_STEP_MS) *
              H2_FINAL_TIME_STEP_MS;

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    s_h2_final_stage_time_ms[stage_index] = time_ms;
    if (interrupt_state == 0U) __enable_irq();
}

uint32_t H2Final_GetStageTime(uint8_t stage_index)
{
    if (stage_index >= 4U) return 0U;
    return s_h2_final_stage_time_ms[stage_index];
}

void H2Final_SetLineWeight(int weight)
{
    uint32_t interrupt_state;

    if (weight < H2_FINAL_LINE_WEIGHT_MIN) {
        weight = H2_FINAL_LINE_WEIGHT_MIN;
    } else if (weight > H2_FINAL_LINE_WEIGHT_MAX) {
        weight = H2_FINAL_LINE_WEIGHT_MAX;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    s_h2_final_line_weight = (int16_t)weight;
    if (interrupt_state == 0U) __enable_irq();
}

int H2Final_GetLineWeight(void)
{
    return (int)s_h2_final_line_weight;
}

void H2Curve_SetDelta(int delta)
{
    if (delta < H2_CURVE_DELTA_MIN) delta = H2_CURVE_DELTA_MIN;
    if (delta > H2_CURVE_DELTA_MAX) delta = H2_CURVE_DELTA_MAX;
    s_h2_curve_delta = delta;
}

int H2Curve_GetDelta(void)
{
    return s_h2_curve_delta;
}

static uint8_t h2_motion_start(H2RunMode mode)
{
    IMU601_Snapshot_t imu;
    uint8_t index;

    jb = 0;
    IMU601_GetSnapshot(&imu);
    memset(&s_h2_motion, 0, sizeof(s_h2_motion));
    s_h2_motion.mode = mode;
    s_h2_motion.status.fault = H2_CAL_FAULT_NONE;
    s_h2_motion.status.curve_delta =
        (int16_t)h2_active_curve_delta();
    for (index = 0U; index < 4U; index++) {
        s_h2_motion.status.stage_time_ms[index] =
            (mode == H2_RUN_FINAL) ?
            s_h2_final_stage_time_ms[index] :
            s_h2_stage_time_ms[index];
        s_h2_motion.status.stage_distance_target[index] =
            (mode == H2_RUN_FINAL) ?
            s_h2_final_stage_distance[index] : 0;
    }
    s_h2_motion.status.line_weight_x10 =
        (int16_t)h2_active_line_weight_x10();

    /*
     * F01/F02/F03不再终止模式七/八。启动瞬间若尚未取得有效帧，先以灰度和
     * 基础差速运行；后续收到有效新帧时自动接管角度辅助。电机、编码器、堵转、
     * 超速和状态超时保护仍然保留。
     */
    if ((IMU601_IsReady() != 0U) &&
        (imu.valid_frames != 0U) &&
        (h2_imu_yaw_is_valid(imu.attitude.yaw) != 0U)) {
        s_h2_motion.imu_available = 1U;
        s_h2_motion.last_imu_frames = imu.valid_frames;
        s_h2_motion.last_frame_yaw = imu.attitude.yaw;
        s_h2_motion.status.yaw = imu.attitude.yaw;
        s_h2_motion.heading_reference = imu.attitude.yaw;
        s_h2_motion.status.imu_available = 1U;
    } else {
        s_h2_motion.imu_available = 0U;
        s_h2_motion.last_imu_frames = imu.valid_frames;
        s_h2_motion.status.yaw = 0.0f;
        s_h2_motion.status.imu_available = 0U;
    }

    s_h2_motion.status.running = 1U;
    s_h2_motion.status.waiting_imu = 0U;
    h2_enter_stage((mode == H2_RUN_CURVE) ?
                   H2_CAL_STAGE_BC : H2_CAL_STAGE_AB,
                   s_h2_motion.status.yaw);
    PID_ResetAll();
    last_pwm_l = 0;
    last_pwm_r = 0;
    Set_Pwm(0, 0, 0, 0);
    if (mode == H2_RUN_TIMED) {
        jb = H2_JB_TIMED_ROUTE;
    } else if (mode == H2_RUN_FINAL) {
        jb = H2_JB_FINAL_ROUTE;
    } else {
        jb = H2_JB_CURVE_TEST;
    }
    return 1U;
}

uint8_t H2Timed_Start(void)
{
    return h2_motion_start(H2_RUN_TIMED);
}

uint8_t H2Final_Start(void)
{
    return h2_motion_start(H2_RUN_FINAL);
}

uint8_t H2Curve_Start(void)
{
    return h2_motion_start(H2_RUN_CURVE);
}

void H2Motion_Stop(void)
{
    if ((jb == H2_JB_TIMED_ROUTE) || (jb == H2_JB_CURVE_TEST) ||
        (jb == H2_JB_FINAL_ROUTE)) {
        jb = 0;
    }
    h2_freeze_final_timer();
    s_h2_motion.status.waiting_imu = 0U;
    h2_stop_outputs();
}

static void h2_update_imu(const IMU601_Snapshot_t *imu)
{
    float yaw_delta;
    float new_rate;

    /*
     * IMU是模式一的辅助通道，不是任务成立条件。
     * 无帧、陈旧帧或异常角速度只会暂时关闭IMU修正，绝不调用h2_fault；
     * 这样模式一会继续由时间、灰度和编码器保护完成任务。
     */
    if ((imu->valid_frames == 0U) ||
        (h2_imu_yaw_is_valid(imu->attitude.yaw) == 0U)) {
        if (s_h2_motion.imu_gap_ticks < 65535U) {
            s_h2_motion.imu_gap_ticks++;
        }
        if (s_h2_motion.imu_gap_ticks >= H2_IMU_STALE_TICKS) {
            s_h2_motion.imu_available = 0U;
            s_h2_motion.status.imu_available = 0U;
            s_h2_motion.status.yaw_rate = 0.0f;
        }
        return;
    }
    s_h2_motion.status.yaw = imu->attitude.yaw;
    if (imu->valid_frames == s_h2_motion.last_imu_frames) {
        if (s_h2_motion.imu_gap_ticks < 65535U) {
            s_h2_motion.imu_gap_ticks++;
        }
        if (s_h2_motion.imu_gap_ticks >= H2_IMU_STALE_TICKS) {
            s_h2_motion.imu_available = 0U;
            s_h2_motion.status.imu_available = 0U;
            s_h2_motion.status.yaw_rate = 0.0f;
        }
        return;
    }

    /*
     * IMU从无效/陈旧状态恢复时只重新建立帧基准，不把通信空窗期间的角度跳变
     * 直接送入闭环，也不重置当前路段计时。
     */
    if (s_h2_motion.imu_available == 0U) {
        s_h2_motion.imu_available = 1U;
        s_h2_motion.status.imu_available = 1U;
        s_h2_motion.last_frame_yaw = imu->attitude.yaw;
        s_h2_motion.last_imu_frames = imu->valid_frames;
        s_h2_motion.imu_gap_ticks = 0U;
        s_h2_motion.status.yaw_rate = 0.0f;
        if (h2_stage_is_curve(s_h2_motion.status.stage) == 0U) {
            s_h2_motion.heading_reference = imu->attitude.yaw;
        }
        return;
    }

    yaw_delta = h_wrap_180(imu->attitude.yaw -
                           s_h2_motion.last_frame_yaw);
    {
        uint16_t frame_ticks = (uint16_t)(s_h2_motion.imu_gap_ticks + 1U);
        new_rate = yaw_delta * (100.0f / (float)frame_ticks);
    }
    if (h_abs_float(new_rate) > H2_IMU_MAX_RATE_DPS) {
        s_h2_motion.imu_available = 0U;
        s_h2_motion.status.imu_available = 0U;
        s_h2_motion.status.yaw_rate = 0.0f;
        return;
    }
    s_h2_motion.status.yaw_rate =
        s_h2_motion.status.yaw_rate * 0.70f + new_rate * 0.30f;
    s_h2_motion.accumulated_yaw += yaw_delta;

    if ((h2_stage_is_curve(s_h2_motion.status.stage) != 0U) &&
        (s_h2_motion.curve_turn_sign == 0) &&
        (h_abs_float(s_h2_motion.accumulated_yaw) >=
         H2_CURVE_SIGN_LEARN_DEG)) {
        s_h2_motion.curve_turn_sign =
            (s_h2_motion.accumulated_yaw >= 0.0f) ? 1 : -1;
    }
    s_h2_motion.status.yaw_progress = h2_curve_progress();
    s_h2_motion.last_frame_yaw = imu->attitude.yaw;
    s_h2_motion.last_imu_frames = imu->valid_frames;
    s_h2_motion.imu_gap_ticks = 0U;
}

static void h2_update_line_control(void)
{
    XunjiSensor sensor;
    int limit;

    Xunji_ReadSensorSnapshot(&sensor);
    s_h2_motion.status.sensor = sensor;

    if (s_h2_motion.mode == H2_RUN_CURVE) {
        /*
         * 模式八必须测到纯粹的固定差速，不能被位置环悄悄改写；
         * 但仍监视是否长时间脱离赛道，避免差速错误时继续冲出8秒。
         */
        if (sensor.valid != 0U) {
            s_h2_motion.line_lost_ticks = 0U;
        } else if (s_h2_motion.line_lost_ticks < 65535U) {
            s_h2_motion.line_lost_ticks++;
        }
        s_h2_motion.line_request = 0;
        s_h2_motion.status.line_correction = 0;
        /* 模式八是纯固定差速标定，长时间丢线仍需停止，避免错误差速持续8秒。 */
        if (s_h2_motion.line_lost_ticks >= H2_LINE_FAULT_TICKS) {
            h2_fault(H2_CAL_FAULT_LINE_LOST);
        }
        return;
    }

    if (h2_stage_is_curve(s_h2_motion.status.stage) != 0U) {
        limit = (s_h2_motion.mode == H2_RUN_FINAL) ?
            H2_FINAL_CURVE_LINE_LIMIT : H2_CURVE_LINE_LIMIT;
    } else {
        limit = H2_STRAIGHT_LINE_LIMIT;
    }

    if (sensor.valid != 0U) {
        s_h2_motion.line_lost_ticks = 0U;
        if (s_h2_motion.position_initialized == 0U) {
            s_h2_motion.filtered_position_x4 =
                (int32_t)sensor.position * 4;
            s_h2_motion.position_initialized = 1U;
            PID_ResetPlace();
        } else {
            /* 25%新样本低通，位置环只做慢修正，避免速度60时左右摇头。 */
            s_h2_motion.filtered_position_x4 =
                (s_h2_motion.filtered_position_x4 * 3 +
                 (int32_t)sensor.position * 4) / 4;
        }
        s_h2_motion.status.filtered_position = (int16_t)(
            (s_h2_motion.filtered_position_x4 + 2) / 4);
        s_h2_motion.position_divider++;
        if (s_h2_motion.position_divider >= H2_POSITION_PERIOD_TICKS) {
            s_h2_motion.position_divider = 0U;
            s_h2_motion.line_request = (int16_t)h_clamp_int(
                (int)place_PID_value_limited(
                    450.0f,
                    (float)s_h2_motion.status.filtered_position,
                    (float)limit),
                -limit, limit);
        }
    } else {
        if (s_h2_motion.line_lost_ticks < 65535U) {
            s_h2_motion.line_lost_ticks++;
        }
        s_h2_motion.position_initialized = 0U;
        s_h2_motion.position_divider = 0U;
        if ((s_h2_motion.line_lost_ticks > H2_LINE_HOLD_TICKS) &&
            !((s_h2_motion.mode == H2_RUN_FINAL) &&
              (h2_stage_is_curve(s_h2_motion.status.stage) != 0U))) {
            s_h2_motion.line_request = (int16_t)h_approach_step(
                s_h2_motion.line_request, 0, 1);
        }
        /*
         * 直线长时间丢线仍是故障；模式七的BC/DA弯道允许灰度暂时全白，
         * 由IMU累计转角继续约束曲率，避免DA半程丢线后直接停车。
         */
        if ((h2_stage_is_curve(s_h2_motion.status.stage) == 0U) &&
            (s_h2_motion.line_lost_ticks >= H2_LINE_FAULT_TICKS)) {
            h2_fault(H2_CAL_FAULT_LINE_LOST);
        }
    }
}

/**
 * @brief 模式一按当前时间结果强制进入下一段，DA段则正常结束。
 *
 * 模式一已经通过实车时间标定。里程和IMU只用于提前确认，不应因为两者
 * 未在补偿窗口内同时满足而触发F4停车。超过窗口后按设定时间继续任务，
 * 但电机、编码器、堵转、超速及直线丢线保护仍然有效。
 */
static void h2_final_advance_or_finish(void)
{
    if (s_h2_motion.status.stage == H2_CAL_STAGE_AB) {
        h2_enter_stage(H2_CAL_STAGE_BC, s_h2_motion.status.yaw);
    } else if (s_h2_motion.status.stage == H2_CAL_STAGE_BC) {
        h2_enter_stage(H2_CAL_STAGE_CD, s_h2_motion.status.yaw);
    } else if (s_h2_motion.status.stage == H2_CAL_STAGE_CD) {
        h2_enter_stage(H2_CAL_STAGE_DA, s_h2_motion.status.yaw);
    } else {
        h2_finish();
    }
}

void H2Motion_Tick10ms(void)
{
    IMU601_Snapshot_t imu;
    uint8_t current_index;
    uint8_t timeout_index;
    uint32_t current_target_ticks;
    uint32_t final_stop_tick;
    uint32_t stage_extension_ticks;
    int32_t distance_gate;
    uint8_t angle_ready;
    uint32_t route_timeout_ticks = H2_ROUTE_TIMEOUT_MARGIN_TICKS;

    if ((jb != H2_JB_TIMED_ROUTE) && (jb != H2_JB_CURVE_TEST) &&
        (jb != H2_JB_FINAL_ROUTE)) return;
    if (s_h2_motion.status.running == 0U) return;

    IMU601_GetSnapshot(&imu);
    h2_update_imu(&imu);
    if (s_h2_motion.status.running == 0U) return;

    s_h2_motion.stage_ticks++;
    s_h2_motion.total_ticks++;
    s_h2_motion.status.stage_elapsed_ms =
        (uint32_t)s_h2_motion.stage_ticks * H2_PERIOD_MS;
    if ((s_h2_motion.mode != H2_RUN_FINAL) ||
        (s_h2_motion.status.timing_complete == 0U)) {
        s_h2_motion.status.total_elapsed_ms =
            (uint32_t)s_h2_motion.total_ticks * H2_PERIOD_MS;
    }

    h2_update_line_control();
    if (s_h2_motion.status.running == 0U) return;

    if (s_h2_motion.mode == H2_RUN_CURVE) {
        if ((s_h2_motion.stage_ticks >= H2_CURVE_MIN_RUN_TICKS) &&
            (s_h2_motion.status.yaw_progress >=
             H2_CAL_CURVE_ANGLE_DEG)) {
            h2_finish();
        } else if (s_h2_motion.stage_ticks >=
                   H2_CURVE_TEST_TIMEOUT_TICKS) {
            h2_fault(H2_CAL_FAULT_TIMEOUT);
        }
        return;
    }

    if (s_h2_motion.mode == H2_RUN_FINAL) {
        route_timeout_ticks =
            H2_MS_TO_TICKS(H2_FINAL_ROUTE_TIMEOUT_MARGIN_MS);
    }
    for (timeout_index = 0U; timeout_index < 4U; timeout_index++) {
        route_timeout_ticks += H2_MS_TO_TICKS(
            h2_active_stage_time_ms(timeout_index));
    }
    if ((s_h2_motion.mode != H2_RUN_FINAL) &&
        ((uint32_t)s_h2_motion.total_ticks >= route_timeout_ticks)) {
        h2_fault(H2_CAL_FAULT_TIMEOUT);
        return;
    }

    current_index = h2_stage_index(s_h2_motion.status.stage);
    current_target_ticks = H2_MS_TO_TICKS(
        h2_active_stage_time_ms(current_index));

    if (s_h2_motion.mode == H2_RUN_FINAL) {
        distance_gate =
            (s_h2_motion.status.stage_distance_target[current_index] *
             (int32_t)H2_FINAL_DISTANCE_GATE_PERCENT) / 100;
        angle_ready = (uint8_t)(
            (h2_stage_is_curve(s_h2_motion.status.stage) == 0U) ||
            (s_h2_motion.imu_available == 0U) ||
            ((s_h2_motion.curve_turn_sign != 0) &&
             (h2_curve_progress() >= H2_CURVE_EXIT_MIN_DEG)));
        s_h2_motion.status.time_ready =
            (uint8_t)(s_h2_motion.stage_ticks >= current_target_ticks);
        s_h2_motion.status.distance_ready =
            (uint8_t)(s_h2_motion.status.stage_distance >= distance_gate);
        s_h2_motion.status.angle_ready = angle_ready;

        /*
         * DA仍保留模式七已经验证过的提前50ms短路制动，但新增里程和转角
         * 门槛。任何一个条件不足都不会提前停车，而是进入受限补偿窗口。
         */
        final_stop_tick = (current_target_ticks >
                           H2_MS_TO_TICKS(H2_FINAL_STOP_LEAD_MS)) ?
                          (current_target_ticks -
                           H2_MS_TO_TICKS(H2_FINAL_STOP_LEAD_MS)) : 0U;
        if ((s_h2_motion.status.stage == H2_CAL_STAGE_DA) &&
            (s_h2_motion.stage_ticks >= final_stop_tick) &&
            (s_h2_motion.status.distance_ready != 0U) &&
            (angle_ready != 0U)) {
            h2_finish();
            return;
        }

        if ((s_h2_motion.stage_ticks >= current_target_ticks) &&
            (s_h2_motion.status.distance_ready != 0U) &&
            (angle_ready != 0U)) {
            h2_final_advance_or_finish();
            return;
        }

        stage_extension_ticks = H2_MS_TO_TICKS(
            (h2_stage_is_curve(s_h2_motion.status.stage) != 0U) ?
            H2_FINAL_CURVE_EXTENSION_MS :
            H2_FINAL_STRAIGHT_EXTENSION_MS);
        if (s_h2_motion.stage_ticks >=
            current_target_ticks + stage_extension_ticks) {
            /*
             * 模式一取消F4：里程或转角反馈未及时满足时按时间标定降级，
             * 进入下一段而不是突然停车。DA段在这里按正常完成结束。
             */
            h2_final_advance_or_finish();
        }
        return;
    }

    /*
     * DA段是整圈最后一段，车辆已经完成回到起始方向后仍会因车体惯性
     * 向前冲出终点。仅在转角已经完成（或IMU不可用时退回时间控制）
     * 且进入结束前50ms窗口时执行Motor_Stop()短路制动；转角未完成时
     * 不提前结束，仍按原补转逻辑运行。
     */
    final_stop_tick = (current_target_ticks >
                       H2_MS_TO_TICKS(H2_FINAL_STOP_LEAD_MS)) ?
                      (current_target_ticks -
                       H2_MS_TO_TICKS(H2_FINAL_STOP_LEAD_MS)) : 0U;
    if ((s_h2_motion.status.stage == H2_CAL_STAGE_DA) &&
        (s_h2_motion.stage_ticks >= final_stop_tick) &&
        ((s_h2_motion.imu_available == 0U) ||
         (h2_curve_progress() >= H2_CURVE_EXIT_MIN_DEG))) {
        h2_finish();
        return;
    }

    if (s_h2_motion.stage_ticks < current_target_ticks) return;

    /*
     * 弯道标称时间是进度参考而不是强制截断点。若累计角度尚未达到176度，
     * 最多追加800ms继续补转；BC正常时不会增加时间，DA不足时才自动延长。
     */
    if ((h2_stage_is_curve(s_h2_motion.status.stage) != 0U) &&
        (s_h2_motion.imu_available != 0U) &&
        (s_h2_motion.curve_turn_sign != 0) &&
        (h2_curve_progress() < H2_CURVE_EXIT_MIN_DEG) &&
        (s_h2_motion.stage_ticks <
         current_target_ticks + H2_CURVE_EXTENSION_TICKS)) {
        return;
    }

    if (s_h2_motion.status.stage == H2_CAL_STAGE_AB) {
        h2_enter_stage(H2_CAL_STAGE_BC, s_h2_motion.status.yaw);
    } else if (s_h2_motion.status.stage == H2_CAL_STAGE_BC) {
        h2_enter_stage(H2_CAL_STAGE_CD, s_h2_motion.status.yaw);
    } else if (s_h2_motion.status.stage == H2_CAL_STAGE_CD) {
        h2_enter_stage(H2_CAL_STAGE_DA, s_h2_motion.status.yaw);
    } else {
        h2_finish();
    }
}

static H2CalFault h2_check_wheel(int target,
                                 int measured,
                                 int pwm,
                                 int stall_pwm_threshold,
                                 uint8_t *stall_ticks,
                                 uint8_t *overspeed_ticks,
                                 H2CalFault stall_fault,
                                 H2CalFault overspeed_fault)
{
    if ((target >= 20) && (h_abs_int(measured) < target / 4) &&
        (h_abs_int(pwm) >= stall_pwm_threshold)) {
        if (*stall_ticks < 255U) (*stall_ticks)++;
    } else {
        *stall_ticks = 0U;
    }
    if (*stall_ticks >= H2_STALL_CONFIRM_TICKS) return stall_fault;

    if ((target >= 20) && (h_abs_int(measured) > target * 2 + 20)) {
        if (*overspeed_ticks < 255U) (*overspeed_ticks)++;
    } else {
        *overspeed_ticks = 0U;
    }
    if (*overspeed_ticks >= H2_OVERSPEED_CONFIRM_TICKS) {
        return overspeed_fault;
    }
    return H2_CAL_FAULT_NONE;
}

/**
 * 将循迹修正乘以0.1分辨率的可调权重。
 *
 * 使用定点数避免在10ms控制中断中增加软浮点开销；正负数均四舍五入，
 * 结果随后仍需经过直线或弯道总修正限幅和每周期渐变限制。
 */
static int h2_apply_line_weight(int correction)
{
    int weighted_x10 =
        correction * h2_active_line_weight_x10();

    s_h2_motion.status.line_weight_x10 =
        (int16_t)h2_active_line_weight_x10();

    if (weighted_x10 >= 0) {
        return (weighted_x10 + 5) / 10;
    }
    return -((-weighted_x10 + 5) / 10);
}

static void h2_plan_targets(int measured_left,
                            int measured_right,
                            int *target_left,
                            int *target_right)
{
    int line_correction = s_h2_motion.line_request;
    int imu_correction = 0;
    int combined;

    if (s_h2_motion.mode == H2_RUN_CURVE) {
        *target_left = H2_CAL_BASE_SPEED;
        *target_right =
            H2_CAL_BASE_SPEED - h2_active_curve_delta();
        s_h2_motion.status.line_correction = 0;
        s_h2_motion.status.imu_correction = 0;
        s_h2_motion.status.curve_delta =
            (int16_t)h2_active_curve_delta();
        return;
    }

    if (h2_stage_is_curve(s_h2_motion.status.stage) != 0U) {
        uint8_t index = h2_stage_index(s_h2_motion.status.stage);
        float target_progress;
        float angle_error;
        float target_rate = H2_CAL_CURVE_ANGLE_DEG * 1000.0f /
                            (float)h2_active_stage_time_ms(index);
        float rate_error;
        int curve_delta;

        target_progress = H2_CAL_CURVE_ANGLE_DEG *
            (float)s_h2_motion.status.stage_elapsed_ms /
            (float)h2_active_stage_time_ms(index);
        if (target_progress > H2_CAL_CURVE_ANGLE_DEG) {
            target_progress = H2_CAL_CURVE_ANGLE_DEG;
        }

        if ((s_h2_motion.imu_available != 0U) &&
            (s_h2_motion.curve_turn_sign != 0)) {
            int requested_trim;

            angle_error = target_progress - h2_curve_progress();
            rate_error = target_rate - h2_curve_rate();
            requested_trim =
                (int)(H2_CURVE_ANGLE_KP * angle_error +
                      H2_CURVE_RATE_KP * rate_error);
            if (s_h2_motion.mode == H2_RUN_FINAL) {
                /*
                 * IMU can add turn authority when yaw is behind schedule.
                 * It may only reduce the calibrated curve by one speed unit,
                 * so a noisy yaw-rate sample cannot flatten the first bend.
                 */
                s_h2_motion.rate_trim = (int16_t)h_clamp_int(
                    requested_trim,
                    -H2_FINAL_CURVE_TRIM_DECREASE_LIMIT,
                    H2_FINAL_CURVE_TRIM_INCREASE_LIMIT);
            } else {
                s_h2_motion.rate_trim = (int16_t)h_clamp_int(
                    requested_trim,
                    -H2_CURVE_IMU_TRIM_LIMIT,
                    H2_CURVE_IMU_TRIM_LIMIT);
            }
        } else {
            s_h2_motion.rate_trim = 0;
        }
        imu_correction = s_h2_motion.rate_trim;

        /*
         * 正位置修正表示黑线在左侧，需要减小顺时针曲率，所以从差速中减去；
         * 负位置修正表示黑线在右侧，需要增大曲率。灰度与IMU合计只允许±3，
         * 不会把29附近的实车标定改成完全不同的弯道。
         */
        {
            int curve_line_limit =
                (s_h2_motion.mode == H2_RUN_FINAL) ?
                H2_FINAL_CURVE_LINE_LIMIT : H2_CURVE_LINE_LIMIT;
            int curve_total_limit =
                (s_h2_motion.mode == H2_RUN_FINAL) ?
                H2_FINAL_CURVE_TOTAL_TRIM_LIMIT :
                H2_CURVE_TOTAL_TRIM_LIMIT;

            line_correction = h_clamp_int(line_correction,
                                          -curve_line_limit,
                                          curve_line_limit);
            combined = h_clamp_int(
                imu_correction - h2_apply_line_weight(line_correction),
                -curve_total_limit,
                curve_total_limit);
        }
        s_h2_motion.applied_correction = (int16_t)h_approach_step(
            s_h2_motion.applied_correction,
            combined,
            H2_CURVE_TRIM_SLEW_STEP);
        curve_delta = h_clamp_int(
            h2_active_curve_delta() +
            s_h2_motion.applied_correction,
            H2_CURVE_DELTA_MIN,
            H2_CURVE_DELTA_MAX);
        *target_left = H2_CAL_BASE_SPEED;
        *target_right = h_clamp_int(H2_CAL_BASE_SPEED - curve_delta,
                                    H2_MIN_WHEEL_SPEED,
                                    H2_CAL_BASE_SPEED);
        s_h2_motion.status.line_correction = (int16_t)line_correction;
        s_h2_motion.status.imu_correction = (int16_t)imu_correction;
        s_h2_motion.status.curve_delta = (int16_t)curve_delta;
        return;
    } else {
        float heading_error = h_wrap_180(
            s_h2_motion.status.yaw - s_h2_motion.heading_reference);
        float heading_rate = s_h2_motion.status.yaw_rate;

        imu_correction = h_clamp_int(
            (int)(H2_HEADING_KP * heading_error +
                  H2_HEADING_KD * heading_rate),
            -H2_HEADING_LIMIT, H2_HEADING_LIMIT);
        line_correction = (int)PID_ApplyPlaceRateDamping(
            (float)line_correction,
            (float)measured_left,
            (float)measured_right,
            (float)H2_STRAIGHT_LINE_LIMIT);
        combined = h_clamp_int(
                               h2_apply_line_weight(line_correction) +
                               imu_correction,
                               -H2_TOTAL_CORRECTION_LIMIT,
                               H2_TOTAL_CORRECTION_LIMIT);
        *target_left = H2_CAL_BASE_SPEED;
        *target_right = H2_CAL_BASE_SPEED;
    }

    s_h2_motion.applied_correction = (int16_t)h_approach_step(
        s_h2_motion.applied_correction,
        combined,
        H2_CORRECTION_SLEW_STEP);
    combined = s_h2_motion.applied_correction;
    if (combined >= 0) {
        *target_left -= combined;
    } else {
        *target_right += combined;
    }
    *target_left = h_clamp_int(*target_left,
                               H2_MIN_WHEEL_SPEED,
                               H2_CAL_BASE_SPEED);
    *target_right = h_clamp_int(*target_right,
                                H2_MIN_WHEEL_SPEED,
                                H2_CAL_BASE_SPEED);
    s_h2_motion.status.line_correction = (int16_t)line_correction;
    s_h2_motion.status.imu_correction = (int16_t)imu_correction;
}

void H2Motion_Control10ms(int encoder_left,
                          int encoder_right,
                          int measured_left,
                          int measured_right)
{
    int target_left;
    int target_right;
    int pwm_left;
    int pwm_right;
    H2CalFault fault;

    if ((jb != H2_JB_TIMED_ROUTE) && (jb != H2_JB_CURVE_TEST) &&
        (jb != H2_JB_FINAL_ROUTE)) return;
    if (s_h2_motion.status.running == 0U) return;

    if (s_h2_motion.mode == H2_RUN_FINAL) {
        int distance_delta =
            (h_abs_int(encoder_left) + h_abs_int(encoder_right)) / 2;

        /*
         * 里程只作为最低门槛，不直接生成轮速。使用饱和累加防止长时间
         * 异常运行时有符号溢出反向绕回。
         */
        if (s_h2_motion.ignore_distance_window != 0U) {
            s_h2_motion.ignore_distance_window = 0U;
        } else if (distance_delta > 0) {
            if (s_h2_motion.status.stage_distance <=
                INT32_MAX - distance_delta) {
                s_h2_motion.status.stage_distance += distance_delta;
            } else {
                s_h2_motion.status.stage_distance = INT32_MAX;
            }
            if (s_h2_motion.status.total_distance <=
                INT32_MAX - distance_delta) {
                s_h2_motion.status.total_distance += distance_delta;
            } else {
                s_h2_motion.status.total_distance = INT32_MAX;
            }
            if ((s_h2_motion.status.stage == H2_CAL_STAGE_DA) &&
                (s_h2_motion.status.stage_distance >=
                 s_h2_motion.status.stage_distance_target[3])) {
                h2_freeze_final_timer();
            }
        }
    }

    if (s_h2_motion.status.waiting_imu != 0U) {
        last_pwm_l = 0;
        last_pwm_r = 0;
        Set_Pwm(0, 0, 0, 0);
        return;
    }

    /*
     * Encoder_ReadAndClear()已经让坏窗口保持上一份有效速度；这里再冻结速度PI，
     * 防止它用陈旧反馈继续积分。短暂毛刺期间两轮维持上一拍命令，连续200ms
     * 仍无效则进入高阻故障停车。恢复后的第一拍按真实PWM重新播种PI。
     */
    if ((encoder_feedback_valid_mask & 1U) == 0U) {
        if (s_h2_motion.encoder_left_bad_ticks < 255U) {
            s_h2_motion.encoder_left_bad_ticks++;
        }
    } else {
        s_h2_motion.encoder_left_bad_ticks = 0U;
    }
    if ((encoder_feedback_valid_mask & 2U) == 0U) {
        if (s_h2_motion.encoder_right_bad_ticks < 255U) {
            s_h2_motion.encoder_right_bad_ticks++;
        }
    } else {
        s_h2_motion.encoder_right_bad_ticks = 0U;
    }
    if (s_h2_motion.encoder_left_bad_ticks >=
        H2_ENCODER_BAD_CONFIRM_TICKS) {
        h2_fault(H2_CAL_FAULT_ENCODER_LEFT);
        return;
    }
    if (s_h2_motion.encoder_right_bad_ticks >=
        H2_ENCODER_BAD_CONFIRM_TICKS) {
        h2_fault(H2_CAL_FAULT_ENCODER_RIGHT);
        return;
    }
    if (encoder_feedback_valid_mask != 0x03U) {
        s_h2_motion.stall_left_ticks = 0U;
        s_h2_motion.stall_right_ticks = 0U;
        s_h2_motion.overspeed_left_ticks = 0U;
        s_h2_motion.overspeed_right_ticks = 0U;
        s_h2_motion.velocity_reprime_pending = 1U;
        Set_Pwm(-last_pwm_l, last_pwm_r, 0, 0);
        return;
    }

    h2_plan_targets(measured_left, measured_right,
                    &target_left, &target_right);
    s_h2_motion.status.target_left = (int16_t)target_left;
    s_h2_motion.status.target_right = (int16_t)target_right;

    /*
     * 直线/半圆切换时按上一拍真实PWM反算新目标下的PI积分，
     * 避免右轮目标从80跳到56时继承直线积分而短时全功率。
     */
    if (s_h2_motion.velocity_reprime_pending != 0U) {
        PID_PrimeVelocity((float)target_left,
                          (float)target_right,
                          (float)measured_left,
                          (float)measured_right,
                          (float)last_pwm_l,
                          (float)last_pwm_r);
        s_h2_motion.velocity_reprime_pending = 0U;
    }

    /* 第二问允许快速启动：直接使用模式二已整定速度PI及其受限前馈。 */
    pwm_left = h_clamp_int(
        (int)velocity_PID_value_l((float)target_left,
                                  (float)measured_left),
        0, MOTOR_PWM_SAFE_LIMIT);
    pwm_right = h_clamp_int(
        (int)velocity_PID_value_r((float)target_right,
                                  (float)measured_right),
        0, MOTOR_PWM_SAFE_LIMIT);
    last_pwm_l = pwm_left;
    last_pwm_r = pwm_right;

    fault = h2_check_wheel(target_left, measured_left, pwm_left,
                           H2_STALL_PWM,
                           &s_h2_motion.stall_left_ticks,
                           &s_h2_motion.overspeed_left_ticks,
                           H2_CAL_FAULT_STALL_LEFT,
                           H2_CAL_FAULT_OVERSPEED_LEFT);
    if (fault == H2_CAL_FAULT_NONE) {
        fault = h2_check_wheel(target_right, measured_right, pwm_right,
                               H2_STALL_PWM,
                               &s_h2_motion.stall_right_ticks,
                               &s_h2_motion.overspeed_right_ticks,
                               H2_CAL_FAULT_STALL_RIGHT,
                               H2_CAL_FAULT_OVERSPEED_RIGHT);
    }
    if (fault != H2_CAL_FAULT_NONE) {
        h2_fault(fault);
        return;
    }

    Set_Pwm(-pwm_left, pwm_right, 0, 0);
}

void H2Motion_GetStatus(H2MotionStatus *status)
{
    uint32_t interrupt_state;
    uint8_t index;

    if (status == 0) return;
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *status = s_h2_motion.status;
    if (s_h2_motion.mode != H2_RUN_FINAL) {
        for (index = 0U; index < 4U; index++) {
            status->stage_time_ms[index] = s_h2_stage_time_ms[index];
        }
    }
    if (interrupt_state == 0U) __enable_irq();
}

/* ========================================================================
 * 2026 H题第四/第五问：滚球平稳运输控制器
 *
 * 与第二问的差别：
 *   1. 固定最高轮速40，速度PI明确关闭前馈；
 *   2. 全局速度使用2秒S形曲线，左右轮目标和PWM再分别做斜率限制；
 *   3. 正常完成先把速度缓慢降为0，再高阻释放，不使用短路急停；
 *   4. 人工中止、编码器故障、堵转或严重丢线仍快速进入安全停机。
 * ======================================================================== */

#define H_BALL_JB_Q4                         9
#define H_BALL_JB_Q5                        10
#define H_BALL_MS_TO_TICKS(ms)              \
    (((ms) + H_BALL_CONTROL_PERIOD_MS - 1U) / \
     H_BALL_CONTROL_PERIOD_MS)
#define H_BALL_STOP_RAMP_TICKS              \
    H_BALL_MS_TO_TICKS(H_BALL_STOP_RAMP_TIME_MS)
#define H_BALL_POSITION_PERIOD_TICKS         2U
#define H_BALL_CORRECTION_SLEW_STEP          2
#define H_BALL_STRAIGHT_LINE_LIMIT           6
#define H_BALL_CURVE_LINE_LIMIT              2
#define H_BALL_TOTAL_CORRECTION_LIMIT        12
#define H_BALL_MIN_CURVE_WHEEL_SPEED        16
#define H_BALL_LINE_HOLD_TICKS              20U
#define H_BALL_LINE_FAULT_TICKS            100U
#define H_BALL_STAGE_TIME_MIN_MS           500U
#define H_BALL_STAGE_TIME_MAX_MS         12000U
#define H_BALL_CURVE_DELTA_MIN               4
#define H_BALL_CURVE_DELTA_MAX              28
#define H_BALL_WEIGHT_MIN_X10                0
#define H_BALL_WEIGHT_MAX_X10               60
#define H_BALL_HEADING_KP                  (0.20f)
#define H_BALL_HEADING_KD                  (0.03f)
#define H_BALL_HEADING_LIMIT                 2
#define H_BALL_CURVE_ANGLE_KP              (0.06f)
#define H_BALL_CURVE_RATE_KP               (0.02f)
#define H_BALL_CURVE_IMU_LIMIT               2
#define H_BALL_CURVE_TARGET_ANGLE_DEG     (180.0f)
#define H_BALL_CURVE_SIGN_LEARN_DEG         (5.0f)
#define H_BALL_IMU_STALE_TICKS              50U
#define H_BALL_IMU_MAX_RATE_DPS           (720.0f)
#define H_BALL_NORMAL_STOP_TIMEOUT_TICKS   200U
#define H_BALL_ROUTE_TIMEOUT_MARGIN_MS    3000U
#define H_BALL_ENCODER_BAD_CONFIRM_TICKS     5U

#if H_BALL_PWM_LIMIT > MOTOR_PWM_SAFE_LIMIT
#error "H_BALL_PWM_LIMIT must not exceed the motor driver hard limit"
#endif

/**
 * @brief 清空双轮同步软启动状态。
 *
 * 该状态机只生成左右目标速度，不直接写PWM。实际PWM仍由各自速度PI、
 * PWM斜率限制和电机驱动硬限幅共同决定。
 */
void HBallLaunch_Reset(HBallLaunchState *state)
{
    if (state == 0) return;
    memset(state, 0, sizeof(*state));
    state->phase = H_BALL_LAUNCH_SYNC;
}

static int hball_launch_abs(int value)
{
    return (value < 0) ? -value : value;
}

static void hball_launch_update_confirm(HBallLaunchState *state,
                                        int measured_left,
                                        int measured_right)
{
    if ((state->active_mask & 1U) != 0U) {
        if (hball_launch_abs(measured_left) >=
            H_BALL_SYNC_MOVING_SPEED) {
            if (state->left_confirm_ticks < 255U) {
                state->left_confirm_ticks++;
            }
            if (state->left_confirm_ticks >=
                H_BALL_SYNC_CONFIRM_TICKS) {
                state->ready_mask |= 1U;
            }
        } else if ((state->ready_mask & 1U) == 0U) {
            state->left_confirm_ticks = 0U;
        }
    }

    if ((state->active_mask & 2U) != 0U) {
        if (hball_launch_abs(measured_right) >=
            H_BALL_SYNC_MOVING_SPEED) {
            if (state->right_confirm_ticks < 255U) {
                state->right_confirm_ticks++;
            }
            if (state->right_confirm_ticks >=
                H_BALL_SYNC_CONFIRM_TICKS) {
                state->ready_mask |= 2U;
            }
        } else if ((state->ready_mask & 2U) == 0U) {
            state->right_confirm_ticks = 0U;
        }
    }
}

/**
 * @brief 生成同步起步阶段的左右轮目标速度。
 *
 * SYNC阶段先把目标缓慢提高到低速爬行值。某一轮先克服静摩擦后，该轮
 * 降到HOLD速度等待，未启动轮继续保持CRAWL目标；这样实车放地启动时不会
 * 出现一轮已经明显前冲、另一轮仍静止。两轮都连续检测到转动后，再共同
 * 按三次S曲线升到最终速度。
 */
int HBallLaunch_Update(HBallLaunchState *state,
                       int final_speed,
                       int measured_left,
                       int measured_right,
                       uint8_t active_mask)
{
    uint32_t ramp_ticks;
    uint8_t required_mask;

    if (state == 0) return 0;
    active_mask &= 0x03U;
    final_speed = hball_launch_abs(final_speed);

    if (state->active_mask != active_mask) {
        HBallLaunch_Reset(state);
        state->active_mask = active_mask;
    }
    required_mask = state->active_mask;

    if (required_mask == 0U) {
        state->phase = H_BALL_LAUNCH_DONE;
        state->target_speed = 0;
        state->target_left = 0;
        state->target_right = 0;
        return 0;
    }

    if (state->phase == H_BALL_LAUNCH_SYNC) {
        uint32_t timeout_ticks =
            H_BALL_MS_TO_TICKS(H_BALL_SYNC_TIMEOUT_MS);
        int sync_target = final_speed;

        if (sync_target > H_BALL_SYNC_CRAWL_SPEED) {
            sync_target = H_BALL_SYNC_CRAWL_SPEED;
        }
        if (state->target_speed < sync_target) {
            state->target_speed = (int16_t)h_approach_step(
                state->target_speed,
                sync_target,
                H_BALL_TARGET_SLEW_STEP);
        } else if (state->target_speed > sync_target) {
            state->target_speed = (int16_t)sync_target;
        }

        hball_launch_update_confirm(
            state, measured_left, measured_right);

        state->target_left =
            ((required_mask & 1U) == 0U) ? 0 :
            (((state->ready_mask & 1U) != 0U) &&
             ((state->ready_mask & required_mask) != required_mask)) ?
                H_BALL_SYNC_HOLD_SPEED : state->target_speed;
        state->target_right =
            ((required_mask & 2U) == 0U) ? 0 :
            (((state->ready_mask & 2U) != 0U) &&
             ((state->ready_mask & required_mask) != required_mask)) ?
                H_BALL_SYNC_HOLD_SPEED : state->target_speed;

        if ((state->ready_mask & required_mask) == required_mask) {
            state->phase_ticks = 0U;
            if (final_speed <= H_BALL_SYNC_CRAWL_SPEED) {
                state->phase = H_BALL_LAUNCH_DONE;
                state->target_speed = (int16_t)final_speed;
                state->target_left =
                    ((required_mask & 1U) != 0U) ?
                    (int16_t)final_speed : 0;
                state->target_right =
                    ((required_mask & 2U) != 0U) ?
                    (int16_t)final_speed : 0;
            } else {
                state->phase = H_BALL_LAUNCH_RAMP;
                state->target_speed = H_BALL_SYNC_CRAWL_SPEED;
                state->target_left =
                    ((required_mask & 1U) != 0U) ?
                    H_BALL_SYNC_CRAWL_SPEED : 0;
                state->target_right =
                    ((required_mask & 2U) != 0U) ?
                    H_BALL_SYNC_CRAWL_SPEED : 0;
            }
        } else {
            if (state->phase_ticks < 65535U) {
                state->phase_ticks++;
            }
            if (state->phase_ticks >= timeout_ticks) {
                state->timeout_mask =
                    required_mask & (uint8_t)(~state->ready_mask);
            }
        }
        return state->target_speed;
    }

    if (state->phase == H_BALL_LAUNCH_RAMP) {
        float x;
        float smooth;
        int target;

        ramp_ticks = H_BALL_MS_TO_TICKS(H_BALL_START_RAMP_TIME_MS);
        if (ramp_ticks == 0U) ramp_ticks = 1U;
        if (state->phase_ticks >= ramp_ticks) {
            state->phase = H_BALL_LAUNCH_DONE;
            state->target_speed = (int16_t)final_speed;
        } else {
            x = (float)state->phase_ticks / (float)ramp_ticks;
            smooth = x * x * (3.0f - 2.0f * x);
            target = H_BALL_SYNC_CRAWL_SPEED +
                (int)(smooth *
                      (float)(final_speed -
                              H_BALL_SYNC_CRAWL_SPEED) +
                      0.5f);
            state->target_speed = (int16_t)h_clamp_int(
                target, H_BALL_SYNC_CRAWL_SPEED, final_speed);
            state->phase_ticks++;
        }
    } else {
        state->target_speed = (int16_t)final_speed;
    }

    state->target_left = ((required_mask & 1U) != 0U) ?
        state->target_speed : 0;
    state->target_right = ((required_mask & 2U) != 0U) ?
        state->target_speed : 0;
    return state->target_speed;
}

uint8_t HBallLaunch_IsComplete(const HBallLaunchState *state)
{
    return (uint8_t)((state != 0) &&
                     (state->phase == H_BALL_LAUNCH_DONE));
}

uint8_t HBallLaunch_GetTimeoutMask(const HBallLaunchState *state)
{
    return (state != 0) ? state->timeout_mask : 0U;
}

typedef struct {
    HBallRouteStatus status;
    HBallLaunchState launch;
    uint32_t total_ticks;
    uint32_t stage_ticks;
    uint32_t post_point_ticks;
    uint32_t decel_ticks;
    uint16_t normal_stop_ticks;
    uint32_t last_imu_frames;
    uint16_t imu_gap_ticks;
    float last_frame_yaw;
    float heading_reference;
    float accumulated_yaw;
    int8_t curve_turn_sign;
    int32_t filtered_position_x4;
    uint8_t position_initialized;
    uint8_t position_divider;
    uint16_t line_lost_ticks;
    int16_t line_request;
    int16_t applied_correction;
    int16_t applied_left;
    int16_t applied_right;
    int16_t pwm_left;
    int16_t pwm_right;
    uint8_t stall_left_ticks;
    uint8_t stall_right_ticks;
    uint8_t overspeed_left_ticks;
    uint8_t overspeed_right_ticks;
    uint8_t encoder_left_bad_ticks;
    uint8_t encoder_right_bad_ticks;
    uint8_t velocity_reprime_pending;
} HBallController;

static HBallController s_hball;
static volatile uint32_t s_hball_q4_time_ms[4] = {
    H_BALL_Q4_AB_TIME_MS,
    H_BALL_Q4_BC_TIME_MS,
    H_BALL_Q4_CD_TIME_MS,
    H_BALL_Q4_DA_TIME_MS
};
static volatile uint32_t s_hball_q5_time_ms[4] = {
    H_BALL_Q5_AB_TIME_MS,
    H_BALL_Q5_BC_TIME_MS,
    H_BALL_Q5_CD_TIME_MS,
    H_BALL_Q5_DA_TIME_MS
};
static volatile int16_t s_hball_q4_delta = H_BALL_Q4_CURVE_DELTA;
static volatile int16_t s_hball_q5_delta = H_BALL_Q5_CURVE_DELTA;
static volatile int16_t s_hball_q4_weight_x10 =
    H_BALL_Q4_LINE_WEIGHT_X10;
static volatile int16_t s_hball_q5_weight_x10 =
    H_BALL_Q5_LINE_WEIGHT_X10;

static uint8_t hball_jb_active(void)
{
    return (uint8_t)((jb == H_BALL_JB_Q4) ||
                     (jb == H_BALL_JB_Q5));
}

static uint8_t hball_stage_is_curve(H2CalStage stage)
{
    return (uint8_t)((stage == H2_CAL_STAGE_BC) ||
                     (stage == H2_CAL_STAGE_DA));
}

static uint8_t hball_stage_index(H2CalStage stage)
{
    if ((stage < H2_CAL_STAGE_AB) || (stage > H2_CAL_STAGE_DA)) {
        return 0U;
    }
    return (uint8_t)(stage - H2_CAL_STAGE_AB);
}

static volatile uint32_t *hball_time_table(HBallRouteMode mode)
{
    return (mode == H_BALL_ROUTE_Q5) ?
        s_hball_q5_time_ms : s_hball_q4_time_ms;
}

static int hball_configured_delta(HBallRouteMode mode)
{
    return (mode == H_BALL_ROUTE_Q5) ?
        (int)s_hball_q5_delta : (int)s_hball_q4_delta;
}

static int hball_configured_weight_x10(HBallRouteMode mode)
{
    return (mode == H_BALL_ROUTE_Q5) ?
        (int)s_hball_q5_weight_x10 : (int)s_hball_q4_weight_x10;
}

static uint32_t hball_stage_time_ms(HBallRouteMode mode,
                                    uint8_t stage_index)
{
    volatile uint32_t *table = hball_time_table(mode);

    if (stage_index >= 4U) return 0U;
    return table[stage_index];
}

static int32_t hball_task_distance_target(HBallRouteMode mode)
{
    return (mode == H_BALL_ROUTE_Q5) ?
        (int32_t)H_BALL_Q5_LAP_DISTANCE_COUNTS :
        (int32_t)H_BALL_Q4_AB_DISTANCE_COUNTS;
}

static void hball_add_distance(int32_t *value, int delta)
{
    if ((value == 0) || (delta <= 0)) return;
    if (*value <= INT32_MAX - delta) {
        *value += delta;
    } else {
        *value = INT32_MAX;
    }
}

static void hball_freeze_task_timer(void)
{
    if (s_hball.status.timing_complete != 0U) return;
    s_hball.status.timing_complete = 1U;
    s_hball.status.total_elapsed_ms =
        s_hball.total_ticks * H_BALL_CONTROL_PERIOD_MS;
}

/** 剩余停车窗口内使用3x^2-2x^3，保证减速起点和终点斜率连续。 */
static int hball_smooth_stop_speed(uint32_t remaining_ticks)
{
    float x;
    float smooth;

    if (remaining_ticks >= H_BALL_STOP_RAMP_TICKS) {
        return H_BALL_BASE_SPEED;
    }
    if (H_BALL_STOP_RAMP_TICKS == 0U) return 0;
    x = (float)remaining_ticks / (float)H_BALL_STOP_RAMP_TICKS;
    smooth = x * x * (3.0f - 2.0f * x);
    return h_clamp_int((int)(smooth * (float)H_BALL_BASE_SPEED + 0.5f),
                       0, H_BALL_BASE_SPEED);
}

static int hball_profile_speed(void)
{
    int speed = s_hball.launch.target_speed;

    if (s_hball.status.phase == H_BALL_PHASE_POST_POINT_DECEL) {
        uint32_t remaining_ticks =
            (s_hball.decel_ticks < H_BALL_STOP_RAMP_TICKS) ?
            (H_BALL_STOP_RAMP_TICKS - s_hball.decel_ticks) : 0U;
        int decel_speed = hball_smooth_stop_speed(remaining_ticks);

        if (decel_speed < speed) speed = decel_speed;
    } else if (s_hball.status.phase == H_BALL_PHASE_STOPPING) {
        speed = 0;
    }
    return speed;
}

/**
 * 最终路段最后的停车窗口允许驶出黑线，避免丢线故障覆盖正常减速曲线。
 */
static uint8_t hball_terminal_decel_active(void)
{
    return (uint8_t)(
        (s_hball.status.phase == H_BALL_PHASE_POST_POINT_DECEL) ||
        (s_hball.status.phase == H_BALL_PHASE_STOPPING));
}

static float hball_curve_progress(void)
{
    if (s_hball.curve_turn_sign > 0) {
        return s_hball.accumulated_yaw;
    }
    if (s_hball.curve_turn_sign < 0) {
        return -s_hball.accumulated_yaw;
    }
    return h_abs_float(s_hball.accumulated_yaw);
}

static float hball_curve_rate(void)
{
    if (s_hball.curve_turn_sign > 0) {
        return s_hball.status.yaw_rate;
    }
    if (s_hball.curve_turn_sign < 0) {
        return -s_hball.status.yaw_rate;
    }
    return h_abs_float(s_hball.status.yaw_rate);
}

static void hball_update_imu(const IMU601_Snapshot_t *imu)
{
    float yaw_delta;
    float new_rate;

    if ((imu->valid_frames == 0U) ||
        (h2_imu_yaw_is_valid(imu->attitude.yaw) == 0U)) {
        if (s_hball.imu_gap_ticks < 65535U) {
            s_hball.imu_gap_ticks++;
        }
        if (s_hball.imu_gap_ticks >= H_BALL_IMU_STALE_TICKS) {
            s_hball.status.imu_available = 0U;
            s_hball.status.yaw_rate = 0.0f;
        }
        return;
    }

    s_hball.status.yaw = imu->attitude.yaw;
    if (imu->valid_frames == s_hball.last_imu_frames) {
        if (s_hball.imu_gap_ticks < 65535U) {
            s_hball.imu_gap_ticks++;
        }
        if (s_hball.imu_gap_ticks >= H_BALL_IMU_STALE_TICKS) {
            s_hball.status.imu_available = 0U;
            s_hball.status.yaw_rate = 0.0f;
        }
        return;
    }

    if (s_hball.status.imu_available == 0U) {
        s_hball.status.imu_available = 1U;
        s_hball.last_imu_frames = imu->valid_frames;
        s_hball.last_frame_yaw = imu->attitude.yaw;
        s_hball.imu_gap_ticks = 0U;
        s_hball.status.yaw_rate = 0.0f;
        s_hball.heading_reference = imu->attitude.yaw;
        return;
    }

    yaw_delta = h_wrap_180(imu->attitude.yaw -
                           s_hball.last_frame_yaw);
    new_rate = yaw_delta *
        (100.0f / (float)(s_hball.imu_gap_ticks + 1U));
    if (h_abs_float(new_rate) > H_BALL_IMU_MAX_RATE_DPS) {
        s_hball.status.imu_available = 0U;
        s_hball.status.yaw_rate = 0.0f;
        return;
    }

    s_hball.status.yaw_rate =
        s_hball.status.yaw_rate * 0.75f + new_rate * 0.25f;
    if (hball_stage_is_curve(s_hball.status.stage) != 0U) {
        s_hball.accumulated_yaw += yaw_delta;
        if ((s_hball.curve_turn_sign == 0) &&
            (h_abs_float(s_hball.accumulated_yaw) >=
             H_BALL_CURVE_SIGN_LEARN_DEG)) {
            s_hball.curve_turn_sign =
                (s_hball.accumulated_yaw >= 0.0f) ? 1 : -1;
        }
        s_hball.status.yaw_progress = hball_curve_progress();
    }
    s_hball.last_imu_frames = imu->valid_frames;
    s_hball.last_frame_yaw = imu->attitude.yaw;
    s_hball.imu_gap_ticks = 0U;
}

static void hball_enter_stage(H2CalStage stage)
{
    uint8_t index = hball_stage_index(stage);

    s_hball.status.stage = stage;
    s_hball.status.segment_index = index;
    s_hball.stage_ticks = 0U;
    s_hball.status.stage_elapsed_ms = 0U;
    s_hball.status.stage_distance = 0;
    s_hball.status.yaw_progress = 0.0f;
    s_hball.accumulated_yaw = 0.0f;
    s_hball.curve_turn_sign = 0;
    s_hball.heading_reference = s_hball.status.yaw;
    s_hball.status.curve_delta =
        (int16_t)hball_configured_delta(s_hball.status.mode);
}

static void hball_clear_outputs(void)
{
    s_hball.status.base_speed = 0;
    s_hball.status.target_left = 0;
    s_hball.status.target_right = 0;
    s_hball.status.pwm_left = 0;
    s_hball.status.pwm_right = 0;
    s_hball.applied_left = 0;
    s_hball.applied_right = 0;
    s_hball.pwm_left = 0;
    s_hball.pwm_right = 0;
    last_pwm_l = 0;
    last_pwm_r = 0;
    PID_ResetAll();
}

static void hball_finish(void)
{
    s_hball.status.running = 0U;
    s_hball.status.normal_stop_pending = 0U;
    s_hball.status.fault = H2_CAL_FAULT_NONE;
    s_hball.status.stage = H2_CAL_STAGE_DONE;
    jb = 0;
    hball_clear_outputs();
    /* 车轮已经通过独立S形减速降为0；此处只释放H桥，依靠静摩擦保持钢球稳定。 */
    Motor_Coast();
}

static void hball_fault(H2CalFault fault)
{
    s_hball.status.running = 0U;
    s_hball.status.normal_stop_pending = 0U;
    s_hball.status.fault = fault;
    s_hball.status.stage = H2_CAL_STAGE_FAULT;
    jb = 0;
    hball_clear_outputs();

    if ((fault == H2_CAL_FAULT_STALL_LEFT) ||
        (fault == H2_CAL_FAULT_STALL_RIGHT) ||
        (fault == H2_CAL_FAULT_OVERSPEED_LEFT) ||
        (fault == H2_CAL_FAULT_OVERSPEED_RIGHT) ||
        (fault == H2_CAL_FAULT_ENCODER_LEFT) ||
        (fault == H2_CAL_FAULT_ENCODER_RIGHT)) {
        Motor_Coast();
    } else {
        Motor_Stop();
    }
}

static void hball_update_line(void)
{
    XunjiSensor sensor;
    int limit;

    Xunji_ReadSensorSnapshot(&sensor);
    s_hball.status.sensor = sensor;
    limit = (hball_stage_is_curve(s_hball.status.stage) != 0U) ?
        H_BALL_CURVE_LINE_LIMIT : H_BALL_STRAIGHT_LINE_LIMIT;

    if (sensor.valid != 0U) {
        s_hball.line_lost_ticks = 0U;
        if (s_hball.position_initialized == 0U) {
            s_hball.filtered_position_x4 =
                (int32_t)sensor.position * 4;
            s_hball.position_initialized = 1U;
            PID_ResetPlace();
        } else {
            s_hball.filtered_position_x4 =
                (s_hball.filtered_position_x4 * 3 +
                 (int32_t)sensor.position * 4) / 4;
        }
        s_hball.status.filtered_position = (int16_t)(
            (s_hball.filtered_position_x4 + 2) / 4);
        s_hball.position_divider++;
        if (s_hball.position_divider >=
            H_BALL_POSITION_PERIOD_TICKS) {
            s_hball.position_divider = 0U;
            s_hball.line_request = (int16_t)h_clamp_int(
                (int)place_PID_value_limited(
                    450.0f,
                    (float)s_hball.status.filtered_position,
                    (float)limit),
                -limit, limit);
        }
        return;
    }

    if (s_hball.line_lost_ticks < 65535U) {
        s_hball.line_lost_ticks++;
    }
    s_hball.position_initialized = 0U;
    s_hball.position_divider = 0U;
    if (s_hball.line_lost_ticks > H_BALL_LINE_HOLD_TICKS) {
        s_hball.line_request = (int16_t)h_approach_step(
            s_hball.line_request, 0, 1);
    }

    /*
     * 半圆中传感器短时全白由固定差速和IMU继续保持曲率；直线连续1秒全白说明
     * 已离开赛道，继续平缓行驶反而会扩大风险，因此进入快速安全停机。
     */
    if ((hball_stage_is_curve(s_hball.status.stage) == 0U) &&
        (hball_terminal_decel_active() == 0U) &&
        (s_hball.line_lost_ticks >= H_BALL_LINE_FAULT_TICKS)) {
        hball_fault(H2_CAL_FAULT_LINE_LOST);
    }
}

static int hball_apply_weight(int correction)
{
    int weighted_x10 =
        correction * hball_configured_weight_x10(s_hball.status.mode);

    if (weighted_x10 >= 0) return (weighted_x10 + 5) / 10;
    return -((-weighted_x10 + 5) / 10);
}

static int hball_scale_for_speed(int value, int base_speed)
{
    int scaled;

    if (base_speed <= 0) return 0;
    scaled = value * base_speed;
    if (scaled >= 0) {
        return (scaled + H_BALL_BASE_SPEED / 2) /
               H_BALL_BASE_SPEED;
    }
    return -((-scaled + H_BALL_BASE_SPEED / 2) /
             H_BALL_BASE_SPEED);
}

static void hball_plan_targets(int measured_left,
                               int measured_right,
                               int *target_left,
                               int *target_right)
{
    int base_speed = hball_profile_speed();
    int line_correction = s_hball.line_request;
    int imu_correction = 0;
    int combined;

    s_hball.status.base_speed = (int16_t)base_speed;
    s_hball.status.line_weight_x10 =
        (int16_t)hball_configured_weight_x10(s_hball.status.mode);

    if (hball_stage_is_curve(s_hball.status.stage) != 0U) {
        uint8_t index = hball_stage_index(s_hball.status.stage);
        uint32_t stage_time =
            hball_stage_time_ms(s_hball.status.mode, index);
        int curve_delta = hball_configured_delta(s_hball.status.mode);

        if ((s_hball.status.imu_available != 0U) &&
            (s_hball.curve_turn_sign != 0) &&
            (stage_time != 0U)) {
            float target_progress = H_BALL_CURVE_TARGET_ANGLE_DEG *
                (float)s_hball.status.stage_elapsed_ms /
                (float)stage_time;
            float target_rate =
                H_BALL_CURVE_TARGET_ANGLE_DEG * 1000.0f /
                (float)stage_time;
            float angle_error;
            float rate_error;

            if (target_progress > H_BALL_CURVE_TARGET_ANGLE_DEG) {
                target_progress = H_BALL_CURVE_TARGET_ANGLE_DEG;
            }
            angle_error = target_progress - hball_curve_progress();
            rate_error = target_rate - hball_curve_rate();
            imu_correction = h_clamp_int(
                (int)(H_BALL_CURVE_ANGLE_KP * angle_error +
                      H_BALL_CURVE_RATE_KP * rate_error),
                -H_BALL_CURVE_IMU_LIMIT,
                H_BALL_CURVE_IMU_LIMIT);
        }

        line_correction = h_clamp_int(
            line_correction,
            -H_BALL_CURVE_LINE_LIMIT,
            H_BALL_CURVE_LINE_LIMIT);
        combined = h_clamp_int(
            imu_correction - hball_apply_weight(line_correction),
            -H_BALL_TOTAL_CORRECTION_LIMIT,
            H_BALL_TOTAL_CORRECTION_LIMIT);
        s_hball.applied_correction = (int16_t)h_approach_step(
            s_hball.applied_correction,
            combined,
            H_BALL_CORRECTION_SLEW_STEP);
        combined = hball_scale_for_speed(
            s_hball.applied_correction, base_speed);
        curve_delta = hball_scale_for_speed(curve_delta, base_speed);
        curve_delta = h_clamp_int(
            curve_delta + combined, 0, base_speed);
        *target_left = base_speed;
        *target_right = h_clamp_int(
            base_speed - curve_delta,
            (base_speed >= H_BALL_MIN_CURVE_WHEEL_SPEED) ?
                H_BALL_MIN_CURVE_WHEEL_SPEED : 0,
            base_speed);
        s_hball.status.curve_delta =
            (int16_t)(curve_delta);
    } else {
        if (s_hball.status.imu_available != 0U) {
            float heading_error = h_wrap_180(
                s_hball.status.yaw - s_hball.heading_reference);
            imu_correction = h_clamp_int(
                (int)(H_BALL_HEADING_KP * heading_error +
                      H_BALL_HEADING_KD *
                      s_hball.status.yaw_rate),
                -H_BALL_HEADING_LIMIT,
                H_BALL_HEADING_LIMIT);
        }
        line_correction = (int)PID_ApplyPlaceRateDamping(
            (float)line_correction,
            (float)measured_left,
            (float)measured_right,
            (float)H_BALL_STRAIGHT_LINE_LIMIT);
        combined = h_clamp_int(
            hball_apply_weight(line_correction) + imu_correction,
            -H_BALL_TOTAL_CORRECTION_LIMIT,
            H_BALL_TOTAL_CORRECTION_LIMIT);
        s_hball.applied_correction = (int16_t)h_approach_step(
            s_hball.applied_correction,
            combined,
            H_BALL_CORRECTION_SLEW_STEP);
        combined = hball_scale_for_speed(
            s_hball.applied_correction, base_speed);
        *target_left = base_speed;
        *target_right = base_speed;
        if (combined >= 0) {
            *target_left -= combined;
        } else {
            *target_right += combined;
        }
        *target_left = h_clamp_int(*target_left, 0, base_speed);
        *target_right = h_clamp_int(*target_right, 0, base_speed);
        s_hball.status.curve_delta =
            (int16_t)hball_configured_delta(s_hball.status.mode);
    }

    s_hball.status.line_correction = (int16_t)line_correction;
    s_hball.status.imu_correction = (int16_t)imu_correction;
}

void HBall_SetStageTime(HBallRouteMode mode,
                        uint8_t stage_index,
                        uint32_t time_ms)
{
    volatile uint32_t *table;
    uint32_t interrupt_state;

    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return;
    if (stage_index >= 4U) return;
    if (time_ms < H_BALL_STAGE_TIME_MIN_MS) {
        time_ms = H_BALL_STAGE_TIME_MIN_MS;
    }
    if (time_ms > H_BALL_STAGE_TIME_MAX_MS) {
        time_ms = H_BALL_STAGE_TIME_MAX_MS;
    }
    time_ms = (time_ms / H_BALL_TIME_STEP_MS) *
              H_BALL_TIME_STEP_MS;
    table = hball_time_table(mode);

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    table[stage_index] = time_ms;
    if (interrupt_state == 0U) __enable_irq();
}

uint32_t HBall_GetStageTime(HBallRouteMode mode, uint8_t stage_index)
{
    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return 0U;
    return hball_stage_time_ms(mode, stage_index);
}

void HBall_SetCurveDelta(HBallRouteMode mode, int delta)
{
    uint32_t interrupt_state;

    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return;
    delta = h_clamp_int(delta,
                        H_BALL_CURVE_DELTA_MIN,
                        H_BALL_CURVE_DELTA_MAX);
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (mode == H_BALL_ROUTE_Q5) {
        s_hball_q5_delta = (int16_t)delta;
    } else {
        s_hball_q4_delta = (int16_t)delta;
    }
    if (interrupt_state == 0U) __enable_irq();
}

int HBall_GetCurveDelta(HBallRouteMode mode)
{
    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return 0;
    return hball_configured_delta(mode);
}

void HBall_SetLineWeightX10(HBallRouteMode mode, int weight_x10)
{
    uint32_t interrupt_state;

    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return;
    weight_x10 = h_clamp_int(weight_x10,
                             H_BALL_WEIGHT_MIN_X10,
                             H_BALL_WEIGHT_MAX_X10);
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (mode == H_BALL_ROUTE_Q5) {
        s_hball_q5_weight_x10 = (int16_t)weight_x10;
    } else {
        s_hball_q4_weight_x10 = (int16_t)weight_x10;
    }
    if (interrupt_state == 0U) __enable_irq();
}

int HBall_GetLineWeightX10(HBallRouteMode mode)
{
    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return 0;
    return hball_configured_weight_x10(mode);
}

uint8_t HBall_Start(HBallRouteMode mode)
{
    IMU601_Snapshot_t imu;
    uint8_t index;

    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) return 0U;

    jb = 0;
    memset(&s_hball, 0, sizeof(s_hball));
    HBallLaunch_Reset(&s_hball.launch);
    s_hball.status.mode = mode;
    s_hball.status.phase = H_BALL_PHASE_ROUTE;
    s_hball.status.fault = H2_CAL_FAULT_NONE;
    s_hball.status.running = 1U;
    s_hball.status.task_distance_target =
        hball_task_distance_target(mode);
    s_hball.filtered_position_x4 = 450 * 4;
    s_hball.status.filtered_position = 450;
    for (index = 0U; index < 4U; index++) {
        s_hball.status.stage_time_ms[index] =
            hball_stage_time_ms(mode, index);
    }
    s_hball.status.line_weight_x10 =
        (int16_t)hball_configured_weight_x10(mode);
    s_hball.status.curve_delta =
        (int16_t)hball_configured_delta(mode);

    IMU601_GetSnapshot(&imu);
    if ((IMU601_IsReady() != 0U) &&
        (imu.valid_frames != 0U) &&
        (h2_imu_yaw_is_valid(imu.attitude.yaw) != 0U)) {
        s_hball.status.imu_available = 1U;
        s_hball.status.yaw = imu.attitude.yaw;
        s_hball.last_imu_frames = imu.valid_frames;
        s_hball.last_frame_yaw = imu.attitude.yaw;
        s_hball.heading_reference = imu.attitude.yaw;
    }

    PID_ResetAll();
    last_pwm_l = 0;
    last_pwm_r = 0;
    Set_Pwm(0, 0, 0, 0);
    hball_enter_stage(H2_CAL_STAGE_AB);
    jb = (mode == H_BALL_ROUTE_Q5) ?
        H_BALL_JB_Q5 : H_BALL_JB_Q4;
    return 1U;
}

void HBall_Abort(void)
{
    if ((hball_jb_active() == 0U) &&
        (s_hball.status.running == 0U)) {
        memset(&s_hball, 0, sizeof(s_hball));
        return;
    }
    if (hball_jb_active() != 0U) jb = 0;
    s_hball.status.running = 0U;
    s_hball.status.normal_stop_pending = 0U;
    s_hball.status.phase = H_BALL_PHASE_STOPPING;
    s_hball.status.stage = H2_CAL_STAGE_IDLE;
    hball_clear_outputs();
    /* 人工中止属于安全停止，不等待2秒正常减速。短刹50ms后驱动层自动高阻。 */
    Motor_Stop();
}

void HBall_Tick10ms(int encoder_left, int encoder_right)
{
    IMU601_Snapshot_t imu;
    uint8_t index;
    uint32_t stage_target_ticks;
    uint32_t route_timeout_ticks;
    uint8_t timeout_index;
    int distance_delta;

    if (hball_jb_active() == 0U) return;
    if (s_hball.status.running == 0U) return;

    s_hball.total_ticks++;
    if (s_hball.status.timing_complete == 0U) {
        s_hball.status.total_elapsed_ms =
            s_hball.total_ticks * H_BALL_CONTROL_PERIOD_MS;
    }
    if (encoder_feedback_valid_mask == 0x03U) {
        distance_delta =
            (h_abs_int(encoder_left) + h_abs_int(encoder_right)) / 2;
        hball_add_distance(&s_hball.status.stage_distance,
                           distance_delta);
        hball_add_distance(&s_hball.status.total_distance,
                           distance_delta);
        if (s_hball.status.phase != H_BALL_PHASE_ROUTE) {
            hball_add_distance(&s_hball.status.post_point_distance,
                               distance_delta);
        }
    }

    IMU601_GetSnapshot(&imu);
    hball_update_imu(&imu);
    hball_update_line();
    if (s_hball.status.running == 0U) return;

    if (s_hball.status.normal_stop_pending != 0U) {
        if (s_hball.normal_stop_ticks < 65535U) {
            s_hball.normal_stop_ticks++;
        }
        if (s_hball.normal_stop_ticks >=
            H_BALL_NORMAL_STOP_TIMEOUT_TICKS) {
            hball_fault(H2_CAL_FAULT_TIMEOUT);
        }
        return;
    }

    s_hball.stage_ticks++;
    s_hball.status.stage_elapsed_ms =
        s_hball.stage_ticks * H_BALL_CONTROL_PERIOD_MS;
    index = hball_stage_index(s_hball.status.stage);
    stage_target_ticks = H_BALL_MS_TO_TICKS(
        hball_stage_time_ms(s_hball.status.mode, index));

    route_timeout_ticks =
        H_BALL_MS_TO_TICKS(H_BALL_ROUTE_TIMEOUT_MARGIN_MS);
    if (s_hball.status.mode == H_BALL_ROUTE_Q4) {
        route_timeout_ticks += H_BALL_MS_TO_TICKS(
            hball_stage_time_ms(H_BALL_ROUTE_Q4, 0U));
    } else {
        for (timeout_index = 0U; timeout_index < 4U;
             timeout_index++) {
            route_timeout_ticks += H_BALL_MS_TO_TICKS(
                hball_stage_time_ms(H_BALL_ROUTE_Q5,
                                    timeout_index));
        }
    }
    if ((s_hball.status.timing_complete == 0U) &&
        (s_hball.total_ticks >= route_timeout_ticks)) {
        hball_fault(H2_CAL_FAULT_TIMEOUT);
        return;
    }

    if (s_hball.status.phase == H_BALL_PHASE_POST_POINT_CRUISE) {
        s_hball.post_point_ticks++;
        if (s_hball.post_point_ticks >=
            H_BALL_MS_TO_TICKS(H_BALL_POST_POINT_CRUISE_MS)) {
            s_hball.status.phase = H_BALL_PHASE_POST_POINT_DECEL;
            s_hball.decel_ticks = 0U;
            return;
        }
        if (s_hball.post_point_ticks >=
            H_BALL_MS_TO_TICKS(H_BALL_POST_POINT_TIMEOUT_MS)) {
            hball_fault(H2_CAL_FAULT_TIMEOUT);
        }
        return;
    }

    if (s_hball.status.phase == H_BALL_PHASE_POST_POINT_DECEL) {
        if (s_hball.decel_ticks < H_BALL_STOP_RAMP_TICKS) {
            s_hball.decel_ticks++;
        }
        if (s_hball.decel_ticks >= H_BALL_STOP_RAMP_TICKS) {
            s_hball.status.phase = H_BALL_PHASE_STOPPING;
            s_hball.status.normal_stop_pending = 1U;
            s_hball.normal_stop_ticks = 0U;
        }
        return;
    }

    if ((s_hball.status.mode == H_BALL_ROUTE_Q4) &&
        (s_hball.status.stage == H2_CAL_STAGE_AB) &&
        (s_hball.status.stage_distance >=
         s_hball.status.task_distance_target)) {
        hball_freeze_task_timer();
        /* Pass B at constant speed for two seconds before the gentle stop. */
        s_hball.status.phase = H_BALL_PHASE_POST_POINT_CRUISE;
        s_hball.status.post_point_distance = 0;
        s_hball.post_point_ticks = 0U;
        s_hball.decel_ticks = 0U;
        return;
    }

    if ((s_hball.status.mode == H_BALL_ROUTE_Q5) &&
        (s_hball.status.stage == H2_CAL_STAGE_DA) &&
        (s_hball.status.total_distance >=
         s_hball.status.task_distance_target)) {
        hball_freeze_task_timer();
        hball_enter_stage(H2_CAL_STAGE_AB);
        s_hball.status.phase = H_BALL_PHASE_POST_POINT_CRUISE;
        s_hball.status.post_point_distance = 0;
        s_hball.post_point_ticks = 0U;
        return;
    }

    if (s_hball.stage_ticks < stage_target_ticks) return;

    if ((s_hball.status.mode == H_BALL_ROUTE_Q4) ||
        (s_hball.status.stage == H2_CAL_STAGE_DA)) {
        /* 计时阶段只按里程结束；标称时间到达后继续匀速等待里程门槛。 */
        return;
    }

    if (s_hball.status.stage == H2_CAL_STAGE_AB) {
        hball_enter_stage(H2_CAL_STAGE_BC);
    } else if (s_hball.status.stage == H2_CAL_STAGE_BC) {
        hball_enter_stage(H2_CAL_STAGE_CD);
    } else if (s_hball.status.stage == H2_CAL_STAGE_CD) {
        hball_enter_stage(H2_CAL_STAGE_DA);
    }
}

void HBall_Control10ms(int measured_left, int measured_right)
{
    int requested_left;
    int requested_right;
    int desired_left;
    int desired_right;
    uint8_t launch_timeout_mask;
    H2CalFault fault;

    if (hball_jb_active() == 0U) return;
    if (s_hball.status.running == 0U) return;

    if ((encoder_feedback_valid_mask & 1U) == 0U) {
        if (s_hball.encoder_left_bad_ticks < 255U) {
            s_hball.encoder_left_bad_ticks++;
        }
    } else {
        s_hball.encoder_left_bad_ticks = 0U;
    }
    if ((encoder_feedback_valid_mask & 2U) == 0U) {
        if (s_hball.encoder_right_bad_ticks < 255U) {
            s_hball.encoder_right_bad_ticks++;
        }
    } else {
        s_hball.encoder_right_bad_ticks = 0U;
    }
    if (s_hball.encoder_left_bad_ticks >=
        H_BALL_ENCODER_BAD_CONFIRM_TICKS) {
        hball_fault(H2_CAL_FAULT_ENCODER_LEFT);
        return;
    }
    if (s_hball.encoder_right_bad_ticks >=
        H_BALL_ENCODER_BAD_CONFIRM_TICKS) {
        hball_fault(H2_CAL_FAULT_ENCODER_RIGHT);
        return;
    }
    if (encoder_feedback_valid_mask != 0x03U) {
        /*
         * 反馈冻结期间不更新轮速PI，也不允许旧的堵转/超速累计跨过坏窗口。
         * 否则恢复后的第一帧可能继承故障前的临界计数而误停。
         */
        s_hball.stall_left_ticks = 0U;
        s_hball.stall_right_ticks = 0U;
        s_hball.overspeed_left_ticks = 0U;
        s_hball.overspeed_right_ticks = 0U;
        s_hball.velocity_reprime_pending = 1U;
        Set_Pwm(-s_hball.pwm_left,
                s_hball.pwm_right, 0, 0);
        return;
    }

    (void)HBallLaunch_Update(&s_hball.launch,
                             H_BALL_BASE_SPEED,
                             measured_left,
                             measured_right,
                             0x03U);
    launch_timeout_mask =
        HBallLaunch_GetTimeoutMask(&s_hball.launch);
    if ((launch_timeout_mask & 1U) != 0U) {
        hball_fault(H2_CAL_FAULT_STALL_LEFT);
        return;
    }
    if ((launch_timeout_mask & 2U) != 0U) {
        hball_fault(H2_CAL_FAULT_STALL_RIGHT);
        return;
    }

    if (s_hball.launch.phase == H_BALL_LAUNCH_SYNC) {
        /*
         * 同步阶段只解决左右轮静摩擦差异，不叠加循迹或IMU修正。
         * 先启动轮被限制在HOLD速度，未启动轮保持CRAWL目标。
         */
        desired_left = s_hball.launch.target_left;
        desired_right = s_hball.launch.target_right;
        s_hball.status.base_speed = s_hball.launch.target_speed;
        s_hball.status.line_correction = 0;
        s_hball.status.imu_correction = 0;
        s_hball.applied_correction = (int16_t)h_approach_step(
            s_hball.applied_correction, 0,
            H_BALL_CORRECTION_SLEW_STEP);
    } else {
        hball_plan_targets(measured_left, measured_right,
                           &desired_left, &desired_right);
    }
    s_hball.applied_left = (int16_t)h_approach_step(
        s_hball.applied_left,
        desired_left,
        H_BALL_TARGET_SLEW_STEP);
    s_hball.applied_right = (int16_t)h_approach_step(
        s_hball.applied_right,
        desired_right,
        H_BALL_TARGET_SLEW_STEP);
    s_hball.status.target_left = s_hball.applied_left;
    s_hball.status.target_right = s_hball.applied_right;

    /*
     * 编码器坏窗口恢复后的第一拍，按保持期间的真实PWM重新播种PI。
     * 否则PI内部历史仍对应坏窗口前的误差，可能在恢复瞬间产生PWM台阶。
     */
    if (s_hball.velocity_reprime_pending != 0U) {
        PID_PrimeVelocityNoFeedforward(
            (float)s_hball.applied_left,
            (float)s_hball.applied_right,
            (float)measured_left,
            (float)measured_right,
            (float)s_hball.pwm_left,
            (float)s_hball.pwm_right);
        s_hball.velocity_reprime_pending = 0U;
    }

    if (s_hball.applied_left == 0) {
        requested_left = 0;
        PID_ResetVelocityLeft();
    } else {
        requested_left = h_clamp_int(
            (int)velocity_PID_value_l_no_ff(
                (float)s_hball.applied_left,
                (float)measured_left),
            0, H_BALL_PWM_LIMIT);
    }
    if (s_hball.applied_right == 0) {
        requested_right = 0;
        PID_ResetVelocityRight();
    } else {
        requested_right = h_clamp_int(
            (int)velocity_PID_value_r_no_ff(
                (float)s_hball.applied_right,
                (float)measured_right),
            0, H_BALL_PWM_LIMIT);
    }

    s_hball.pwm_left = (int16_t)h_approach_step(
        s_hball.pwm_left,
        requested_left,
        H_BALL_PWM_SLEW_STEP);
    s_hball.pwm_right = (int16_t)h_approach_step(
        s_hball.pwm_right,
        requested_right,
        H_BALL_PWM_SLEW_STEP);

    /*
     * 左右轮机械差异会让两侧PWM斜率限制在不同周期触发。只能播种实际受限
     * 的一侧，否则另一侧积分会被无关轮子的量化波动反复清除。
     */
    if (s_hball.pwm_left != requested_left) {
        PID_PrimeVelocityLeftNoFeedforward(
            (float)s_hball.applied_left,
            (float)measured_left,
            (float)s_hball.pwm_left);
    }
    if (s_hball.pwm_right != requested_right) {
        PID_PrimeVelocityRightNoFeedforward(
            (float)s_hball.applied_right,
            (float)measured_right,
            (float)s_hball.pwm_right);
    }

    s_hball.status.pwm_left = s_hball.pwm_left;
    s_hball.status.pwm_right = s_hball.pwm_right;
    last_pwm_l = s_hball.pwm_left;
    last_pwm_r = s_hball.pwm_right;

    fault = h2_check_wheel(
        s_hball.applied_left,
        measured_left,
        s_hball.pwm_left,
        H_BALL_STALL_PWM_THRESHOLD,
        &s_hball.stall_left_ticks,
        &s_hball.overspeed_left_ticks,
        H2_CAL_FAULT_STALL_LEFT,
        H2_CAL_FAULT_OVERSPEED_LEFT);
    if (fault == H2_CAL_FAULT_NONE) {
        fault = h2_check_wheel(
            s_hball.applied_right,
            measured_right,
            s_hball.pwm_right,
            H_BALL_STALL_PWM_THRESHOLD,
            &s_hball.stall_right_ticks,
            &s_hball.overspeed_right_ticks,
            H2_CAL_FAULT_STALL_RIGHT,
            H2_CAL_FAULT_OVERSPEED_RIGHT);
    }
    if (fault != H2_CAL_FAULT_NONE) {
        hball_fault(fault);
        return;
    }

    Set_Pwm(-s_hball.pwm_left,
            s_hball.pwm_right, 0, 0);

    if ((s_hball.status.normal_stop_pending != 0U) &&
        (s_hball.applied_left == 0) &&
        (s_hball.applied_right == 0) &&
        (s_hball.pwm_left == 0) &&
        (s_hball.pwm_right == 0)) {
        hball_finish();
    }
}

void HBall_GetStatus(HBallRouteStatus *status)
{
    uint32_t interrupt_state;
    uint8_t index;
    HBallRouteMode mode;

    if (status == 0) return;
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *status = s_hball.status;
    mode = status->mode;
    if ((mode != H_BALL_ROUTE_Q4) &&
        (mode != H_BALL_ROUTE_Q5)) {
        mode = H_BALL_ROUTE_Q4;
    }
    for (index = 0U; index < 4U; index++) {
        status->stage_time_ms[index] =
            hball_stage_time_ms(mode, index);
    }
    if (interrupt_state == 0U) __enable_irq();
}
