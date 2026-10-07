#include "xunji.h"
#include "encoder.h"
#include "moter.h"
#include "pid.h"
#include <string.h>

/*
 * 逻辑参数区：这些量描述的是消抖和状态持续时间，不是从参考工程照搬的
 * 电机参数。当前H题运球任务基准使用 empty.c 中的 purpose=45。
 */
#define XUNJI_CENTER_POSITION          450
#define XUNJI_CENTER_CONFIRM_TICKS       2U   /* 160ms保护后只需连续回中20ms，避免确认期间继续转过头 */
#define XUNJI_CORNER_MIN_TICKS          16U   /* 实车标定的最短转向160ms；防旧线但不强迫转过200ms */
#define XUNJI_CORNER_GUARD_TICKS        30U   /* 出弯后 300ms 内不重复计角 */
#define XUNJI_CORNER_INTENT_TICKS       15U   /* 角点证据在丢线后保留150ms */
#define XUNJI_CORNER_INTENT_CONFIRM      2U   /* 外侧探头至少连续20ms，才作为计角证据 */
#define XUNJI_MARKER_CONFIRM_TICKS       2U   /* 同侧最外三探头连续20ms，确认宽拐角标记 */
#define XUNJI_MARKER_ADVANCE_COUNTS      30  /* 约一个50ms测速窗口：只越过宽线前沿，不吞掉下一条边 */
#define XUNJI_MARKER_ADVANCE_PERCENT     80  /* 前探时使用目标速度的80%，保留制动余量 */
#define XUNJI_CORNER_DEPART_TICKS         5U  /* 前探入弯后至少离开旧中心50ms才允许结束 */
#define XUNJI_CORNER_REARM_TICKS        10U   /* 下一条直线连续回中100ms才重新允许计角 */
#define XUNJI_CORNER_REARM_SPEED_WINDOWS 8   /* 未学到边长前至少前进约8个满速窗口 */
#define XUNJI_CORNER_REARM_MIN_COUNTS   400  /* 低速调试时仍保留最小空间隔离 */
#define XUNJI_LOST_ENTER_CONFIRM_TICKS   6U   /* 每计数10ms；当前连续60ms才进入LOST */
#define XUNJI_LOST_TO_CORNER_TICKS       16U  /* 每拍10ms：恢复实车验证过的160ms判弯窗口 */
#define XUNJI_SHORT_RECOVERY_TICKS        0U  /* 短丢线恢复不二次降速，避免反复100ms脉冲 */
#define XUNJI_RECOVERY_TICKS            50U   /* 出弯后 500ms 平滑恢复速度 */
#define XUNJI_CORNER_TIMEOUT_TICKS      250U  /* 2.5s覆盖低速带载完成90度转向 */
#define XUNJI_LOST_STOP_TICKS           100U  /* 连续丢线1s后禁止继续盲走 */
#define XUNJI_SENSOR_STUCK_TICKS        100U  /* 8路全触发1s才判异常，避免拐角黑区误报 */
#define XUNJI_STALL_CONFIRM_WINDOWS      20U  /* 10ms×20=200ms，高PWM半堵转及时撤销驱动 */
#define XUNJI_OVERSPEED_CONFIRM_WINDOWS  15U  /* 10ms×15=150ms */
#define XUNJI_TRACK_MIN_PERCENT         75
#define XUNJI_CORNER_APPROACH_PERCENT   70   /* target60时约42，接近原实车预减速绝对值 */
#define XUNJI_TURN_SPEED_PERCENT        85   /* target60时外轮约51，保留足够转向静摩擦力 */
#define XUNJI_TARGET_RISE_DIVISOR        25  /* target60时每10ms升约2 */
#define XUNJI_TARGET_FALL_DIVISOR        10  /* target60时每10ms降约6 */
#define XUNJI_TRACK_TARGET_SLEW_DIVISOR  50  /* 直线每10ms只改约1 */
#define XUNJI_TRACK_CORRECTION_PERCENT   35  /* 直线方向修正最多占当前基速35% */
#define XUNJI_CORRECTION_BUILD_STEP       1  /* 偏离中心时每10ms最多增加1 */
#define XUNJI_CORRECTION_RELEASE_STEP     2  /* 回中时更快撤销，避免修正穿过中心 */
#define XUNJI_DIRECTION_MEMORY_ERROR     100  /* 400/500中心量化不更新丢线搜索方向 */
#define XUNJI_POSITION_PID_DIVIDER         2U /* 10ms基准上每2拍=20ms */
#define XUNJI_TASK_TARGET_SLEW_STEP        1  /* 轮速目标每10ms最多变化1 */
#define XUNJI_TASK_PWM_SLEW_STEP          12  /* PWM每10ms最多变化12，限制驱动冲击 */

/*
 * 模式一独立高速循迹参数。80是外轮最高目标，不会修改purpose，也不会影响
 * 模式二、模式四或模式五。较小的最大差速配合独立阻尼，优先抑制高速摇头。
 */
#define XUNJI_MODE1_BASE_SPEED             80
#define XUNJI_MODE1_CORRECTION_PERCENT     30
#define XUNJI_MODE1_CORRECTION_BUILD_STEP   2
#define XUNJI_MODE1_CORRECTION_RELEASE_STEP 3
#define XUNJI_MODE1_TARGET_SLEW_STEP        2
#define XUNJI_MODE1_PWM_SLEW_STEP          18

/* 模式五连续位置环调试：短暂全白先消抖，确认丢线后保持前进并向最后见线侧搜线。
 * 外轮目标不超过purpose，内轮降低60%；既有明确差速，也不会因丢线突然超速。 */
#define XUNJI_TUNE_LOST_ENTER_TICKS         3U /* 连续全白30ms才切入搜线 */
#define XUNJI_TUNE_SEARCH_CORRECTION_PERCENT 60

#define XUNJI_STRAIGHT_PERIOD_MS            10U
#define XUNJI_STRAIGHT_MS_TO_TICKS(ms) \
    (((ms) + XUNJI_STRAIGHT_PERIOD_MS - 1U) / XUNJI_STRAIGHT_PERIOD_MS)
#define XUNJI_STRAIGHT_START_RAMP_MS      1800U
#define XUNJI_STRAIGHT_DECEL_MS           4000U
#define XUNJI_STRAIGHT_ROUTE_TIMEOUT_MS  30000U
#define XUNJI_STRAIGHT_STOP_SETTLE_TICKS    10U
#define XUNJI_STRAIGHT_STOP_SETTLE_SPEED      2
#define XUNJI_TIMED_TASK_BASE_SPEED           80
#define XUNJI_TIMED_WEIGHT_DEFAULT_X10         10
#define XUNJI_TIMED_WEIGHT_MIN_X10              0
#define XUNJI_TIMED_WEIGHT_MAX_X10             50

/* 2026 H题第二问：A点横向黑线是唯一圈终点标志，赛道本体为连续黑线。 */
#define H2_MARKER_MIN_ACTIVE                5U /* 5cm横线应覆盖至少5路，普通1.8cm轨迹不会 */
#define H2_START_RELEASE_TICKS              5U /* 横线释放且正常见线50ms后才武装终点 */
#define H2_FINISH_CONFIRM_TICKS             2U /* 终点横线连续20ms，抑制单拍毛刺 */
#define H2_FINISH_MIN_RUNTIME_TICKS        300U /* 3s内禁止终点，避免起点附近重复触发 */
#define H2_ROUTE_TIMEOUT_TICKS            2200U /* 22s硬保护；赛题合格时间仍是20s */
#define H2_LOST_ENTER_TICKS                 3U /* 连续全白30ms后开始按最后方向搜线 */
#define H2_LOST_TIMEOUT_TICKS              80U /* 搜线800ms仍失败则急停 */
#define H2_SEARCH_SPEED_PERCENT            65  /* 搜线基速，外轮不超过已验证purpose */
#define H2_SEARCH_CORRECTION_PERCENT       70  /* 降低内轮，禁止反转 */
#define H2_FINISH_APPROACH_PERCENT         50  /* 停车偏移阶段降速，减小停车误差 */
#define H2_STOP_SETTLE_SPEED                2  /* 两轮低于该速度才允许宣布完成 */
#define H2_STOP_SETTLE_TICKS               10U /* 连续100ms稳定后再进入DONE */
/*
 * 终点横线首次确认时，前置灰度排通常还在车体参考点前方。这里必须用实车
 * “传感器到计时参考点”的编码器计数标定；0表示确认后立即开始平滑减速。
 */
#define H2_A_STOP_OFFSET_COUNTS              0

/* 可选的赛前单边编码器标定值。保持 0 时首圈自动学习；填入实测值后，
 * 上电后的第一圈也能立即提供连续的 0..1000 圈内进度。 */
#define XUNJI_SIDE_COUNTS_PRESET          0

extern volatile float purpose;
extern volatile int jb;
extern volatile int last_pwm_l;
extern volatile int last_pwm_r;

volatile float xun = (float)XUNJI_CENTER_POSITION;
volatile int xun_pwm_l = 0;
volatile int xun_pwm_r = 0;

typedef struct {
    XunjiStatus status;

    uint8_t has_last_position;
    int16_t last_position;
    int8_t last_error_direction;

    uint8_t corner_candidate_ticks;
    int8_t corner_candidate_direction;
    uint8_t center_ticks;
    uint8_t corner_ticks;
    uint8_t corner_guard_ticks;
    uint8_t corner_armed;
    uint8_t corner_rearm_ticks;
    uint8_t corner_intent_ticks;
    int8_t corner_intent_direction;
    uint8_t corner_departed_center;
    uint8_t line_missing_ticks;
    uint8_t lost_ticks;
    int8_t lost_direction;
    uint8_t lost_has_corner_evidence;
    uint8_t turn_count_enabled;
    int8_t advance_direction;
    uint8_t advance_distance_started;
    int32_t advance_start_distance;
    int32_t advance_event_distance;
    uint8_t recovery_ticks;
    uint8_t sensor_stuck_ticks;
    uint8_t stall_left_windows;
    uint8_t stall_right_windows;
    uint8_t overspeed_left_windows;
    uint8_t overspeed_right_windows;

    int16_t applied_left;
    int16_t applied_right;
    int16_t applied_correction;
    int16_t position_pid_correction;
    uint8_t position_pid_divider;

    int32_t filtered_position_x4;
    uint8_t position_filter_initialized;
    int16_t position_samples[3];
    uint8_t position_sample_index;

    int32_t first_corner_distance;
    int32_t last_corner_distance;
    int32_t last_corner_exit_distance;
    int32_t lost_start_distance;
    int32_t corner_event_distance;
    int32_t side_distance_sum;
    uint8_t side_distance_samples;

    uint8_t waiting_lap_finish;
    int32_t lap_finish_distance;
    int32_t lap_start_distance;

    /* 状态切换后的首个10ms速度周期按实际PWM反算PI，禁止恢复过期积分。 */
    uint8_t velocity_reprime_pending;
    uint8_t velocity_bridge_windows;

    uint32_t h2_elapsed_ticks;
    uint8_t h2_start_release_ticks;
    uint8_t h2_finish_marker_ticks;
    uint8_t h2_finish_armed;
    uint8_t h2_finish_pending;
    uint8_t h2_finish_decelerating;
    uint8_t h2_stop_settle_ticks;
    uint8_t h2_lost_ticks;
    int8_t h2_search_direction;
    int32_t h2_finish_stop_distance;

    uint32_t straight_elapsed_ticks;
    uint32_t straight_run_ticks;
    uint32_t straight_post_ticks;
    uint32_t straight_post_target_ticks;
    uint32_t straight_decel_ticks;
    uint32_t straight_time_target_ticks;
    uint8_t straight_stop_settle_ticks;
} XunjiController;

static XunjiController s_control;
static int32_t s_learned_side_distance = XUNJI_SIDE_COUNTS_PRESET;
static uint8_t s_direction_tune_mode = 0U;
static uint8_t s_h2_lap_mode = 0U;
static uint8_t s_mode1_test_mode = 0U;
static uint8_t s_straight_task_mode = 0U;
static uint8_t s_timed_task_mode = 0U;
static volatile int16_t s_timed_line_weight_x10 =
    XUNJI_TIMED_WEIGHT_DEFAULT_X10;

/** 同时清除位置PID本体和20ms调度缓存，禁止状态切换后泄漏旧修正。 */
static void reset_position_control(void)
{
    PID_ResetPlace();
    PID_ResetMode1Place();
    s_control.position_pid_correction = 0;
    s_control.position_pid_divider = 0U;
    /*
     * 状态切换或重新见线时，旧的搜索/转向差速已经失去物理含义，必须同步清除。
     * 左右轮目标仍由后续1单位/10ms斜坡改变，因此这里不会造成速度阶跃。
     */
    s_control.applied_correction = 0;
}

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int round_float_to_int(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) :
                             (int)(value - 0.5f);
}

static int16_t median_position(int16_t first,
                               int16_t second,
                               int16_t third)
{
    int16_t temporary;

    if (first > second) {
        temporary = first;
        first = second;
        second = temporary;
    }
    if (second > third) {
        temporary = second;
        second = third;
        third = temporary;
    }
    if (first > second) {
        second = first;
    }
    return second;
}

/**
 * @brief 对轮速目标使用“慢加速、快减速”的非对称斜坡。
 *
 * 加速过快会增加电流和打滑；减速本身不会增加驱动功率，却直接决定入弯是否及时，
 * 因此下降步长应大于上升步长。
 */
static int approach_speed_target(int current,
                                 int target,
                                 int rise_step,
                                 int fall_step)
{
    if (current < target) {
        current += rise_step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= fall_step;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static int straight_profile_speed(int final_speed)
{
    uint32_t ticks;
    uint32_t duration_ticks;
    float ratio;
    float smooth;

    if (s_straight_task_mode == 0U) return final_speed;

    if (s_control.status.straight_phase == XUNJI_STRAIGHT_RUN) {
        ticks = s_control.straight_run_ticks;
        duration_ticks =
            XUNJI_STRAIGHT_MS_TO_TICKS(XUNJI_STRAIGHT_START_RAMP_MS);
        if (ticks >= duration_ticks) return final_speed;
        ratio = (duration_ticks == 0U) ? 1.0f :
                (float)ticks / (float)duration_ticks;
    } else if (s_control.status.straight_phase ==
               XUNJI_STRAIGHT_POST_CRUISE) {
        return final_speed;
    } else if (s_control.status.straight_phase ==
               XUNJI_STRAIGHT_DECEL) {
        duration_ticks =
            XUNJI_STRAIGHT_MS_TO_TICKS(XUNJI_STRAIGHT_DECEL_MS);
        if (s_control.straight_decel_ticks >= duration_ticks) return 0;
        ratio = (duration_ticks == 0U) ? 0.0f :
                (float)(duration_ticks - s_control.straight_decel_ticks) /
                (float)duration_ticks;
    } else {
        return 0;
    }

    smooth = ratio * ratio * (3.0f - 2.0f * ratio);
    return round_float_to_int((float)final_speed * smooth);
}

/**
 * 修正建立较慢、撤销较快；若请求跨过零，必须先回到零，禁止一拍内反向。
 */
static int approach_correction(int current, int target)
{
    int same_direction =
        ((current >= 0) && (target >= 0)) ||
        ((current <= 0) && (target <= 0));
    int current_abs = abs_int(current);
    int target_abs = abs_int(target);
    int step;
    int build_step = (s_mode1_test_mode != 0U) ?
        XUNJI_MODE1_CORRECTION_BUILD_STEP : XUNJI_CORRECTION_BUILD_STEP;
    int release_step = (s_mode1_test_mode != 0U) ?
        XUNJI_MODE1_CORRECTION_RELEASE_STEP : XUNJI_CORRECTION_RELEASE_STEP;

    if (current == target) {
        return current;
    }
    if (same_direction == 0) {
        target = 0;
        step = release_step;
    } else if (target_abs > current_abs) {
        step = build_step;
    } else {
        step = release_step;
    }
    return approach_speed_target(current, target, step, step);
}

static void seed_position_filter(int16_t position)
{
    uint8_t index;

    s_control.filtered_position_x4 = (int32_t)position * 4;
    s_control.position_filter_initialized = 1U;
    s_control.position_sample_index = 0U;
    for (index = 0U; index < 3U; index++) {
        s_control.position_samples[index] = position;
    }
    s_control.status.control_position = position;
}

static void update_position_filter(int16_t position)
{
    int16_t median;

    if (s_control.position_filter_initialized == 0U) {
        seed_position_filter(position);
        return;
    }

    s_control.position_samples[s_control.position_sample_index] = position;
    s_control.position_sample_index++;
    if (s_control.position_sample_index >= 3U) {
        s_control.position_sample_index = 0U;
    }

    median = median_position(s_control.position_samples[0],
                             s_control.position_samples[1],
                             s_control.position_samples[2]);
    /* 三点中值先去除单拍跳格，再用50%新样本低通；比旧25%新样本延迟更小。 */
    s_control.filtered_position_x4 =
        (s_control.filtered_position_x4 + (int32_t)median * 4) / 2;
    s_control.status.control_position =
        (int16_t)((s_control.filtered_position_x4 + 2) / 4);
}

/* GPIO pin 宏均为单 bit 掩码，求其位序后保持原工程已经验证的取反逻辑。 */
static uint16_t find_set_bit(uint32_t value)
{
    uint16_t bit_position = 0U;

    while ((value & 1U) == 0U) {
        value >>= 1U;
        bit_position++;
    }
    return bit_position;
}

uint8_t my_GPIO_readPin(GPIO_Regs *gpio, uint32_t pins)
{
    uint16_t bit_position = find_set_bit(pins);
    uint32_t read_value = DL_GPIO_readPins(gpio, pins);

    /* 返回值 1 的含义保持为“该通道检测到循迹线”。禁止在没有实车校验时
     * 根据通用模块资料改动这里的极性。 */
    return (uint8_t)!((read_value >> bit_position) & 1U);
}

static XunjiSensor read_sensor(void)
{
    static const int16_t weights[8] = {100, 200, 300, 400, 500, 600, 700, 800};
    static const uint32_t pins[8] = {
        GPIO_XUNJI_L4_PIN, GPIO_XUNJI_L3_PIN,
        GPIO_XUNJI_L2_PIN, GPIO_XUNJI_L1_PIN,
        GPIO_XUNJI_R1_PIN, GPIO_XUNJI_R2_PIN,
        GPIO_XUNJI_R3_PIN, GPIO_XUNJI_R4_PIN
    };
    XunjiSensor sensor;
    int32_t weighted_sum = 0;
    uint8_t index;

    memset(&sensor, 0, sizeof(sensor));

    for (index = 0U; index < 8U; index++) {
        if (my_GPIO_readPin(GPIO_XUNJI_PORT, pins[index]) != 0U) {
            sensor.bits |= (uint8_t)(1U << index);
            sensor.active_count++;
            weighted_sum += weights[index];
        }
    }

    sensor.valid = (sensor.active_count != 0U) ? 1U : 0U;
    if (sensor.valid != 0U) {
        sensor.position = (int16_t)(weighted_sum / sensor.active_count);
        s_control.last_position = sensor.position;
        s_control.has_last_position = 1U;
    } else if (s_control.has_last_position != 0U) {
        /* 短时丢线保留最后方向，具体保持/搜索时限由状态机控制。 */
        sensor.position = s_control.last_position;
    } else {
        /* 启动前尚未见线时保持中性，避免默认向某一侧猛转。 */
        sensor.position = XUNJI_CENTER_POSITION;
    }

    sensor.error = (int16_t)(sensor.position - XUNJI_CENTER_POSITION);
    return sensor;
}

void Xunji_ReadSensorSnapshot(XunjiSensor *sensor)
{
    if (sensor != 0) {
        *sensor = read_sensor();
    }
}

void xunji_shua_xin(void)
{
    s_control.status.sensor = read_sensor();
    xun = (float)s_control.status.sensor.position;

    /* 状态识别使用原始位图；位置环单独使用三点中值+50%低通。 */
    if (s_control.status.sensor.valid != 0U) {
        update_position_filter(s_control.status.sensor.position);
    }
}

int xunji_biao_zhi(void)
{
    xunji_shua_xin();
    return (s_control.status.sensor.valid != 0U) ? 0 : 1;
}

static uint8_t is_centered(const XunjiSensor *sensor)
{
    uint8_t center_bits = sensor->bits & (XUNJI_BIT_L1 | XUNJI_BIT_R1);

    return (uint8_t)((sensor->valid != 0U) &&
                     (center_bits != 0U) &&
                     (sensor->active_count <= 4U) &&
                     (abs_int(sensor->error) <= 150));
}

/**
 * @brief 返回当前外侧探头给出的拐角接近方向：-1=左，+1=右，0=证据不足。
 *
 * 这里只负责提前减速和给后续丢线提供“计角证据”，不再直接切入转弯状态。
 * 真正的转弯入口统一由 LOST 持续时间决定，避免宽线/瞬时走偏直接虚增角点。
 */
static int8_t corner_approach_direction(const XunjiSensor *sensor)
{
    uint8_t left_outer = sensor->bits & (XUNJI_BIT_L4 | XUNJI_BIT_L3);
    uint8_t right_outer = sensor->bits & (XUNJI_BIT_R3 | XUNJI_BIT_R4);
    uint8_t enough_pattern = (sensor->active_count >= 2U) ? 1U : 0U;

    if ((sensor->valid == 0U) ||
        (s_control.corner_guard_ticks != 0U) ||
        (s_control.recovery_ticks != 0U) ||
        (s_control.corner_armed == 0U) ||
        (s_control.waiting_lap_finish != 0U)) {
        return 0;
    }

    if ((left_outer != 0U) && (right_outer == 0U) &&
        ((enough_pattern != 0U) || (sensor->error <= -180))) {
        return -1;
    }
    if ((right_outer != 0U) && (left_outer == 0U) &&
        ((enough_pattern != 0U) || (sensor->error >= 180))) {
        return 1;
    }

    /* 两侧外探头同时命中时不盲猜；只有重心明显偏向一侧才保留方向证据。 */
    if ((left_outer != 0U) && (right_outer != 0U)) {
        if (sensor->error <= -180) {
            return -1;
        }
        if (sensor->error >= 180) {
            return 1;
        }
    }
    return 0;
}

/** 同侧最外三探头同时压到黑线，才把图样当作宽拐角标记。 */
static int8_t outer_three_direction(const XunjiSensor *sensor)
{
    const uint8_t left_mask = XUNJI_BIT_L4 | XUNJI_BIT_L3 | XUNJI_BIT_L2;
    const uint8_t right_mask = XUNJI_BIT_R2 | XUNJI_BIT_R3 | XUNJI_BIT_R4;
    uint8_t left_match = ((sensor->bits & left_mask) == left_mask) ? 1U : 0U;
    uint8_t right_match = ((sensor->bits & right_mask) == right_mask) ? 1U : 0U;

    if ((left_match != 0U) && (right_match == 0U)) {
        return -1;
    }
    if ((right_match != 0U) && (left_match == 0U)) {
        return 1;
    }

    /* 两侧都命中时可能是交叉黑区；只有重心明确偏向一侧才选择方向。 */
    if ((left_match != 0U) && (right_match != 0U)) {
        if (sensor->error <= -180) {
            return -1;
        }
        if (sensor->error >= 180) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 计算两个不同物理角点之间必须满足的最小里程。
 *
 * 首圈尚未学到边长时，用目标速度乘以8个50ms窗口建立约400ms的空间隔离；
 * 学到边长后要求至少走过半条边。它与时间保护叠加，避免宽角点或出弯抖动
 * 在同一位置被重复累计。
 */
static int32_t corner_rearm_min_distance(void)
{
    int32_t minimum_distance;

    if (s_control.status.estimated_side > 0) {
        minimum_distance = s_control.status.estimated_side / 2;
    } else {
        int target = (purpose < 0.0f) ? (int)(-purpose) : (int)purpose;
        minimum_distance = (int32_t)target * XUNJI_CORNER_REARM_SPEED_WINDOWS;
    }

    if (minimum_distance < XUNJI_CORNER_REARM_MIN_COUNTS) {
        minimum_distance = XUNJI_CORNER_REARM_MIN_COUNTS;
    }
    return minimum_distance;
}

/** 只有离开旧角点、走够距离并在下一条直线上连续回中后，才重新允许计角。 */
static void update_corner_rearm(const XunjiSensor *sensor)
{
    int32_t travelled_since_corner;

    if (s_control.corner_armed != 0U) {
        return;
    }

    travelled_since_corner = s_control.status.total_distance -
                             s_control.last_corner_exit_distance;
    if ((s_control.corner_guard_ticks == 0U) &&
        (travelled_since_corner >= corner_rearm_min_distance()) &&
        (is_centered(sensor) != 0U)) {
        if (s_control.corner_rearm_ticks < 255U) {
            s_control.corner_rearm_ticks++;
        }
        if (s_control.corner_rearm_ticks >= XUNJI_CORNER_REARM_TICKS) {
            s_control.corner_armed = 1U;
            s_control.corner_rearm_ticks = 0U;
        }
    } else {
        s_control.corner_rearm_ticks = 0U;
    }
}

static void update_progress(void)
{
    int32_t side = s_control.status.estimated_side;

    if (s_h2_lap_mode != 0U) {
        if (s_control.status.state == XUNJI_STATE_FINISHED) {
            s_control.status.progress_permille = 1000U;
        } else {
            uint32_t elapsed = s_control.h2_elapsed_ticks;
            uint32_t requirement_ticks = 2000U; /* 赛题第二问要求20s内完成。 */
            if (elapsed >= requirement_ticks) elapsed = requirement_ticks - 1U;
            s_control.status.progress_permille =
                (uint16_t)((elapsed * 1000U) / requirement_ticks);
        }
        return;
    }

    if (s_control.status.state == XUNJI_STATE_FINISHED) {
        s_control.status.progress_permille = 1000U;
    } else if (side > 0) {
        int32_t lap_distance = s_control.status.total_distance -
                               s_control.lap_start_distance;
        int32_t lap_length = side * 4;
        int64_t scaled;

        if (lap_distance < 0) {
            lap_distance = 0;
        }
        scaled = ((int64_t)lap_distance * 1000) / lap_length;
        if (scaled > 999) {
            scaled = 999;
        }
        s_control.status.progress_permille = (uint16_t)scaled;
    } else {
        uint16_t coarse = (uint16_t)s_control.status.corners_in_lap * 250U;
        s_control.status.progress_permille = (coarse > 999U) ? 999U : coarse;
    }
}

static void finish_run(void)
{
    uint8_t straight_finish = s_straight_task_mode;

    s_control.status.state = XUNJI_STATE_FINISHED;
    s_control.status.progress_permille = 1000U;
    s_control.applied_left = 0;
    s_control.applied_right = 0;
    s_control.status.target_left = 0;
    s_control.status.target_right = 0;
    s_control.status.h2_finish_pending = 0U;
    s_control.status.h2_phase = XUNJI_H2_PHASE_TRACK;
    if (straight_finish != 0U) {
        s_control.status.straight_phase = XUNJI_STRAIGHT_DONE;
        s_control.status.timing_complete = 1U;
    } else {
        s_control.status.elapsed_ms = s_control.h2_elapsed_ticks * 10U;
    }
    xun_pwm_l = 0;
    xun_pwm_r = 0;
    last_pwm_l = 0;
    last_pwm_r = 0;
    jb = 0;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;
    /*
     * 正常到站已由目标速度和PWM斜坡降到零，这里只撤销驱动。
     * 不再追加短路制动，避免最后一次制动力跃变扰动钢珠。
     */
    Motor_Coast();
    PID_ResetAll();
}

static void finish_timed_run(void)
{
    s_control.status.state = XUNJI_STATE_FINISHED;
    s_control.status.progress_permille = 1000U;
    s_control.status.straight_phase = XUNJI_STRAIGHT_DONE;
    s_control.status.timing_complete = 1U;
    s_control.applied_left = 0;
    s_control.applied_right = 0;
    s_control.applied_correction = 0;
    s_control.status.target_left = 0;
    s_control.status.target_right = 0;
    xun_pwm_l = 0;
    xun_pwm_r = 0;
    last_pwm_l = 0;
    last_pwm_r = 0;
    jb = 0;
    s_direction_tune_mode = 0U;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;
    Motor_Stop();
    PID_ResetAll();
}

/**
 * @brief 任何可能导致持续大电流的异常都统一进入故障停车。
 *
 * 堵转/超速可能伴随电气异常，使用高阻滑行避免追加制动电流；纯状态机或
 * 传感器故障则短路制动，禁止小车靠惯性继续冲出赛道。
 */
static void fault_stop(XunjiFault fault)
{
    if ((s_straight_task_mode != 0U) &&
        (s_control.status.timing_complete == 0U)) {
        s_control.status.elapsed_ms =
            s_control.straight_elapsed_ticks * XUNJI_STRAIGHT_PERIOD_MS;
        s_control.status.timing_complete = 1U;
    }
    s_control.status.state = XUNJI_STATE_FAULT;
    s_control.status.fault = fault;
    s_control.applied_left = 0;
    s_control.applied_right = 0;
    s_control.status.target_left = 0;
    s_control.status.target_right = 0;
    xun_pwm_l = 0;
    xun_pwm_r = 0;
    jb = 0;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;
    if ((fault == XUNJI_FAULT_STALL_LEFT) ||
        (fault == XUNJI_FAULT_STALL_RIGHT) ||
        (fault == XUNJI_FAULT_OVERSPEED_LEFT) ||
        (fault == XUNJI_FAULT_OVERSPEED_RIGHT)) {
        /* 功率或反馈异常优先撤销驱动，避免短路制动继续产生大电流。 */
        Motor_Coast();
    } else {
        /* 丢线/状态机超时属于失控风险，仍必须快速制止车辆冲出赛道。 */
        Motor_Stop();
    }
    PID_ResetAll();
}

/**
 * @brief 记录一个已经完成并通过证据确认的物理拐角。
 * @param event_distance 首次进入全白时的里程，而不是出弯里程。
 *
 * 用丢线入口作为角点位置，可避免把转弯阶段的单轮路程误算进“起点到第一角”的
 * 距离。单圈第四次出弯后，还需前进“边长-本圈起点到第一角”
 * 才真正回到原点；从角点起步时该补偿为0。
 */
static void record_corner(int32_t event_distance)
{
    int32_t current_distance = s_control.status.total_distance;

    if (s_control.status.corners_in_lap == 0U) {
        s_control.first_corner_distance = event_distance;
    } else if ((s_control.status.completed_laps == 0U) &&
               (s_control.side_distance_samples < 3U)) {
        int32_t interval = event_distance - s_control.last_corner_distance;

        if (interval > 0) {
            s_control.side_distance_sum += interval;
            s_control.side_distance_samples++;
        }
    }

    s_control.last_corner_distance = event_distance;
    s_control.last_corner_exit_distance = current_distance;
    /* 锁住计角入口，必须进入下一条直线并满足空间条件后才能重新武装。 */
    s_control.corner_armed = 0U;
    s_control.corner_rearm_ticks = 0U;
    if (s_control.status.corners_in_lap < 4U) {
        s_control.status.corners_in_lap++;
    }

    if ((s_control.status.completed_laps == 0U) &&
        (s_control.side_distance_samples != 0U)) {
        s_control.status.estimated_side =
            s_control.side_distance_sum / s_control.side_distance_samples;
        if (s_control.side_distance_samples >= 3U) {
            s_learned_side_distance = s_control.status.estimated_side;
        }
    }

    if (s_control.status.corners_in_lap >= 4U) {
        int32_t start_to_first_corner = s_control.first_corner_distance -
                                        s_control.lap_start_distance;
        int32_t return_distance = s_control.status.estimated_side -
                                  start_to_first_corner;

        /* 起点到第一角走了d，第四角后回到起点只应再走side-d。
         * 若从角点起步，d约等于一条边，补走距离应为0；直接再走d会恰好多走一边。 */
        if (return_distance < 0) {
            return_distance = 0;
        }
        s_control.lap_finish_distance = current_distance + return_distance;
        s_control.waiting_lap_finish = 1U;
    }
}

/** 丢线恢复后用新观测重置位置低通，避免旧位置让方向环继续朝错误方向输出。 */
static void reseed_position_filter(const XunjiSensor *sensor)
{
    seed_position_filter(sensor->position);
}

static void begin_corner(int8_t direction,
                         int32_t event_distance,
                         uint8_t has_count_evidence,
                         uint8_t already_departed_center)
{
    /* 下一次10ms速度控制按当时目标、测速和上一拍PWM反算PI，避免突跳。 */
    s_control.velocity_reprime_pending = 1U;
    s_control.status.state = (direction < 0) ?
        XUNJI_STATE_CORNER_LEFT : XUNJI_STATE_CORNER_RIGHT;
    s_control.corner_ticks = 0U;
    s_control.center_ticks = 0U;
    s_control.corner_departed_center = already_departed_center;
    s_control.corner_event_distance = event_distance;
    s_control.turn_count_enabled =
        (uint8_t)((s_control.corner_armed != 0U) &&
                  (s_control.waiting_lap_finish == 0U) &&
                  (has_count_evidence != 0U));
    reset_position_control();
}

/** 由全白超时统一进入左右转状态；是否计角由丢线前的外侧证据单独决定。 */
static void enter_corner_from_lost(void)
{
    int8_t direction = s_control.lost_direction;

    if (direction == 0) {
        direction = s_control.last_error_direction;
    }
    if (direction == 0) {
        direction = -1; /* E题默认逆时针，仅在完全没有方向历史时使用。 */
    }

    begin_corner(direction,
                 s_control.lost_start_distance,
                 s_control.lost_has_corner_evidence,
                 1U); /* 已经连续全白160ms，按实车标定进入定向转弯。 */
}

/** 完成左右转共同的退出处理；恢复运动与是否计角分离，防止走偏恢复导致提前DONE。 */
static void finish_corner(void)
{
    if (s_control.turn_count_enabled != 0U) {
        record_corner(s_control.corner_event_distance);
    }
    s_control.status.state = XUNJI_STATE_TRACK;
    s_control.corner_guard_ticks = XUNJI_CORNER_GUARD_TICKS;
    s_control.recovery_ticks = XUNJI_RECOVERY_TICKS;
    s_control.center_ticks = 0U;
    s_control.corner_intent_ticks = 0U;
    s_control.corner_departed_center = 0U;
    s_control.line_missing_ticks = 0U;
    s_control.turn_count_enabled = 0U;
    s_control.lost_has_corner_evidence = 0U;
    reseed_position_filter(&s_control.status.sensor);
    /* 运球任务不允许前馈桥接；仅按当前实际PWM无扰接回无前馈PI。 */
    s_control.velocity_bridge_windows = 0U;
    s_control.velocity_reprime_pending = 1U;
    reset_position_control();
}

static void check_lap_finish(void)
{
    if ((s_control.waiting_lap_finish == 0U) ||
        (s_control.status.total_distance < s_control.lap_finish_distance)) {
        return;
    }

    s_control.waiting_lap_finish = 0U;
    s_control.status.completed_laps++;
    s_control.status.corners_in_lap = 0U;
    s_control.lap_start_distance = s_control.status.total_distance;

    if (s_control.status.completed_laps >= s_control.status.target_laps) {
        finish_run();
    }
}

#if 0
/* 已移除的旧正方形循迹实现留在条件编译外，仅便于历史对照。 */
/** 清除模式一当前角点的短时证据，不影响已完成角点数。 */
static void simple_reset_corner_evidence(void)
{
    s_control.simple_marker_ticks = 0U;
    s_control.simple_marker_age_ticks = 0U;
    s_control.simple_white_ticks = 0U;
    s_control.simple_advance_ticks = 0U;
    s_control.simple_direction = 0;
    s_control.corner_candidate_direction = 0;
}

/** 模式一完成一个物理角点：非第4n角回TRACK，第4n角立即急停。 */
static void simple_finish_corner(const XunjiSensor *sensor)
{
    if (s_control.status.corners_in_lap < 4U) {
        s_control.status.corners_in_lap++;
    }

    if (s_control.status.corners_in_lap >= 4U) {
        s_control.status.completed_laps++;
        if (s_control.status.completed_laps >= s_control.status.target_laps) {
            /* 用户定义的原点就是完成第4n个角后的新直线中心，禁止再补走一边。 */
            finish_run();
            return;
        }
        s_control.status.corners_in_lap = 0U;
    }

    s_control.status.state = XUNJI_STATE_TRACK;
    s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_REARM;
    s_control.status.simple_corner_armed = 0U;
    s_control.simple_rearm_ticks = 0U;
    /* 记录“完成回正”而不是“开始转弯”时的里程。之后的空间隔离只统计
     * 在新边上的前进量，不把转弯时的单轮路程算进去。 */
    s_control.last_corner_exit_distance =
        s_control.status.total_distance;
    s_control.center_ticks = 0U;
    s_control.corner_ticks = 0U;
    s_control.applied_correction = 0;
    s_control.velocity_bridge_windows = 0U;
    s_control.velocity_reprime_pending = 1U;
    simple_reset_corner_evidence();
    reseed_position_filter(sensor);
    reset_position_control();
    update_progress();
}

/** 模式一的10ms状态机。对外只产生 TRACK 与 TURN，角点证据在TRACK内完成。 */
static void simple_square_tick(const XunjiSensor *sensor)
{
    int8_t marker_direction;

    if (s_control.status.state == XUNJI_STATE_TRACK) {
        if (s_control.simple_initial_line_seen == 0U) {
            if (sensor->valid == 0U) {
                s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_WAIT_LINE;
                return;
            }
            s_control.simple_initial_line_seen = 1U;
            s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_TRACK;
        }

        /* 完成上一角后的重新使能必须同时满足：
         * 1) 编码器已经驶离旧角点；2) 三外侧标记已经释放；3) 新直线连续回中100ms。
         * 只用时间会让同一宽角在车身尚未离开时被反复计数。 */
        if (s_control.status.simple_corner_armed == 0U) {
            int32_t rearm_distance = s_control.status.total_distance -
                                     s_control.last_corner_exit_distance;
            marker_direction = outer_three_direction(sensor);
            if ((rearm_distance >= SIMPLE_REARM_MIN_COUNTS) &&
                (marker_direction == 0) &&
                (is_centered(sensor) != 0U)) {
                if (s_control.simple_rearm_ticks < 255U) {
                    s_control.simple_rearm_ticks++;
                }
                if (s_control.simple_rearm_ticks >=
                    SIMPLE_REARM_CENTER_TICKS) {
                    s_control.status.simple_corner_armed = 1U;
                    s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_TRACK;
                    s_control.simple_rearm_ticks = 0U;
                }
            } else {
                s_control.simple_rearm_ticks = 0U;
            }
            return;
        }

        /* 全白确认后的150ms直行是TRACK内的固定阶段，期间完全屏蔽新标记。 */
        if (s_control.status.simple_phase == XUNJI_SIMPLE_PHASE_ADVANCE) {
            if (s_control.simple_advance_ticks <
                SIMPLE_FORWARD_AFTER_WHITE_TICKS) {
                s_control.simple_advance_ticks++;
            }
            if (s_control.simple_advance_ticks >=
                SIMPLE_FORWARD_AFTER_WHITE_TICKS) {
                s_control.status.state = (s_control.simple_direction < 0) ?
                    XUNJI_STATE_CORNER_LEFT : XUNJI_STATE_CORNER_RIGHT;
                s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_TURN;
                s_control.status.simple_corner_armed = 0U;
                s_control.corner_ticks = 0U;
                s_control.center_ticks = 0U;
                s_control.applied_correction = 0;
                s_control.velocity_reprime_pending = 1U;
                reset_position_control();
            }
            return;
        }

        marker_direction = outer_three_direction(sensor);
        if (s_control.simple_direction == 0) {
            if (marker_direction != 0) {
                if (s_control.corner_candidate_direction != marker_direction) {
                    s_control.corner_candidate_direction = marker_direction;
                    s_control.simple_marker_ticks = 1U;
                } else if (s_control.simple_marker_ticks < 255U) {
                    s_control.simple_marker_ticks++;
                }
                if (s_control.simple_marker_ticks >=
                    SIMPLE_MARKER_CONFIRM_TICKS) {
                    s_control.simple_direction =
                        s_control.corner_candidate_direction;
                    s_control.simple_marker_age_ticks = 0U;
                    s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_MARKER;
                }
            } else {
                s_control.simple_marker_ticks = 0U;
                s_control.corner_candidate_direction = 0;
            }
            return;
        }

        /* 已锁存左/右标记，后续即使标记离开探头也不丢方向，只等待全白。 */
        if (s_control.simple_marker_age_ticks < 255U) {
            s_control.simple_marker_age_ticks++;
        }
        if (s_control.simple_marker_age_ticks >=
            SIMPLE_MARKER_TIMEOUT_TICKS) {
            simple_reset_corner_evidence();
            s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_TRACK;
            return;
        }

        if (sensor->valid == 0U) {
            s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_WHITE;
            if (s_control.simple_white_ticks < 255U) {
                s_control.simple_white_ticks++;
            }
            if (s_control.simple_white_ticks >= SIMPLE_WHITE_CONFIRM_TICKS) {
                s_control.simple_advance_ticks = 0U;
                s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_ADVANCE;
                reset_position_control();
            }
        } else {
            s_control.simple_white_ticks = 0U;
            s_control.status.simple_phase = XUNJI_SIMPLE_PHASE_MARKER;
        }
        return;
    }

    if ((s_control.status.state == XUNJI_STATE_CORNER_LEFT) ||
        (s_control.status.state == XUNJI_STATE_CORNER_RIGHT)) {
        if (s_control.corner_ticks < 255U) {
            s_control.corner_ticks++;
        }
        if (s_control.corner_ticks >= XUNJI_CORNER_TIMEOUT_TICKS) {
            fault_stop(XUNJI_FAULT_CORNER_TIMEOUT);
            return;
        }

        /* TURN中不再调用任何标记检测。只在最短转向保护后连续见到中心线，
         * 才认为车头已经对准下一条边。 */
        if ((s_control.corner_ticks >= SIMPLE_TURN_MIN_TICKS) &&
            (is_centered(sensor) != 0U)) {
            if (s_control.center_ticks < 255U) {
                s_control.center_ticks++;
            }
            if (s_control.center_ticks >= SIMPLE_CENTER_CONFIRM_TICKS) {
                simple_finish_corner(sensor);
            }
        } else {
            s_control.center_ticks = 0U;
        }
    }
}
#endif

static uint8_t h2_is_finish_marker(const XunjiSensor *sensor)
{
    return (uint8_t)(sensor->active_count >= H2_MARKER_MIN_ACTIVE);
}

static void h2_finish_marker(void)
{
    s_control.h2_finish_pending = 1U;
    s_control.status.h2_finish_pending = 1U;
    s_control.status.h2_phase = XUNJI_H2_PHASE_STOP_OFFSET;
    s_control.h2_finish_stop_distance =
        s_control.status.total_distance + H2_A_STOP_OFFSET_COUNTS;
    s_control.h2_finish_decelerating =
        (H2_A_STOP_OFFSET_COUNTS == 0) ? 1U : 0U;
    s_control.h2_stop_settle_ticks = 0U;
    s_control.applied_correction = 0;
    reset_position_control();
}

/**
 * 2026 H题第二问状态机：赛道为连续黑线，不再把全白当作90度拐角。
 * A点横线先经历“起点释放”，完整运行后再次检测到宽横线才算终点。
 */
static void h2_lap_tick(const XunjiSensor *sensor)
{
    uint8_t marker = h2_is_finish_marker(sensor);

    if (s_control.h2_elapsed_ticks < UINT32_MAX) {
        s_control.h2_elapsed_ticks++;
    }
    s_control.status.elapsed_ms = s_control.h2_elapsed_ticks * 10U;

    if ((s_mode1_test_mode == 0U) &&
        (s_control.h2_elapsed_ticks >= H2_ROUTE_TIMEOUT_TICKS)) {
        fault_stop(XUNJI_FAULT_ROUTE_TIMEOUT);
        return;
    }

    if (s_control.h2_finish_pending != 0U) {
        if ((s_control.h2_finish_decelerating == 0U) &&
            (s_control.status.total_distance >=
             s_control.h2_finish_stop_distance)) {
            s_control.h2_finish_decelerating = 1U;
            s_control.h2_stop_settle_ticks = 0U;
        }
        return;
    }

    /* 起点可能恰好压在横线上，也可能位于横线后方。只有连续看到普通窄线，
     * 才认为已经离开A点并允许终点识别。 */
    if (s_control.h2_finish_armed == 0U) {
        s_control.status.state = XUNJI_STATE_TRACK;
        s_control.status.h2_finish_armed = 0U;
        s_control.h2_finish_marker_ticks = 0U;
        s_control.status.h2_marker_ticks = 0U;

        if ((sensor->valid != 0U) && (marker == 0U) &&
            (sensor->active_count <= 4U)) {
            if (s_control.h2_start_release_ticks < 255U) {
                s_control.h2_start_release_ticks++;
            }
            s_control.status.h2_phase = XUNJI_H2_PHASE_TRACK;
            if (s_control.h2_start_release_ticks >=
                H2_START_RELEASE_TICKS) {
                s_control.h2_finish_armed = 1U;
                s_control.status.h2_finish_armed = 1U;
                reseed_position_filter(sensor);
                reset_position_control();
                s_control.applied_correction = 0;
            }
        } else {
            s_control.h2_start_release_ticks = 0U;
            s_control.status.h2_phase =
                (sensor->valid == 0U) ? XUNJI_H2_PHASE_WAIT_LINE :
                                       XUNJI_H2_PHASE_START_MARKER;
        }
        return;
    }

    if (sensor->valid != 0U) {
        if (s_control.status.state == XUNJI_STATE_LOST) {
            reseed_position_filter(sensor);
            reset_position_control();
            s_control.velocity_reprime_pending = 1U;
        }
        s_control.status.state = XUNJI_STATE_TRACK;
        s_control.line_missing_ticks = 0U;
        s_control.h2_lost_ticks = 0U;
        s_control.status.lost_ticks = 0U;

        if ((s_mode1_test_mode == 0U) &&
            (marker != 0U) &&
            (s_control.h2_elapsed_ticks >= H2_FINISH_MIN_RUNTIME_TICKS)) {
            if (s_control.h2_finish_marker_ticks < 255U) {
                s_control.h2_finish_marker_ticks++;
            }
            s_control.status.h2_marker_ticks =
                s_control.h2_finish_marker_ticks;
            s_control.status.h2_phase = XUNJI_H2_PHASE_FINISH_CONFIRM;
            if (s_control.h2_finish_marker_ticks >=
                H2_FINISH_CONFIRM_TICKS) {
                h2_finish_marker();
            }
        } else {
            s_control.h2_finish_marker_ticks = 0U;
            s_control.status.h2_marker_ticks = 0U;
            s_control.status.h2_phase = XUNJI_H2_PHASE_TRACK;
        }
        return;
    }

    s_control.h2_finish_marker_ticks = 0U;
    s_control.status.h2_marker_ticks = 0U;
    if (s_control.line_missing_ticks < 255U) {
        s_control.line_missing_ticks++;
    }
    s_control.status.line_missing_ticks = s_control.line_missing_ticks;

    if (s_control.line_missing_ticks < H2_LOST_ENTER_TICKS) {
        /* 20ms以内的灰度闪断继续沿用上一拍位置，不重置速度PI。 */
        return;
    }

    if (s_control.status.state != XUNJI_STATE_LOST) {
        s_control.status.state = XUNJI_STATE_LOST;
        s_control.h2_lost_ticks = 0U;
        s_control.h2_search_direction = s_control.last_error_direction;
        reset_position_control();
    }
    if (s_control.h2_lost_ticks < 255U) {
        s_control.h2_lost_ticks++;
    }
    s_control.status.lost_ticks = s_control.h2_lost_ticks;
    s_control.status.search_direction = s_control.h2_search_direction;
    s_control.status.h2_phase = XUNJI_H2_PHASE_LOST;

    if (s_control.h2_lost_ticks >= H2_LOST_TIMEOUT_TICKS) {
        fault_stop(XUNJI_FAULT_LINE_LOST_TIMEOUT);
    }
}

void Xunji_ControlInit(void)
{
    memset(&s_control, 0, sizeof(s_control));
    s_control.status.state = XUNJI_STATE_IDLE;
    s_control.status.target_laps = 1U;
    s_control.status.estimated_side = s_learned_side_distance;
    s_control.last_position = XUNJI_CENTER_POSITION;
    /*
     * 未获得明确偏差信息前不预设搜索方向。中心探头会在400/500间量化跳变，
     * 若默认或随单拍跳变选边，首次丢线会随机向某一侧猛转。
     */
    s_control.last_error_direction = 0;
    s_control.lost_direction = 0;
    s_control.corner_armed = 1U;         /* 起跑后允许识别第一个真实角点。 */
    s_control.filtered_position_x4 = (int32_t)XUNJI_CENTER_POSITION * 4;
    s_control.status.control_position = XUNJI_CENTER_POSITION;
    xun = (float)XUNJI_CENTER_POSITION;
    xun_pwm_l = 0;
    xun_pwm_r = 0;
}

void Xunji_StartDirectionTune(void)
{
    Motor_Stop();
    Encoder_Clear();
    PID_ResetAll();
    s_h2_lap_mode = 0U;
    s_direction_tune_mode = 1U;
    s_mode1_test_mode = 0U;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;
    Xunji_ControlInit();
    xunji_shua_xin();
    s_control.recovery_ticks = XUNJI_RECOVERY_TICKS;
    s_control.velocity_reprime_pending = 1U;
    /*
     * 即使启动瞬间恰好采到单拍全白，也先按TRACK保持30ms，不在第一拍使用
     * 默认方向猛转。确认丢线后才锁存最后一次真实见线方向。
     */
    s_control.status.state = XUNJI_STATE_TRACK;
    if (s_control.status.sensor.valid == 0U) {
        s_control.last_error_direction = 0;
        s_control.lost_direction = 0;
    }
    jb = 2;
}

void Xunji_StartStraightTask(int32_t distance_target,
                             uint32_t post_cruise_ms)
{
    Xunji_StartDirectionTune();
    s_straight_task_mode = 1U;
    s_control.status.straight_phase = XUNJI_STRAIGHT_RUN;
    s_control.status.timing_complete = 0U;
    s_control.status.elapsed_ms = 0U;
    s_control.status.task_distance_target =
        (distance_target > 0) ? distance_target :
        XUNJI_STRAIGHT_DISTANCE_COUNTS;
    s_control.straight_post_target_ticks =
        XUNJI_STRAIGHT_MS_TO_TICKS(post_cruise_ms);
}

void Xunji_StartTimedTask(uint32_t run_time_ms)
{
    if (run_time_ms < 100U) run_time_ms = 100U;
    Xunji_StartStraightTask(XUNJI_STRAIGHT_DISTANCE_COUNTS, 0U);
    s_timed_task_mode = 1U;
    s_control.status.task_distance_target = 0;
    s_control.straight_time_target_ticks =
        XUNJI_STRAIGHT_MS_TO_TICKS(run_time_ms);
}

void Xunji_SetTimedLineWeightX10(int weight_x10)
{
    if (weight_x10 < XUNJI_TIMED_WEIGHT_MIN_X10) {
        weight_x10 = XUNJI_TIMED_WEIGHT_MIN_X10;
    } else if (weight_x10 > XUNJI_TIMED_WEIGHT_MAX_X10) {
        weight_x10 = XUNJI_TIMED_WEIGHT_MAX_X10;
    }
    s_timed_line_weight_x10 = (int16_t)weight_x10;
}

int Xunji_GetTimedLineWeightX10(void)
{
    return (int)s_timed_line_weight_x10;
}

void Xunji_StartMode1Test(void)
{
    Motor_Stop();
    Encoder_Clear();
    PID_ResetAll();
    s_direction_tune_mode = 0U;
    s_h2_lap_mode = 1U;
    s_mode1_test_mode = 1U;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;
    Xunji_ControlInit();

    /*
     * 当前阶段只测试高速循迹稳定性和人工计时：
     * 起点释放逻辑仍负责确认真正进入窄线，终点横线不触发自动停车。
     */
    s_control.status.target_laps = 0U;
    s_control.status.completed_laps = 0U;
    s_control.status.h2_phase = XUNJI_H2_PHASE_WAIT_LINE;
    s_control.status.h2_finish_armed = 0U;
    s_control.status.h2_finish_pending = 0U;
    s_control.status.h2_marker_ticks = 0U;
    s_control.status.elapsed_ms = 0U;
    xunji_shua_xin();
    s_control.status.state = XUNJI_STATE_TRACK;
    s_control.recovery_ticks = 0U;
    s_control.velocity_reprime_pending = 1U;
    s_control.velocity_bridge_windows = 0U;
    last_pwm_l = 0;
    last_pwm_r = 0;
    jb = 2;
}

void Xunji_StartH2Lap(void)
{
    Motor_Stop();
    Encoder_Clear();
    PID_ResetAll();
    s_direction_tune_mode = 0U;
    s_h2_lap_mode = 1U;
    s_mode1_test_mode = 0U;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;
    Xunji_ControlInit();

    s_control.status.target_laps = 1U;
    s_control.status.completed_laps = 0U;
    s_control.status.h2_phase = XUNJI_H2_PHASE_WAIT_LINE;
    s_control.status.h2_finish_armed = 0U;
    s_control.status.h2_finish_pending = 0U;
    s_control.status.h2_marker_ticks = 0U;
    s_control.status.elapsed_ms = 0U;
    xunji_shua_xin();
    s_control.status.state = XUNJI_STATE_TRACK;
    s_control.recovery_ticks = XUNJI_RECOVERY_TICKS;
    s_control.velocity_reprime_pending = 1U;
    s_control.velocity_bridge_windows = 0U;
    last_pwm_l = 0;
    last_pwm_r = 0;
    jb = 2;
}

void Xunji_Stop(void)
{
    if ((s_straight_task_mode != 0U) &&
        (s_control.status.timing_complete == 0U)) {
        s_control.status.elapsed_ms =
            s_control.straight_elapsed_ticks * XUNJI_STRAIGHT_PERIOD_MS;
        s_control.status.timing_complete = 1U;
    }
    if (s_straight_task_mode != 0U) {
        s_control.status.straight_phase = XUNJI_STRAIGHT_DONE;
    }
    jb = 0;
    Motor_Stop();
    Encoder_Clear();
    PID_ResetAll();
    s_direction_tune_mode = 0U;
    s_h2_lap_mode = 0U;
    s_mode1_test_mode = 0U;
    s_straight_task_mode = 0U;
    s_timed_task_mode = 0U;

    s_control.status.state = XUNJI_STATE_IDLE;
    s_control.status.fault = XUNJI_FAULT_NONE;
    s_control.applied_left = 0;
    s_control.applied_right = 0;
    s_control.status.target_left = 0;
    s_control.status.target_right = 0;
    s_control.status.h2_phase = XUNJI_H2_PHASE_WAIT_LINE;
    s_control.status.h2_finish_armed = 0U;
    s_control.status.h2_finish_pending = 0U;
    s_control.status.h2_marker_ticks = 0U;
    xun_pwm_l = 0;
    xun_pwm_r = 0;
}

void Xunji_Tick10ms(void)
{
    XunjiSensor *sensor;

    if ((jb != 2) ||
        (s_control.status.state == XUNJI_STATE_IDLE) ||
        (s_control.status.state == XUNJI_STATE_FINISHED) ||
        (s_control.status.state == XUNJI_STATE_FAULT)) {
        return;
    }

    if (s_straight_task_mode != 0U) {
        if (s_control.status.timing_complete == 0U) {
            if (s_control.straight_elapsed_ticks < UINT32_MAX) {
                s_control.straight_elapsed_ticks++;
            }
            s_control.status.elapsed_ms =
                s_control.straight_elapsed_ticks *
                XUNJI_STRAIGHT_PERIOD_MS;
        }

        if (s_control.status.straight_phase == XUNJI_STRAIGHT_RUN) {
            if (s_control.straight_run_ticks < UINT32_MAX) {
                s_control.straight_run_ticks++;
            }
            if ((s_timed_task_mode != 0U) &&
                (s_control.straight_elapsed_ticks >=
                 s_control.straight_time_target_ticks)) {
                finish_timed_run();
                return;
            }
            if ((s_timed_task_mode == 0U) &&
                (s_control.straight_elapsed_ticks >=
                XUNJI_STRAIGHT_MS_TO_TICKS(
                    XUNJI_STRAIGHT_ROUTE_TIMEOUT_MS))) {
                fault_stop(XUNJI_FAULT_ROUTE_TIMEOUT);
                return;
            }
        } else if (s_control.status.straight_phase ==
                   XUNJI_STRAIGHT_POST_CRUISE) {
            if (s_control.straight_post_ticks < UINT32_MAX) {
                s_control.straight_post_ticks++;
            }
            if (s_control.straight_post_ticks >=
                s_control.straight_post_target_ticks) {
                s_control.status.straight_phase =
                    XUNJI_STRAIGHT_DECEL;
                s_control.straight_decel_ticks = 0U;
            }
        } else if (s_control.status.straight_phase ==
                   XUNJI_STRAIGHT_DECEL) {
            uint32_t decel_target =
                XUNJI_STRAIGHT_MS_TO_TICKS(XUNJI_STRAIGHT_DECEL_MS);
            if (s_control.straight_decel_ticks < decel_target) {
                s_control.straight_decel_ticks++;
            }
        }
    }

    xunji_shua_xin();
    sensor = &s_control.status.sensor;

    /*
     * 模式五是连续位置环调试，不参与角点、圈数或终点识别。
     * 单拍全白保持上一轮速；连续30ms确认丢线后，锁存最后见线方向并持续
     * 搜线。丢线本身永不置FAULT，真正的堵转/超速仍由速度执行层停车。
     */
    if (s_direction_tune_mode != 0U) {
        if (sensor->valid != 0U) {
            uint8_t left_outer =
                sensor->bits & (XUNJI_BIT_L4 | XUNJI_BIT_L3);
            uint8_t right_outer =
                sensor->bits & (XUNJI_BIT_R3 | XUNJI_BIT_R4);

            if ((left_outer != 0U) && (right_outer == 0U)) {
                s_control.last_error_direction = -1;
            } else if ((right_outer != 0U) && (left_outer == 0U)) {
                s_control.last_error_direction = 1;
            } else if (abs_int(sensor->error) >=
                       XUNJI_DIRECTION_MEMORY_ERROR) {
                s_control.last_error_direction =
                    (sensor->error < 0) ? -1 : 1;
            }

            if (s_control.status.state == XUNJI_STATE_LOST) {
                /*
                 * 找回线时立即用真实位置重装位置滤波器；若继续沿用丢线前的
                 * 滤波历史，车辆会多走数拍才开始向新线纠正。
                 */
                reseed_position_filter(sensor);
                reset_position_control();
            }
            s_control.status.state = XUNJI_STATE_TRACK;
            s_control.line_missing_ticks = 0U;
            s_control.lost_ticks = 0U;
            s_control.lost_direction = s_control.last_error_direction;
        } else {
            if (s_control.line_missing_ticks < 255U) {
                s_control.line_missing_ticks++;
            }
            if (s_control.line_missing_ticks >=
                XUNJI_TUNE_LOST_ENTER_TICKS) {
                if (s_control.status.state != XUNJI_STATE_LOST) {
                    s_control.lost_direction =
                        s_control.last_error_direction;
                    s_control.lost_ticks = 0U;
                    reset_position_control();
                }
                s_control.status.state = XUNJI_STATE_LOST;
                if (s_control.lost_ticks < 255U) {
                    s_control.lost_ticks++;
                }
            }
        }

        s_control.status.fault = XUNJI_FAULT_NONE;
        s_control.corner_candidate_ticks = 0U;
        s_control.corner_intent_ticks = 0U;
        update_progress();
        return;
    }

    /* 正常1.8cm黑线一般只覆盖少量通道。7/8路持续触发通常意味着极性、
     * 接线或传感器供电异常；继续运行会把错误图样识别为永久弯道。 */
    if (sensor->active_count == 8U) {
        if (s_control.sensor_stuck_ticks < 255U) {
            s_control.sensor_stuck_ticks++;
        }
        if (s_control.sensor_stuck_ticks >= XUNJI_SENSOR_STUCK_TICKS) {
            fault_stop(XUNJI_FAULT_SENSOR_PATTERN);
            return;
        }
    } else {
        s_control.sensor_stuck_ticks = 0U;
    }

    if (s_control.corner_guard_ticks != 0U) {
        s_control.corner_guard_ticks--;
    }
    if (s_control.recovery_ticks != 0U) {
        s_control.recovery_ticks--;
    }
    if (s_control.corner_intent_ticks != 0U) {
        s_control.corner_intent_ticks--;
    }

    if (sensor->valid != 0U) {
        uint8_t left_outer = sensor->bits & (XUNJI_BIT_L4 | XUNJI_BIT_L3);
        uint8_t right_outer = sensor->bits & (XUNJI_BIT_R3 | XUNJI_BIT_R4);

        /* 外侧探头比加权误差更能表示“最后从哪侧离线”，优先锁存搜索方向。 */
        if ((left_outer != 0U) && (right_outer == 0U)) {
            s_control.last_error_direction = -1;
        } else if ((right_outer != 0U) && (left_outer == 0U)) {
            s_control.last_error_direction = 1;
        } else if (abs_int(sensor->error) >=
                   XUNJI_DIRECTION_MEMORY_ERROR) {
            s_control.last_error_direction = (sensor->error < 0) ? -1 : 1;
        }
    }

    if (s_h2_lap_mode != 0U) {
        h2_lap_tick(sensor);
        update_progress();
        return;
    }

    switch (s_control.status.state) {
        case XUNJI_STATE_TRACK:
        {
            int8_t approach_direction;
            int8_t marker_direction;

            update_corner_rearm(sensor);
            if (sensor->valid == 0U) {
                /* 数字灰度在车体振动、线边缘和电机干扰下可能偶发单拍全白。
                 * 先保持上一拍位置与轮速目标，连续确认后才真正进入LOST。 */
                if (s_control.line_missing_ticks == 0U) {
                    s_control.lost_start_distance =
                        s_control.status.total_distance;
                }
                if (s_control.line_missing_ticks < 255U) {
                    s_control.line_missing_ticks++;
                }
                if (s_control.line_missing_ticks <
                    XUNJI_LOST_ENTER_CONFIRM_TICKS) {
                    break;
                }

                s_control.status.state = XUNJI_STATE_LOST;
                /* lost_ticks从第一次全白开始计，累计160ms才判定为真实拐角。 */
                s_control.lost_ticks = s_control.line_missing_ticks;
                s_control.lost_direction = s_control.last_error_direction;
                s_control.lost_has_corner_evidence =
                    (uint8_t)((s_control.corner_intent_ticks != 0U) &&
                              (s_control.corner_intent_direction ==
                               s_control.lost_direction));
                s_control.corner_candidate_ticks = 0U;
                s_control.corner_candidate_direction = 0;
                break;
            }

            s_control.line_missing_ticks = 0U;

            approach_direction = corner_approach_direction(sensor);
            if (approach_direction != 0) {
                if (s_control.corner_candidate_direction != approach_direction) {
                    s_control.corner_candidate_ticks = 1U;
                    s_control.corner_candidate_direction = approach_direction;
                } else if (s_control.corner_candidate_ticks < 255U) {
                    s_control.corner_candidate_ticks++;
                }

                /* 20ms同向外侧证据保留150ms；只决定能否计角，不直接切入转弯。 */
                if (s_control.corner_candidate_ticks >=
                    XUNJI_CORNER_INTENT_CONFIRM) {
                    s_control.corner_intent_ticks = XUNJI_CORNER_INTENT_TICKS;
                    s_control.corner_intent_direction = approach_direction;
                }

                marker_direction = outer_three_direction(sensor);
                if ((s_control.corner_candidate_ticks >=
                     XUNJI_MARKER_CONFIRM_TICKS) &&
                    (marker_direction == approach_direction)) {
                    /* 宽角点先按编码器里程越过角点前沿，再开始单轮转向。
                     * 这样不会把角点入口附近仍可见的旧直线误当作出弯新线。 */
                    s_control.status.state = XUNJI_STATE_CORNER_ADVANCE;
                    s_control.velocity_reprime_pending = 1U;
                    s_control.advance_direction = marker_direction;
                    s_control.advance_distance_started = 0U;
                    s_control.advance_start_distance =
                        s_control.status.total_distance;
                    s_control.advance_event_distance =
                        s_control.status.total_distance;
                    s_control.corner_candidate_ticks = 0U;
                    s_control.corner_candidate_direction = 0;
                    reset_position_control();
                    break;
                }
            } else {
                s_control.corner_candidate_ticks = 0U;
                s_control.corner_candidate_direction = 0;
            }
            break;
        }

        case XUNJI_STATE_CORNER_ADVANCE:
            if ((s_control.advance_distance_started != 0U) &&
                ((s_control.status.total_distance -
                 s_control.advance_start_distance) >=
                 XUNJI_MARKER_ADVANCE_COUNTS)) {
                begin_corner(s_control.advance_direction,
                             s_control.advance_event_distance,
                             1U,
                             0U);
            }
            break;

        case XUNJI_STATE_CORNER_LEFT:
        case XUNJI_STATE_CORNER_RIGHT:
            if (s_control.corner_ticks < 255U) {
                s_control.corner_ticks++;
            }

            if (s_control.corner_ticks >= XUNJI_CORNER_TIMEOUT_TICKS) {
                fault_stop(XUNJI_FAULT_CORNER_TIMEOUT);
                return;
            }

            if ((s_control.corner_ticks >= XUNJI_CORNER_DEPART_TICKS) &&
                (is_centered(sensor) == 0U)) {
                s_control.corner_departed_center = 1U;
            }

            if ((s_control.corner_ticks >= XUNJI_CORNER_MIN_TICKS) &&
                (s_control.corner_departed_center != 0U) &&
                (is_centered(sensor) != 0U)) {
                if (s_control.center_ticks < 255U) {
                    s_control.center_ticks++;
                }
                if (s_control.center_ticks >= XUNJI_CENTER_CONFIRM_TICKS) {
                    finish_corner();
                }
            } else {
                s_control.center_ticks = 0U;
            }
            break;

        case XUNJI_STATE_LOST:
            if (s_control.lost_ticks < 255U) {
                s_control.lost_ticks++;
            }

            if (sensor->valid != 0U) {
                /* 160ms内找回线属于普通走偏。LOST全程不重置速度PI，
                 * 因此速度PI必须连续运行，禁止每次灰度闪断都重新装载。 */
                s_control.status.state = XUNJI_STATE_TRACK;
                s_control.recovery_ticks = XUNJI_SHORT_RECOVERY_TICKS;
                s_control.line_missing_ticks = 0U;
                s_control.lost_ticks = 0U;
                s_control.corner_candidate_ticks = 0U;
                s_control.corner_candidate_direction = 0;
                s_control.corner_intent_ticks = 0U;
                s_control.lost_has_corner_evidence = 0U;
                reseed_position_filter(sensor);
                reset_position_control();
            } else if (s_control.lost_ticks >= XUNJI_LOST_TO_CORNER_TICKS) {
                /* 全白达到阈值后，按丢线瞬间锁存的方向进入左/右转向恢复。 */
                enter_corner_from_lost();
            } else if (s_control.lost_ticks >= XUNJI_LOST_STOP_TICKS) {
                /* 正常配置不会到达；保留作为以后改阈值时的最终盲走保护。 */
                fault_stop(XUNJI_FAULT_LINE_LOST_TIMEOUT);
                return;
            }
            break;

        default:
            break;
    }

    update_progress();
}

void Xunji_UpdateEncoder(int left_delta, int right_delta)
{
    int32_t left_distance;
    int32_t right_distance;

    if (jb != 2) {
        return;
    }

    left_distance = (left_delta < 0) ? -(int32_t)left_delta : (int32_t)left_delta;
    right_distance = (right_delta < 0) ? -(int32_t)right_delta : (int32_t)right_delta;
    s_control.status.total_distance += (left_distance + right_distance) / 2;

    if ((s_straight_task_mode != 0U) &&
        (s_control.status.straight_phase == XUNJI_STRAIGHT_RUN) &&
        (s_control.status.task_distance_target > 0) &&
        (s_control.status.total_distance >=
         s_control.status.task_distance_target)) {
        s_control.status.timing_complete = 1U;
        s_control.status.elapsed_ms =
            s_control.straight_elapsed_ticks * XUNJI_STRAIGHT_PERIOD_MS;
        s_control.straight_post_ticks = 0U;
        s_control.straight_decel_ticks = 0U;
        s_control.status.straight_phase =
            (s_control.straight_post_target_ticks != 0U) ?
            XUNJI_STRAIGHT_POST_CRUISE : XUNJI_STRAIGHT_DECEL;
    }

    /* 状态切换可能发生在10ms测速窗中间。舍弃首个混合窗口，保证前探
     * 里程不包含进入FWD之前的直线距离，从而不会随机少走0~10ms。 */
    if ((s_control.status.state == XUNJI_STATE_CORNER_ADVANCE) &&
        (s_control.advance_distance_started == 0U)) {
        s_control.advance_start_distance = s_control.status.total_distance;
        s_control.advance_distance_started = 1U;
    }

    /* 模式一当前只人工计时，彻底隔离旧圈数完成入口。 */
    if (s_mode1_test_mode == 0U) {
        check_lap_finish();
    }
    update_progress();
}

/**
 * @brief 检查单个车轮是否堵转或速度反馈异常。
 *
 * 本工程没有电流传感器，只能用“目标速度 + 实测速度 + PWM”联合判断。
 * 高PWM连续200ms仍低于目标1/4时立即停车；实测速度连续150ms超过
 * 目标两倍并带安全余量时也停车，兼顾编码器毛刺和控制失稳。
 */
static XunjiFault check_wheel_safety(int target,
                                     int measured,
                                     int pwm,
                                     int reference_speed,
                                     uint8_t *stall_windows,
                                     uint8_t *overspeed_windows,
                                     XunjiFault stall_fault,
                                     XunjiFault overspeed_fault)
{
    int target_abs = abs_int(target);
    int measured_abs = abs_int(measured);
    int pwm_abs = abs_int(pwm);
    int active_target = reference_speed / 4;
    int low_speed_threshold = target_abs / 4;
    int stall_pwm = (MOTOR_PWM_SAFE_LIMIT * 4) / 5;

    if (active_target < 15) {
        active_target = 15;
    }
    if (low_speed_threshold < 3) {
        low_speed_threshold = 3;
    }

    if ((target_abs >= active_target) &&
        (measured_abs < low_speed_threshold) &&
        (pwm_abs >= stall_pwm)) {
        if (*stall_windows < 255U) {
            (*stall_windows)++;
        }
    } else {
        *stall_windows = 0U;
    }

    if (*stall_windows >= XUNJI_STALL_CONFIRM_WINDOWS) {
        return stall_fault;
    }

    if ((target_abs >= active_target) &&
        (measured_abs > (target_abs * 2 + 20))) {
        if (*overspeed_windows < 255U) {
            (*overspeed_windows)++;
        }
    } else {
        *overspeed_windows = 0U;
    }

    if (*overspeed_windows >= XUNJI_OVERSPEED_CONFIRM_WINDOWS) {
        return overspeed_fault;
    }

    return XUNJI_FAULT_NONE;
}

void Xunji_SpeedControl(int measured_left, int measured_right)
{
    int base_speed;
    int control_speed;
    int correction_percent;
    int correction;
    int correction_limit;
    int target_left;
    int target_right;
    int target_limit;
    int rise_step;
    int fall_step;
    int pwm_left;
    int pwm_right;
    int requested_pwm_left;
    int requested_pwm_right;
    int position_output_limit;
    int previous_target_left = s_control.applied_left;
    int previous_target_right = s_control.applied_right;
    XunjiFault safety_fault;
    int error_abs = abs_int(s_control.status.control_position -
                            XUNJI_CENTER_POSITION);

    if ((jb != 2) ||
        (s_control.status.state == XUNJI_STATE_IDLE) ||
        (s_control.status.state == XUNJI_STATE_FINISHED) ||
        (s_control.status.state == XUNJI_STATE_FAULT)) {
        Motor_Stop();
        return;
    }

    control_speed = (s_mode1_test_mode != 0U) ?
                    XUNJI_MODE1_BASE_SPEED :
                    (s_timed_task_mode != 0U) ?
                    XUNJI_TIMED_TASK_BASE_SPEED : (int)purpose;
    if (control_speed < 0) {
        control_speed = 0;
    }
    correction_percent = (s_mode1_test_mode != 0U) ?
                         XUNJI_MODE1_CORRECTION_PERCENT :
                         XUNJI_TRACK_CORRECTION_PERCENT;
    base_speed = control_speed;
    if (base_speed < 0) {
        base_speed = 0;
    }
    base_speed = straight_profile_speed(base_speed);
    position_output_limit =
        (base_speed * correction_percent) / 100;
    if ((base_speed > 0) && (position_output_limit < 1)) {
        position_output_limit = 1;
    }

    /*
     * 非线性位置PI固定20ms更新；轮速差阻尼每10ms使用最新编码器速度。
     * 单拍全白仍保持上一修正，真正进入LOST时状态机会统一复位。
     */
    s_control.position_pid_divider++;
    if (s_control.position_pid_divider >= XUNJI_POSITION_PID_DIVIDER) {
        s_control.position_pid_divider = 0U;
        if ((s_control.status.state == XUNJI_STATE_TRACK) &&
            (s_control.status.sensor.valid != 0U) &&
            ((s_h2_lap_mode == 0U) ||
             (s_control.status.h2_phase == XUNJI_H2_PHASE_TRACK))) {
            float position_output = (s_mode1_test_mode != 0U) ?
                PID_Mode1PlaceLimited(
                    (float)XUNJI_CENTER_POSITION,
                    (float)s_control.status.control_position,
                    (float)position_output_limit) :
                place_PID_value_limited(
                    (float)XUNJI_CENTER_POSITION,
                    (float)s_control.status.control_position,
                    (float)position_output_limit);
            s_control.position_pid_correction =
                (int16_t)round_float_to_int(position_output);
        } else if ((s_control.status.sensor.valid != 0U) ||
                   (s_control.status.state != XUNJI_STATE_TRACK)) {
            /*
             * 无有效位置量或离开TRACK时，同时清除通用与模式一位置环。
             * 否则模式一后续启用Ki后，重新见线会继承丢线前积分并产生瞬时打舵。
             */
            reset_position_control();
        }
    }
    if (s_mode1_test_mode != 0U) {
        correction = round_float_to_int(PID_Mode1ApplyPlaceRateDamping(
            (float)s_control.position_pid_correction,
            (float)measured_left,
            (float)measured_right,
            (float)position_output_limit));
    } else {
        correction = round_float_to_int(PID_ApplyPlaceRateDamping(
            (float)s_control.position_pid_correction,
            (float)measured_left,
            (float)measured_right,
            (float)position_output_limit));
    }
    if (s_timed_task_mode != 0U) {
        int weighted_x10 = correction *
                           (int)s_timed_line_weight_x10;
        correction = (weighted_x10 >= 0) ?
            (weighted_x10 + 5) / 10 :
            -((-weighted_x10 + 5) / 10);
        correction = clamp_int(correction,
                               -position_output_limit,
                               position_output_limit);
    }
    if (s_control.status.state == XUNJI_STATE_TRACK) {
        if ((s_h2_lap_mode != 0U) &&
            (s_control.status.h2_phase == XUNJI_H2_PHASE_WAIT_LINE)) {
            /* 起步前尚未看到任何黑线时保持双轮短路制动，禁止默认方向盲走。 */
            base_speed = 0;
            correction = 0;
            s_control.applied_correction = 0;
        } else if ((s_h2_lap_mode != 0U) &&
                   (s_control.h2_finish_pending != 0U) &&
                   (s_control.h2_finish_decelerating != 0U)) {
            /* 正常到站仅通过斜坡把双轮目标降到零，不触发短路急停。 */
            base_speed = 0;
            correction = 0;
            s_control.applied_correction = 0;
        } else if ((s_h2_lap_mode != 0U) &&
                   (s_control.status.h2_phase ==
                    XUNJI_H2_PHASE_START_MARKER) &&
                   (s_control.h2_finish_armed == 0U)) {
            /* 从A点横线起跑时必须主动驶离横线；保持等速低速，禁止把横线
             * 加权重心误当作位置误差。 */
            base_speed = (base_speed * H2_FINISH_APPROACH_PERCENT) / 100;
            correction = 0;
            s_control.applied_correction = 0;
        } else {
            int speed_percent = (s_h2_lap_mode != 0U) ? 100 :
                100 - clamp_int((error_abs * 35) / 350, 0, 35);

            if ((s_h2_lap_mode != 0U) &&
                (s_control.h2_finish_pending != 0U)) {
                speed_percent = H2_FINISH_APPROACH_PERCENT;
            }
            if ((s_h2_lap_mode == 0U) &&
                (s_control.recovery_ticks != 0U)) {
                int recovered_ticks = XUNJI_RECOVERY_TICKS -
                                      s_control.recovery_ticks;
                int recovery_percent = XUNJI_TRACK_MIN_PERCENT +
                    (recovered_ticks * (100 - XUNJI_TRACK_MIN_PERCENT)) /
                    XUNJI_RECOVERY_TICKS;
                if (recovery_percent < speed_percent) {
                    speed_percent = recovery_percent;
                }
            }

            /* 旧角点状态机仅供未开放的历史接口，H2连续赛道不参与判角。 */
            if ((s_h2_lap_mode == 0U) &&
                (s_control.corner_candidate_ticks >=
                 XUNJI_CORNER_INTENT_CONFIRM)) {
                int approach_speed = ((int)purpose *
                                      XUNJI_CORNER_APPROACH_PERCENT) / 100;
                if (base_speed > approach_speed) {
                    base_speed = approach_speed;
                }
            }

            base_speed = (base_speed * speed_percent) / 100;
            correction_limit =
                (base_speed * correction_percent) / 100;
            correction = clamp_int(correction,
                                   -correction_limit,
                                   correction_limit);
            correction = approach_correction(s_control.applied_correction,
                                             correction);
            s_control.applied_correction = (int16_t)correction;
        }
    } else if (s_control.status.state == XUNJI_STATE_CORNER_ADVANCE) {
        /* 已确认宽角点后只按里程匀速前探，不再让宽黑区的位置重心左右拉扯。 */
        base_speed = (base_speed * XUNJI_MARKER_ADVANCE_PERCENT) / 100;
        correction = 0;
        s_control.applied_correction = 0;
    } else if ((s_control.status.state == XUNJI_STATE_CORNER_LEFT) ||
               (s_control.status.state == XUNJI_STATE_CORNER_RIGHT)) {
        int minimum_turn_correction;
        int turn_left = (s_control.status.state == XUNJI_STATE_CORNER_LEFT);

        base_speed = (base_speed * XUNJI_TURN_SPEED_PERCENT) / 100;
        correction_limit = base_speed;
        minimum_turn_correction = (base_speed * 4) / 5;

        if (s_control.status.sensor.valid == 0U) {
            /* 弯内丢线时停住内轮、保持外轮前进；不采用带载反转。 */
            minimum_turn_correction = base_speed;
        }
        if (turn_left != 0) {
            if (correction < minimum_turn_correction) {
                correction = minimum_turn_correction;
            }
            correction = clamp_int(correction, 0, correction_limit);
        } else {
            if (correction > -minimum_turn_correction) {
                correction = -minimum_turn_correction;
            }
            correction = clamp_int(correction, -correction_limit, 0);
        }
        s_control.applied_correction = 0;
    } else if ((s_h2_lap_mode != 0U) &&
               (s_control.status.state == XUNJI_STATE_LOST)) {
        int search_base =
            (base_speed * H2_SEARCH_SPEED_PERCENT) / 100;
        int search_correction =
            (search_base * H2_SEARCH_CORRECTION_PERCENT) / 100;

        base_speed = search_base;
        if (s_control.h2_search_direction < 0) {
            correction = search_correction;
        } else if (s_control.h2_search_direction > 0) {
            correction = -search_correction;
        } else {
            /*
             * 居中时直接丢线没有可靠左右证据，保持低速直行比盲选方向更安全。
             * 800ms仍未重新见线时由既有丢线超时保护停车。
             */
            correction = 0;
        }
        s_control.applied_correction = (int16_t)correction;
    } else if (s_direction_tune_mode != 0U) {
        /*
         * 连续调试丢线后不停车：保持外轮不超过purpose，只降低内轮并向
         * 最后见线侧转向。若启动后从未见过线，方向为0，先等速直行而不
         * 凭默认值猛转；一旦见过线，后续丢线都使用真实锁存方向。
         */
        correction = 0;
        if (s_control.lost_direction < 0) {
            correction =
                (base_speed * XUNJI_TUNE_SEARCH_CORRECTION_PERCENT) / 100;
        } else if (s_control.lost_direction > 0) {
            correction =
                -(base_speed * XUNJI_TUNE_SEARCH_CORRECTION_PERCENT) / 100;
        }
        s_control.applied_correction = (int16_t)correction;
    } else {
        s_control.applied_correction = 0;
        /* LOST的160ms判弯窗口内，不能继续锁住上一拍差速：
         * 出弯后的最后一拍往往仍是大转向量，锁住它会把短暂全白放大成左右大摆。
         * 这里保持两轮目标的平均值，并将差速平滑收回0；速度PI不重置，
         * 所以既不会继续拐向，也不会恢复成50~100ms的PWM脉冲。 */
        base_speed = ((int)s_control.applied_left +
                      (int)s_control.applied_right + 1) / 2;
        correction = 0;
        s_control.applied_correction = 0;
    }

    /* 安全差速：转向只减小内轮，不再通过抬高外轮获得差速。这样第三模式
     * 任一车轮的目标都不会超过已在速度模式验证过的 purpose。
     *
     * 灰度位序固定为 L4,L3,L2,L1,R1,R2,R3,R4：
     *   左侧见线 -> position<450 -> PID correction>0
     *             -> 左轮减速、右轮保持 -> 车向左修正；
     *   右侧见线 -> position>450 -> PID correction<0
     *             -> 右轮减速、左轮保持 -> 车向右修正。
     * 这与“L4见黑表示车偏右，需要左转”的实车几何关系一致。
     */
    if (correction >= 0) {
        target_left = base_speed - correction;
        target_right = base_speed;
    } else {
        target_left = base_speed;
        target_right = base_speed + correction;
    }

    target_limit = control_speed;
    target_left = clamp_int(target_left, 0, target_limit);
    target_right = clamp_int(target_right, 0, target_limit);
    s_control.status.control_correction = (int16_t)correction;

    /*
     * 目标升高时保持柔和，目标降低时更快响应。旧版统一步长20会让内轮从80降到
     * 约10仍耗时200ms，容易在已经识别到直角后继续冲出赛道。
     */
    rise_step = control_speed / XUNJI_TARGET_RISE_DIVISOR;
    fall_step = control_speed / XUNJI_TARGET_FALL_DIVISOR;
    if (rise_step < 2) {
        rise_step = 2;
    }
    if (fall_step < 4) {
        fall_step = 4;
    }
    if ((s_control.status.state == XUNJI_STATE_TRACK) ||
        (s_control.status.state == XUNJI_STATE_CORNER_ADVANCE) ||
        (s_control.status.state == XUNJI_STATE_LOST)) {
        int track_step = control_speed / XUNJI_TRACK_TARGET_SLEW_DIVISOR;
        if (track_step < 1) {
            track_step = 1;
        }
        /* 出弯后也使用稳定直线的10ms缓变速度，防止目标跑在反馈前面。 */
        rise_step = track_step;
        fall_step = track_step;
    } else if ((s_control.status.state == XUNJI_STATE_CORNER_LEFT) ||
               (s_control.status.state == XUNJI_STATE_CORNER_RIGHT)) {
        /* 正式转向需要停住内轮时，一拍内把目标降到0并进入短路制动，
         * 不再用两拍斜坡减速，更不能依靠机械惯性滑行。 */
        fall_step = target_limit;
    }
    if (s_h2_lap_mode != 0U) {
        /*
         * 运球任务的加速、减速、丢线搜索和恢复统一使用同一斜率，
         * 禁止状态切换产生一快一慢的速度阶跃。
         */
        if (s_mode1_test_mode != 0U) {
            rise_step = XUNJI_MODE1_TARGET_SLEW_STEP;
            fall_step = XUNJI_MODE1_TARGET_SLEW_STEP;
        } else {
            rise_step = XUNJI_TASK_TARGET_SLEW_STEP;
            fall_step = XUNJI_TASK_TARGET_SLEW_STEP;
        }
    }
    s_control.applied_left = (int16_t)approach_speed_target(
        s_control.applied_left, target_left, rise_step, fall_step);
    s_control.applied_right = (int16_t)approach_speed_target(
        s_control.applied_right, target_right, rise_step, fall_step);

    /*
     * 单轮从0重新启动也属于控制工况切换。任务路径不使用前馈，
     * 只按当前实际PWM反算积分，使下一次PI输出连续。
     */
    if (((previous_target_left == 0) && (s_control.applied_left > 0)) ||
        ((previous_target_right == 0) && (s_control.applied_right > 0))) {
        s_control.velocity_reprime_pending = 1U;
    }
    if (s_control.velocity_reprime_pending != 0U) {
        float prime_output_left =
            (s_control.applied_left == 0) ? 0.0f : (float)last_pwm_l;
        float prime_output_right =
            (s_control.applied_right == 0) ? 0.0f : (float)last_pwm_r;

        if (s_mode1_test_mode != 0U) {
            PID_PrimeMode1VelocityNoFeedforward(
                (float)s_control.applied_left,
                (float)s_control.applied_right,
                (float)measured_left,
                (float)measured_right,
                prime_output_left,
                prime_output_right);
        } else {
            PID_PrimeVelocityNoFeedforward(
                (float)s_control.applied_left,
                (float)s_control.applied_right,
                (float)measured_left,
                (float)measured_right,
                prime_output_left,
                prime_output_right);
        }
        s_control.velocity_reprime_pending = 0U;
    }

    s_control.status.target_left = s_control.applied_left;
    s_control.status.target_right = s_control.applied_right;
    xun_pwm_l = s_control.applied_left;
    xun_pwm_r = s_control.applied_right;

    if (s_control.applied_left == 0) {
        pwm_left = 0;
        if (s_mode1_test_mode != 0U) {
            PID_ResetMode1VelocityLeft();
        } else {
            PID_ResetVelocityLeft();
        }
    } else {
        pwm_left = (s_mode1_test_mode != 0U) ?
            (int)PID_Mode1VelocityLeftNoFeedforward(
                (float)s_control.applied_left, (float)measured_left) :
            (int)velocity_PID_value_l_no_ff(
                (float)s_control.applied_left, (float)measured_left);
    }
    if (s_control.applied_right == 0) {
        pwm_right = 0;
        if (s_mode1_test_mode != 0U) {
            PID_ResetMode1VelocityRight();
        } else {
            PID_ResetVelocityRight();
        }
    } else {
        pwm_right = (s_mode1_test_mode != 0U) ?
            (int)PID_Mode1VelocityRightNoFeedforward(
                (float)s_control.applied_right, (float)measured_right) :
            (int)velocity_PID_value_r_no_ff(
                (float)s_control.applied_right, (float)measured_right);
    }

    requested_pwm_left = clamp_int(pwm_left, 0, MOTOR_PWM_SAFE_LIMIT);
    requested_pwm_right = clamp_int(pwm_right, 0, MOTOR_PWM_SAFE_LIMIT);
    {
        int pwm_slew_step = (s_mode1_test_mode != 0U) ?
                            XUNJI_MODE1_PWM_SLEW_STEP :
                            XUNJI_TASK_PWM_SLEW_STEP;
        pwm_left = approach_speed_target(last_pwm_l, requested_pwm_left,
                                         pwm_slew_step, pwm_slew_step);
        pwm_right = approach_speed_target(last_pwm_r, requested_pwm_right,
                                          pwm_slew_step, pwm_slew_step);
    }

    /*
     * PWM斜率限制属于单轮执行器饱和。受限轮需要按真实PWM回算积分，
     * 未受限轮必须保留自己的积分，不能被另一轮的量化波动反复覆盖。
     */
    if (pwm_left != requested_pwm_left) {
        if (s_mode1_test_mode != 0U) {
            PID_PrimeMode1VelocityLeftNoFeedforward(
                (float)s_control.applied_left,
                (float)measured_left,
                (float)pwm_left);
        } else {
            PID_PrimeVelocityLeftNoFeedforward(
                (float)s_control.applied_left,
                (float)measured_left,
                (float)pwm_left);
        }
    }
    if (pwm_right != requested_pwm_right) {
        if (s_mode1_test_mode != 0U) {
            PID_PrimeMode1VelocityRightNoFeedforward(
                (float)s_control.applied_right,
                (float)measured_right,
                (float)pwm_right);
        } else {
            PID_PrimeVelocityRightNoFeedforward(
                (float)s_control.applied_right,
                (float)measured_right,
                (float)pwm_right);
        }
    }
    last_pwm_l = pwm_left;
    last_pwm_r = pwm_right;

    safety_fault = check_wheel_safety(s_control.applied_left,
                                      measured_left,
                                      pwm_left,
                                      control_speed,
                                      &s_control.stall_left_windows,
                                      &s_control.overspeed_left_windows,
                                      XUNJI_FAULT_STALL_LEFT,
                                      XUNJI_FAULT_OVERSPEED_LEFT);
    if (safety_fault == XUNJI_FAULT_NONE) {
        safety_fault = check_wheel_safety(s_control.applied_right,
                                          measured_right,
                                          pwm_right,
                                          control_speed,
                                          &s_control.stall_right_windows,
                                          &s_control.overspeed_right_windows,
                                          XUNJI_FAULT_STALL_RIGHT,
                                          XUNJI_FAULT_OVERSPEED_RIGHT);
    }
    if (safety_fault != XUNJI_FAULT_NONE) {
        fault_stop(safety_fault);
        return;
    }

    /* 本工程实车方向：逻辑左轮前进命令必须在 Set_Pwm 前取反。 */
    Set_Pwm(-pwm_left, pwm_right, 0, 0);

    if ((s_h2_lap_mode != 0U) &&
        (s_mode1_test_mode == 0U) &&
        (s_control.h2_finish_pending != 0U) &&
        (s_control.h2_finish_decelerating != 0U) &&
        (s_control.applied_left == 0) &&
        (s_control.applied_right == 0) &&
        (pwm_left == 0) && (pwm_right == 0) &&
        (abs_int(measured_left) <= H2_STOP_SETTLE_SPEED) &&
        (abs_int(measured_right) <= H2_STOP_SETTLE_SPEED)) {
        if (s_control.h2_stop_settle_ticks < 255U) {
            s_control.h2_stop_settle_ticks++;
        }
        if (s_control.h2_stop_settle_ticks >= H2_STOP_SETTLE_TICKS) {
            s_control.status.completed_laps = 1U;
            finish_run();
        }
    } else {
        s_control.h2_stop_settle_ticks = 0U;
    }

    if ((s_straight_task_mode != 0U) &&
        (s_control.status.straight_phase == XUNJI_STRAIGHT_DECEL) &&
        (s_control.straight_decel_ticks >=
         XUNJI_STRAIGHT_MS_TO_TICKS(XUNJI_STRAIGHT_DECEL_MS)) &&
        (s_control.applied_left == 0) &&
        (s_control.applied_right == 0) &&
        (pwm_left == 0) && (pwm_right == 0) &&
        (abs_int(measured_left) <= XUNJI_STRAIGHT_STOP_SETTLE_SPEED) &&
        (abs_int(measured_right) <= XUNJI_STRAIGHT_STOP_SETTLE_SPEED)) {
        if (s_control.straight_stop_settle_ticks < 255U) {
            s_control.straight_stop_settle_ticks++;
        }
        if (s_control.straight_stop_settle_ticks >=
            XUNJI_STRAIGHT_STOP_SETTLE_TICKS) {
            finish_run();
        }
    } else {
        s_control.straight_stop_settle_ticks = 0U;
    }
}

void Xunji_GetStatus(XunjiStatus *status)
{
    if (status != 0) {
        /* 运行诊断量只在取快照时同步，不参与控制，便于VOFA定位是否仍有灰度闪断。 */
        *status = s_control.status;
        status->line_missing_ticks = s_control.line_missing_ticks;
        status->lost_ticks = s_control.lost_ticks;
        status->search_direction = s_control.lost_direction;
        status->velocity_bridge_windows = s_control.velocity_bridge_windows;
        status->velocity_reprime_pending = s_control.velocity_reprime_pending;
    }
}
