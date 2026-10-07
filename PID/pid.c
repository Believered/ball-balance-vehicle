#include "pid.h"
#include "moter.h"
#include <stdint.h>
#include <math.h>

/*
 * 参数区：保留实车已经调过的数值。后续实车整定通常只需要修改本区和
 * XUNJI/xunji.c 中的基础目标速度，不需要改状态机逻辑。
 */
float VKp_l = 13.5f;
float VKi_l = 5.5f;
float VKd_l = 0.0f;

float VKp_r = 13.0f;
float VKi_r = 6.5f;
float VKd_r = 0.0f;

/*
 * 模式一固定速度80的独立速度PI。初值沿用当前已经验证过的左右轮参数，
 * 但后续模式二在线调参不会再影响模式一。
 */
float mode1_VKp_l = 26.0f;
float mode1_VKi_l = 12.0f;
float mode1_VKd_l = 0.0f;
float mode1_VKp_r = 17.5f;
float mode1_VKi_r = 16.0f;
float mode1_VKd_r = 0.0f;

/*
 * 八路数字灰度的位置环：
 *   Kp = 非线性位置误差增益；
 *   Ki = 仅对持续中等偏差生效的弱积分（默认关闭）；
 *   Kd = 左右轮实测速度差阻尼，不再对离散位置直接微分。
 * OLED/模式五仍按参数乘1000显示，因此默认值分别显示50、0、200。
 */
float place_Kp = 0.30f;
float place_Ki = 0.06f;
float place_Kd = 0.24f;

/*
 * 模式一独立位置环。高速循迹先关闭积分，避免残留积分把车辆推过中心；
 * 轮速差阻尼略加强，用于抑制高速回线后的左右摇头。
 */
float mode1_place_Kp = 0.16f;
float mode1_place_Ki = 0.00f;
float mode1_place_Kd = 0.18f;

float angle_Kp = 0.0f;
float angle_Ki = 0.0f;
float angle_Kd = 0.0f;

/* TIMA0 PWM 周期为 1000。速度 PI 输出必须受物理 PWM 范围约束。 */
#define VELOCITY_OUTPUT_LIMIT   ((float)MOTOR_PWM_SAFE_LIMIT)
#define VELOCITY_SUM_LIMIT      ((float)MOTOR_PWM_SAFE_LIMIT)
#define VELOCITY_FF_LEFT_PER_SPEED  (6.90f)
#define VELOCITY_FF_RIGHT_PER_SPEED (5.05f)
#define VELOCITY_FF_LIMIT       (700.0f)
#define PLACE_SUM_LIMIT                  (5000.0f)
#define PLACE_CENTER_ENTER_ERROR           (55.0f)
#define PLACE_CENTER_EXIT_ERROR            (90.0f)
#define PLACE_SENSOR_MAX_ERROR             (350.0f)
#define PLACE_NONLINEAR_RANGE              \
    (PLACE_SENSOR_MAX_ERROR - PLACE_CENTER_ENTER_ERROR)
#define PLACE_INTEGRAL_MIN_ERROR            (90.0f)
#define PLACE_INTEGRAL_MAX_ERROR           (220.0f)
#define PLACE_INTEGRAL_OUTPUT_LIMIT          (2.0f)
#define PLACE_INTEGRAL_CONFIRM_TICKS          3U
#define PLACE_CENTER_INTEGRAL_DECAY          (0.50f)
#define PLACE_OUTER_INTEGRAL_DECAY           (0.90f)
/* 速度PID由原50ms改为10ms。反馈仍换算成“等效50ms边沿数”，因此P和目标值
 * 不变；积分每拍只累计1/5误差，微分乘5，保持原实车参数的时间尺度。 */
#define VELOCITY_PID_DT_RATIO     (0.20f)
/* 位置积分仍按20ms调用，0.4表示相对旧50ms参数的时间尺度。 */
#define PLACE_PID_DT_RATIO        (0.40f)

float velocity_sum_l = 0.0f;
float last_velocity_err_l = 0.0f;
float velocity_err_l = 0.0f;

float velocity_sum_r = 0.0f;
float last_velocity_err_r = 0.0f;
float velocity_err_r = 0.0f;

static float mode1_velocity_sum_l = 0.0f;
static float mode1_last_velocity_err_l = 0.0f;
static float mode1_velocity_err_l = 0.0f;
static float mode1_velocity_sum_r = 0.0f;
static float mode1_last_velocity_err_r = 0.0f;
static float mode1_velocity_err_r = 0.0f;

float place_err = 0.0f;
float place_last_err = 0.0f;
float place_err_sum = 0.0f;
float place_err_difference = 0.0f;
static uint8_t place_center_locked = 1U;
static uint8_t place_integral_confirm_ticks = 0U;
static int8_t place_integral_direction = 0;

static float mode1_place_err = 0.0f;
static float mode1_place_last_err = 0.0f;
static float mode1_place_err_sum = 0.0f;
static float mode1_place_err_difference = 0.0f;
static uint8_t mode1_place_center_locked = 1U;
static uint8_t mode1_place_integral_confirm_ticks = 0U;
static int8_t mode1_place_integral_direction = 0;

float angle_err = 0.0f;
float angle_last_err = 0.0f;
float angle_err_sum = 0.0f;
float angle_err_difference = 0.0f;

static float limit_float(float value, float limit)
{
    /*
     * IEEE-754 NaN与上下限比较都为false，若不先拦截会穿过PID限幅并在
     * 浮点转整型时产生不可预测命令。正常有限值的控制结果不受影响。
     */
    if (value != value) {
        return 0.0f;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 按指定斜率计算带符号前馈，并限制前馈本身而不是限制PID总输出。 */
static float velocity_feedforward(float target, float pwm_per_speed)
{
    float magnitude = (target < 0.0f) ? -target : target;
    float output = magnitude * pwm_per_speed;

    if (output > VELOCITY_FF_LIMIT) {
        output = VELOCITY_FF_LIMIT;
    }
    return (target < 0.0f) ? -output : output;
}

float PID_VelocityFeedforwardLeft(float target)
{
    return velocity_feedforward(target, VELOCITY_FF_LEFT_PER_SPEED);
}

float PID_VelocityFeedforwardRight(float target)
{
    return velocity_feedforward(target, VELOCITY_FF_RIGHT_PER_SPEED);
}

/**
 * @brief 带条件积分抗饱和的位置式速度 PID。
 *
 * 当前 Ki/Kd 数值保持不变。只有当输出已经到达 PWM 极限且误差还在推动
 * 输出继续饱和时，才暂停积分，避免丢线或堵转后恢复时积分长期残留。
 */
static float velocity_pid_step(float target,
                               float measured,
                               float feedforward,
                               float kp,
                               float ki,
                               float kd,
                               float *sum,
                               float *last_error,
                               float *current_error)
{
    if (!isfinite(target) || !isfinite(measured) || !isfinite(feedforward) ||
        !isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
        *sum = 0.0f;
        *last_error = 0.0f;
        *current_error = 0.0f;
        return 0.0f;
    }
    float error = target - measured;
    float candidate_sum = limit_float(*sum + error * VELOCITY_PID_DT_RATIO,
                                      VELOCITY_SUM_LIMIT);
    float derivative = (error - *last_error) / VELOCITY_PID_DT_RATIO;
    float output = feedforward + kp * error +
                   ki * candidate_sum + kd * derivative;

    if (((output > VELOCITY_OUTPUT_LIMIT) && (error > 0.0f)) ||
        ((output < -VELOCITY_OUTPUT_LIMIT) && (error < 0.0f)) ||
        ((target >= 0.0f) && (output < 0.0f) && (error < 0.0f))) {
        /* 饱和方向与误差方向相同，保留旧积分。 */
        candidate_sum = *sum;
        output = feedforward + kp * error +
                 ki * candidate_sum + kd * derivative;
    }

    /* 前进目标的物理下限是0 PWM，禁止积分在该下限外继续向负方向积累。 */
    if ((target >= 0.0f) && (output < 0.0f)) {
        output = 0.0f;
    }

    *sum = candidate_sum;
    *last_error = error;
    *current_error = error;
    return limit_float(output, VELOCITY_OUTPUT_LIMIT);
}

float velocity_PID_value_l(float measure, float calcu)
{
    return velocity_pid_step(measure, calcu,
                             PID_VelocityFeedforwardLeft(measure),
                             VKp_l, VKi_l, VKd_l,
                             &velocity_sum_l,
                             &last_velocity_err_l,
                             &velocity_err_l);
}

float velocity_PID_value_r(float measure, float calcu)
{
    return velocity_pid_step(measure, calcu,
                             PID_VelocityFeedforwardRight(measure),
                             VKp_r, VKi_r, VKd_r,
                             &velocity_sum_r,
                             &last_velocity_err_r,
                             &velocity_err_r);
}

float velocity_PID_value_l_no_ff(float measure, float calcu)
{
    return velocity_pid_step(measure, calcu, 0.0f,
                             VKp_l, VKi_l, VKd_l,
                             &velocity_sum_l,
                             &last_velocity_err_l,
                             &velocity_err_l);
}

float velocity_PID_value_r_no_ff(float measure, float calcu)
{
    return velocity_pid_step(measure, calcu, 0.0f,
                             VKp_r, VKi_r, VKd_r,
                             &velocity_sum_r,
                             &last_velocity_err_r,
                             &velocity_err_r);
}

float PID_Mode1VelocityLeftNoFeedforward(float target, float measured)
{
    return velocity_pid_step(target, measured, 0.0f,
                             mode1_VKp_l, mode1_VKi_l, mode1_VKd_l,
                             &mode1_velocity_sum_l,
                             &mode1_last_velocity_err_l,
                             &mode1_velocity_err_l);
}

float PID_Mode1VelocityRightNoFeedforward(float target, float measured)
{
    return velocity_pid_step(target, measured, 0.0f,
                             mode1_VKp_r, mode1_VKi_r, mode1_VKd_r,
                             &mode1_velocity_sum_r,
                             &mode1_last_velocity_err_r,
                             &mode1_velocity_err_r);
}

/**
 * @brief 八路数字灰度专用的非线性位置 PI。
 *
 * 位置量仍为100..800、中心为450，固定20ms调用。中心使用55/90双阈值迟滞：
 * 进入中心后必须真正偏出90才恢复纠偏，避免400/500量化值反复翻转方向。
 * P项按有效误差平方增长，中心柔和、外侧有足够修正；积分仅在持续中等偏差
 * 时启用并限制到±2轮速单位。D不在这里计算，改由左右轮实测速度差提供阻尼。
 */
static float place_pid_step(float measure,
                            float calcu,
                            float output_limit)
{
    float output;
    float candidate_sum;
    float error_magnitude;
    float effective_magnitude;
    float shaped_error;
    float integral_sum_limit = PLACE_SUM_LIMIT;
    int8_t error_direction;

    if (!isfinite(measure) || !isfinite(calcu) || !isfinite(output_limit)) {
        PID_ResetPlace();
        return 0.0f;
    }
    place_err = measure - calcu;
    error_magnitude = abs_float(place_err);

    if (place_center_locked != 0U) {
        if (error_magnitude > PLACE_CENTER_EXIT_ERROR) {
            place_center_locked = 0U;
        }
    } else if (error_magnitude <= PLACE_CENTER_ENTER_ERROR) {
        place_center_locked = 1U;
    }

    if (place_center_locked != 0U) {
        place_err_sum *= PLACE_CENTER_INTEGRAL_DECAY;
        if (abs_float(place_err_sum * place_Ki) < 0.05f) {
            place_err_sum = 0.0f;
        }
        place_integral_confirm_ticks = 0U;
        place_integral_direction = 0;
        place_last_err = 0.0f;
        place_err_difference = 0.0f;
        return 0.0f;
    }

    effective_magnitude = error_magnitude - PLACE_CENTER_ENTER_ERROR;
    if (effective_magnitude < 0.0f) {
        effective_magnitude = 0.0f;
    } else if (effective_magnitude > PLACE_NONLINEAR_RANGE) {
        effective_magnitude = PLACE_NONLINEAR_RANGE;
    }
    shaped_error = (effective_magnitude * effective_magnitude) /
                   PLACE_NONLINEAR_RANGE;
    if (place_err < 0.0f) {
        shaped_error = -shaped_error;
        error_direction = -1;
    } else {
        error_direction = 1;
    }

    candidate_sum = place_err_sum;
    if (abs_float(place_Ki) < 0.000001f) {
        candidate_sum = 0.0f;
        place_integral_confirm_ticks = 0U;
        place_integral_direction = 0;
    } else if ((error_magnitude >= PLACE_INTEGRAL_MIN_ERROR) &&
               (error_magnitude <= PLACE_INTEGRAL_MAX_ERROR)) {
        if (place_integral_direction != error_direction) {
            place_integral_direction = error_direction;
            place_integral_confirm_ticks = 1U;
            candidate_sum = 0.0f;
        } else if (place_integral_confirm_ticks <
                   PLACE_INTEGRAL_CONFIRM_TICKS) {
            place_integral_confirm_ticks++;
        } else {
            candidate_sum += shaped_error * PLACE_PID_DT_RATIO;
        }

        integral_sum_limit =
            PLACE_INTEGRAL_OUTPUT_LIMIT / abs_float(place_Ki);
        if (integral_sum_limit > PLACE_SUM_LIMIT) {
            integral_sum_limit = PLACE_SUM_LIMIT;
        }
        candidate_sum = limit_float(candidate_sum, integral_sum_limit);
    } else {
        candidate_sum *= PLACE_OUTER_INTEGRAL_DECAY;
        place_integral_confirm_ticks = 0U;
        place_integral_direction = error_direction;
    }

    output = place_Kp * shaped_error + place_Ki * candidate_sum;

    /* 输出饱和且积分仍在同向推动时保留旧积分，禁止离线后残留单向修正。 */
    if ((output_limit > 0.0f) &&
        (((output > output_limit) && (shaped_error > 0.0f)) ||
         ((output < -output_limit) && (shaped_error < 0.0f)))) {
        candidate_sum = place_err_sum;
        output = place_Kp * shaped_error + place_Ki * candidate_sum;
    }
    place_err_sum = candidate_sum;
    place_err_difference = 0.0f;
    place_last_err = place_err;

    if (output_limit > 0.0f) {
        output = limit_float(output, output_limit);
    }
    return output;
}

/**
 * @brief 模式一独立的非线性位置PI。
 *
 * 非线性整形和中心迟滞与模式五一致，但参数、积分、迟滞状态完全独立。
 */
static float mode1_place_pid_step(float measure,
                                  float calcu,
                                  float output_limit)
{
    float output;
    float candidate_sum;
    float error_magnitude;
    float effective_magnitude;
    float shaped_error;
    float integral_sum_limit = PLACE_SUM_LIMIT;
    int8_t error_direction;

    if (!isfinite(measure) || !isfinite(calcu) || !isfinite(output_limit)) {
        PID_ResetMode1Place();
        return 0.0f;
    }
    mode1_place_err = measure - calcu;
    error_magnitude = abs_float(mode1_place_err);

    if (mode1_place_center_locked != 0U) {
        if (error_magnitude > PLACE_CENTER_EXIT_ERROR) {
            mode1_place_center_locked = 0U;
        }
    } else if (error_magnitude <= PLACE_CENTER_ENTER_ERROR) {
        mode1_place_center_locked = 1U;
    }

    if (mode1_place_center_locked != 0U) {
        mode1_place_err_sum *= PLACE_CENTER_INTEGRAL_DECAY;
        if (abs_float(mode1_place_err_sum * mode1_place_Ki) < 0.05f) {
            mode1_place_err_sum = 0.0f;
        }
        mode1_place_integral_confirm_ticks = 0U;
        mode1_place_integral_direction = 0;
        mode1_place_last_err = 0.0f;
        mode1_place_err_difference = 0.0f;
        return 0.0f;
    }

    effective_magnitude = error_magnitude - PLACE_CENTER_ENTER_ERROR;
    if (effective_magnitude < 0.0f) {
        effective_magnitude = 0.0f;
    } else if (effective_magnitude > PLACE_NONLINEAR_RANGE) {
        effective_magnitude = PLACE_NONLINEAR_RANGE;
    }
    shaped_error = (effective_magnitude * effective_magnitude) /
                   PLACE_NONLINEAR_RANGE;
    if (mode1_place_err < 0.0f) {
        shaped_error = -shaped_error;
        error_direction = -1;
    } else {
        error_direction = 1;
    }

    candidate_sum = mode1_place_err_sum;
    if (abs_float(mode1_place_Ki) < 0.000001f) {
        candidate_sum = 0.0f;
        mode1_place_integral_confirm_ticks = 0U;
        mode1_place_integral_direction = 0;
    } else if ((error_magnitude >= PLACE_INTEGRAL_MIN_ERROR) &&
               (error_magnitude <= PLACE_INTEGRAL_MAX_ERROR)) {
        if (mode1_place_integral_direction != error_direction) {
            mode1_place_integral_direction = error_direction;
            mode1_place_integral_confirm_ticks = 1U;
            candidate_sum = 0.0f;
        } else if (mode1_place_integral_confirm_ticks <
                   PLACE_INTEGRAL_CONFIRM_TICKS) {
            mode1_place_integral_confirm_ticks++;
        } else {
            candidate_sum += shaped_error * PLACE_PID_DT_RATIO;
        }

        integral_sum_limit =
            PLACE_INTEGRAL_OUTPUT_LIMIT / abs_float(mode1_place_Ki);
        if (integral_sum_limit > PLACE_SUM_LIMIT) {
            integral_sum_limit = PLACE_SUM_LIMIT;
        }
        candidate_sum = limit_float(candidate_sum, integral_sum_limit);
    } else {
        candidate_sum *= PLACE_OUTER_INTEGRAL_DECAY;
        mode1_place_integral_confirm_ticks = 0U;
        mode1_place_integral_direction = error_direction;
    }

    output = mode1_place_Kp * shaped_error +
             mode1_place_Ki * candidate_sum;
    if ((output_limit > 0.0f) &&
        (((output > output_limit) && (shaped_error > 0.0f)) ||
         ((output < -output_limit) && (shaped_error < 0.0f)))) {
        candidate_sum = mode1_place_err_sum;
        output = mode1_place_Kp * shaped_error +
                 mode1_place_Ki * candidate_sum;
    }

    mode1_place_err_sum = candidate_sum;
    mode1_place_err_difference = 0.0f;
    mode1_place_last_err = mode1_place_err;
    if (output_limit > 0.0f) {
        output = limit_float(output, output_limit);
    }
    return output;
}

float place_PID_value(float measure, float calcu)
{
    return place_pid_step(measure, calcu, 0.0f);
}

float place_PID_value_limited(float measure,
                              float calcu,
                              float output_limit)
{
    if (output_limit < 0.0f) {
        output_limit = -output_limit;
    }
    return place_pid_step(measure, calcu, output_limit);
}

float PID_Mode1PlaceLimited(float measure,
                            float calcu,
                            float output_limit)
{
    if (output_limit < 0.0f) {
        output_limit = -output_limit;
    }
    return mode1_place_pid_step(measure, calcu, output_limit);
}

/**
 * @brief 用左右轮实际速度差抑制过度转向。
 *
 * 正位置修正会降低左轮，正常情况下右轮速度高于左轮；R-L为正，阻尼项会
 * 减小继续左转的请求。负修正同理。阻尼只调整已有位置修正，绝不在中心
 * 凭轮速差自行产生反向转向，也不允许输出越过零改变方向。
 */
float PID_ApplyPlaceRateDamping(float correction,
                                float measured_left,
                                float measured_right,
                                float output_limit)
{
    float output;
    float wheel_rate;

    if ((correction < 0.0001f) && (correction > -0.0001f)) {
        return 0.0f;
    }

    wheel_rate = measured_right - measured_left;
    output = correction - place_Kd * wheel_rate;

    if ((correction > 0.0f) && (output < 0.0f)) {
        output = 0.0f;
    } else if ((correction < 0.0f) && (output > 0.0f)) {
        output = 0.0f;
    }

    if (output_limit < 0.0f) {
        output_limit = -output_limit;
    }
    if (output_limit > 0.0f) {
        output = limit_float(output, output_limit);
    }
    return output;
}

float PID_Mode1ApplyPlaceRateDamping(float correction,
                                     float measured_left,
                                     float measured_right,
                                     float output_limit)
{
    float output;
    float wheel_rate;

    if ((correction < 0.0001f) && (correction > -0.0001f)) {
        return 0.0f;
    }

    wheel_rate = measured_right - measured_left;
    output = correction - mode1_place_Kd * wheel_rate;

    /* 阻尼只减小原修正，禁止越过零后反向打舵。 */
    if ((correction > 0.0f) && (output < 0.0f)) {
        output = 0.0f;
    } else if ((correction < 0.0f) && (output > 0.0f)) {
        output = 0.0f;
    }

    if (output_limit < 0.0f) {
        output_limit = -output_limit;
    }
    if (output_limit > 0.0f) {
        output = limit_float(output, output_limit);
    }
    return output;
}

float angle_PID_value(float measure, float calcu)
{
    angle_err = measure - calcu;
    angle_err_sum += angle_err;
    angle_err_difference = angle_err - angle_last_err;
    angle_last_err = angle_err;

    return angle_Kp * angle_err +
           angle_Ki * angle_err_sum +
           angle_Kd * angle_err_difference;
}

void PID_ResetVelocityLeft(void)
{
    velocity_sum_l = 0.0f;
    last_velocity_err_l = 0.0f;
    velocity_err_l = 0.0f;
}

void PID_ResetVelocityRight(void)
{
    velocity_sum_r = 0.0f;
    last_velocity_err_r = 0.0f;
    velocity_err_r = 0.0f;
}

void PID_ResetMode1VelocityLeft(void)
{
    mode1_velocity_sum_l = 0.0f;
    mode1_last_velocity_err_l = 0.0f;
    mode1_velocity_err_l = 0.0f;
}

void PID_ResetMode1VelocityRight(void)
{
    mode1_velocity_sum_r = 0.0f;
    mode1_last_velocity_err_r = 0.0f;
    mode1_velocity_err_r = 0.0f;
}

void PID_ResetMode1Velocity(void)
{
    PID_ResetMode1VelocityLeft();
    PID_ResetMode1VelocityRight();
}

void PID_ResetVelocity(void)
{
    PID_ResetVelocityLeft();
    PID_ResetVelocityRight();
}

/** 根据期望的下一拍输出反算位置式PI积分初值。 */
static void velocity_pid_prime_one(float target,
                                   float measured,
                                   float output,
                                   float feedforward,
                                   float kp,
                                   float ki,
                                   float *sum,
                                   float *last_error,
                                   float *current_error)
{
    float error = target - measured;
    /* 下一拍计算会先执行candidate_sum=sum+error*dt，因此这里提前减去该增量。
     * 同时把last_error置为当前误差，使D项在接管瞬间为0，实现无扰切换。 */
    if ((ki > 0.000001f) || (ki < -0.000001f)) {
        float next_sum = (output - feedforward - kp * error) / ki;
        *sum = limit_float(next_sum - error * VELOCITY_PID_DT_RATIO,
                           VELOCITY_SUM_LIMIT);
    } else {
        *sum = 0.0f;
    }
    *last_error = error;
    *current_error = error;
}

void PID_PrimeVelocity(float target_left,
                       float target_right,
                       float measured_left,
                       float measured_right,
                       float output_left,
                       float output_right)
{
    velocity_pid_prime_one(target_left, measured_left, output_left,
                           PID_VelocityFeedforwardLeft(target_left),
                           VKp_l, VKi_l,
                           &velocity_sum_l,
                           &last_velocity_err_l,
                           &velocity_err_l);
    velocity_pid_prime_one(target_right, measured_right, output_right,
                           PID_VelocityFeedforwardRight(target_right),
                           VKp_r, VKi_r,
                           &velocity_sum_r,
                           &last_velocity_err_r,
                           &velocity_err_r);
}

void PID_PrimeVelocityNoFeedforward(float target_left,
                                    float target_right,
                                    float measured_left,
                                    float measured_right,
                                    float output_left,
                                    float output_right)
{
    velocity_pid_prime_one(target_left, measured_left, output_left, 0.0f,
                           VKp_l, VKi_l,
                           &velocity_sum_l,
                           &last_velocity_err_l,
                           &velocity_err_l);
    velocity_pid_prime_one(target_right, measured_right, output_right, 0.0f,
                           VKp_r, VKi_r,
                           &velocity_sum_r,
                           &last_velocity_err_r,
                           &velocity_err_r);
}

/**
 * @brief 仅按左轮真实PWM播种左轮PI。
 *
 * PWM斜率限制是左右独立发生的。如果因为任意一轮受限就同时播种两轮，
 * 另一轮的积分会被反复覆盖，表现为目标未达到但PWM长期不再增加。
 */
void PID_PrimeVelocityLeftNoFeedforward(float target,
                                        float measured,
                                        float output)
{
    velocity_pid_prime_one(target, measured, output, 0.0f,
                           VKp_l, VKi_l,
                           &velocity_sum_l,
                           &last_velocity_err_l,
                           &velocity_err_l);
}

/** @brief 仅按右轮真实PWM播种右轮PI，避免左轮量化波动重置右轮积分。 */
void PID_PrimeVelocityRightNoFeedforward(float target,
                                         float measured,
                                         float output)
{
    velocity_pid_prime_one(target, measured, output, 0.0f,
                           VKp_r, VKi_r,
                           &velocity_sum_r,
                           &last_velocity_err_r,
                           &velocity_err_r);
}

void PID_PrimeMode1VelocityNoFeedforward(float target_left,
                                         float target_right,
                                         float measured_left,
                                         float measured_right,
                                         float output_left,
                                         float output_right)
{
    velocity_pid_prime_one(target_left, measured_left, output_left, 0.0f,
                           mode1_VKp_l, mode1_VKi_l,
                           &mode1_velocity_sum_l,
                           &mode1_last_velocity_err_l,
                           &mode1_velocity_err_l);
    velocity_pid_prime_one(target_right, measured_right, output_right, 0.0f,
                           mode1_VKp_r, mode1_VKi_r,
                           &mode1_velocity_sum_r,
                           &mode1_last_velocity_err_r,
                           &mode1_velocity_err_r);
}

/** @brief 仅按左轮真实PWM播种模式一左轮PI。 */
void PID_PrimeMode1VelocityLeftNoFeedforward(float target,
                                             float measured,
                                             float output)
{
    velocity_pid_prime_one(target, measured, output, 0.0f,
                           mode1_VKp_l, mode1_VKi_l,
                           &mode1_velocity_sum_l,
                           &mode1_last_velocity_err_l,
                           &mode1_velocity_err_l);
}

/** @brief 仅按右轮真实PWM播种模式一右轮PI。 */
void PID_PrimeMode1VelocityRightNoFeedforward(float target,
                                              float measured,
                                              float output)
{
    velocity_pid_prime_one(target, measured, output, 0.0f,
                           mode1_VKp_r, mode1_VKi_r,
                           &mode1_velocity_sum_r,
                           &mode1_last_velocity_err_r,
                           &mode1_velocity_err_r);
}

void PID_ResetPlace(void)
{
    place_err = 0.0f;
    place_last_err = 0.0f;
    place_err_sum = 0.0f;
    place_err_difference = 0.0f;
    place_center_locked = 1U;
    place_integral_confirm_ticks = 0U;
    place_integral_direction = 0;
}

void PID_ResetMode1Place(void)
{
    mode1_place_err = 0.0f;
    mode1_place_last_err = 0.0f;
    mode1_place_err_sum = 0.0f;
    mode1_place_err_difference = 0.0f;
    mode1_place_center_locked = 1U;
    mode1_place_integral_confirm_ticks = 0U;
    mode1_place_integral_direction = 0;
}

void PID_ResetAll(void)
{
    PID_ResetVelocity();
    PID_ResetMode1Velocity();
    PID_ResetPlace();
    PID_ResetMode1Place();

    angle_err = 0.0f;
    angle_last_err = 0.0f;
    angle_err_sum = 0.0f;
    angle_err_difference = 0.0f;
}
