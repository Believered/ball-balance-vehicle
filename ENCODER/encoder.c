#include "encoder.h"
#include "board.h"
#include "ti_msp_dl_config.h"

volatile int _encoder_l_count = 0, _encoder_r_count = 0;
volatile uint32_t encoder_l_total_edges = 0U;
volatile uint32_t encoder_r_total_edges = 0U;
volatile uint8_t encoder_l_a_level = 0U;
volatile uint8_t encoder_l_b_level = 0U;
volatile uint8_t encoder_r_a_level = 0U;
volatile uint8_t encoder_r_b_level = 0U;
volatile uint32_t encoder_l_invalid_transitions = 0U;
volatile uint32_t encoder_r_invalid_transitions = 0U;
volatile int encoder_l_a_edges_window = 0;
volatile int encoder_l_b_edges_window = 0;
volatile int encoder_r_a_edges_window = 0;
volatile int encoder_r_b_edges_window = 0;
volatile int encoder_l_quadrature_window = 0;
volatile int encoder_r_quadrature_window = 0;
volatile uint8_t encoder_phase_warning_mask = 0U;
volatile uint8_t encoder_feedback_valid_mask = 0x03U;
volatile uint8_t encoder_irq_storm_latched = 0U;
volatile uint32_t encoder_irq_overrun_count = 0U;
volatile uint8_t encoder_mapping_swapped = 0U;
/* 编码器插座正确连接时保持直连映射：左插座反馈左轮、右插座反馈右轮。 */
volatile uint8_t encoder_mapping_valid = 1U;

static uint8_t s_encoder_l_previous_state = 0U;
static uint8_t s_encoder_r_previous_state = 0U;
static volatile int s_encoder_l_quadrature_count = 0;
static volatile int s_encoder_r_quadrature_count = 0;
static volatile uint16_t s_encoder_l_a_edges = 0U;
static volatile uint16_t s_encoder_l_b_edges = 0U;
static volatile uint16_t s_encoder_r_a_edges = 0U;
static volatile uint16_t s_encoder_r_b_edges = 0U;
static volatile uint16_t s_encoder_l_legal_window = 0U;
static volatile uint16_t s_encoder_r_legal_window = 0U;
static volatile uint16_t s_encoder_l_invalid_window = 0U;
static volatile uint16_t s_encoder_r_invalid_window = 0U;
static uint8_t s_encoder_l_half_step_remainder = 0U;
static uint8_t s_encoder_r_half_step_remainder = 0U;

#define ENCODER_ISR_MAX_EVENTS 16U
/*
 * 商家资料给出每相信号为电机轴每转11个脉冲，输出轴脉冲还要乘齿轮减速比。
 * 本工程返回值是完整x4 Gray计数除以2，即保持旧版“单相双边沿”的尺度。
 * 由于当前齿轮减速比没有写入固件，不能假装精确换算RPM；这里仅设置极宽的
 * 物理跳变上限（80计数/10ms，等效VOFA速度400），只拦截毛刺尖峰，不限制
 * 目标20~100的正常速度范围。
 */
#define ENCODER_IMPLAUSIBLE_COUNT_10MS 80

static uint32_t encoder_all_pins(void)
{
    return ENCODER_Left_A_PIN | ENCODER_Left_B_PIN |
           ENCODER_Right_A_PIN | ENCODER_Right_B_PIN;
}

/* bit2诊断条件：右A与左B几乎逐边沿镜像、但右B明显更少。
 * 该模式已在2026-07-30模式二单轮数据中实测出现，说明PB16不能继续作为
 * 右轮闭环反馈。它只置诊断位，不改变已经固定选择的反馈源。 */
static uint8_t encoder_cross_channel_mirror_bad(int left_b,
                                                int right_a,
                                                int right_b)
{
    int mirror_difference = left_b - right_a;

    if (mirror_difference < 0) mirror_difference = -mirror_difference;
    if ((left_b < 4) || (right_a < 4)) return 0U;

    return (uint8_t)((mirror_difference <= 1) &&
                     ((right_b * 2 + 2) < right_a));
}

/* 标准Gray码正交跃迁表。每个合法A/B边沿产生一个有符号x4步数。 */
static const int8_t s_quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t encoder_state_from_pins(uint32_t pins,
                                       uint32_t a_pin,
                                       uint32_t b_pin)
{
    return (uint8_t)(((pins & a_pin) ? 2U : 0U) |
                     ((pins & b_pin) ? 1U : 0U));
}

/* A/B每窗边沿差异只用于健康告警，绝不用于生成速度倍率。 */
static uint8_t encoder_phase_counts_bad(int a_edges, int b_edges)
{
    int larger;
    int smaller;

    /* 健康正交编码器两相频率应接近。低速时允许边界量化误差；较大计数下，
     * 若较大相比较小相高出约1/3并超过2个边沿，就置诊断位。该告警只用于
     * VOFA，不再自动切换反馈源或触发停车。 */
    if ((a_edges < 4) && (b_edges < 4)) return 0U;
    larger = (a_edges > b_edges) ? a_edges : b_edges;
    smaller = (a_edges > b_edges) ? b_edges : a_edges;
    return (uint8_t)((larger - smaller) > (larger / 3 + 2));
}

static int encoder_abs_count(int value)
{
    return (value < 0) ? -value : value;
}

/*
 * JGB37-520的完整Gray解码是x4计数，原工程目标值和PID则按单相双边沿x2
 * 标定。除以2可保持原target/PID尺度；余数跨10ms窗口保留，避免奇数步
 * 每窗截断造成长期低估。
 */
static int encoder_gray_to_legacy_scale(int gray_count, uint8_t *remainder)
{
    uint32_t combined =
        (uint32_t)encoder_abs_count(gray_count) + (uint32_t)(*remainder);

    *remainder = (uint8_t)(combined & 1U);
    return (int)(combined >> 1U);
}

/*
 * 健康正交编码器的A/B边沿数应接近，且绝大多数中断都应形成合法Gray跃迁。
 * 合法的正转和反转都计入legal；不能用有符号净位移判断信号质量，否则电机
 * 低速起步抖动时正反步数抵消，会被误报为编码器故障。
 */
static uint8_t encoder_gray_window_warning(int a_edges,
                                           int b_edges,
                                           int legal_count,
                                           int invalid_count)
{
    int total_edges = a_edges + b_edges;
    int allowance;

    if (total_edges < 6) return 0U;
    if (encoder_phase_counts_bad(a_edges, b_edges) != 0U) return 1U;

    allowance = total_edges / 4 + 2;
    if (invalid_count > allowance) return 1U;
    return (uint8_t)((legal_count + allowance) < total_edges);
}

void Encoder_ReadAndClear(int *left_count, int *right_count)
{
    int decoded_left;
    int decoded_right;
    int left_a;
    int left_b;
    int right_a;
    int right_b;
    int left_quadrature;
    int right_quadrature;
    int left_legal;
    int right_legal;
    int left_invalid;
    int right_invalid;
    uint8_t left_quality_warning;
    uint8_t right_quality_warning;
    uint8_t left_feedback_bad;
    uint8_t right_feedback_bad;
    uint8_t cross_channel_mirror_warning;
    uint32_t irq_was_enabled;

    /* 编码器GPIO优先级高于控制定时器。这里只在快照/清零的极短临界区屏蔽，
     * 保证左右计数来自同一个10ms窗口。 */
    irq_was_enabled = NVIC_GetEnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    left_a = (int)s_encoder_l_a_edges;
    left_b = (int)s_encoder_l_b_edges;
    right_a = (int)s_encoder_r_a_edges;
    right_b = (int)s_encoder_r_b_edges;
    left_quadrature = s_encoder_l_quadrature_count;
    right_quadrature = s_encoder_r_quadrature_count;
    left_legal = (int)s_encoder_l_legal_window;
    right_legal = (int)s_encoder_r_legal_window;
    left_invalid = (int)s_encoder_l_invalid_window;
    right_invalid = (int)s_encoder_r_invalid_window;
    left_quality_warning = encoder_gray_window_warning(
        left_a, left_b, left_legal, left_invalid);
    right_quality_warning = encoder_gray_window_warning(
        right_a, right_b, right_legal, right_invalid);

    /*
     * 相位失衡/非法跃迁只作为诊断：Gray表本身已经只累计合法跃迁，不能再因
     * 一个10ms窗口边界不理想而把整窗速度清零。否则11PPR编码器起步时会在
     * 只有1~4个边沿的量化区被误锁死，表现为PWM刚爬升轮子就停。
     */
    decoded_left = encoder_gray_to_legacy_scale(
        left_quadrature, &s_encoder_l_half_step_remainder);
    decoded_right = encoder_gray_to_legacy_scale(
        right_quadrature, &s_encoder_r_half_step_remainder);

    /*
     * 只有无法由电机物理速度解释的尖峰或GPIO中断风暴才判反馈无效。缺相时
     * Gray净计数会趋近0，后级“高PWM+持续低速”的堵转保护负责安全停车。
     */
    left_feedback_bad =
        (uint8_t)(decoded_left > ENCODER_IMPLAUSIBLE_COUNT_10MS);
    right_feedback_bad =
        (uint8_t)(decoded_right > ENCODER_IMPLAUSIBLE_COUNT_10MS);
    if (encoder_irq_storm_latched != 0U) {
        left_feedback_bad = 1U;
        right_feedback_bad = 1U;
    }
    if (left_feedback_bad != 0U) decoded_left = 0;
    if (right_feedback_bad != 0U) decoded_right = 0;

    /* bit2描述PCB物理脚PB13/PB16的关系，必须在可选的逻辑左右映射交换前锁存。 */
    cross_channel_mirror_warning =
        encoder_cross_channel_mirror_bad(left_b, right_a, right_b);

    if (encoder_mapping_swapped != 0U) {
        int temporary = decoded_left;
        decoded_left = decoded_right;
        decoded_right = temporary;
        temporary = left_a; left_a = right_a; right_a = temporary;
        temporary = left_b; left_b = right_b; right_b = temporary;
        temporary = left_quadrature;
        left_quadrature = right_quadrature;
        right_quadrature = temporary;
        temporary = left_invalid;
        left_invalid = right_invalid;
        right_invalid = temporary;
        {
            uint8_t temporary_warning = left_quality_warning;
            uint8_t temporary_bad = left_feedback_bad;
            left_quality_warning = right_quality_warning;
            right_quality_warning = temporary_warning;
            left_feedback_bad = right_feedback_bad;
            right_feedback_bad = temporary_bad;
        }
    }

    encoder_l_a_edges_window = left_a;
    encoder_l_b_edges_window = left_b;
    encoder_r_a_edges_window = right_a;
    encoder_r_b_edges_window = right_b;
    encoder_l_quadrature_window = left_quadrature;
    encoder_r_quadrature_window = right_quadrature;
    encoder_phase_warning_mask =
        (encoder_phase_counts_bad(left_a, left_b) ? 1U : 0U) |
        (encoder_phase_counts_bad(right_a, right_b) ? 2U : 0U) |
        (cross_channel_mirror_warning ? 4U : 0U) |
        (left_quality_warning ? 8U : 0U) |
        (right_quality_warning ? 16U : 0U) |
        ((encoder_irq_storm_latched != 0U) ? 32U : 0U) |
        (left_feedback_bad ? 64U : 0U) |
        (right_feedback_bad ? 128U : 0U);
    encoder_feedback_valid_mask =
        (left_feedback_bad ? 0U : 1U) |
        (right_feedback_bad ? 0U : 2U);

    if (left_count != 0) *left_count = decoded_left;
    if (right_count != 0) *right_count = decoded_right;
    /* 速度和里程只消费经过合法正交质量检查的同尺度计数。 */
    encoder_l_total_edges += (uint32_t)decoded_left;
    encoder_r_total_edges += (uint32_t)decoded_right;

    _encoder_l_count = 0;
    _encoder_r_count = 0;
    s_encoder_l_a_edges = 0U;
    s_encoder_l_b_edges = 0U;
    s_encoder_r_a_edges = 0U;
    s_encoder_r_b_edges = 0U;
    s_encoder_l_quadrature_count = 0;
    s_encoder_r_quadrature_count = 0;
    s_encoder_l_legal_window = 0U;
    s_encoder_r_legal_window = 0U;
    s_encoder_l_invalid_window = 0U;
    s_encoder_r_invalid_window = 0U;

    if (irq_was_enabled != 0U) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    }
}

void Encoder_Clear(void)
{
    uint32_t irq_was_enabled;

    Encoder_ReadAndClear(0, 0);
    if (encoder_irq_storm_latched != 0U) {
        Encoder_EnableInterrupts();
    }

    irq_was_enabled = NVIC_GetEnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    encoder_phase_warning_mask = 0U;
    encoder_feedback_valid_mask = 0x03U;
    if (irq_was_enabled != 0U) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    }
}

void Encoder_ResetDiagnostics(void)
{
    uint32_t state;
    uint32_t irq_was_enabled =
        NVIC_GetEnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    _encoder_l_count = 0;
    _encoder_r_count = 0;
    s_encoder_l_a_edges = 0U;
    s_encoder_l_b_edges = 0U;
    s_encoder_r_a_edges = 0U;
    s_encoder_r_b_edges = 0U;
    s_encoder_l_quadrature_count = 0;
    s_encoder_r_quadrature_count = 0;
    s_encoder_l_legal_window = 0U;
    s_encoder_r_legal_window = 0U;
    s_encoder_l_invalid_window = 0U;
    s_encoder_r_invalid_window = 0U;
    s_encoder_l_half_step_remainder = 0U;
    s_encoder_r_half_step_remainder = 0U;
    encoder_l_a_edges_window = 0;
    encoder_l_b_edges_window = 0;
    encoder_r_a_edges_window = 0;
    encoder_r_b_edges_window = 0;
    encoder_l_quadrature_window = 0;
    encoder_r_quadrature_window = 0;
    encoder_phase_warning_mask = 0U;
    encoder_feedback_valid_mask = 0x03U;
    encoder_l_total_edges = 0U;
    encoder_r_total_edges = 0U;
    encoder_l_invalid_transitions = 0U;
    encoder_r_invalid_transitions = 0U;
    encoder_irq_storm_latched = 0U;
    encoder_irq_overrun_count = 0U;
    state = ENCODER_PORT->DIN31_0;
    s_encoder_l_previous_state = encoder_state_from_pins(
        state, ENCODER_Left_A_PIN, ENCODER_Left_B_PIN);
    s_encoder_r_previous_state = encoder_state_from_pins(
        state, ENCODER_Right_A_PIN, ENCODER_Right_B_PIN);
    if (irq_was_enabled != 0U) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    }
}

void Encoder_SetChannelMapping(uint8_t swapped, uint8_t valid)
{
    uint32_t irq_was_enabled =
        NVIC_GetEnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    encoder_mapping_swapped = (swapped != 0U) ? 1U : 0U;
    encoder_mapping_valid = (valid != 0U) ? 1U : 0U;
    _encoder_l_count = 0;
    _encoder_r_count = 0;
    s_encoder_l_a_edges = 0U;
    s_encoder_l_b_edges = 0U;
    s_encoder_r_a_edges = 0U;
    s_encoder_r_b_edges = 0U;
    s_encoder_l_quadrature_count = 0;
    s_encoder_r_quadrature_count = 0;
    s_encoder_l_legal_window = 0U;
    s_encoder_r_legal_window = 0U;
    s_encoder_l_invalid_window = 0U;
    s_encoder_r_invalid_window = 0U;
    s_encoder_l_half_step_remainder = 0U;
    s_encoder_r_half_step_remainder = 0U;
    encoder_l_a_edges_window = 0;
    encoder_l_b_edges_window = 0;
    encoder_r_a_edges_window = 0;
    encoder_r_b_edges_window = 0;
    encoder_l_quadrature_window = 0;
    encoder_r_quadrature_window = 0;
    encoder_phase_warning_mask = 0U;
    encoder_feedback_valid_mask = 0x03U;
    if (irq_was_enabled != 0U) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    }
}

void Encoder_InitQuadrature(void)
{
    uint32_t all_encoder_pins = encoder_all_pins();

    /* PB1/PB13属于低16位，PB16/PB17属于高16位；四路都启用双边沿，
     * 才能用完整A/B状态表判断合法跃迁和识别缺相/噪声。 */
    DL_GPIO_setLowerPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_1_EDGE_RISE_FALL | DL_GPIO_PIN_13_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_16_EDGE_RISE_FALL | DL_GPIO_PIN_17_EDGE_RISE_FALL);
    /* 8个总线周期数字滤波抑制电机电刷/PWM造成的亚微秒毛刺，不会削弱
     * 330RPM编码器的有效脉冲。 */
    DL_GPIO_setLowerPinsInputFilter(
        ENCODER_PORT,
        DL_GPIO_PIN_1_INPUT_FILTER_8_CYCLES |
        DL_GPIO_PIN_13_INPUT_FILTER_8_CYCLES);
    DL_GPIO_setUpperPinsInputFilter(
        ENCODER_PORT,
        DL_GPIO_PIN_16_INPUT_FILTER_8_CYCLES |
        DL_GPIO_PIN_17_INPUT_FILTER_8_CYCLES);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, all_encoder_pins);
    DL_GPIO_enableInterrupt(ENCODER_PORT, all_encoder_pins);
    Encoder_ResetDiagnostics();
}

void Encoder_EnableInterrupts(void)
{
    uint32_t all_encoder_pins = encoder_all_pins();
    uint32_t state;

    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    DL_GPIO_disableInterrupt(ENCODER_PORT, all_encoder_pins);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, all_encoder_pins);

    /* 丢弃初始化/故障期间的旧相位，从当前真实A/B状态重新开始。 */
    state = ENCODER_PORT->DIN31_0;
    s_encoder_l_previous_state = encoder_state_from_pins(
        state, ENCODER_Left_A_PIN, ENCODER_Left_B_PIN);
    s_encoder_r_previous_state = encoder_state_from_pins(
        state, ENCODER_Right_A_PIN, ENCODER_Right_B_PIN);
    s_encoder_l_a_edges = 0U;
    s_encoder_l_b_edges = 0U;
    s_encoder_r_a_edges = 0U;
    s_encoder_r_b_edges = 0U;
    s_encoder_l_quadrature_count = 0;
    s_encoder_r_quadrature_count = 0;
    s_encoder_l_legal_window = 0U;
    s_encoder_r_legal_window = 0U;
    s_encoder_l_invalid_window = 0U;
    s_encoder_r_invalid_window = 0U;
    encoder_irq_storm_latched = 0U;
    encoder_feedback_valid_mask = 0x03U;

    DL_GPIO_enableInterrupt(ENCODER_PORT, all_encoder_pins);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
}

static void encoder_update_left(uint8_t is_a)
{
    uint32_t state = ENCODER_PORT->DIN31_0;
    uint8_t current = encoder_state_from_pins(
        state, ENCODER_Left_A_PIN, ENCODER_Left_B_PIN);
    int8_t step = s_quadrature_table[
        (s_encoder_l_previous_state << 2U) | current];

    if (is_a != 0U) {
        s_encoder_l_a_edges++;
    } else {
        s_encoder_l_b_edges++;
    }
    encoder_l_a_level = (current >> 1U) & 1U;
    encoder_l_b_level = current & 1U;
    if (step != 0) {
        s_encoder_l_quadrature_count += step;
        s_encoder_l_legal_window++;
    } else {
        encoder_l_invalid_transitions++;
        s_encoder_l_invalid_window++;
    }
    s_encoder_l_previous_state = current;
}

static void encoder_update_right(uint8_t is_a)
{
    uint32_t state = ENCODER_PORT->DIN31_0;
    uint8_t current = encoder_state_from_pins(
        state, ENCODER_Right_A_PIN, ENCODER_Right_B_PIN);
    int8_t step = s_quadrature_table[
        (s_encoder_r_previous_state << 2U) | current];

    if (is_a != 0U) {
        s_encoder_r_a_edges++;
    } else {
        s_encoder_r_b_edges++;
    }
    encoder_r_a_level = (current >> 1U) & 1U;
    encoder_r_b_level = current & 1U;
    if (step != 0) {
        s_encoder_r_quadrature_count += step;
        s_encoder_r_legal_window++;
    } else {
        encoder_r_invalid_transitions++;
        s_encoder_r_invalid_window++;
    }
    s_encoder_r_previous_state = current;
}

static void encoder_function2(void)
{
    uint8_t handled;

    /*
     * IIDX每次读取会只应答一个最高优先级GPIO事件；循环排空可避免MIS快照
     * 把多个边沿合成一次。读取IIDX后禁止再clear RIS，否则会竞态清掉刚到的
     * 下一边沿。
     */
    for (handled = 0U; handled < ENCODER_ISR_MAX_EVENTS; handled++) {
        DL_GPIO_IIDX pending = DL_GPIO_getPendingInterrupt(ENCODER_PORT);

        switch (pending) {
        case DL_GPIO_IIDX_DIO1:
            encoder_update_left(1U);
            break;
        case DL_GPIO_IIDX_DIO13:
            encoder_update_left(0U);
            break;
        case DL_GPIO_IIDX_DIO16:
            encoder_update_right(1U);
            break;
        case DL_GPIO_IIDX_DIO17:
            encoder_update_right(0U);
            break;
        case DL_GPIO_IIDX_NO_INTR:
            return;
        default:
            /* 共享GPIOB上的其他事件已由IIDX应答；编码器模块不处理它。 */
            break;
        }
    }

    /*
     * 正常330RPM运行时一次ISR通常只有1个事件。单次连续16个仍排不空说明
     * 输入毛刺或电平异常；必须退出ISR并关闭编码器中断，不能饿死主循环。
     * 下一次进入控制模式时Encoder_Clear会重新对齐并尝试恢复。
     */
    encoder_irq_storm_latched = 1U;
    encoder_irq_overrun_count++;
    encoder_feedback_valid_mask = 0U;
    DL_GPIO_disableInterrupt(ENCODER_PORT, encoder_all_pins());
    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
}

/**
 * @brief 处理中断组1的中断服务函数
 *
 * 该函数用于处理中断组1中的中断。根据中断组1中待处理的中断，确定中断源，并执行相应的处理函数。
 * 不接受任何参数，也不返回任何值。
 */
void GROUP1_IRQHandler(void)
{
     
    // 确定中断源并进行相应的处理
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
    case DL_INTERRUPT_GROUP1_IIDX_GPIOB: // 当中断源为编码器中断时
    {
        /*
         * GPIOB is shared by encoder edges and nRF24 IRQ. The radio hook only
         * clears PB14 and sets a flag; all SPI work stays in foreground.
         */
        encoder_function2();
    }
    break;
//    case GPIO_MPU_INT_IIDX: // 当中断源为GPIO或MPU中断时
//    {
//        // 从MPU获取姿态数据
////        mpu_dmp_get_data(&pitch, &roll, &yaw);
////        MPU_Get_Gyroscope(&gx, &gy, &gz);
//    }
//    break;
    default: // 对于其他中断源，不进行处理
        break;
    }
}
