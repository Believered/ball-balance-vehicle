#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

extern volatile int _encoder_l_count;
extern volatile int _encoder_r_count;

/* 编码器诊断量：累计经过正交质量检查的反馈计数，以及最近一次A/B电平。 */
extern volatile uint32_t encoder_l_total_edges;
extern volatile uint32_t encoder_r_total_edges;
extern volatile uint8_t encoder_l_a_level;
extern volatile uint8_t encoder_l_b_level;
extern volatile uint8_t encoder_r_a_level;
extern volatile uint8_t encoder_r_b_level;
extern volatile uint32_t encoder_l_invalid_transitions;
extern volatile uint32_t encoder_r_invalid_transitions;
/* 最近10ms逻辑通道诊断：原始A/B边沿和有符号x4 Gray净计数。 */
extern volatile int encoder_l_a_edges_window;
extern volatile int encoder_l_b_edges_window;
extern volatile int encoder_r_a_edges_window;
extern volatile int encoder_r_b_edges_window;
extern volatile int encoder_l_quadrature_window;
extern volatile int encoder_r_quadrature_window;
/* bit0=左A/B失衡，bit1=右A/B失衡，bit2=跨通道串扰，
 * bit3=左Gray窗口质量告警，bit4=右Gray窗口质量告警，bit5=中断风暴，
 * bit6/bit7=左/右物理不可能速度样本。
 * bit0~4只用于诊断，不会因11PPR编码器低速量化直接停车。 */
extern volatile uint8_t encoder_phase_warning_mask;
/* bit0=左反馈有效，bit1=右反馈有效；仅物理不可能尖峰/中断风暴会清除。 */
extern volatile uint8_t encoder_feedback_valid_mask;
/* ISR无法在16个事件内排空时锁存，防止GPIO毛刺饿死主循环。 */
extern volatile uint8_t encoder_irq_storm_latched;
extern volatile uint32_t encoder_irq_overrun_count;
extern volatile uint8_t encoder_mapping_swapped;
extern volatile uint8_t encoder_mapping_valid;

/**
 * @brief 原子读取本测速窗口内的左右编码器计数并清零。
 *
 * 返回最近10ms内合法x4 Gray净计数绝对值的一半，保持原单相双边沿标度。
 * 该接口由固定周期控制中断调用，让速度和里程使用同一份编码器快照。
 */
void Encoder_ReadAndClear(int *left_count, int *right_count);

/**
 * @brief 清除尚未消费的编码器计数。
 *
 * 每次开始或退出控制模式时调用，防止上一次运行残留的脉冲造成启动偏转。
 */
void Encoder_Clear(void);

/** 清除测速窗口和累计边沿诊断计数，用于模式一重新观察编码器。 */
void Encoder_ResetDiagnostics(void);

/** 在SysConfig之后启用两路A/B全边沿中断并装载正交解码初态。 */
void Encoder_InitQuadrature(void);

/** 清除初始化期间的旧边沿，从当前A/B状态重新对齐并使能编码器中断。 */
void Encoder_EnableInterrupts(void);

/** 设置电机左右通道与编码器左右插座是否需要交换；同时清除残留计数。 */
void Encoder_SetChannelMapping(uint8_t swapped, uint8_t valid);

#endif
