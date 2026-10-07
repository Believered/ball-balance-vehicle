#ifndef XUNJI_H_
#define XUNJI_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 使用工程现有 SysConfig 灰度引脚，顺序为左外到右外。 */
#define GPIO_XUNJI_PORT      (Huidu_PORT)
#define GPIO_XUNJI_L4_PIN    (Huidu_L4_PIN)
#define GPIO_XUNJI_L3_PIN    (Huidu_L3_PIN)
#define GPIO_XUNJI_L2_PIN    (Huidu_L2_PIN)
#define GPIO_XUNJI_L1_PIN    (Huidu_L1_PIN)
#define GPIO_XUNJI_R1_PIN    (Huidu_R1_PIN)
#define GPIO_XUNJI_R2_PIN    (Huidu_R2_PIN)
#define GPIO_XUNJI_R3_PIN    (Huidu_R3_PIN)
#define GPIO_XUNJI_R4_PIN    (Huidu_R4_PIN)

enum {
    XUNJI_BIT_L4 = (1U << 0),
    XUNJI_BIT_L3 = (1U << 1),
    XUNJI_BIT_L2 = (1U << 2),
    XUNJI_BIT_L1 = (1U << 3),
    XUNJI_BIT_R1 = (1U << 4),
    XUNJI_BIT_R2 = (1U << 5),
    XUNJI_BIT_R3 = (1U << 6),
    XUNJI_BIT_R4 = (1U << 7)
};

typedef enum {
    XUNJI_STATE_IDLE = 0,
    XUNJI_STATE_TRACK,
    XUNJI_STATE_CORNER_LEFT,
    XUNJI_STATE_LOST,
    XUNJI_STATE_FINISHED,
    XUNJI_STATE_FAULT,
    /* 追加状态，避免改变既有 VOFA 状态编号。 */
    XUNJI_STATE_CORNER_RIGHT,
    XUNJI_STATE_CORNER_ADVANCE
} XunjiState;

typedef enum {
    XUNJI_FAULT_NONE = 0,
    XUNJI_FAULT_STALL_LEFT,
    XUNJI_FAULT_STALL_RIGHT,
    XUNJI_FAULT_OVERSPEED_LEFT,
    XUNJI_FAULT_OVERSPEED_RIGHT,
    XUNJI_FAULT_CORNER_TIMEOUT,
    XUNJI_FAULT_LINE_LOST_TIMEOUT,
    XUNJI_FAULT_SENSOR_PATTERN,
    XUNJI_FAULT_ROUTE_TIMEOUT
} XunjiFault;

/* 模式一对应2026 H题第二问：起点横线释放、连续循迹、终点横线确认。 */
typedef enum {
    XUNJI_H2_PHASE_WAIT_LINE = 0,
    XUNJI_H2_PHASE_START_MARKER,
    XUNJI_H2_PHASE_TRACK,
    XUNJI_H2_PHASE_LOST,
    XUNJI_H2_PHASE_FINISH_CONFIRM,
    XUNJI_H2_PHASE_STOP_OFFSET
} XunjiH2Phase;

typedef enum {
    XUNJI_STRAIGHT_IDLE = 0,
    XUNJI_STRAIGHT_RUN,
    XUNJI_STRAIGHT_POST_CRUISE,
    XUNJI_STRAIGHT_DECEL,
    XUNJI_STRAIGHT_DONE
} XunjiStraightPhase;

#define XUNJI_STRAIGHT_DISTANCE_COUNTS       4000
#define XUNJI_MODE5_POST_CRUISE_MS           5000U
#define XUNJI_MODE6_POST_CRUISE_MS           1200U

typedef struct {
    uint8_t bits;          /* 检测到轨迹的通道位图，bit0=L4 ... bit7=R4 */
    uint8_t active_count;  /* 当前检测到轨迹的通道数 */
    uint8_t valid;         /* 至少一个通道检测到轨迹 */
    int16_t position;      /* 与旧控制器兼容的 100..800 加权位置 */
    int16_t error;         /* position - 450；负值表示轨迹在左侧 */
} XunjiSensor;

typedef struct {
    XunjiState state;
    XunjiFault fault;
    XunjiSensor sensor;
    int16_t control_position; /* 位置环使用的低通位置，原始sensor仍用于状态识别。 */
    int16_t control_correction; /* 正值表示减小左轮目标。 */
    uint8_t target_laps;
    uint8_t completed_laps;
    uint8_t corners_in_lap;
    uint16_t progress_permille;
    int16_t target_left;
    int16_t target_right;
    int32_t total_distance;
    int32_t estimated_side;
    uint8_t line_missing_ticks; /* 连续全白确认计数，10ms/计数。 */
    uint8_t lost_ticks;         /* LOST累计计数，10ms/计数。 */
    int8_t search_direction;    /* -1=向左搜索，+1=向右搜索。 */
    uint8_t velocity_bridge_windows; /* 出弯前馈桥接剩余的50ms窗口数。 */
    uint8_t velocity_reprime_pending; /* 1=下一拍速度PI将执行无扰重装。 */
    uint8_t h2_phase;           /* 2026 H题第二问阶段，见 XunjiH2Phase。 */
    uint8_t h2_finish_armed;    /* 1=已离开起点横线，允许识别终点横线。 */
    uint8_t h2_finish_pending;  /* 1=已确认终点横线，正在走停车标定偏移。 */
    uint8_t h2_marker_ticks;    /* 横线连续确认计数，10ms/计数。 */
    uint32_t elapsed_ms;        /* 从按下启动到停车/故障的冻结计时。 */
    uint8_t straight_phase;
    uint8_t timing_complete;
    int32_t task_distance_target;
} XunjiStatus;

/* 兼容现有菜单和调试界面的公共量。 */
extern volatile float xun;
extern volatile int xun_pwm_l;
extern volatile int xun_pwm_r;

uint8_t my_GPIO_readPin(GPIO_Regs *gpio, uint32_t pins);
void xunji_shua_xin(void);
int xunji_biao_zhi(void);

/** 读取一次原始8路灰度快照，供独立模式四复用现有引脚极性和权重。 */
void Xunji_ReadSensorSnapshot(XunjiSensor *sensor);

/** 初始化循迹控制器，保持停车状态。 */
void Xunji_ControlInit(void);

/**
 * 连续位置环调试：禁用角点/圈数/终点识别；丢线后按最后见线方向持续搜线，
 * 仅手动退出或电机堵转、超速等严重执行器故障才停车。
 */
void Xunji_StartDirectionTune(void);

/** Start a finite mileage task using the mode-5 position-PID follower. */
void Xunji_StartStraightTask(int32_t distance_target,
                             uint32_t post_cruise_ms);

/** Start pure line following and brake immediately at the configured time. */
void Xunji_StartTimedTask(uint32_t run_time_ms);

void Xunji_SetTimedLineWeightX10(int weight_x10);
int Xunji_GetTimedLineWeightX10(void);

/**
 * 模式一高速循迹测试：固定独立速度80，使用独立速度/位置PID。
 * 不识别终点、不按圈数停车，仅按键人工停止；严重执行器和丢线故障保护仍保留。
 */
void Xunji_StartMode1Test(void);

/** 模式一：启动2026 H题第二问，沿黑线顺时针一圈并在A点横线平滑停车。 */
void Xunji_StartH2Lap(void);

/** 立即退出循迹并清理 PID、编码器和状态机历史量。 */
void Xunji_Stop(void);

/** 10ms 固定周期：采集灰度，按短丢线恢复/长丢线左右转状态机规划目标轮速。 */
void Xunji_Tick10ms(void);

/** 50ms 固定周期：累计里程并更新圈数定位。 */
void Xunji_UpdateEncoder(int left_delta, int right_delta);

/** 10ms 固定周期：执行位置修正、左右轮速度 PI 并输出 PWM。 */
void Xunji_SpeedControl(int measured_left, int measured_right);

/** 获取供 OLED、串口和后续视觉同步使用的只读状态快照。 */
void Xunji_GetStatus(XunjiStatus *status);

#endif
