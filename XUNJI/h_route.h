#ifndef H_ROUTE_H_
#define H_ROUTE_H_

#include <stdint.h>
#include "xunji.h"

/*
 * H题第三问实车绝对航向。以A->B为0度，角度按IMU601的0..360度顺时针增加。
 * 本题实际只使用A->C和B->D；D->B、C->A保留为装车方向校验值。
 */
#define H_ROUTE_HEADING_AB_DEG   (0.0f)
#define H_ROUTE_HEADING_AC_DEG  (38.0f)
#define H_ROUTE_HEADING_BD_DEG (132.0f)
#define H_ROUTE_HEADING_DB_DEG (320.0f)
#define H_ROUTE_HEADING_CA_DEG (224.0f)

/* 圆弧实际出圈切线角，和出圈后的直线路径角必须分开设置。
 * C->B先以180°离开圆弧，再切到B->D的132°；D->A先以0°离开圆弧，
 * 再切到A->C/最终停车姿态的38°。 */
#define H_ROUTE_ARC_CB_EXIT_YAW_DEG (180.0f)
#define H_ROUTE_ARC_DA_EXIT_YAW_DEG   (0.0f)

/*
 * 航向环输出的是“左右轮目标速度差”，不是PWM。初值刻意保守，避免把参考
 * 工程直接输出PWM的Kp=7错误套入本车已经闭环的速度环。
 */
extern float h_heading_kp;
extern float h_heading_kd;

typedef enum {
    H_ROUTE_STATE_IDLE = 0,
    H_ROUTE_STATE_STRAIGHT_AC,
    H_ROUTE_STATE_ARC_CB,
    H_ROUTE_STATE_STRAIGHT_BD,
    H_ROUTE_STATE_ARC_DA,
    H_ROUTE_STATE_FINISHED,
    H_ROUTE_STATE_FAULT,
    H_ROUTE_STATE_ALIGN_A
} HRouteState;

typedef enum {
    H_ROUTE_FAULT_NONE = 0,
    H_ROUTE_FAULT_IMU_NOT_READY,
    H_ROUTE_FAULT_START_HEADING,
    H_ROUTE_FAULT_IMU_STALE,
    H_ROUTE_FAULT_STRAIGHT_TIMEOUT,
    H_ROUTE_FAULT_ARC_LINE_LOST,
    H_ROUTE_FAULT_ROUTE_TIMEOUT,
    H_ROUTE_FAULT_STALL_LEFT,
    H_ROUTE_FAULT_STALL_RIGHT,
    H_ROUTE_FAULT_OVERSPEED_LEFT,
    H_ROUTE_FAULT_OVERSPEED_RIGHT,
    H_ROUTE_FAULT_ARC_NOT_ACQUIRED,
    H_ROUTE_FAULT_FINAL_ALIGN_TIMEOUT
} HRouteFault;

typedef struct {
    HRouteState state;
    HRouteFault fault;
    XunjiSensor sensor;
    float yaw;
    float target_yaw;
    float heading_error;
    int16_t target_left;
    int16_t target_right;
    int16_t correction;
    int32_t segment_distance;
    int32_t total_distance;
    uint16_t state_ticks;
    uint8_t point_index;       /* 0=A起点，1=C，2=B，3=D，4=A终点。 */
    uint8_t imu_stale_ticks;
    uint8_t arc_lost_ticks;    /* 圆弧连续全白时间，10ms/计数。 */
    int8_t search_direction;   /* -1=向左找线，+1=向右找线，0=不搜索。 */
    uint8_t completed_laps;    /* 已完整经过 C、B、D、A 的圈数。 */
    uint8_t target_laps;       /* 目标圈数，限制为1..5。 */
    uint8_t arc_line_acquired; /* 0=刚入弧寻线，1=曾稳定捕获弧线。 */
    uint8_t arc_exit_armed;    /* 1=圆弧已运行满1.5s；开始允许全白30ms判终点。 */
} HRouteStatus;

/*
 * Mode 5 uses the very same position PID parameters and timing as the arc
 * tracker in mode 4, but deliberately omits the H-route state transitions.
 * This makes the measured Kp/Ki/Kd directly transferable to mode 4.
 */
typedef struct {
    XunjiSensor sensor;
    int16_t filtered_position;
    int16_t correction;
    int16_t target_left;
    int16_t target_right;
    uint8_t running;
    uint8_t waiting_line;
    HRouteFault fault;
} HPositionTuneStatus;

/*
 * 2026 H题第二问标定参数。
 * 赛道顺时针运行，AB/CD为直线，BC/DA为右转半圆。
 * 模式七和模式八统一使用模式二已经整定的左右轮速度PID。
 */
#define H2_CAL_BASE_SPEED          60
#define H2_CAL_TIME_STEP_MS        25U
#define H2_CAL_LINE_WEIGHT_STEP_X10 1
#define H2_CAL_CURVE_DELTA_STEP     1
#define H2_CAL_CURVE_ANGLE_DEG    180.0f

/*
 * 模式一：第二问最终综合控制参数。
 *
 * 每段前两个里程取两次实测平均值，时间使用用户最新实测值。
 * 时间仍是主进度，里程只作为阶段切换门槛，避免编码器离散误差改变
 * 已经由模式七验证成功的赛道轨迹。权重50表示5.0。
 */
#define H2_FINAL_AB_TIME_MS                 3350U
#define H2_FINAL_BC_TIME_MS                 4800U
#define H2_FINAL_CD_TIME_MS                 3425U
#define H2_FINAL_DA_TIME_MS                 3625U
#define H2_FINAL_AB_DISTANCE_COUNTS          3699
#define H2_FINAL_BC_DISTANCE_COUNTS          4682
#define H2_FINAL_CD_DISTANCE_COUNTS          3622
#define H2_FINAL_DA_DISTANCE_COUNTS          4733
#define H2_FINAL_LINE_WEIGHT_X10               50
#define H2_FINAL_CURVE_DELTA                    21
#define H2_FINAL_DISTANCE_GATE_PERCENT         85U
#define H2_FINAL_STRAIGHT_EXTENSION_MS         500U
#define H2_FINAL_CURVE_EXTENSION_MS            800U
#define H2_FINAL_ROUTE_TIMEOUT_MARGIN_MS      3000U
#define H2_FINAL_TIME_STEP_MS                   25U
#define H2_FINAL_LINE_WEIGHT_MIN                 0
#define H2_FINAL_LINE_WEIGHT_MAX                99
#define H2_FINAL_LINE_WEIGHT_STEP                1

typedef enum {
    H2_CAL_STAGE_IDLE = 0,
    H2_CAL_STAGE_AB,
    H2_CAL_STAGE_BC,
    H2_CAL_STAGE_CD,
    H2_CAL_STAGE_DA,
    H2_CAL_STAGE_DONE,
    H2_CAL_STAGE_FAULT
} H2CalStage;

typedef enum {
    H2_CAL_FAULT_NONE = 0,
    /*
     * 1~3原为IMU未就绪/陈旧/数据异常。模式一、七、八现已改为自动降级，
     * 不再把IMU通信瞬态作为停车故障；编号留空，避免现有F04~F11含义变化。
     * F04仍供模式七/八测试超时使用，模式一超过补偿窗口会按时间切换下一段。
     */
    H2_CAL_FAULT_TIMEOUT = 4,
    H2_CAL_FAULT_STALL_LEFT,
    H2_CAL_FAULT_STALL_RIGHT,
    H2_CAL_FAULT_OVERSPEED_LEFT,
    H2_CAL_FAULT_OVERSPEED_RIGHT,
    H2_CAL_FAULT_ENCODER_LEFT,
    H2_CAL_FAULT_ENCODER_RIGHT,
    H2_CAL_FAULT_LINE_LOST
} H2CalFault;

typedef struct {
    int32_t segment_counts[4]; /* AB, BC, CD, DA */
    int32_t current_counts;
    uint8_t segment_index;     /* 0=AB, 1=BC, 2=CD, 3=DA */
    uint8_t completed;
    uint8_t active;
} H2MileageStatus;

typedef struct {
    H2CalStage stage;
    H2CalFault fault;
    XunjiSensor sensor;
    uint32_t stage_time_ms[4];
    uint32_t stage_elapsed_ms;
    uint32_t total_elapsed_ms;
    int16_t target_left;
    int16_t target_right;
    int16_t line_correction;
    int16_t imu_correction;
    int16_t curve_delta;
    int16_t filtered_position;
    int16_t line_weight_x10;
    int32_t stage_distance_target[4];
    int32_t stage_distance;
    int32_t total_distance;
    float yaw;
    float yaw_progress;
    float yaw_rate;
    uint8_t running;
    uint8_t waiting_imu;
    uint8_t imu_available;
    uint8_t time_ready;
    uint8_t distance_ready;
    uint8_t angle_ready;
    uint8_t segment_index;
    uint8_t timing_complete;
} H2MotionStatus;

/** 初始化/清除模式四状态，不启动电机。 */
void HRoute_Init(void);

/**
 * 从A点启动第三问。为避免起步大转弯，车头必须预先对准A->C约38度。
 * 返回1表示已启动；返回0表示IMU或初始朝向不满足安全条件。
 */
uint8_t HRoute_Start(void);

/** 设置模式四目标圈数；小于1按1处理，大于5按5处理。 */
void HRoute_SetLapTarget(uint8_t laps);

/** 人工停止或离开菜单；使用工程统一的50ms短路急停。 */
void HRoute_Stop(void);

/** 10ms状态机：灰度采样、IMU健康检查和路段切换。 */
void HRoute_Tick10ms(void);

/** 10ms控制：里程累计、航向/循迹规划、速度PI和电机输出。 */
void HRoute_Control10ms(int encoder_left,
                        int encoder_right,
                        int measured_left,
                        int measured_right);

/** 原子获取OLED/VOFA只读快照。 */
void HRoute_GetStatus(HRouteStatus *status);

/** Mode 5: start/stop a continuous line-following position-PID test. */
void HPositionTune_Start(void);
void HPositionTune_Stop(void);

/** 10 ms speed loop; the position PID itself is updated every 20 ms. */
void HPositionTune_Control10ms(int measured_left, int measured_right);

/** Atomic read-only snapshot for OLED and VOFA. */
void HPositionTune_GetStatus(HPositionTuneStatus *status);

/* 模式六：架空/断开驱动后手推小车，依次记录AB、BC、CD、DA编码器里程。 */
void H2Mileage_Start(void);
void H2Mileage_Stop(void);
void H2Mileage_Reset(void);
void H2Mileage_MarkPoint(void);
void H2Mileage_Control10ms(int encoder_left, int encoder_right);
void H2Mileage_GetStatus(H2MileageStatus *status);

/* 模式七：四段时间状态机；时间步长25ms，循迹权重步长0.1。 */
void H2Timed_SetStageTime(uint8_t stage_index, uint32_t time_ms);
uint32_t H2Timed_GetStageTime(uint8_t stage_index);
void H2Timed_SetLineWeightX10(int weight_x10);
int H2Timed_GetLineWeightX10(void);
uint8_t H2Timed_Start(void);

/* 模式一：实测时间、里程门槛、IMU和可实时调整的整数循迹权重。 */
void H2Final_SetStageTime(uint8_t stage_index, uint32_t time_ms);
uint32_t H2Final_GetStageTime(uint8_t stage_index);
void H2Final_SetLineWeight(int weight);
int H2Final_GetLineWeight(void);
uint8_t H2Final_Start(void);

/* 模式八：固定180度右弯标定；差速单位是轮速目标，不是PWM。 */
void H2Curve_SetDelta(int delta);
int H2Curve_GetDelta(void);
uint8_t H2Curve_Start(void);

/* 模式七/八共用的停止、10ms调度和状态快照。 */
void H2Motion_Stop(void);
void H2Motion_Tick10ms(void);
void H2Motion_Control10ms(int encoder_left,
                          int encoder_right,
                          int measured_left,
                          int measured_right);
void H2Motion_GetStatus(H2MotionStatus *status);

/*
 * 2026 H题第四/第五问滚球平稳控制。
 *
 * 第四问只执行AB直线；第五问执行AB、BC、CD、DA一整圈。两种模式都使用
 * 模式二整定好的左右速度PI，但明确关闭速度前馈，并在目标速度与PWM两层做
 * 斜率限制。启动时先同步克服左右轮不同的静摩擦，再共同加速；正常结束使用
 * 平滑减速后高阻释放，人工停止/故障仍快速安全停机。
 *
 * 下列宏是后续实车标定时最常修改的参数。菜单中的时间、弯道差速和循迹权重
 * 都有独立RAM副本，修改模式三不会污染模式四。
 */
#define H_BALL_BASE_SPEED                    40

/*
 * 模式三/四以及模式二双轮曲线测试共用的启动参数。
 *
 * 修改方法：
 * - 启动仍太慢：减小 START_RAMP_TIME_MS，建议不要低于600ms；
 * - 一轮先跑偏：减小 SYNC_CRAWL_SPEED，或增大 SYNC_CONFIRM_TICKS；
 * - 某轮长期不起步：增大 PWM_LIMIT/PWM_SLEW_STEP前先确认机械和供电；
 * - PWM_LIMIT只限制滚球模式和模式二的同步曲线，不改变电机驱动最终1000硬限幅。
 */
#define H_BALL_CONTROL_PERIOD_MS             10U
#define H_BALL_START_RAMP_TIME_MS          2000U
#define H_BALL_STOP_RAMP_TIME_MS           4000U
#define H_BALL_SYNC_CRAWL_SPEED              10
#define H_BALL_SYNC_MOVING_SPEED              5
#define H_BALL_SYNC_HOLD_SPEED                4
#define H_BALL_SYNC_CONFIRM_TICKS             3U
#define H_BALL_SYNC_TIMEOUT_MS             2500U
#define H_BALL_TARGET_SLEW_STEP               1
#define H_BALL_PWM_SLEW_STEP                  8
#define H_BALL_PWM_LIMIT                     800
/*
 * 堵转保护阈值随滚球运行限幅联动，避免降低PWM上限后永远达不到固定阈值。
 * 90%只用于确认“控制器已经接近全力但车轮仍不动”，不改变实际PWM上限。
 */
#define H_BALL_STALL_PWM_THRESHOLD \
    ((H_BALL_PWM_LIMIT * 9) / 10)

#define H_BALL_TIME_STEP_MS                  25U
#define H_BALL_WEIGHT_STEP_X10                1
#define H_BALL_CURVE_DELTA_STEP               1

/* 第四问：7200ms为AB标称时间；4000计数到点后锁存成绩并直行缓停。 */
#define H_BALL_Q4_AB_TIME_MS               7200U
#define H_BALL_Q4_BC_TIME_MS               7000U
#define H_BALL_Q4_CD_TIME_MS               5925U
#define H_BALL_Q4_DA_TIME_MS               6650U
#define H_BALL_Q4_CURVE_DELTA                14
#define H_BALL_Q4_LINE_WEIGHT_X10            20

/*
 * 第五问：四段时间、弯道差速和循迹权重使用实车标定值。菜单只保留AB、
 * DEL和WGT，BC/CD/DA继续作为整圈状态机的内部固定参数。
 */
#define H_BALL_Q5_AB_TIME_MS               7000U
#define H_BALL_Q5_BC_TIME_MS               6650U
#define H_BALL_Q5_CD_TIME_MS               5700U
#define H_BALL_Q5_DA_TIME_MS               7100U
#define H_BALL_Q5_CURVE_DELTA                15
#define H_BALL_Q5_LINE_WEIGHT_X10            30

/*
 * Distance uses the same legacy encoder-count scale as H2 mileage
 * calibration. Q4 freezes its timer at B. Q5 freezes at the completed lap,
 * cruises past A on AB, and only then starts the smooth stop.
 */
#define H_BALL_Q4_AB_DISTANCE_COUNTS        4000
#define H_BALL_Q5_LAP_DISTANCE_COUNTS \
    (H2_FINAL_AB_DISTANCE_COUNTS + H2_FINAL_BC_DISTANCE_COUNTS + \
     H2_FINAL_CD_DISTANCE_COUNTS + H2_FINAL_DA_DISTANCE_COUNTS)
#define H_BALL_POST_POINT_CRUISE_MS         2000U
#define H_BALL_POST_POINT_TIMEOUT_MS        3500U

typedef enum {
    H_BALL_ROUTE_Q4 = 4,
    H_BALL_ROUTE_Q5 = 5
} HBallRouteMode;

typedef enum {
    H_BALL_PHASE_ROUTE = 0,
    H_BALL_PHASE_POST_POINT_CRUISE,
    H_BALL_PHASE_POST_POINT_DECEL,
    H_BALL_PHASE_STOPPING
} HBallRunPhase;

typedef enum {
    H_BALL_LAUNCH_SYNC = 0,
    H_BALL_LAUNCH_RAMP,
    H_BALL_LAUNCH_DONE
} HBallLaunchPhase;

/*
 * 双轮同步软启动状态。active_mask bit0=左轮、bit1=右轮。
 * ready_mask会锁存已经稳定转动的轮，防止快轮在等待慢轮时反复退出确认。
 */
typedef struct {
    uint16_t phase_ticks;
    uint8_t left_confirm_ticks;
    uint8_t right_confirm_ticks;
    uint8_t active_mask;
    uint8_t ready_mask;
    uint8_t timeout_mask;
    HBallLaunchPhase phase;
    int16_t target_speed;
    int16_t target_left;
    int16_t target_right;
} HBallLaunchState;

typedef struct {
    HBallRouteMode mode;
    HBallRunPhase phase;
    H2CalStage stage;
    H2CalFault fault;
    XunjiSensor sensor;
    uint32_t stage_time_ms[4];
    uint32_t stage_elapsed_ms;
    uint32_t total_elapsed_ms;
    int32_t stage_distance;
    int32_t total_distance;
    int32_t post_point_distance;
    int32_t task_distance_target;
    int16_t base_speed;
    int16_t target_left;
    int16_t target_right;
    int16_t pwm_left;
    int16_t pwm_right;
    int16_t line_correction;
    int16_t imu_correction;
    int16_t curve_delta;
    int16_t line_weight_x10;
    int16_t filtered_position;
    float yaw;
    float yaw_progress;
    float yaw_rate;
    uint8_t running;
    uint8_t normal_stop_pending;
    uint8_t timing_complete;
    uint8_t imu_available;
    uint8_t segment_index;
} HBallRouteStatus;

void HBallLaunch_Reset(HBallLaunchState *state);
int HBallLaunch_Update(HBallLaunchState *state,
                       int final_speed,
                       int measured_left,
                       int measured_right,
                       uint8_t active_mask);
uint8_t HBallLaunch_IsComplete(const HBallLaunchState *state);
uint8_t HBallLaunch_GetTimeoutMask(const HBallLaunchState *state);

void HBall_SetStageTime(HBallRouteMode mode,
                        uint8_t stage_index,
                        uint32_t time_ms);
uint32_t HBall_GetStageTime(HBallRouteMode mode, uint8_t stage_index);
void HBall_SetCurveDelta(HBallRouteMode mode, int delta);
int HBall_GetCurveDelta(HBallRouteMode mode);
void HBall_SetLineWeightX10(HBallRouteMode mode, int weight_x10);
int HBall_GetLineWeightX10(HBallRouteMode mode);
uint8_t HBall_Start(HBallRouteMode mode);
void HBall_Abort(void);
void HBall_Tick10ms(int encoder_left, int encoder_right);
void HBall_Control10ms(int measured_left, int measured_right);
void HBall_GetStatus(HBallRouteStatus *status);

#endif
