#ifndef IMU601_H
#define IMU601_H

#include "ti_msp_dl_config.h"
#include "board.h"

typedef enum {
    IMU601_STATE_OFF = 0,
    IMU601_STATE_RESET_WAIT,
    IMU601_STATE_WAIT_FRAME,
    IMU601_STATE_READY,
    IMU601_STATE_TX_FAULT,
    IMU601_STATE_NO_FRAME
} IMU601_State;

void IMU601_init(void);
void IMU601_Restart(void);
void IMU601_Service(void);
uint8_t IMU601_IsReady(void);
uint8_t IMU601_GetInitFault(void);

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Attitude_t;

/**
 * @brief IMU601 接收状态快照。
 *
 * 姿态数据由 UART3 中断更新，菜单只能通过快照接口读取，避免主循环分别读取
 * yaw/pitch/roll 时恰好跨越一帧，导致三个角度不属于同一个采样时刻。
 */
typedef struct {
    Attitude_t attitude;
    uint32_t raw_bytes;
    uint32_t header_hits;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t uart_errors;
    uint8_t retry_count;
    uint8_t last_bytes[4];
    IMU601_State state;
} IMU601_Snapshot_t;

/** 原子复制最近一帧姿态和接收诊断计数；尚未收到有效帧时 valid_frames 为 0。 */
void IMU601_GetSnapshot(IMU601_Snapshot_t *snapshot);

#endif /* IMU601_H */
