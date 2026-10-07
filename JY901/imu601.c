#include "imu601.h"

/*
 * IMU601_Service() is called every 50 ms by the menu pages. Startup and
 * recovery must remain non-blocking because the IMU is optional at boot.
 */
#define IMU601_RESET_SETTLE_SERVICE_TICKS 10U /* 10 * 50 ms = 500 ms */
#define IMU601_FRAME_TIMEOUT_SERVICE_TICKS 40U /* 40 * 50 ms = 2 s */
#define IMU601_MAX_AUTO_RETRIES          3U
#define IMU601_TX_WAIT_SPINS        200000U
#define IMU601_UART_ERROR_MASK                                           \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                              \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                                \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                              \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

/* Parser state is written only by the UART3 ISR. */
static uint8_t s_rx_buffer[12] = {0};
static uint8_t s_rx_index = 0U;
static uint8_t s_last_byte = 0U;
static uint8_t s_is_receiving = 0U;

/* Raw angles avoid slow software floating-point work inside the UART ISR. */
static volatile uint16_t s_yaw_raw = 0U;
static volatile int16_t s_pitch_raw = 0;
static volatile int16_t s_roll_raw = 0;
static volatile uint32_t s_raw_bytes = 0U;
static volatile uint32_t s_header_hits = 0U;
static volatile uint32_t s_valid_frames = 0U;
static volatile uint32_t s_checksum_errors = 0U;
static volatile uint32_t s_uart_errors = 0U;
static volatile uint8_t s_last_bytes[4] = {0U, 0U, 0U, 0U};

static volatile IMU601_State s_init_state = IMU601_STATE_OFF;
static uint8_t s_service_ticks = 0U;
static volatile uint8_t s_retry_count = 0U;
static uint32_t s_ready_after_frame = 0U;

static uint8_t uart_wait_idle_bounded(UART_Regs *uart)
{
    uint32_t remaining = IMU601_TX_WAIT_SPINS;

    while (DL_UART_isBusy(uart) == true) {
        if (remaining == 0U) {
            return 0U;
        }
        remaining--;
    }
    return 1U;
}

static uint8_t uart_send_buffer_bounded(UART_Regs *uart,
                                        const uint8_t *buf,
                                        uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++) {
        if (uart_wait_idle_bounded(uart) == 0U) {
            return 0U;
        }
        DL_UART_Main_transmitData(uart, buf[i]);
    }
    return uart_wait_idle_bounded(uart);
}

static void imu601_enable_receiver(void)
{
    DL_UART_Main_enable(IMU601_INST);
    DL_UART_Main_enableInterrupt(IMU601_INST,
        DL_UART_MAIN_INTERRUPT_RX | IMU601_UART_ERROR_MASK);
    NVIC_SetPriority(IMU601_INST_INT_IRQN, 3U);
    NVIC_ClearPendingIRQ(IMU601_INST_INT_IRQN);
    NVIC_EnableIRQ(IMU601_INST_INT_IRQN);
}

static uint8_t imu601_send_reset(void)
{
    static const uint8_t imu_reset[] =
        {0xAAU, 0x55U, 0x60U, 0x12U, 0x00U, 0x72U};

    if (uart_send_buffer_bounded(IMU601_INST, imu_reset,
                                 (uint8_t)sizeof(imu_reset)) == 0U) {
        s_init_state = IMU601_STATE_TX_FAULT;
        return 0U;
    }

    s_service_ticks = 0U;
    s_ready_after_frame = s_valid_frames;
    s_init_state = IMU601_STATE_RESET_WAIT;
    return 1U;
}

void IMU601_Restart(void)
{
    uint8_t discarded;

    /*
     * SysConfig owns UART3 baud and pin configuration. Disable only its NVIC
     * line while stale RX data and the partial parser state are discarded.
     */
    NVIC_DisableIRQ(IMU601_INST_INT_IRQN);
    while (DL_UART_Main_isRXFIFOEmpty(IMU601_INST) == false) {
        discarded = DL_UART_Main_receiveData(IMU601_INST);
        (void)discarded;
    }

    s_rx_index = 0U;
    s_last_byte = 0U;
    s_is_receiving = 0U;
    s_service_ticks = 0U;
    s_retry_count = 0U;
    s_ready_after_frame = s_valid_frames;

    DL_UART_Main_clearInterruptStatus(IMU601_INST,
        DL_UART_MAIN_INTERRUPT_RX | IMU601_UART_ERROR_MASK);
    imu601_enable_receiver();
    (void)imu601_send_reset();
}

void IMU601_init(void)
{
    /*
     * Re-arm RX every time an IMU mode is entered. This recovers if another
     * startup path left UART3 configured but its NVIC line masked.
     */
    imu601_enable_receiver();

    if ((s_init_state == IMU601_STATE_OFF) ||
        (s_init_state == IMU601_STATE_TX_FAULT) ||
        (s_init_state == IMU601_STATE_NO_FRAME)) {
        IMU601_Restart();
    }
}

void IMU601_Service(void)
{
    static const uint8_t imu_cali[] =
        {0xAAU, 0x55U, 0x60U, 0x14U, 0x04U,
         0xCDU, 0xCCU, 0xB3U, 0x43U, 0x07U};

    if (s_init_state == IMU601_STATE_RESET_WAIT) {
        if (s_service_ticks < IMU601_RESET_SETTLE_SERVICE_TICKS) {
            s_service_ticks++;
            return;
        }

        if (uart_send_buffer_bounded(IMU601_INST, imu_cali,
                                     (uint8_t)sizeof(imu_cali)) == 0U) {
            s_init_state = IMU601_STATE_TX_FAULT;
            return;
        }

        /* READY requires a complete frame received after calibration. */
        s_ready_after_frame = s_valid_frames;
        s_service_ticks = 0U;
        s_init_state = IMU601_STATE_WAIT_FRAME;
        return;
    }

    if (s_init_state == IMU601_STATE_WAIT_FRAME) {
        if (s_valid_frames != s_ready_after_frame) {
            s_init_state = IMU601_STATE_READY;
            return;
        }

        if (s_service_ticks < IMU601_FRAME_TIMEOUT_SERVICE_TICKS) {
            s_service_ticks++;
            return;
        }

        if (s_retry_count < IMU601_MAX_AUTO_RETRIES) {
            s_retry_count++;
            (void)imu601_send_reset();
        } else {
            s_init_state = IMU601_STATE_NO_FRAME;
        }
    }
}

uint8_t IMU601_IsReady(void)
{
    return (uint8_t)(s_init_state == IMU601_STATE_READY);
}

uint8_t IMU601_GetInitFault(void)
{
    return (uint8_t)((s_init_state == IMU601_STATE_TX_FAULT) ||
                     (s_init_state == IMU601_STATE_NO_FRAME));
}

static void parse_imu601_data(void)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 2U; i < 11U; i++) {
        checksum = (uint8_t)(checksum + s_rx_buffer[i]);
    }

    if (checksum == s_rx_buffer[11]) {
        /* Publish all three raw axes only after the frame is validated. */
        s_yaw_raw = ((uint16_t)s_rx_buffer[6] << 8) |
                    (uint16_t)s_rx_buffer[5];
        s_pitch_raw = (int16_t)(((uint16_t)s_rx_buffer[8] << 8) |
                                (uint16_t)s_rx_buffer[7]);
        s_roll_raw = (int16_t)(((uint16_t)s_rx_buffer[10] << 8) |
                               (uint16_t)s_rx_buffer[9]);
        s_valid_frames++;

        if ((s_init_state == IMU601_STATE_WAIT_FRAME) &&
            (s_valid_frames != s_ready_after_frame)) {
            s_init_state = IMU601_STATE_READY;
        }
    } else {
        s_checksum_errors++;
    }
}

void IMU601_GetSnapshot(IMU601_Snapshot_t *snapshot)
{
    uint32_t interrupt_state;
    uint16_t yaw_raw;
    int16_t pitch_raw;
    int16_t roll_raw;

    if (snapshot == 0) {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    yaw_raw = s_yaw_raw;
    pitch_raw = s_pitch_raw;
    roll_raw = s_roll_raw;
    snapshot->raw_bytes = s_raw_bytes;
    snapshot->header_hits = s_header_hits;
    snapshot->valid_frames = s_valid_frames;
    snapshot->checksum_errors = s_checksum_errors;
    snapshot->uart_errors = s_uart_errors;
    snapshot->retry_count = s_retry_count;
    snapshot->last_bytes[0] = s_last_bytes[0];
    snapshot->last_bytes[1] = s_last_bytes[1];
    snapshot->last_bytes[2] = s_last_bytes[2];
    snapshot->last_bytes[3] = s_last_bytes[3];
    snapshot->state = s_init_state;
    if (interrupt_state == 0U) {
        __enable_irq();
    }

    /* Software floating-point conversion runs outside the UART ISR. */
    snapshot->attitude.yaw = (float)yaw_raw / 100.0f;
    snapshot->attitude.pitch = (float)pitch_raw / 100.0f;
    snapshot->attitude.roll = (float)roll_raw / 100.0f;
}

static void imu601_process_rx_byte(uint8_t recv)
{
    s_raw_bytes++;
    s_last_bytes[0] = s_last_bytes[1];
    s_last_bytes[1] = s_last_bytes[2];
    s_last_bytes[2] = s_last_bytes[3];
    s_last_bytes[3] = recv;

    if (s_is_receiving == 0U) {
        if ((recv == 0x55U) && (s_last_byte == 0xAAU)) {
            s_rx_buffer[0] = 0xAAU;
            s_rx_buffer[1] = 0x55U;
            s_rx_index = 2U;
            s_is_receiving = 1U;
            s_header_hits++;
        }
        s_last_byte = recv;
        return;
    }

    /*
     * Consume exactly one 12-byte frame after accepting a header. AA 55 is
     * legal inside payload and must not restart the parser mid-frame.
     */
    if (s_rx_index < sizeof(s_rx_buffer)) {
        s_rx_buffer[s_rx_index++] = recv;
    } else {
        s_rx_index = 0U;
        s_is_receiving = 0U;
    }

    s_last_byte = recv;
    if (s_rx_index >= sizeof(s_rx_buffer)) {
        s_rx_index = 0U;
        s_is_receiving = 0U;
        parse_imu601_data();
    }
}

void IMU601_INST_IRQHandler(void)
{
    uint32_t error_status = DL_UART_Main_getRawInterruptStatus(
        IMU601_INST, IMU601_UART_ERROR_MASK);

    if (error_status != 0U) {
        s_uart_errors++;
        DL_UART_Main_clearInterruptStatus(IMU601_INST, error_status);
    }

    /*
     * Drain every available byte. This works with FIFO disabled and remains
     * correct if the SysConfig RX FIFO is enabled later.
     */
    while (DL_UART_Main_isRXFIFOEmpty(IMU601_INST) == false) {
        imu601_process_rx_byte(DL_UART_Main_receiveData(IMU601_INST));
    }
}
