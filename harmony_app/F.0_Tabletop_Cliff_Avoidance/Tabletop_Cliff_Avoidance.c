/*
 * Phase F: tabletop cliff avoidance on the Task24 UART2 protocol link.
 *
 * Infrared sensors:
 *   GPIO13 = left, GPIO14 = right
 *   raw low  = reflecting surface detected = floor present
 *   raw high = no reflecting surface       = possible cliff
 *
 * Motor link:
 *   GPIO11 UART2_TXD -> STM32 PA10 USART1_RX
 *   GPIO12 UART2_RXD <- STM32 PA9  USART1_TX
 *   115200 8N1, AA | CMD | LEN | PAYLOAD | CHECK
 *
 * Telemetry: GET_STATUS polls during RECOVER return STATUS frames carrying
 * STM32 cumulative encoder odometry; recovery turns close the loop on
 * differential pulses instead of blind timing (RECOVER_TIMEOUT_TICKS is
 * the fail-safe if telemetry stalls).
 *
 * Safety: motion is disabled by default. Set PHASE_F_ENABLE_MOTION to 1
 * only for an attended test with the battery, a clear table, and a stop
 * switch or hand ready. Sensor failure, stale data, and failed recovery
 * always result in STOP.
 */
#include <stdio.h>
#include <stdint.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_uart_ex.h"
#include "hi_time.h"

/* Keep this 0 until the static and wheels-off-ground checks are complete. */
#define PHASE_F_ENABLE_MOTION             0

#define PROTOCOL_SOF                      0xAAU
#define MAX_PAYLOAD                       16U
#define PROTOCOL_FRAME_MAX                (MAX_PAYLOAD + 4U)
#define PROTOCOL_CMD_SET_SPEED            0x01U
#define PROTOCOL_CMD_STOP                 0x02U
#define PROTOCOL_CMD_PING                 0x03U
#define PROTOCOL_CMD_GET_STATUS           0x04U
#define PROTOCOL_CMD_ACK                  0x81U
#define PROTOCOL_CMD_STATUS               0x82U

/* STATUS payload: odo_left i32 LE | odo_right i32 LE |
   speed_left i16 LE | speed_right i16 LE | flags u8 (bit0 = motion lease). */
#define STATUS_PAYLOAD_LEN                13U

#define PROTOCOL_STATUS_OK                0U
#define PROTOCOL_STATUS_BAD_LENGTH        1U
#define PROTOCOL_STATUS_BAD_CHECKSUM      2U
#define PROTOCOL_STATUS_INVALID_PARAM     3U
#define PROTOCOL_STATUS_UNKNOWN_CMD       4U

#define PHASE_F_THREAD_STACK_SIZE         4096U
#define PHASE_F_THREAD_PRIORITY           25
#define ACK_READ_BUFFER_SIZE              64U
#define ACK_POLL_DELAY_TICKS              1U

/* Hi3861 uses a 10 ms RTOS tick in this project. */
#define SENSOR_SAMPLE_TICKS               5U   /* 50 ms */
#define SENSOR_DANGER_DEBOUNCE_SAMPLES    1U   /* edge detection must be fast */
#define SENSOR_SAFE_DEBOUNCE_SAMPLES      2U   /* 100 ms stable floor before recovery */
#define CONTROL_LOOP_TICKS                1U   /* 10 ms response to a new snapshot */
#define COMMAND_RESEND_TICKS              10U  /* 100 ms, below STM32's 300 ms lease */
#define SENSOR_STALE_TICKS                12U  /* 120 ms without a fresh sample => STOP */
#define STARTUP_GUARD_TICKS               100U /* 1 s initial STOP guard */

#define DRIVE_SPEED                       140
#define REVERSE_SPEED                     140
#define TURN_SPEED                        140

/* All action times are measured in 10 ms RTOS ticks. */
#define EDGE_BRAKE_TICKS                  10U  /* STOP before reversing: 100 ms */
#define EDGE_REVERSE_MIN_TICKS            50U  /* 500 ms fallback when no safe mileage (boot near edge) */
#define EDGE_REVERSE_MAX_TICKS            200U /* 2.0 s cap ~= 14 cm of known-safe ground */
#define RECOVER_BRAKE_TICKS               10U  /* STOP before turning: 100 ms */
/* Recovery turns are closed-loop on STM32 odometry reported over STATUS:
   target = angle x differential pulses per degree (|dLeft| + |dRight|). */
#define TURN_PULSES_PER_DEG               24U  /* 8-29 desktop telemetry calibration: 7860p/330deg, 9548p/400deg (~7cm track) */
#define RECOVER_TURN_BASE_DEG             180U /* U-turn away from the edge */
#define RECOVER_TURN_JITTER_DEG           30U  /* +/-30 deg exploration jitter around the U-turn */
#define RECOVER_TIMEOUT_TICKS             400U /* 4.0 s fail-safe cap if STATUS telemetry stalls */
#define STATUS_POLL_TICKS                 10U  /* 100 ms GET_STATUS cadence during RECOVER */
#define RECOVER_SETTLE_TICKS              10U  /* 100 ms */
#define PHASE_F_MAX_RECOVER_RETRIES       3U
#define BOTH_EDGE_TURNS_RIGHT             1

#define IR_LEFT_PIN                       WIFI_IOT_IO_NAME_GPIO_13
#define IR_RIGHT_PIN                      WIFI_IOT_IO_NAME_GPIO_14

/* The SDK provides this symbol even when older headers omit the declaration. */
extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

typedef enum {
    RX_WAIT_SOF = 0,
    RX_CMD,
    RX_LEN,
    RX_PAYLOAD,
    RX_CHECK
} ProtocolRxState;

typedef struct {
    ProtocolRxState state;
    uint8_t cmd;
    uint8_t len;
    uint8_t index;
    uint8_t checksum;
    uint8_t payload[MAX_PAYLOAD];
} ProtocolRxParser;

typedef struct {
    uint8_t candidate;
    uint8_t stable;
    uint8_t count;
    uint8_t ready;
} SensorDebounce;

typedef struct {
    uint8_t raw_left;
    uint8_t raw_right;
    uint8_t floor_left;
    uint8_t floor_right;
    uint8_t ready;
    uint8_t healthy;
    uint32_t updated_tick;
    uint32_t sequence;
} SensorSnapshot;

typedef enum {
    PHASE_F_NORMAL = 0,
    PHASE_F_EDGE_LEFT,
    PHASE_F_EDGE_RIGHT,
    PHASE_F_EDGE_BOTH,
    PHASE_F_RECOVER,
    PHASE_F_FAULT_STOP
} PhaseFState;

typedef enum {
    TURN_LEFT = 0,
    TURN_RIGHT
} TurnDirection;

typedef struct {
    PhaseFState state;
    TurnDirection recover_turn;
    uint32_t recover_target_pulses; /* differential pulses to accumulate for the turn */
    uint8_t recover_motion_done;    /* pulse target reached: stop driving the spin */
    uint8_t odo_base_valid;
    int32_t odo_base_left;
    int32_t odo_base_right;
    uint32_t odo_base_seq;          /* status sequence when the base was requested */
    uint32_t last_status_poll_tick;
    uint32_t state_enter_tick;
    uint8_t armed;
    uint8_t active;
    uint8_t fault_latched;
    uint8_t recover_retries;
    uint32_t safe_forward_ticks;  /* 10 ms beats of forward driving since arm/last recovery */
    uint32_t reverse_ticks;       /* computed at edge entry: clamp(safe_forward_ticks, MIN, MAX) */
} PhaseFController;

typedef struct {
    int16_t left;
    int16_t right;
    uint8_t stop;
} MotionCommand;

typedef struct {
    int32_t odo_left;
    int32_t odo_right;
    int16_t speed_left;
    int16_t speed_right;
    uint8_t flags;
    uint32_t sequence;
    uint32_t updated_tick;
} StatusSnapshot;

static osMutexId_t sensor_mutex;
static SensorSnapshot sensor_snapshot;
static volatile uint8_t sensor_reset_requested;

static osMutexId_t status_mutex;
static StatusSnapshot status_snapshot;

static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t i;
    uint8_t checksum = (uint8_t)(cmd + len);

    for (i = 0U; i < len; i++) {
        checksum = (uint8_t)(checksum + payload[i]);
    }
    return checksum;
}

static uint8_t protocol_encode_frame(uint8_t cmd, uint8_t len, const uint8_t *payload,
    uint8_t *frame, uint8_t capacity)
{
    uint8_t i;
    uint8_t frame_len = (uint8_t)(len + 4U);

    if (frame == NULL || len > MAX_PAYLOAD || capacity < frame_len) {
        return 0U;
    }
    if (len != 0U && payload == NULL) {
        return 0U;
    }

    frame[0] = PROTOCOL_SOF;
    frame[1] = cmd;
    frame[2] = len;
    for (i = 0U; i < len; i++) {
        frame[3U + i] = payload[i];
    }
    frame[3U + len] = protocol_checksum(cmd, len, payload);
    return frame_len;
}

static int protocol_send_frame(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t frame[PROTOCOL_FRAME_MAX];
    uint8_t frame_len = protocol_encode_frame(cmd, len, payload, frame, sizeof(frame));
    int written;

    if (frame_len == 0U) {
        printf("[PhaseF] frame encode failed: cmd=0x%02X len=%u\r\n", cmd, len);
        return -1;
    }

    written = UartWrite(WIFI_IOT_UART_IDX_2, frame, frame_len);
    if (written != frame_len) {
        printf("[PhaseF] UART2 write failed: cmd=0x%02X wrote=%d/%u\r\n",
            cmd, written, frame_len);
        return -1;
    }
    return 0;
}

static int protocol_send_ping(void)
{
    return protocol_send_frame(PROTOCOL_CMD_PING, 0U, NULL);
}

static int protocol_send_stop(void)
{
    return protocol_send_frame(PROTOCOL_CMD_STOP, 0U, NULL);
}

static int protocol_send_get_status(void)
{
    return protocol_send_frame(PROTOCOL_CMD_GET_STATUS, 0U, NULL);
}

static int32_t decode_i32_le(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

static int16_t decode_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/* Wrap-safe odometer delta: two's-complement subtraction stays correct
   across an int32 wrap as long as the real delta is below 2^31 pulses. */
static int32_t status_odo_delta(int32_t now, int32_t base)
{
    return (int32_t)((uint32_t)now - (uint32_t)base);
}

static void status_publish(const uint8_t *payload)
{
    if (osMutexAcquire(status_mutex, osWaitForever) != osOK) {
        return;
    }
    status_snapshot.odo_left = decode_i32_le(&payload[0]);
    status_snapshot.odo_right = decode_i32_le(&payload[4]);
    status_snapshot.speed_left = decode_i16_le(&payload[8]);
    status_snapshot.speed_right = decode_i16_le(&payload[10]);
    status_snapshot.flags = payload[12];
    status_snapshot.updated_tick = hi_get_tick();
    status_snapshot.sequence++;
    osMutexRelease(status_mutex);
}

static StatusSnapshot status_take_snapshot(void)
{
    StatusSnapshot snapshot = {0};

    if (osMutexAcquire(status_mutex, osWaitForever) != osOK) {
        return snapshot;
    }
    snapshot = status_snapshot;
    osMutexRelease(status_mutex);
    return snapshot;
}

static int protocol_send_speed(int16_t left, int16_t right)
{
    uint8_t payload[4];
    uint16_t left_raw = (uint16_t)left;
    uint16_t right_raw = (uint16_t)right;

    payload[0] = (uint8_t)(left_raw & 0xFFU);
    payload[1] = (uint8_t)((left_raw >> 8) & 0xFFU);
    payload[2] = (uint8_t)(right_raw & 0xFFU);
    payload[3] = (uint8_t)((right_raw >> 8) & 0xFFU);
    return protocol_send_frame(PROTOCOL_CMD_SET_SPEED, sizeof(payload), payload);
}

static const char *protocol_command_name(uint8_t cmd)
{
    switch (cmd) {
        case PROTOCOL_CMD_SET_SPEED:
            return "SET_SPEED";
        case PROTOCOL_CMD_STOP:
            return "STOP";
        case PROTOCOL_CMD_PING:
            return "PING";
        case PROTOCOL_CMD_GET_STATUS:
            return "GET_STATUS";
        case PROTOCOL_CMD_ACK:
            return "ACK";
        case PROTOCOL_CMD_STATUS:
            return "STATUS";
        default:
            return "UNKNOWN_CMD";
    }
}

static const char *protocol_status_name(uint8_t status)
{
    switch (status) {
        case PROTOCOL_STATUS_OK:
            return "OK";
        case PROTOCOL_STATUS_BAD_LENGTH:
            return "BAD_LENGTH";
        case PROTOCOL_STATUS_BAD_CHECKSUM:
            return "BAD_CHECKSUM";
        case PROTOCOL_STATUS_INVALID_PARAM:
            return "INVALID_PARAM";
        case PROTOCOL_STATUS_UNKNOWN_CMD:
            return "UNKNOWN_CMD";
        default:
            return "UNKNOWN_STATUS";
    }
}

static void protocol_rx_reset(ProtocolRxParser *parser)
{
    parser->state = RX_WAIT_SOF;
    parser->cmd = 0U;
    parser->len = 0U;
    parser->index = 0U;
    parser->checksum = 0U;
}

static void protocol_rx_start(ProtocolRxParser *parser)
{
    parser->state = RX_CMD;
    parser->len = 0U;
    parser->index = 0U;
    parser->checksum = 0U;
}

static void protocol_handle_received_frame(const ProtocolRxParser *parser)
{
    uint8_t original_cmd;
    uint8_t status;

    if (parser->cmd == PROTOCOL_CMD_STATUS) {
        if (parser->len != STATUS_PAYLOAD_LEN) {
            printf("[PhaseF] STATUS invalid length: %u\r\n", parser->len);
            return;
        }
        status_publish(parser->payload); /* silent: polled at 10 Hz during RECOVER */
        return;
    }
    if (parser->cmd != PROTOCOL_CMD_ACK) {
        printf("[PhaseF] RX ignored: cmd=0x%02X len=%u\r\n", parser->cmd, parser->len);
        return;
    }
    if (parser->len != 2U) {
        printf("[PhaseF] ACK invalid length: %u\r\n", parser->len);
        return;
    }

    original_cmd = parser->payload[0];
    status = parser->payload[1];
    printf("[PhaseF] ACK %s (0x%02X): %s (%u)\r\n",
        protocol_command_name(original_cmd), original_cmd,
        protocol_status_name(status), status);
}

static void protocol_parse_byte(ProtocolRxParser *parser, uint8_t byte)
{
    switch (parser->state) {
        case RX_WAIT_SOF:
            if (byte == PROTOCOL_SOF) {
                protocol_rx_start(parser);
            }
            break;

        case RX_CMD:
            parser->cmd = byte;
            parser->checksum = byte;
            parser->state = RX_LEN;
            break;

        case RX_LEN:
            parser->len = byte;
            parser->checksum = (uint8_t)(parser->checksum + byte);
            if (parser->len > MAX_PAYLOAD) {
                printf("[PhaseF] RX bad length: %u\r\n", parser->len);
                protocol_rx_reset(parser);
                if (byte == PROTOCOL_SOF) {
                    protocol_rx_start(parser);
                }
            } else if (parser->len == 0U) {
                parser->state = RX_CHECK;
            } else {
                parser->index = 0U;
                parser->state = RX_PAYLOAD;
            }
            break;

        case RX_PAYLOAD:
            parser->payload[parser->index++] = byte;
            parser->checksum = (uint8_t)(parser->checksum + byte);
            if (parser->index >= parser->len) {
                parser->state = RX_CHECK;
            }
            break;

        case RX_CHECK:
            if (byte == parser->checksum) {
                protocol_handle_received_frame(parser);
            } else {
                printf("[PhaseF] RX checksum error: cmd=0x%02X\r\n", parser->cmd);
            }
            protocol_rx_reset(parser);
            if (byte == PROTOCOL_SOF) {
                protocol_rx_start(parser);
            }
            break;

        default:
            protocol_rx_reset(parser);
            break;
    }
}

static void phase_f_ack_thread(void *arg)
{
    uint8_t buffer[ACK_READ_BUFFER_SIZE];
    ProtocolRxParser parser;

    (void)arg;
    protocol_rx_reset(&parser);

    while (1) {
        unsigned char empty = 1U;
        unsigned int result = UartIsBufEmpty(WIFI_IOT_UART_IDX_2, &empty);

        if (result == WIFI_IOT_SUCCESS && empty == 0U) {
            int count = UartRead(WIFI_IOT_UART_IDX_2, buffer, sizeof(buffer));
            int i;

            if (count < 0) {
                printf("[PhaseF] UART2 read failed\r\n");
            } else {
                for (i = 0; i < count; i++) {
                    protocol_parse_byte(&parser, buffer[i]);
                }
            }
        } else if (result != WIFI_IOT_SUCCESS) {
            printf("[PhaseF] UART2 buffer check failed: %u\r\n", result);
        }
        osDelay(ACK_POLL_DELAY_TICKS);
    }
}

static void sensor_debounce_reset(SensorDebounce *filter)
{
    filter->candidate = 0U;
    filter->stable = 0U;
    filter->count = 0U;
    filter->ready = 0U;
}

static void sensor_debounce_update(SensorDebounce *filter, uint8_t sample)
{
    uint8_t required_samples = (sample == 0U) ?
        SENSOR_DANGER_DEBOUNCE_SAMPLES : SENSOR_SAFE_DEBOUNCE_SAMPLES;

    if (filter->count == 0U || filter->candidate != sample) {
        filter->candidate = sample;
        filter->count = 1U;
    } else if (filter->count < required_samples) {
        filter->count++;
    }

    if (filter->count >= required_samples) {
        filter->stable = filter->candidate;
        filter->ready = 1U;
    }
}

static void sensor_request_reset(void)
{
    if (osMutexAcquire(sensor_mutex, osWaitForever) != osOK) {
        return;
    }
    sensor_reset_requested = 1U;
    sensor_snapshot.ready = 0U;
    sensor_snapshot.healthy = 0U;
    osMutexRelease(sensor_mutex);
}

static uint8_t sensor_take_reset_request(void)
{
    uint8_t requested = 0U;

    if (osMutexAcquire(sensor_mutex, osWaitForever) != osOK) {
        return 0U;
    }
    requested = sensor_reset_requested;
    sensor_reset_requested = 0U;
    osMutexRelease(sensor_mutex);
    return requested;
}

static void sensor_publish(uint8_t raw_left, uint8_t raw_right,
    const SensorDebounce *left, const SensorDebounce *right,
    uint8_t healthy, uint32_t now)
{
    if (osMutexAcquire(sensor_mutex, osWaitForever) != osOK) {
        return;
    }

    sensor_snapshot.raw_left = raw_left;
    sensor_snapshot.raw_right = raw_right;
    sensor_snapshot.floor_left = left->stable;
    sensor_snapshot.floor_right = right->stable;
    sensor_snapshot.ready = (uint8_t)(healthy != 0U &&
        left->ready != 0U && right->ready != 0U);
    sensor_snapshot.healthy = healthy;
    sensor_snapshot.updated_tick = now;
    sensor_snapshot.sequence++;
    osMutexRelease(sensor_mutex);
}

static SensorSnapshot sensor_take_snapshot(void)
{
    SensorSnapshot snapshot = {0};

    if (osMutexAcquire(sensor_mutex, osWaitForever) != osOK) {
        return snapshot;
    }
    snapshot = sensor_snapshot;
    osMutexRelease(sensor_mutex);
    return snapshot;
}

static void phase_f_sensor_thread(void *arg)
{
    SensorDebounce left = {0};
    SensorDebounce right = {0};
    uint8_t last_floor_left = 0U;
    uint8_t last_floor_right = 0U;
    uint8_t last_logged_ready = 0U;
    uint8_t was_healthy = 0U;

    (void)arg;

    while (1) {
        WifiIotGpioValue raw_left = WIFI_IOT_GPIO_VALUE1;
        WifiIotGpioValue raw_right = WIFI_IOT_GPIO_VALUE1;
        unsigned int left_result;
        unsigned int right_result;
        uint32_t now = hi_get_tick();

        if (sensor_take_reset_request() != 0U) {
            sensor_debounce_reset(&left);
            sensor_debounce_reset(&right);
            last_logged_ready = 0U;
        }

        left_result = GpioGetInputVal(IR_LEFT_PIN, &raw_left);
        right_result = GpioGetInputVal(IR_RIGHT_PIN, &raw_right);

        if (left_result != WIFI_IOT_SUCCESS || right_result != WIFI_IOT_SUCCESS ||
            (raw_left != WIFI_IOT_GPIO_VALUE0 && raw_left != WIFI_IOT_GPIO_VALUE1) ||
            (raw_right != WIFI_IOT_GPIO_VALUE0 && raw_right != WIFI_IOT_GPIO_VALUE1)) {
            sensor_debounce_reset(&left);
            sensor_debounce_reset(&right);
            sensor_publish((uint8_t)raw_left, (uint8_t)raw_right,
                &left, &right, 0U, now);
            if (was_healthy != 0U) {
                printf("[PhaseF] infrared read failed: left=%u right=%u\r\n",
                    left_result, right_result);
            }
            was_healthy = 0U;
            last_logged_ready = 0U;
        } else {
            uint8_t floor_left = (raw_left == WIFI_IOT_GPIO_VALUE0) ? 1U : 0U;
            uint8_t floor_right = (raw_right == WIFI_IOT_GPIO_VALUE0) ? 1U : 0U;

            sensor_debounce_update(&left, floor_left);
            sensor_debounce_update(&right, floor_right);
            sensor_publish((uint8_t)raw_left, (uint8_t)raw_right,
                &left, &right, 1U, now);

            if (was_healthy == 0U) {
                printf("[PhaseF] infrared read recovered; debouncing again\r\n");
            }
            was_healthy = 1U;

            if (left.ready != 0U && right.ready != 0U &&
                (last_logged_ready == 0U || left.stable != last_floor_left ||
                 right.stable != last_floor_right)) {
                printf("[PhaseF] sensor raw L=%u R=%u, floor L=%u R=%u\r\n",
                    (uint8_t)raw_left, (uint8_t)raw_right,
                    left.stable, right.stable);
                last_floor_left = left.stable;
                last_floor_right = right.stable;
                last_logged_ready = 1U;
            }
        }
        osDelay(SENSOR_SAMPLE_TICKS);
    }
}

static const char *phase_f_state_name(PhaseFState state)
{
    switch (state) {
        case PHASE_F_NORMAL:
            return "NORMAL";
        case PHASE_F_EDGE_LEFT:
            return "EDGE_LEFT";
        case PHASE_F_EDGE_RIGHT:
            return "EDGE_RIGHT";
        case PHASE_F_EDGE_BOTH:
            return "EDGE_BOTH";
        case PHASE_F_RECOVER:
            return "RECOVER";
        case PHASE_F_FAULT_STOP:
            return "FAULT_STOP";
        default:
            return "UNKNOWN";
    }
}

static PhaseFState phase_f_edge_state(const SensorSnapshot *sensor)
{
    if (sensor->floor_left != 0U && sensor->floor_right != 0U) {
        return PHASE_F_NORMAL;
    }
    if (sensor->floor_left == 0U && sensor->floor_right != 0U) {
        return PHASE_F_EDGE_LEFT;
    }
    if (sensor->floor_left != 0U && sensor->floor_right == 0U) {
        return PHASE_F_EDGE_RIGHT;
    }
    return PHASE_F_EDGE_BOTH;
}

static void phase_f_enter_state(PhaseFController *controller, PhaseFState state,
    const SensorSnapshot *sensor, uint32_t now)
{
    PhaseFState previous = controller->state;

    controller->state = state;
    controller->state_enter_tick = now;
    printf("[PhaseF] state %s -> %s, floor L=%u R=%u\r\n",
        phase_f_state_name(previous), phase_f_state_name(state),
        sensor->floor_left, sensor->floor_right);
}

static uint32_t phase_f_rand_state;

static uint32_t phase_f_next_rand(void)
{
    /* Numerical Recipes LCG, seeded from the boot tick on first use.
       Only the control thread calls this, so no locking is needed. */
    if (phase_f_rand_state == 0U) {
        phase_f_rand_state = hi_get_tick() | 1U;
    }
    phase_f_rand_state = phase_f_rand_state * 1664525U + 1013904223U;
    return phase_f_rand_state;
}

static uint32_t phase_f_turn_pulses_with_jitter(void)
{
    int32_t jitter_deg = (int32_t)(phase_f_next_rand() %
        (2U * RECOVER_TURN_JITTER_DEG + 1U)) - (int32_t)RECOVER_TURN_JITTER_DEG;
    int32_t deg = (int32_t)RECOVER_TURN_BASE_DEG + jitter_deg;

    /* 150..210 deg: keeps a strong away-from-edge component, never parallel. */
    return (uint32_t)(deg * (int32_t)TURN_PULSES_PER_DEG);
}

static void phase_f_prepare_recover(PhaseFController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    PhaseFState edge = controller->state;

    controller->recover_target_pulses = phase_f_turn_pulses_with_jitter();
    if (edge == PHASE_F_EDGE_LEFT) {
        controller->recover_turn = TURN_RIGHT;
    } else if (edge == PHASE_F_EDGE_RIGHT) {
        controller->recover_turn = TURN_LEFT;
    } else {
#if BOTH_EDGE_TURNS_RIGHT
        controller->recover_turn = TURN_RIGHT;
#else
        controller->recover_turn = TURN_LEFT;
#endif
    }
    /* Odometry base for the closed-loop turn: captured from the first STATUS
       that arrives after this request (sequence must advance). If telemetry
       stalls, RECOVER_TIMEOUT_TICKS stops the turn as the old timed logic did. */
    controller->recover_motion_done = 0U;
    controller->odo_base_valid = 0U;
    controller->odo_base_seq = status_take_snapshot().sequence;
    controller->last_status_poll_tick = now;
    (void)protocol_send_get_status();
    printf("[PhaseF] recover turn %s, target=%lu pulses\r\n",
        controller->recover_turn == TURN_RIGHT ? "RIGHT" : "LEFT",
        (unsigned long)controller->recover_target_pulses);
    phase_f_enter_state(controller, PHASE_F_RECOVER, sensor, now);
}

static void phase_f_update_controller(PhaseFController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    uint32_t elapsed;

    if (controller->fault_latched != 0U) {
        controller->active = 0U;
        controller->armed = 0U;
        controller->state = PHASE_F_FAULT_STOP;
        return;
    }

    if (controller->armed == 0U) {
        if (sensor->floor_left != 0U && sensor->floor_right != 0U) {
            controller->armed = 1U;
            controller->active = 1U;
            controller->recover_retries = 0U;
            phase_f_enter_state(controller, PHASE_F_NORMAL, sensor, now);
            printf("[PhaseF] controller armed\r\n");
        }
        return;
    }

    if (controller->active == 0U) {
        return;
    }

    elapsed = (uint32_t)(now - controller->state_enter_tick);
    switch (controller->state) {
        case PHASE_F_NORMAL:
            if (sensor->floor_left == 0U || sensor->floor_right == 0U) {
                controller->recover_retries = 0U;
                /* Snapshot the safe forward mileage as the blind reverse
                   duration; edge-to-edge switches keep this first value. */
                controller->reverse_ticks = controller->safe_forward_ticks;
                if (controller->reverse_ticks < EDGE_REVERSE_MIN_TICKS) {
                    controller->reverse_ticks = EDGE_REVERSE_MIN_TICKS;
                }
                if (controller->reverse_ticks > EDGE_REVERSE_MAX_TICKS) {
                    controller->reverse_ticks = EDGE_REVERSE_MAX_TICKS;
                }
                controller->safe_forward_ticks = 0U;
                phase_f_enter_state(controller, phase_f_edge_state(sensor), sensor, now);
            }
            break;

        case PHASE_F_EDGE_LEFT:
        case PHASE_F_EDGE_RIGHT:
        case PHASE_F_EDGE_BOTH:
            if (controller->state == PHASE_F_EDGE_BOTH &&
                (sensor->floor_left != 0U || sensor->floor_right != 0U) &&
                (sensor->floor_left == 0U || sensor->floor_right == 0U)) {
                /* Exactly one side still unsafe: switch edge state so the
                   next turn is chosen from that side. The reverse timer
                   restarts with the same odometer-derived duration. */
                phase_f_enter_state(controller, phase_f_edge_state(sensor),
                    sensor, now);
                break;
            }
            if (controller->state != PHASE_F_EDGE_BOTH &&
                sensor->floor_left == 0U && sensor->floor_right == 0U) {
                phase_f_enter_state(controller, PHASE_F_EDGE_BOTH, sensor, now);
                break;
            }
            /* Blind reverse: near the edge the sensors report floor while
               the wheels may still be on the line, so only the safe-mileage
               timer decides when reversing stops. */
            if (elapsed >= (EDGE_BRAKE_TICKS + controller->reverse_ticks)) {
                phase_f_prepare_recover(controller, sensor, now);
            }
            break;

        case PHASE_F_RECOVER: {
            StatusSnapshot status = status_take_snapshot();
            uint32_t progress = 0U;
            uint8_t done = 0U;

            if (controller->odo_base_valid == 0U) {
                if (status.sequence != controller->odo_base_seq) {
                    controller->odo_base_left = status.odo_left;
                    controller->odo_base_right = status.odo_right;
                    controller->odo_base_valid = 1U;
                }
            } else {
                int32_t dl = status_odo_delta(status.odo_left, controller->odo_base_left);
                int32_t dr = status_odo_delta(status.odo_right, controller->odo_base_right);
                progress = (uint32_t)(dl < 0 ? -dl : dl) + (uint32_t)(dr < 0 ? -dr : dr);
            }

            if (elapsed >= RECOVER_BRAKE_TICKS && controller->odo_base_valid != 0U &&
                progress >= controller->recover_target_pulses) {
                done = 1U;
            }
            /* Fail-safe: telemetry stall falls back to a timed stop so the car
               can never spin forever; the 300 ms lease is the last resort. */
            if (elapsed >= (RECOVER_BRAKE_TICKS + RECOVER_TIMEOUT_TICKS +
                RECOVER_SETTLE_TICKS)) {
                if (done == 0U) {
                    printf("[PhaseF] recover timeout: achieved=%lu/%lu pulses (base_valid=%u)\r\n",
                        (unsigned long)progress,
                        (unsigned long)controller->recover_target_pulses,
                        controller->odo_base_valid);
                }
                done = 1U;
            }

            if (done != 0U) {
                controller->recover_motion_done = 1U;
                printf("[PhaseF] recover done: target=%lu achieved=%lu pulses\r\n",
                    (unsigned long)controller->recover_target_pulses,
                    (unsigned long)progress);
                if (sensor->floor_left != 0U && sensor->floor_right != 0U) {
                    controller->recover_retries = 0U;
                    phase_f_enter_state(controller, PHASE_F_NORMAL, sensor, now);
                } else if (controller->recover_retries < PHASE_F_MAX_RECOVER_RETRIES) {
                    controller->recover_retries++;
                    phase_f_enter_state(controller, phase_f_edge_state(sensor), sensor, now);
                } else {
                    controller->active = 0U;
                    controller->armed = 0U;
                    controller->fault_latched = 1U;
                    phase_f_enter_state(controller, PHASE_F_FAULT_STOP, sensor, now);
                    printf("[PhaseF] recovery limit reached; latched STOP\r\n");
                }
            }
            break;
        }

        case PHASE_F_FAULT_STOP:
            controller->active = 0U;
            controller->armed = 0U;
            break;

        default:
            controller->active = 0U;
            controller->armed = 0U;
            break;
    }
}

static MotionCommand phase_f_motion_command(const PhaseFController *controller,
    uint32_t now)
{
    MotionCommand command = {0, 0, 1U};
    uint32_t elapsed;

    if (controller->armed == 0U || controller->active == 0U) {
        return command;
    }

    elapsed = (uint32_t)(now - controller->state_enter_tick);
    switch (controller->state) {
        case PHASE_F_NORMAL:
            command.left = DRIVE_SPEED;
            command.right = DRIVE_SPEED;
            command.stop = 0U;
            break;

        case PHASE_F_EDGE_LEFT:
        case PHASE_F_EDGE_RIGHT:
        case PHASE_F_EDGE_BOTH:
            if (elapsed >= EDGE_BRAKE_TICKS &&
                elapsed < (EDGE_BRAKE_TICKS + controller->reverse_ticks)) {
                command.left = -REVERSE_SPEED;
                command.right = -REVERSE_SPEED;
                command.stop = 0U;
            }
            break;

        case PHASE_F_RECOVER:
            if (controller->recover_motion_done == 0U &&
                elapsed >= RECOVER_BRAKE_TICKS &&
                elapsed < (RECOVER_BRAKE_TICKS + RECOVER_TIMEOUT_TICKS)) {
                if (controller->recover_turn == TURN_RIGHT) {
                    /* Right turn: physical left wheel forward, right backward. */
                    command.left = TURN_SPEED;
                    command.right = -TURN_SPEED;
                } else {
                    command.left = -TURN_SPEED;
                    command.right = TURN_SPEED;
                }
                command.stop = 0U;
            }
            break;

        case PHASE_F_FAULT_STOP:
        default:
            break;
    }
    return command;
}

static void phase_f_send_command(const MotionCommand *command, uint32_t now)
{
    static MotionCommand previous = {0, 0, 1U};
    static uint32_t last_send_tick = 0U;
    static uint8_t sent_once = 0U;
    MotionCommand actual = *command;
    uint8_t changed;
    int result;

#if !PHASE_F_ENABLE_MOTION
    actual.left = 0;
    actual.right = 0;
    actual.stop = 1U;
#endif

    changed = (uint8_t)(sent_once == 0U || actual.stop != previous.stop ||
        actual.left != previous.left || actual.right != previous.right);
    if (changed == 0U && (uint32_t)(now - last_send_tick) < COMMAND_RESEND_TICKS) {
        return;
    }

    if (actual.stop != 0U) {
        result = protocol_send_stop();
    } else {
        result = protocol_send_speed(actual.left, actual.right);
    }

    if (result == 0) {
        previous = actual;
        last_send_tick = now;
        sent_once = 1U;
    }
}

static uint8_t phase_f_sensor_is_fresh(const SensorSnapshot *sensor, uint32_t now)
{
    if (sensor->sequence == 0U || sensor->healthy == 0U || sensor->ready == 0U) {
        return 0U;
    }
    return (uint8_t)((uint32_t)(now - sensor->updated_tick) <= SENSOR_STALE_TICKS);
}

static void phase_f_control_thread(void *arg)
{
    PhaseFController controller = {0};
    uint8_t sensor_unavailable_logged = 0U;
    uint8_t stale_logged = 0U;
    uint8_t reset_requested = 0U;
    uint32_t guard_start;

    (void)arg;
    controller.recover_turn = TURN_RIGHT;

    printf("[PhaseF] control started: motion=%d speed=%d danger_debounce=%u safe_debounce=%u\r\n",
        PHASE_F_ENABLE_MOTION, DRIVE_SPEED,
        SENSOR_DANGER_DEBOUNCE_SAMPLES, SENSOR_SAFE_DEBOUNCE_SAMPLES);
    protocol_send_ping();
    protocol_send_stop();

    guard_start = hi_get_tick();
    while ((uint32_t)(hi_get_tick() - guard_start) < STARTUP_GUARD_TICKS) {
        phase_f_send_command(&(MotionCommand){0, 0, 1U}, hi_get_tick());
        osDelay(CONTROL_LOOP_TICKS);
    }

#if !PHASE_F_ENABLE_MOTION
    printf("[PhaseF] motion disabled; all commands are forced STOP\r\n");
#endif

    while (1) {
        SensorSnapshot sensor = sensor_take_snapshot();
        uint32_t now = hi_get_tick();
        MotionCommand command = {0, 0, 1U};

        if (phase_f_sensor_is_fresh(&sensor, now) == 0U) {
            if (reset_requested == 0U) {
                sensor_request_reset();
                reset_requested = 1U;
            }
            if (sensor_unavailable_logged == 0U) {
                printf("[PhaseF] sensor data unavailable or stale; motors stopped\r\n");
                sensor_unavailable_logged = 1U;
            }
            if (sensor.sequence != 0U &&
                (uint32_t)(now - sensor.updated_tick) > SENSOR_STALE_TICKS &&
                stale_logged == 0U) {
                printf("[PhaseF] sensor snapshot stale; forcing local STOP\r\n");
                stale_logged = 1U;
            }
            controller.active = 0U;
            controller.armed = 0U;
            controller.recover_retries = 0U;
            controller.safe_forward_ticks = 0U;
        } else {
            reset_requested = 0U;
            sensor_unavailable_logged = 0U;
            stale_logged = 0U;
            phase_f_update_controller(&controller, &sensor, now);
            command = phase_f_motion_command(&controller, now);
        }

        phase_f_send_command(&command, now);
        if (controller.armed != 0U && controller.active != 0U &&
            controller.state == PHASE_F_RECOVER &&
            (uint32_t)(now - controller.last_status_poll_tick) >= STATUS_POLL_TICKS) {
            /* Telemetry cadence for the closed-loop turn; doubles as a second
               heartbeat alongside the 100 ms command resend. */
            controller.last_status_poll_tick = now;
            (void)protocol_send_get_status();
        }
        if (controller.armed != 0U && controller.active != 0U &&
            controller.state == PHASE_F_NORMAL &&
            controller.safe_forward_ticks < EDGE_REVERSE_MAX_TICKS) {
            /* NORMAL always drives straight at +140/+140, so elapsed beats
               measure distance over ground already proven safe. */
            controller.safe_forward_ticks++;
        }
        osDelay(CONTROL_LOOP_TICKS);
    }
}

static int phase_f_hardware_init(void)
{
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[PhaseF] GPIO init failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11,
        WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[PhaseF] GPIO11 UART2 TX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12,
        WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[PhaseF] GPIO12 UART2 RX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(IR_LEFT_PIN, WIFI_IOT_IO_FUNC_GPIO_13_GPIO) != WIFI_IOT_SUCCESS ||
        IoSetFunc(IR_RIGHT_PIN, WIFI_IOT_IO_FUNC_GPIO_14_GPIO) != WIFI_IOT_SUCCESS) {
        printf("[PhaseF] infrared GPIO function setup failed\r\n");
        return -1;
    }
    if (GpioSetDir(IR_LEFT_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS ||
        GpioSetDir(IR_RIGHT_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS) {
        printf("[PhaseF] infrared GPIO direction setup failed\r\n");
        return -1;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[PhaseF] UART2 init failed\r\n");
        return -1;
    }
    return 0;
}

static int phase_f_create_thread(const char *name, osThreadFunc_t function)
{
    osThreadAttr_t attr = {0};

    attr.name = name;
    attr.stack_size = PHASE_F_THREAD_STACK_SIZE;
    attr.priority = PHASE_F_THREAD_PRIORITY;
    if (osThreadNew(function, NULL, &attr) == NULL) {
        printf("[PhaseF] thread create failed: %s\r\n", name);
        return -1;
    }
    return 0;
}

static void TabletopCliffAvoidanceEntry(void)
{
    if (phase_f_hardware_init() != 0) {
        return;
    }

    sensor_mutex = osMutexNew(NULL);
    if (sensor_mutex == NULL) {
        printf("[PhaseF] sensor mutex create failed\r\n");
        return;
    }
    status_mutex = osMutexNew(NULL);
    if (status_mutex == NULL) {
        printf("[PhaseF] status mutex create failed\r\n");
        return;
    }

    if (phase_f_create_thread("PhaseFAck", (osThreadFunc_t)phase_f_ack_thread) != 0) {
        return;
    }
    if (phase_f_create_thread("PhaseFSensor", (osThreadFunc_t)phase_f_sensor_thread) != 0) {
        return;
    }
    if (phase_f_create_thread("PhaseFControl", (osThreadFunc_t)phase_f_control_thread) != 0) {
        return;
    }

    printf("[PhaseF] ready: IR GPIO13/14, UART2 GPIO11/12, 115200 8N1\r\n");
}

APP_FEATURE_INIT(TabletopCliffAvoidanceEntry);
