#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_uart_ex.h"
#include "hi_time.h"

#define TRACE_ENABLE_MOTION               1

#define TRACE_DEMO_SCRIPT                 1
#define DEMO_ARC_INNER                    0
#define DEMO_ARC_OUTER                    80
#define DEMO_ARC_MIN_TICKS                30U
#define DEMO_ARC_TIMEOUT_TICKS            200U
#define DEMO_REARM_S2                     (10L * S2_PER_CM)
#define DEMO_PAUSE_TICKS                  300U
#define DEMO_BAR_EXPIRE_TICKS             6000U

#define DEMO_REV_SPEED                    80
#define DEMO_REV_DELTA                    40
#define DEMO_REV_MAX_S2                   (60L * S2_PER_CM)
#define DEMO_RETURN_TICKS                 300U
#define DEMO_PH_DRIVE                     0
#define DEMO_PH_REVERSE                   1
#define DEMO_PH_RETURN                    2

#define CAPTURE_INNER_SPEED               40
#define CAPTURE_OUTER_SPEED               160
#define CAPTURE_MIN_TICKS                 20U
#define CAPTURE_TIMEOUT_TICKS             150U

#define TRACE_BLE_DEBUG                   1
#define BLE_DEBUG_BAUD                    9600U
#define BLE_HEARTBEAT_TICKS               100U

#define PROTOCOL_SOF                      0xAAU
#define MAX_PAYLOAD                       16U
#define PROTOCOL_FRAME_MAX                (MAX_PAYLOAD + 4U)
#define PROTOCOL_CMD_SET_SPEED            0x01U
#define PROTOCOL_CMD_STOP                 0x02U
#define PROTOCOL_CMD_PING                 0x03U
#define PROTOCOL_CMD_GET_STATUS           0x04U
#define PROTOCOL_CMD_ACK                  0x81U
#define PROTOCOL_CMD_STATUS               0x82U
#define STATUS_PAYLOAD_LEN                13U

#define PROTOCOL_STATUS_OK                0U
#define PROTOCOL_STATUS_BAD_LENGTH        1U
#define PROTOCOL_STATUS_BAD_CHECKSUM      2U
#define PROTOCOL_STATUS_INVALID_PARAM     3U
#define PROTOCOL_STATUS_UNKNOWN_CMD       4U

#define TRACE_THREAD_STACK_SIZE           4096U
#define TRACE_THREAD_PRIORITY             25
#define ACK_READ_BUFFER_SIZE              64U
#define ACK_POLL_DELAY_TICKS              1U

#define SENSOR_SAMPLE_TICKS               1U
#define TRACE_DEBOUNCE_SAMPLES            2U
#define CONTROL_LOOP_TICKS                1U
#define COMMAND_RESEND_TICKS              3U
#define SENSOR_STALE_TICKS                12U
#define STARTUP_GUARD_TICKS               100U
#define STATUS_LOG_TICKS                  10U
#define STATUS_POLL_TICKS                 3U

#define DRIVE_SPEED                       80
#define CORRECT_INNER_SPEED               0
#define CORRECT_OUTER_SPEED               80
#define CORRECT_PULSE_TICKS               50U
#define SETTLE_SPEED                      40
#define SETTLE_TICKS                      50U

#define S2_PER_CM                         392L

#define HEADING_PULSES_PER_DEG            24L
#define DEG_TO_PULSES(d)                  ((int32_t)(d) * HEADING_PULSES_PER_DEG)

#define WATCH_CONFIRM_TICKS               2U
#define WATCH_BAR2_TICKS                  4U

#define WATCH_BAR1_MIN_TICKS              8U
#define WATCH_TIMEOUT_TICKS               500U
#define GOAL_GAP_MIN_S2                   (S2_PER_CM / 2L)
#define GOAL_GAP_MAX_S2                   (4L * S2_PER_CM)

#define DEAD_SILENCE_S2                   (3L * S2_PER_CM)

#define BAR2_GUARD_S2                     (8L * S2_PER_CM)
#define BAR2_WINDOW_S2                    (35L * S2_PER_CM)

#define W2_WINDOW_TICKS                   40U
#define W2_HEADING_GUARD                  DEG_TO_PULSES(30)

#define WIGGLE_PIVOT_SPEED                40
#define WIGGLE_ARC_DEG                    25
#define WIGGLE_ARCS                       4U
#define WIGGLE_ARC_TIMEOUT_TICKS          250U

#define REJOIN_PIVOT_SPEED                40
#define REJOIN_CONFIRM_DEG                90
#define REJOIN_SWITCH_DEG                 150
#define REJOIN_CENTER_DEG                 120

#define BRANCH_SIDE_RIGHT                 (-1)
#define BRANCH_SIDE_LEFT                  1

#define REJOIN_PHASE_CONFIRM              0U
#define REJOIN_PHASE_SWITCH               1U
#define REJOIN_PHASE_CENTER               2U

#define REV_SPEED                         40
#define REV_BUDGET_S2                     (150L * S2_PER_CM)
#define REV_GUARD_S2                      (10L * S2_PER_CM)
#define REV_DOSE_DEG                      8
#define REV_DOSE_SPEED                    40
#define REV_DOSE_TIMEOUT_TICKS            100U

#define BAR_SUPPRESS_S2                   (10L * S2_PER_CM)

#define IR_LEFT_PIN                       WIFI_IOT_IO_NAME_GPIO_13
#define IR_RIGHT_PIN                      WIFI_IOT_IO_NAME_GPIO_14

extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

#if TRACE_BLE_DEBUG
static void ble_debug_printf(const char *format, ...)
{
    char buffer[160];
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length <= 0) {
        return;
    }
    if (length >= (int)sizeof(buffer)) {
        length = (int)sizeof(buffer) - 1;
    }
    (void)UartWrite(WIFI_IOT_UART_IDX_1, (const uint8_t *)buffer, (unsigned int)length);
}
#else
#define ble_debug_printf(...) ((void)0)
#endif

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
    uint8_t line_left;
    uint8_t line_right;
    uint8_t ready;
    uint8_t healthy;
    uint32_t updated_tick;
    uint32_t sequence;
} SensorSnapshot;

typedef struct {
    int32_t odo_left;
    int32_t odo_right;
    int16_t speed_left;
    int16_t speed_right;
    uint8_t flags;
    uint32_t updated_tick;
    uint32_t sequence;
} StatusSnapshot;

typedef enum {
    TRACE_STRAIGHT = 0,
    TRACE_LEFT,
    TRACE_RIGHT,
    TRACE_SETTLE,
    TRACE_WIGGLE,
    TRACE_REVERSE,
    TRACE_REJOIN,
    TRACE_CAPTURE,
    TRACE_GOAL
} TraceState;

typedef struct {
    TraceState state;
    uint32_t state_enter_tick;

    int32_t s2;
    int32_t theta;
    int32_t prev_odo_left;
    int32_t prev_odo_right;
    uint8_t odo_valid;
    uint32_t odo_seq;

    uint8_t watch_active;
    uint8_t watch_ww;
    uint8_t watch_seen_black;
    int32_t watch_trail_s2;
    int32_t watch_last_white_s2;
    uint8_t watch_bar2;
    uint8_t watch_bar1;
    uint32_t watch_start_tick;

    uint8_t w2_left_valid;
    uint8_t w2_right_valid;
    uint32_t w2_left_tick;
    uint32_t w2_right_tick;
    int32_t w2_left_theta;
    int32_t w2_right_theta;
    int32_t bar_suppress_s2;

    uint8_t bar_valid;
    int32_t bar_valid_s2;
    uint8_t side_recorded;

    int branch_side;

    uint32_t wiggle_arc;
    int32_t wiggle_arc_theta;
    uint32_t wiggle_arc_tick;

    int32_t rev_prev_s2;
    int32_t rev_budget_s2;
    uint8_t rev_started;
    int32_t rev_guard_s2;
    int16_t rev_cmd_left;
    int16_t rev_cmd_right;

    uint8_t rev_dosing;
    int rev_dose_dir;
    int32_t rev_dose_theta;
    uint32_t rev_dose_tick;
    uint8_t rev_dose_lock;
    uint8_t rev_luck;

    uint32_t rejoin_phase;
    uint32_t rejoin_edges;
    uint8_t rejoin_last;
    int32_t rejoin_theta;

    uint8_t demo_ww;
    uint8_t demo_bars;
    int demo_script;
    uint32_t demo_script_tick;
    int32_t demo_rearm_s2;
    uint32_t demo_pause_start;
    uint8_t demo_phase;
    int16_t demo_cmd_left;
    int16_t demo_cmd_right;
    int32_t demo_rev_s2;
    int32_t demo_entry_s2;
    int demo_entry_dir;
    int demo_return_dir;
    uint32_t demo_bar1_tick;
    uint32_t demo_phase_tick;
} TraceController;

typedef struct {
    int16_t left;
    int16_t right;
    uint8_t stop;
} MotionCommand;

static osMutexId_t sensor_mutex;
static SensorSnapshot sensor_snapshot;
static volatile uint8_t sensor_reset_requested;
static osMutexId_t status_mutex;
static StatusSnapshot status_snapshot;
static uint32_t link_ack_ok;
static uint32_t link_ack_bad;

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
        printf("[Trace] frame encode failed: cmd=0x%02X len=%u\r\n", cmd, len);
        return -1;
    }

    written = UartWrite(WIFI_IOT_UART_IDX_2, frame, frame_len);
    if (written != frame_len) {
        printf("[Trace] UART2 write failed: cmd=0x%02X wrote=%d/%u\r\n",
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

static int32_t decode_i32_le(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

static int16_t decode_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

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
            printf("[Trace v7.3] STATUS invalid length: %u\r\n", parser->len);
            return;
        }
        status_publish(parser->payload);
        return;
    }
    if (parser->cmd != PROTOCOL_CMD_ACK) {
        printf("[Trace v7.3] RX ignored: cmd=0x%02X len=%u\r\n", parser->cmd, parser->len);
        return;
    }
    if (parser->len != 2U) {
        printf("[Trace v7.3] ACK invalid length: %u\r\n", parser->len);
        return;
    }

    original_cmd = parser->payload[0];
    status = parser->payload[1];
    if (status != PROTOCOL_STATUS_OK) {
        link_ack_bad++;
        printf("[Trace v7.3] ACK %s (0x%02X): %s (%u)\r\n",
            protocol_command_name(original_cmd), original_cmd,
            protocol_status_name(status), status);
    } else {
        link_ack_ok++;
    }
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
                printf("[Trace v7.3] RX bad length: %u\r\n", parser->len);
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
                printf("[Trace v7.3] RX checksum error: cmd=0x%02X\r\n", parser->cmd);
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

static void trace_ack_thread(void *arg)
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
                printf("[Trace v7.3] UART2 read failed\r\n");
            } else {
                for (i = 0; i < count; i++) {
                    protocol_parse_byte(&parser, buffer[i]);
                }
            }
        } else if (result != WIFI_IOT_SUCCESS) {
            printf("[Trace v7.3] UART2 buffer check failed: %u\r\n", result);
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
    if (filter->count == 0U || filter->candidate != sample) {
        filter->candidate = sample;
        filter->count = 1U;
    } else if (filter->count < TRACE_DEBOUNCE_SAMPLES) {
        filter->count++;
    }

    if (filter->count >= TRACE_DEBOUNCE_SAMPLES) {
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
    sensor_snapshot.line_left = left->stable;
    sensor_snapshot.line_right = right->stable;
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

static void trace_sensor_thread(void *arg)
{
    SensorDebounce left = {0};
    SensorDebounce right = {0};
    uint8_t last_line_left = 0U;
    uint8_t last_line_right = 0U;
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
                printf("[Trace v7.3] infrared read failed: left=%u right=%u\r\n",
                    left_result, right_result);
            }
            was_healthy = 0U;
            last_logged_ready = 0U;
        } else {

            uint8_t line_left = (raw_left == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;
            uint8_t line_right = (raw_right == WIFI_IOT_GPIO_VALUE1) ? 1U : 0U;

            sensor_debounce_update(&left, line_left);
            sensor_debounce_update(&right, line_right);
            sensor_publish((uint8_t)raw_left, (uint8_t)raw_right,
                &left, &right, 1U, now);

            if (was_healthy == 0U) {
                printf("[Trace v7.3] infrared read recovered; debouncing again\r\n");
            }
            was_healthy = 1U;

            if (left.ready != 0U && right.ready != 0U &&
                (last_logged_ready == 0U || left.stable != last_line_left ||
                 right.stable != last_line_right)) {
                printf("[Trace v7.3] sensor raw L=%u R=%u, line L=%s R=%s\r\n",
                    (uint8_t)raw_left, (uint8_t)raw_right,
                    left.stable != 0U ? "white" : "black",
                    right.stable != 0U ? "white" : "black");
                ble_debug_printf("[Trace v7.3] sensor L=%s R=%s\r\n",
                    left.stable != 0U ? "white" : "black",
                    right.stable != 0U ? "white" : "black");
                last_line_left = left.stable;
                last_line_right = right.stable;
                last_logged_ready = 1U;
            }
        }
        osDelay(SENSOR_SAMPLE_TICKS);
    }
}

static const char *trace_state_name(TraceState state)
{
    switch (state) {
        case TRACE_STRAIGHT:
            return "STRAIGHT";
        case TRACE_LEFT:
            return "LEFT";
        case TRACE_RIGHT:
            return "RIGHT";
        case TRACE_SETTLE:
            return "SETTLE";
        case TRACE_WIGGLE:
            return "WIGGLE";
        case TRACE_REVERSE:
            return "REVERSE";
        case TRACE_REJOIN:
            return "REJOIN";
        case TRACE_CAPTURE:
            return "CAPTURE";
        case TRACE_GOAL:
            return "GOAL";
        default:
            return "UNKNOWN";
    }
}

static void reckoning_update(TraceController *controller, const StatusSnapshot *status)
{
    int32_t dl;
    int32_t dr;

    if (status->sequence == 0U || status->sequence == controller->odo_seq) {
        return;
    }
    controller->odo_seq = status->sequence;
    if (controller->odo_valid == 0U) {
        controller->prev_odo_left = status->odo_left;
        controller->prev_odo_right = status->odo_right;
        controller->odo_valid = 1U;
        return;
    }
    dl = status_odo_delta(status->odo_left, controller->prev_odo_left);
    dr = status_odo_delta(status->odo_right, controller->prev_odo_right);
    controller->prev_odo_left = status->odo_left;
    controller->prev_odo_right = status->odo_right;
    controller->s2 += dl + dr;
    controller->theta += dr - dl;
}

static void trace_enter_state(TraceController *controller, TraceState state,
    const SensorSnapshot *sensor, uint32_t now)
{
    TraceState previous = controller->state;

    controller->state = state;
    controller->state_enter_tick = now;
    printf("[Trace v7.3] %s -> %s (L=%s R=%s s=%ld h=%ld)\r\n",
        trace_state_name(previous), trace_state_name(state),
        sensor->line_left != 0U ? "white" : "black",
        sensor->line_right != 0U ? "white" : "black",
        (long)(controller->s2 / S2_PER_CM),
        (long)(controller->theta / HEADING_PULSES_PER_DEG));
    ble_debug_printf("[Trace v7.3] %s -> %s (L=%s R=%s s=%ld)\r\n",
        trace_state_name(previous), trace_state_name(state),
        sensor->line_left != 0U ? "w" : "b",
        sensor->line_right != 0U ? "w" : "b",
        (long)(controller->s2 / S2_PER_CM));
}

static void trace_enter_reverse(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now, const char *why)
{
    if (controller->rev_started == 0U) {
        controller->rev_started = 1U;
        controller->rev_budget_s2 = REV_BUDGET_S2;
    }
    controller->rev_prev_s2 = controller->s2;

    controller->rev_guard_s2 = controller->s2 - REV_GUARD_S2;
    controller->rev_cmd_left = -REV_SPEED;
    controller->rev_cmd_right = -REV_SPEED;
    controller->rev_dosing = 0U;
    controller->rev_dose_lock = 0U;
    controller->rev_luck = 0U;
    controller->watch_active = 0U;
    controller->watch_ww = 0U;
    controller->bar_valid = 0U;
    printf("[Trace v7.3] REVERSE (%s): backing out, budget %ld cm\r\n", why,
        (long)(controller->rev_budget_s2 / S2_PER_CM));
    ble_debug_printf("[Trace v7.3] REVERSE (%s)\r\n", why);
    trace_enter_state(controller, TRACE_REVERSE, sensor, now);
}

static void trace_watch_arm(TraceController *controller, uint32_t now, const char *how)
{
    controller->watch_active = 1U;
    controller->watch_seen_black = 0U;
    controller->watch_trail_s2 = controller->s2;
    controller->watch_last_white_s2 = controller->s2;
    controller->watch_bar2 = 0U;
    controller->watch_bar1 = 0U;
    controller->watch_start_tick = now;
    controller->bar_valid = 1U;
    controller->bar_valid_s2 = controller->s2;
    controller->side_recorded = 0U;
    printf("[Trace v7.3] WATCH armed (%s) at s=%ld cm h=%ld deg\r\n", how,
        (long)(controller->s2 / S2_PER_CM),
        (long)(controller->theta / HEADING_PULSES_PER_DEG));
    ble_debug_printf("[Trace v7.3] WATCH on (%s) s=%ld\r\n", how,
        (long)(controller->s2 / S2_PER_CM));
}

static void trace_wiggle_arc_begin(TraceController *controller, uint32_t now)
{
    controller->wiggle_arc_theta = controller->theta;
    controller->wiggle_arc_tick = now;
}

static void trace_enter_wiggle(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    controller->wiggle_arc = 0U;
    trace_wiggle_arc_begin(controller, now);
    printf("[Trace v7.3] WIGGLE: %ld cm silent, probing (%u x %d deg arcs)\r\n",
        (long)(DEAD_SILENCE_S2 / S2_PER_CM), (unsigned int)WIGGLE_ARCS,
        WIGGLE_ARC_DEG);
    ble_debug_printf("[Trace v7.3] WIGGLE start\r\n");
    trace_enter_state(controller, TRACE_WIGGLE, sensor, now);
}

static void trace_demo_tick(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    uint8_t both_white = (uint8_t)(sensor->line_left != 0U && sensor->line_right != 0U);
    uint8_t followish = (uint8_t)(controller->state == TRACE_STRAIGHT ||
        controller->state == TRACE_LEFT || controller->state == TRACE_RIGHT ||
        controller->state == TRACE_SETTLE);
    uint32_t elapsed;

    if (controller->demo_pause_start != 0U) {

        if ((uint32_t)(now - controller->demo_pause_start) < DEMO_PAUSE_TICKS) {
            return;
        }
        controller->demo_pause_start = 0U;
        printf("[TraceDemo] pause over, resume (s=%ld)\r\n",
            (long)(controller->s2 / S2_PER_CM));
        trace_enter_state(controller, TRACE_SETTLE, sensor, now);
        return;
    }
    if (controller->demo_script != 0) {

        uint8_t outer_black = (uint8_t)(controller->demo_script > 0 ?
            (sensor->line_right == 0U) : (sensor->line_left == 0U));
        elapsed = (uint32_t)(now - controller->demo_script_tick);
        if ((elapsed >= DEMO_ARC_MIN_TICKS && outer_black != 0U) ||
            elapsed >= DEMO_ARC_TIMEOUT_TICKS) {
            printf("[TraceDemo] scripted %s pivot done (%s, s=%ld L=%c R=%c)\r\n",
                controller->demo_script > 0 ? "LEFT" : "RIGHT",
                elapsed >= DEMO_ARC_TIMEOUT_TICKS ? "timeout" : "outer black",
                (long)(controller->s2 / S2_PER_CM),
                sensor->line_left != 0U ? 'w' : 'b',
                sensor->line_right != 0U ? 'w' : 'b');
            ble_debug_printf("[TraceDemo] pivot done\r\n");
            controller->demo_script = 0;
            controller->demo_rearm_s2 = controller->s2 + DEMO_REARM_S2;
            controller->demo_pause_start = now;
            printf("[TraceDemo] pause 3s -- look around (s=%ld)\r\n",
                (long)(controller->s2 / S2_PER_CM));
        }
        return;
    }
    if (controller->demo_phase == DEMO_PH_REVERSE) {

        int32_t backed = controller->demo_rev_s2 - controller->s2;

        if (sensor->line_left != 0U && sensor->line_right == 0U) {
            controller->demo_cmd_left = -DEMO_REV_SPEED;
            controller->demo_cmd_right = (int16_t)(-(DEMO_REV_SPEED + DEMO_REV_DELTA));
        } else if (sensor->line_right != 0U && sensor->line_left == 0U) {
            controller->demo_cmd_left = (int16_t)(-(DEMO_REV_SPEED + DEMO_REV_DELTA));
            controller->demo_cmd_right = -DEMO_REV_SPEED;
        } else {
            controller->demo_cmd_left = -DEMO_REV_SPEED;
            controller->demo_cmd_right = -DEMO_REV_SPEED;
        }
        if (backed >= DEMO_REV_MAX_S2) {
            printf("[TraceDemo] reverse cap %ld cm -- park (s=%ld)\r\n",
                (long)(DEMO_REV_MAX_S2 / S2_PER_CM),
                (long)(controller->s2 / S2_PER_CM));
            ble_debug_printf("[TraceDemo] rev cap, park\r\n");
            controller->demo_phase = DEMO_PH_DRIVE;
            trace_enter_state(controller, TRACE_GOAL, sensor, now);
            return;
        }
        if (controller->s2 <= controller->demo_entry_s2) {

            controller->demo_return_dir = (controller->demo_entry_dir != 0) ?
                -controller->demo_entry_dir : 1;
            printf("[TraceDemo] entry path reversed (%ld cm): return pivot %s %ums fixed (s=%ld)\r\n",
                (long)(backed / S2_PER_CM),
                controller->demo_return_dir > 0 ? "LEFT" : "RIGHT",
                (unsigned int)(DEMO_RETURN_TICKS * 10U),
                (long)(controller->s2 / S2_PER_CM));
            ble_debug_printf("[TraceDemo] return pivot\r\n");
            controller->demo_phase = DEMO_PH_RETURN;
            controller->demo_phase_tick = now;
            return;
        }
        return;
    }
    if (controller->demo_phase == DEMO_PH_RETURN) {

        elapsed = (uint32_t)(now - controller->demo_phase_tick);
        if (elapsed >= DEMO_RETURN_TICKS) {
            printf("[TraceDemo] return pivot %s done (%ums fixed): back on the main line, FOLLOW (s=%ld)\r\n",
                controller->demo_return_dir > 0 ? "LEFT" : "RIGHT",
                (unsigned int)(DEMO_RETURN_TICKS * 10U),
                (long)(controller->s2 / S2_PER_CM));
            ble_debug_printf("[TraceDemo] returned\r\n");
            controller->demo_phase = DEMO_PH_DRIVE;
            controller->demo_rearm_s2 = controller->s2 + DEMO_REARM_S2;
            trace_enter_state(controller, TRACE_SETTLE, sensor, now);
        }
        return;
    }

    if (controller->demo_bars == 1U &&
        (uint32_t)(now - controller->demo_bar1_tick) >= DEMO_BAR_EXPIRE_TICKS) {
        controller->demo_bars = 0U;
        controller->demo_entry_dir = 0;
        printf("[TraceDemo] bar #1 expired (60 s, no second bar) -- voided, next bar re-arms (s=%ld)\r\n",
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[TraceDemo] bar1 expired\r\n");
    }

    if (controller->demo_bars == 1U && controller->demo_entry_dir == 0) {
        if (controller->state == TRACE_LEFT) {
            controller->demo_entry_dir = 1;
        } else if (controller->state == TRACE_RIGHT) {
            controller->demo_entry_dir = -1;
        }
        if (controller->demo_entry_dir != 0) {
            printf("[TraceDemo] entry turn %s recorded (s=%ld)\r\n",
                controller->demo_entry_dir > 0 ? "LEFT" : "RIGHT",
                (long)(controller->s2 / S2_PER_CM));
            ble_debug_printf("[TraceDemo] entry %s\r\n",
                controller->demo_entry_dir > 0 ? "LEFT" : "RIGHT");
        }
    }
    if (controller->s2 < controller->demo_rearm_s2) {
        controller->demo_ww = 0U;
        return;
    }
    if (followish == 0U) {

        controller->demo_ww = 0U;
        return;
    }
    if (both_white == 0U) {
        controller->demo_ww = 0U;
        return;
    }
    controller->demo_ww++;
    if (controller->demo_ww < WATCH_CONFIRM_TICKS) {
        return;
    }
    controller->demo_ww = 0U;
    controller->demo_bars++;
    if (controller->demo_bars == 1U) {

        controller->demo_entry_s2 = controller->s2;
        controller->demo_bar1_tick = now;
        controller->demo_entry_dir = 0;
        controller->demo_rearm_s2 = controller->s2 + DEMO_REARM_S2;
        printf("[TraceDemo] bar #1 => junction armed: FOLLOW on, first turn recorded, 60 s expiry (s=%ld)\r\n",
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[TraceDemo] bar1 armed\r\n");
    } else if (controller->demo_bars == 2U) {

        controller->demo_phase = DEMO_PH_REVERSE;
        controller->demo_rev_s2 = controller->s2;
        controller->demo_cmd_left = -DEMO_REV_SPEED;
        controller->demo_cmd_right = -DEMO_REV_SPEED;
        printf("[TraceDemo] bar #2 => dead end, reverse %ld cm (recorded) (s=%ld)\r\n",
            (long)((controller->s2 - controller->demo_entry_s2) / S2_PER_CM),
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[TraceDemo] dead-end reverse\r\n");
    } else if (controller->demo_bars == 3U) {

        controller->demo_script = -1;
        controller->demo_script_tick = now;
        printf("[TraceDemo] bar #3 => scripted RIGHT pivot at fork 2 (s=%ld)\r\n",
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[TraceDemo] bar3 RIGHT\r\n");
    } else {
        printf("[TraceDemo] bar #%u => finish line, park (s=%ld)\r\n",
            (unsigned int)controller->demo_bars,
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[TraceDemo] finish, park\r\n");
        trace_enter_state(controller, TRACE_GOAL, sensor, now);
    }
}

static void trace_bar_watch(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    uint8_t both_white = (uint8_t)(sensor->line_left != 0U && sensor->line_right != 0U);
    uint8_t both_black = (uint8_t)(sensor->line_left == 0U && sensor->line_right == 0U);
    uint8_t single = (uint8_t)(sensor->line_left != sensor->line_right);
    uint8_t followish = (uint8_t)(controller->state == TRACE_STRAIGHT ||
        controller->state == TRACE_LEFT || controller->state == TRACE_RIGHT ||
        controller->state == TRACE_SETTLE);

    if (TRACE_DEMO_SCRIPT != 0) {

        trace_demo_tick(controller, sensor, now);
        return;
    }

    if (followish == 0U) {

        controller->watch_ww = 0U;
        controller->w2_left_valid = 0U;
        controller->w2_right_valid = 0U;
        return;
    }

    if (controller->watch_active == 0U) {

        if (controller->s2 < controller->bar_suppress_s2) {
            controller->watch_ww = 0U;
            return;
        }

        if (sensor->line_left != 0U) {
            controller->w2_left_valid = 1U;
            controller->w2_left_tick = now;
            controller->w2_left_theta = controller->theta;
        }
        if (sensor->line_right != 0U) {
            controller->w2_right_valid = 1U;
            controller->w2_right_tick = now;
            controller->w2_right_theta = controller->theta;
        }
        if (both_white != 0U) {
            controller->watch_ww++;
            if (controller->watch_ww >= WATCH_CONFIRM_TICKS) {
                int32_t gap = controller->s2 - controller->bar_valid_s2;

                if (controller->bar_valid != 0U &&
                    gap >= (int32_t)BAR2_GUARD_S2 &&
                    gap <= (int32_t)BAR2_WINDOW_S2) {
                    printf("[Trace v7.3] second bar at +%ld cm of the arming => dead end\r\n",
                        (long)(gap / S2_PER_CM));
                    ble_debug_printf("[Trace v7.3] 2nd bar => REVERSE\r\n");
                    trace_enter_reverse(controller, sensor, now, "second bar");
                    return;
                }
                if (controller->bar_valid != 0U &&
                    gap < (int32_t)BAR2_GUARD_S2) {

                    controller->watch_ww = 0U;
                } else {
                    trace_watch_arm(controller, now, "double white");
                }
            }
        } else {
            uint32_t latest;

            controller->watch_ww = 0U;

            latest = (controller->w2_left_tick >= controller->w2_right_tick)
                ? controller->w2_left_tick : controller->w2_right_tick;
            if (controller->w2_left_valid != 0U && controller->w2_right_valid != 0U &&
                controller->w2_left_tick != controller->w2_right_tick &&
                (uint32_t)(now - latest) <= W2_WINDOW_TICKS) {
                uint32_t span;
                int32_t dtheta;

                span = (controller->w2_left_tick >= controller->w2_right_tick)
                    ? (controller->w2_left_tick - controller->w2_right_tick)
                    : (controller->w2_right_tick - controller->w2_left_tick);
                dtheta = controller->w2_left_theta - controller->w2_right_theta;
                if (dtheta < 0) {
                    dtheta = -dtheta;
                }
                if (span <= W2_WINDOW_TICKS && dtheta <= W2_HEADING_GUARD) {
                    trace_watch_arm(controller, now, "W2 seq white");
                }
            }
        }
        return;
    }

    if (both_white != 0U || single != 0U) {
        controller->watch_last_white_s2 = controller->s2;
    }
    if (both_white != 0U && controller->watch_seen_black == 0U &&
        controller->watch_bar1 < 255U) {
        controller->watch_bar1++;
    }
    if (both_black != 0U && controller->watch_seen_black == 0U) {
        controller->watch_seen_black = 1U;
        controller->watch_trail_s2 = controller->s2;
    }

    if (controller->side_recorded == 0U && single != 0U) {
        controller->branch_side = (sensor->line_left != 0U) ?
            BRANCH_SIDE_LEFT : BRANCH_SIDE_RIGHT;
        controller->side_recorded = 1U;
        printf("[Trace v7.3] branch side %s (first white, s=%ld)\r\n",
            controller->branch_side < 0 ? "RIGHT" : "LEFT",
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[Trace v7.3] branch %s\r\n",
            controller->branch_side < 0 ? "R" : "L");

        printf("[Trace v7.3] capture %s (commit)\r\n",
            controller->branch_side < 0 ? "RIGHT" : "LEFT");
        ble_debug_printf("[Trace v7.3] capture %s\r\n",
            controller->branch_side < 0 ? "R" : "L");
        trace_enter_state(controller, TRACE_CAPTURE, sensor, now);
    }

    if (both_white != 0U && controller->watch_seen_black != 0U) {
        int32_t gap = controller->s2 - controller->watch_trail_s2;

        controller->watch_bar2++;
        if (controller->watch_bar2 >= WATCH_BAR2_TICKS) {
            if (gap >= GOAL_GAP_MIN_S2 && gap <= GOAL_GAP_MAX_S2 &&
                controller->watch_bar1 >= WATCH_BAR1_MIN_TICKS) {
                printf("[Trace v7.3] GOAL: second bar at net gap %ld mm (bar1 %u ticks)\r\n",
                    (long)(gap * 10L / S2_PER_CM), controller->watch_bar1);
                ble_debug_printf("[Trace v7.3] GOAL! gap=%ldmm\r\n",
                    (long)(gap * 10L / S2_PER_CM));
                controller->watch_active = 0U;
                trace_enter_state(controller, TRACE_GOAL, sensor, now);
                return;
            }
            if (gap > GOAL_GAP_MAX_S2) {

                controller->watch_seen_black = 0U;
                controller->watch_trail_s2 = controller->s2;
                controller->watch_bar2 = 0U;
                controller->watch_bar1 = 0U;
            }
        }
    } else {
        controller->watch_bar2 = 0U;
    }

    if ((int32_t)(controller->s2 - controller->watch_last_white_s2) >= DEAD_SILENCE_S2) {
        trace_enter_wiggle(controller, sensor, now);
        return;
    }

    if ((uint32_t)(now - controller->watch_start_tick) >= WATCH_TIMEOUT_TICKS) {

        printf("[Trace v7.3] WATCH timeout => disarm\r\n");
        ble_debug_printf("[Trace v7.3] WATCH timeout\r\n");
        controller->watch_active = 0U;
        controller->watch_ww = 0U;
    }
}

static void trace_wiggle_tick(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    uint8_t contact = (uint8_t)(sensor->line_left != 0U || sensor->line_right != 0U);
    int32_t swing;
    int32_t target;

    if (contact != 0U) {
        printf("[Trace v7.3] WIGGLE: contact => line alive, follow on\r\n");
        ble_debug_printf("[Trace v7.3] WIGGLE: line alive\r\n");
        controller->watch_active = 0U;
        controller->watch_ww = 0U;
        trace_enter_state(controller, TRACE_SETTLE, sensor, now);
        return;
    }
    swing = controller->theta - controller->wiggle_arc_theta;
    if (swing < 0) {
        swing = -swing;
    }

    target = DEG_TO_PULSES((controller->wiggle_arc == 0U ||
        controller->wiggle_arc == WIGGLE_ARCS - 1U) ? WIGGLE_ARC_DEG :
        2 * WIGGLE_ARC_DEG);
    if (swing >= target ||
        (uint32_t)(now - controller->wiggle_arc_tick) >= WIGGLE_ARC_TIMEOUT_TICKS) {
        controller->wiggle_arc++;
        if (controller->wiggle_arc >= WIGGLE_ARCS) {
            printf("[Trace v7.3] WIGGLE: no contact => dead end\r\n");
            ble_debug_printf("[Trace v7.3] WIGGLE: dead end\r\n");
            trace_enter_reverse(controller, sensor, now, "dead end (wiggle)");
            return;
        }
        trace_wiggle_arc_begin(controller, now);
    }
}

static TraceState trace_target_state(const SensorSnapshot *sensor)
{
    if (sensor->line_left != 0U && sensor->line_right != 0U) {

        return TRACE_STRAIGHT;
    }
    if (sensor->line_left != 0U) {
        return TRACE_LEFT;
    }
    if (sensor->line_right != 0U) {
        return TRACE_RIGHT;
    }
    return TRACE_STRAIGHT;
}

static void trace_follow_tick(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    TraceState target = trace_target_state(sensor);
    uint32_t dwell = (uint32_t)(now - controller->state_enter_tick);

    switch (controller->state) {
        case TRACE_STRAIGHT:

            if (target == TRACE_LEFT || target == TRACE_RIGHT) {
                trace_enter_state(controller, target, sensor, now);
            }
            break;

        case TRACE_LEFT:
        case TRACE_RIGHT:
            if (target == controller->state) {

                if (dwell >= CORRECT_PULSE_TICKS) {
                    trace_enter_state(controller, TRACE_SETTLE, sensor, now);
                }
            } else if (target == TRACE_STRAIGHT) {
                trace_enter_state(controller, TRACE_SETTLE, sensor, now);
            } else {
                trace_enter_state(controller, target, sensor, now);
            }
            break;

        case TRACE_SETTLE:
            if (target == TRACE_LEFT || target == TRACE_RIGHT) {
                trace_enter_state(controller, target, sensor, now);
            } else if (dwell >= SETTLE_TICKS) {
                trace_enter_state(controller, TRACE_STRAIGHT, sensor, now);
            }
            break;

        case TRACE_CAPTURE:

            {
                uint8_t far_white = (controller->branch_side > 0) ?
                    sensor->line_right : sensor->line_left;
                uint8_t done = (uint8_t)(far_white != 0U ||
                    (dwell >= CAPTURE_MIN_TICKS && target == TRACE_STRAIGHT) ||
                    dwell >= CAPTURE_TIMEOUT_TICKS);

                if (done != 0U) {
                    controller->watch_last_white_s2 = controller->s2;
                    printf("[Trace v7.3] capture done (%s, %lu ms)\r\n",
                        far_white != 0U ? "far probe on tape" :
                        target == TRACE_STRAIGHT ? "tape straddled" : "cap",
                        (unsigned long)(dwell * 10U));
                    ble_debug_printf("[Trace v7.3] capture done\r\n");
                    trace_enter_state(controller, TRACE_SETTLE, sensor, now);
                }
            }
            break;

        default:
            break;
    }
}

static uint8_t rejoin_grammar_white(const TraceController *controller,
    const SensorSnapshot *sensor)
{

    return (controller->branch_side > 0) ? sensor->line_right : sensor->line_left;
}

static void trace_rejoin_phase_begin(TraceController *controller, uint32_t phase)
{
    controller->rejoin_phase = phase;
    controller->rejoin_edges = 0U;
    controller->rejoin_theta = controller->theta;
}

static void trace_enter_rejoin(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    trace_rejoin_phase_begin(controller, REJOIN_PHASE_CONFIRM);
    controller->rejoin_last = rejoin_grammar_white(controller, sensor);
    printf("[Trace v7.3] REJOIN (%s branch): CONFIRM sweep at s=%ld h=%ld\r\n",
        controller->branch_side > 0 ? "left" : "right",
        (long)(controller->s2 / S2_PER_CM),
        (long)(controller->theta / HEADING_PULSES_PER_DEG));
    ble_debug_printf("[Trace v7.3] REJOIN confirm\r\n");
    trace_enter_state(controller, TRACE_REJOIN, sensor, now);
}

static void trace_rejoin_park(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now, const char *why)
{
    printf("[Trace v7.3] REJOIN stuck (%s) => park\r\n", why);
    ble_debug_printf("[Trace v7.3] REJOIN park (%s)\r\n", why);
    trace_enter_state(controller, TRACE_GOAL, sensor, now);
}

static void trace_rejoin_tick(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    uint8_t grammar_white = rejoin_grammar_white(controller, sensor);
    int32_t swing = controller->theta - controller->rejoin_theta;

    if (swing < 0) {
        swing = -swing;
    }

    if (grammar_white != controller->rejoin_last) {
        controller->rejoin_last = grammar_white;
        if (controller->rejoin_edges < 255U) {
            controller->rejoin_edges++;
        }
        printf("[Trace v7.3] REJOIN edge #%lu: %s (h=%ld)\r\n",
            (unsigned long)controller->rejoin_edges,
            grammar_white != 0U ? "white" : "black",
            (long)(controller->theta / HEADING_PULSES_PER_DEG));
    }

    switch (controller->rejoin_phase) {
        case REJOIN_PHASE_CONFIRM:
            if (controller->rejoin_edges >= 2U) {
                printf("[Trace v7.3] REJOIN: second tape at %ld deg => fork, SWITCH\r\n",
                    (long)(swing / HEADING_PULSES_PER_DEG));
                ble_debug_printf("[Trace v7.3] REJOIN: fork, switch\r\n");
                trace_rejoin_phase_begin(controller, REJOIN_PHASE_SWITCH);
                return;
            }
            if (swing >= DEG_TO_PULSES(REJOIN_CONFIRM_DEG)) {
                if (controller->rejoin_edges == 1U) {
                    printf("[Trace v7.3] REJOIN: %ld deg silent => mid-branch, CENTER\r\n",
                        (long)(swing / HEADING_PULSES_PER_DEG));
                    ble_debug_printf("[Trace v7.3] REJOIN: mid-branch\r\n");
                    trace_rejoin_phase_begin(controller, REJOIN_PHASE_CENTER);
                } else {
                    trace_rejoin_park(controller, sensor, now, "buried white");
                }
                return;
            }
            break;

        case REJOIN_PHASE_SWITCH:
            if (controller->rejoin_edges >= 3U) {
                controller->bar_suppress_s2 = controller->s2 + BAR_SUPPRESS_S2;
                controller->watch_active = 0U;
                controller->watch_ww = 0U;
                printf("[Trace v7.3] REJOIN: on the far arm, resume FOLLOW\r\n");
                ble_debug_printf("[Trace v7.3] REJOIN done\r\n");
                trace_enter_state(controller, TRACE_SETTLE, sensor, now);
                return;
            }
            if (swing >= DEG_TO_PULSES(REJOIN_SWITCH_DEG)) {
                trace_rejoin_park(controller, sensor, now, "switch budget");
                return;
            }
            break;

        case REJOIN_PHASE_CENTER:
            if (controller->rejoin_edges >= 2U) {
                printf("[Trace v7.3] REJOIN: recentered, resume REVERSE\r\n");
                ble_debug_printf("[Trace v7.3] REJOIN recentered\r\n");
                trace_enter_reverse(controller, sensor, now, "rejoin recenter");

                controller->rev_guard_s2 = controller->s2;
                return;
            }
            if (swing >= DEG_TO_PULSES(REJOIN_CENTER_DEG)) {
                trace_rejoin_park(controller, sensor, now, "center budget");
                return;
            }
            break;

        default:
            break;
    }
}

static void trace_reverse_tick(TraceController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    int32_t step = controller->rev_prev_s2 - controller->s2;
    uint8_t grammar_white = rejoin_grammar_white(controller, sensor);
    uint8_t other_white = (controller->branch_side > 0) ?
        sensor->line_left : sensor->line_right;
    uint8_t past_guard = (uint8_t)(controller->s2 <= controller->rev_guard_s2);

    if (step > 0) {
        controller->rev_budget_s2 -= step;
    }
    controller->rev_prev_s2 = controller->s2;

    if (controller->rev_budget_s2 <= 0) {
        printf("[Trace v7.3] reverse budget exhausted => park\r\n");
        ble_debug_printf("[Trace v7.3] budget out, park\r\n");
        trace_enter_state(controller, TRACE_GOAL, sensor, now);
        return;
    }

    if (other_white == 0U) {
        controller->rev_dose_lock = 0U;
    }

    if (controller->rev_luck != 0U) {
        if (grammar_white != other_white) {
            controller->rev_luck = 0U;
            printf("[Trace v7.3] luck: single white at s=%ld, FOLLOW\r\n",
                (long)(controller->s2 / S2_PER_CM));
            ble_debug_printf("[Trace v7.3] luck shot\r\n");
            trace_enter_state(controller, TRACE_SETTLE, sensor, now);
            return;
        }
        controller->rev_cmd_left = -REV_SPEED;
        controller->rev_cmd_right = -REV_SPEED;
        return;
    }

    if (controller->rev_dosing != 0U) {
        int32_t dose_swing = controller->theta - controller->rev_dose_theta;

        if (dose_swing < 0) {
            dose_swing = -dose_swing;
        }
        if (other_white == 0U) {
            controller->rev_dosing = 0U;
        } else if (dose_swing >= DEG_TO_PULSES(REV_DOSE_DEG) ||
            (uint32_t)(now - controller->rev_dose_tick) >= REV_DOSE_TIMEOUT_TICKS) {
            controller->rev_dosing = 0U;
            controller->rev_dose_lock = 1U;

        } else if (grammar_white == 0U || past_guard == 0U) {
            return;
        } else {
            controller->rev_dosing = 0U;
        }
    }

    if (grammar_white != 0U && other_white != 0U && past_guard != 0U) {
        controller->rev_luck = 1U;
        controller->rev_dosing = 0U;
        printf("[Trace v7.3] double white in REVERSE at s=%ld => luck path\r\n",
            (long)(controller->s2 / S2_PER_CM));
        ble_debug_printf("[Trace v7.3] luck path\r\n");
        controller->rev_cmd_left = -REV_SPEED;
        controller->rev_cmd_right = -REV_SPEED;
        return;
    }

    if (grammar_white != 0U && other_white == 0U && past_guard != 0U) {
        trace_enter_rejoin(controller, sensor, now);
        return;
    }

    if (other_white != 0U && grammar_white == 0U && past_guard != 0U &&
        controller->rev_dose_lock == 0U) {
        controller->rev_dosing = 1U;
        controller->rev_dose_dir = -controller->branch_side;
        controller->rev_dose_theta = controller->theta;
        controller->rev_dose_tick = now;
        printf("[Trace v7.3] REV dose dir=%d at h=%ld\r\n",
            controller->rev_dose_dir,
            (long)(controller->theta / HEADING_PULSES_PER_DEG));
        return;
    }

    controller->rev_cmd_left = -REV_SPEED;
    controller->rev_cmd_right = -REV_SPEED;
}

static MotionCommand trace_motion_command(const TraceController *controller,
    uint32_t now)
{
    MotionCommand command = {0, 0, 1U};

    (void)now;
    if (TRACE_DEMO_SCRIPT != 0 && controller->demo_pause_start != 0U) {
        return command;
    }
    if (TRACE_DEMO_SCRIPT != 0 && controller->demo_phase == DEMO_PH_REVERSE) {
        command.left = controller->demo_cmd_left;
        command.right = controller->demo_cmd_right;
        command.stop = 0U;
        return command;
    }
    if (TRACE_DEMO_SCRIPT != 0 && controller->demo_phase == DEMO_PH_RETURN) {

        command.left = (int16_t)(controller->demo_return_dir > 0 ?
            DEMO_ARC_INNER : DEMO_ARC_OUTER);
        command.right = (int16_t)(controller->demo_return_dir > 0 ?
            DEMO_ARC_OUTER : DEMO_ARC_INNER);
        command.stop = 0U;
        return command;
    }
    if (TRACE_DEMO_SCRIPT != 0 && controller->demo_script != 0) {

        command.left = (int16_t)(controller->demo_script > 0 ?
            DEMO_ARC_INNER : DEMO_ARC_OUTER);
        command.right = (int16_t)(controller->demo_script > 0 ?
            DEMO_ARC_OUTER : DEMO_ARC_INNER);
        command.stop = 0U;
        return command;
    }
    switch (controller->state) {
        case TRACE_STRAIGHT:
            command.left = DRIVE_SPEED;
            command.right = DRIVE_SPEED;
            command.stop = 0U;
            break;

        case TRACE_LEFT:
            command.left = CORRECT_INNER_SPEED;
            command.right = CORRECT_OUTER_SPEED;
            command.stop = 0U;
            break;

        case TRACE_RIGHT:
            command.left = CORRECT_OUTER_SPEED;
            command.right = CORRECT_INNER_SPEED;
            command.stop = 0U;
            break;

        case TRACE_SETTLE:
            command.left = SETTLE_SPEED;
            command.right = SETTLE_SPEED;
            command.stop = 0U;
            break;

        case TRACE_WIGGLE:

            command.left = (controller->wiggle_arc % 2U == 0U) ?
                -WIGGLE_PIVOT_SPEED : WIGGLE_PIVOT_SPEED;
            command.right = -command.left;
            command.stop = 0U;
            break;

        case TRACE_REVERSE:
            if (controller->rev_dosing != 0U) {

                command.left = (int16_t)(-controller->rev_dose_dir * REV_DOSE_SPEED);
                command.right = (int16_t)(controller->rev_dose_dir * REV_DOSE_SPEED);
            } else {
                command.left = controller->rev_cmd_left;
                command.right = controller->rev_cmd_right;
            }
            command.stop = 0U;
            break;

        case TRACE_REJOIN:

            {
                int dir = (controller->rejoin_phase == REJOIN_PHASE_CONFIRM)
                    ? controller->branch_side : -controller->branch_side;

                command.left = (int16_t)(-dir * REJOIN_PIVOT_SPEED);
                command.right = (int16_t)(dir * REJOIN_PIVOT_SPEED);
            }
            command.stop = 0U;
            break;

        case TRACE_CAPTURE:

            command.left = (int16_t)(controller->branch_side > 0 ?
                CAPTURE_INNER_SPEED : CAPTURE_OUTER_SPEED);
            command.right = (int16_t)(controller->branch_side > 0 ?
                CAPTURE_OUTER_SPEED : CAPTURE_INNER_SPEED);
            command.stop = 0U;
            break;

        case TRACE_GOAL:
        default:
            break;
    }
    return command;
}

static void trace_send_command(const MotionCommand *command, uint32_t now)
{
    static MotionCommand previous = {0, 0, 1U};
    static uint32_t last_send_tick = 0U;
    static uint8_t sent_once = 0U;
    MotionCommand actual = *command;
    uint8_t changed;
    int result;

#if !TRACE_ENABLE_MOTION
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

static uint8_t trace_sensor_is_fresh(const SensorSnapshot *sensor, uint32_t now)
{
    if (sensor->sequence == 0U || sensor->healthy == 0U || sensor->ready == 0U) {
        return 0U;
    }
    return (uint8_t)((uint32_t)(now - sensor->updated_tick) <= SENSOR_STALE_TICKS);
}

static void trace_control_thread(void *arg)
{
    TraceController controller = {0};
    uint8_t sensor_unavailable_logged = 0U;
    uint8_t stale_logged = 0U;
    uint8_t reset_requested = 0U;
    uint32_t last_log_tick = 0U;
    uint32_t last_ble_tick = 0U;
    uint32_t last_status_poll = 0U;
    uint32_t guard_start;

    (void)arg;
    controller.state = TRACE_STRAIGHT;

    controller.branch_side = BRANCH_SIDE_RIGHT;

    printf("[Trace v7.3] control started: motion=%d drive=%d correct=%d/%d pulse=%ums settle=%d/%u rev=%d goal_gap=[%ld,%ld]mm silent=%ldcm wiggle=%d/%udegx%u w2=%ums/%lddeg rejoin=%d/%u/%u/%udeg dose=%udeg@%d bar2=%ld-%ldcm capture=%d/%dx%ums\r\n",
        TRACE_ENABLE_MOTION, DRIVE_SPEED, CORRECT_INNER_SPEED, CORRECT_OUTER_SPEED,
        (unsigned int)(CORRECT_PULSE_TICKS * 10U), SETTLE_SPEED, SETTLE_TICKS,
        REV_SPEED,
        (long)(GOAL_GAP_MIN_S2 * 10L / S2_PER_CM),
        (long)(GOAL_GAP_MAX_S2 * 10L / S2_PER_CM),
        (long)(DEAD_SILENCE_S2 / S2_PER_CM),
        WIGGLE_PIVOT_SPEED, (unsigned int)WIGGLE_ARC_DEG, (unsigned int)WIGGLE_ARCS,
        (unsigned int)(W2_WINDOW_TICKS * 10U),
        (long)(W2_HEADING_GUARD / HEADING_PULSES_PER_DEG),
        REJOIN_PIVOT_SPEED, (unsigned int)REJOIN_CONFIRM_DEG,
        (unsigned int)REJOIN_SWITCH_DEG, (unsigned int)REJOIN_CENTER_DEG,
        (unsigned int)REV_DOSE_DEG, REV_DOSE_SPEED,
        (long)(BAR2_GUARD_S2 / S2_PER_CM), (long)(BAR2_WINDOW_S2 / S2_PER_CM),
        CAPTURE_INNER_SPEED, CAPTURE_OUTER_SPEED,
        (unsigned int)(CAPTURE_MIN_TICKS * 10U));
    ble_debug_printf("[Trace v7.3] control started: motion=%d drive=%d rev=%d wiggle=%d/%u rejoin=%d\r\n",
        TRACE_ENABLE_MOTION, DRIVE_SPEED, REV_SPEED, WIGGLE_PIVOT_SPEED,
        (unsigned int)WIGGLE_ARC_DEG, REJOIN_PIVOT_SPEED);
    if (TRACE_DEMO_SCRIPT != 0) {
        printf("[TraceDemo] scripted route v5.2: bar#1=>junction armed (FOLLOW on, first turn recorded, 60s expiry), bar#2=>reverse to anchor + return pivot OPPOSITE recorded turn 3s fixed, bar#3=>RIGHT at fork2, bar#4=>park; pivot=%d/%d min=%ums rearm=%ldcm pause=%ums rev=-%d/-%d\r\n",
            DEMO_ARC_INNER, DEMO_ARC_OUTER,
            (unsigned int)(DEMO_ARC_MIN_TICKS * 10U),
            (long)(DEMO_REARM_S2 / S2_PER_CM),
            (unsigned int)(DEMO_PAUSE_TICKS * 10U),
            (int)DEMO_REV_SPEED, (int)(DEMO_REV_SPEED + DEMO_REV_DELTA));
        ble_debug_printf("[TraceDemo] scripted route\r\n");
    }
    protocol_send_ping();
    protocol_send_stop();

    guard_start = hi_get_tick();
    while ((uint32_t)(hi_get_tick() - guard_start) < STARTUP_GUARD_TICKS) {
        trace_send_command(&(MotionCommand){0, 0, 1U}, hi_get_tick());
        osDelay(CONTROL_LOOP_TICKS);
    }

#if !TRACE_ENABLE_MOTION
    printf("[Trace v7.3] motion disabled; all commands are forced STOP\r\n");
    ble_debug_printf("[Trace v7.3] motion disabled (safe build)\r\n");
#endif

    while (1) {
        SensorSnapshot sensor = sensor_take_snapshot();
        StatusSnapshot status = status_take_snapshot();
        uint32_t now = hi_get_tick();
        MotionCommand command = {0, 0, 1U};

        reckoning_update(&controller, &status);

        if ((uint32_t)(now - last_status_poll) >= STATUS_POLL_TICKS) {
            protocol_send_get_status();
            last_status_poll = now;
        }

        if (trace_sensor_is_fresh(&sensor, now) == 0U) {
            if (reset_requested == 0U) {
                sensor_request_reset();
                reset_requested = 1U;
            }
            if (sensor_unavailable_logged == 0U) {
                printf("[Trace v7.3] sensor data unavailable or stale; motors stopped\r\n");
                ble_debug_printf("[Trace v7.3] sensor stale; STOP\r\n");
                sensor_unavailable_logged = 1U;
            }
            if (sensor.sequence != 0U &&
                (uint32_t)(now - sensor.updated_tick) > SENSOR_STALE_TICKS &&
                stale_logged == 0U) {
                printf("[Trace v7.3] sensor snapshot stale; forcing local STOP\r\n");
                stale_logged = 1U;
            }
            controller.watch_ww = 0U;
            controller.w2_left_valid = 0U;
            controller.w2_right_valid = 0U;
            controller.rev_dosing = 0U;
            controller.rev_dose_lock = 0U;
            controller.rev_luck = 0U;
        } else {
            reset_requested = 0U;
            sensor_unavailable_logged = 0U;
            stale_logged = 0U;

            switch (controller.state) {
                case TRACE_WIGGLE:
                    trace_wiggle_tick(&controller, &sensor, now);
                    break;
                case TRACE_REVERSE:
                    trace_reverse_tick(&controller, &sensor, now);
                    break;
                case TRACE_REJOIN:
                    trace_rejoin_tick(&controller, &sensor, now);
                    break;
                case TRACE_GOAL:
                    break;
                default:
                    trace_bar_watch(&controller, &sensor, now);
                    if (controller.state != TRACE_GOAL &&
                        controller.state != TRACE_REVERSE &&
                        controller.state != TRACE_REJOIN &&
                        controller.state != TRACE_WIGGLE) {
                        trace_follow_tick(&controller, &sensor, now);
                    }
                    break;
            }
            command = trace_motion_command(&controller, now);
        }

        trace_send_command(&command, now);
        if ((uint32_t)(now - last_log_tick) >= STATUS_LOG_TICKS) {
            printf("[Trace v7.3] st=%s L=%s R=%s cmd=%d/%d s=%ldcm h=%lddeg w=%d ack=%lu/%lu\r\n",
                trace_state_name(controller.state),
                sensor.line_left != 0U ? "white" : "black",
                sensor.line_right != 0U ? "white" : "black",
                command.left, command.right,
                (long)(controller.s2 / S2_PER_CM),
                (long)(controller.theta / HEADING_PULSES_PER_DEG),
                controller.watch_active,
                (unsigned long)link_ack_ok, (unsigned long)link_ack_bad);
            last_log_tick = now;
        }
        if ((uint32_t)(now - last_ble_tick) >= BLE_HEARTBEAT_TICKS) {
            ble_debug_printf("[Trace v7.3] st=%s L=%s R=%s s=%ld h=%ld w=%d\r\n",
                trace_state_name(controller.state),
                sensor.line_left != 0U ? "w" : "b",
                sensor.line_right != 0U ? "w" : "b",
                (long)(controller.s2 / S2_PER_CM),
                (long)(controller.theta / HEADING_PULSES_PER_DEG),
                controller.watch_active);
            last_ble_tick = now;
        }
        osDelay(CONTROL_LOOP_TICKS);
    }
}

static int trace_hardware_init(void)
{
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[Trace] GPIO init failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11,
        WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[Trace] GPIO11 UART2 TX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12,
        WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[Trace] GPIO12 UART2 RX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(IR_LEFT_PIN, WIFI_IOT_IO_FUNC_GPIO_13_GPIO) != WIFI_IOT_SUCCESS ||
        IoSetFunc(IR_RIGHT_PIN, WIFI_IOT_IO_FUNC_GPIO_14_GPIO) != WIFI_IOT_SUCCESS) {
        printf("[Trace] infrared GPIO function setup failed\r\n");
        return -1;
    }
    if (GpioSetDir(IR_LEFT_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS ||
        GpioSetDir(IR_RIGHT_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS) {
        printf("[Trace] infrared GPIO direction setup failed\r\n");
        return -1;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[Trace] UART2 init failed\r\n");
        return -1;
    }
#if TRACE_BLE_DEBUG

    {
        WifiIotUartAttribute ble_attr = {
            .baudRate = BLE_DEBUG_BAUD,
            .dataBits = 8,
            .stopBits = 1,
            .parity = 0,
        };

        (void)UartDeinit(WIFI_IOT_UART_IDX_1);
        if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0,
            WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD) != WIFI_IOT_SUCCESS ||
            IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1,
            WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD) != WIFI_IOT_SUCCESS ||
            UartInit(WIFI_IOT_UART_IDX_1, &ble_attr, NULL) != WIFI_IOT_SUCCESS) {
            printf("[Trace] UART1 BLE debug init failed; continuing without it\r\n");
        }
    }
#endif
    return 0;
}

static int trace_create_thread(const char *name, osThreadFunc_t function)
{
    osThreadAttr_t attr = {0};

    attr.name = name;
    attr.stack_size = TRACE_THREAD_STACK_SIZE;
    attr.priority = TRACE_THREAD_PRIORITY;
    if (osThreadNew(function, NULL, &attr) == NULL) {
        printf("[Trace] thread create failed: %s\r\n", name);
        return -1;
    }
    return 0;
}

static void TraceFollowingEntry(void)
{
    if (trace_hardware_init() != 0) {
        return;
    }

    sensor_mutex = osMutexNew(NULL);
    if (sensor_mutex == NULL) {
        printf("[Trace] sensor mutex create failed\r\n");
        return;
    }
    status_mutex = osMutexNew(NULL);
    if (status_mutex == NULL) {
        printf("[Trace] status mutex create failed\r\n");
        return;
    }

    if (trace_create_thread("TraceAck", (osThreadFunc_t)trace_ack_thread) != 0) {
        return;
    }
    if (trace_create_thread("TraceSensor", (osThreadFunc_t)trace_sensor_thread) != 0) {
        return;
    }
    if (trace_create_thread("TraceControl", (osThreadFunc_t)trace_control_thread) != 0) {
        return;
    }

    printf("[Trace v7.3] ready: IR GPIO13/14, UART2 GPIO11/12 115200 + GET_STATUS 10Hz, BLE debug UART1 GPIO0/1 9600\r\n");
    ble_debug_printf("[Trace v7.3] ready: BLE debug link up\r\n");
}

APP_FEATURE_INIT(TraceFollowingEntry);
