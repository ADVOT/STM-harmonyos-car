/*
 * Safeguard (F.1): tabletop cliff avoidance + forward obstacle avoidance,
 * fused on the Task24 UART2 protocol link. Evolved from Phase F (F.0);
 * internal identifiers keep the phase_f_* prefix to minimize churn, logs
 * use the [Safeguard] tag.
 *
 * Infrared cliff sensors (unchanged from F.0):
 *   GPIO13 = left, GPIO14 = right
 *   raw low  = reflecting surface detected = floor present
 *   raw high = no reflecting surface       = possible cliff
 *
 * Forward sonar (HC-SR04, new):
 *   GPIO7 = Trig (output), GPIO8 = Echo (input)
 *   Distance bands: >40cm FAR (full speed) | 15..40cm SLOW | <=15cm NEAR
 *   (enter AVOID). No-echo timeout = far space = FAR; a stalled sonar
 *   thread degrades to FAR with a warning (cliff safety stays independent).
 *
 * Gimbal servo (SG90, GPIO2): a thread feeds a continuous pulse train (no
 * train = limp gimbal, vibration could steer the sonar off-axis). Locked
 * at 90 deg while DRIVING so the forward look is never blind; sweeps
 * L/C/R only while parked for AVOID judging (v11). v3: pulse
 * kernel-locked and priority above the sonar thread after the 8-30
 * twitch (hardware PWM cannot reach 50 Hz on this chip: u16 period
 * count at 24M/160M clock).
 *
 * Fusion rule: cliff wins over obstacle. EDGE states and RECOVER ignore
 * the sonar; an edge detected during AVOID preempts it via the same entry
 * path as NORMAL->EDGE.
 *
 * Obstacle avoidance style (8-30, USER): pivot in place (wheels
 * counter-rotate, ~zero turn radius) in closed-loop slices of 30~60 deg,
 * using the same odometry pulses-per-degree calibration as RECOVER. After
 * each slice the car parks and sweeps the servo over left/center/right
 * (v11): two raw pings per look, and the sweep MINIMUM decides - a
 * multipath echo can only inflate a reading, never shrink it. Minimum <=
 * NEAR keeps the pivot going (same direction within an episode,
 * alternating between episodes); otherwise the servo re-centers and the
 * car drives on. A full 360 deg of walled scans forces NORMAL as the
 * livelock guard.
 * v12 (8-31): sweep widened to +-60 deg (+-45 still missed the wall once
 * the heading was 70~80 deg off the wall normal - every beam landed
 * outside the +-15~20 deg specular window). And an all-echoless scan
 * inside a NEAR episode is no longer believed as "open": the wall can be
 * right there with all three beams flying. Up to AVOID_SCAN_ECHOLESS_MAX
 * consecutive blind scans escalate the next slice to a wide 90 deg pivot
 * before NORMAL is granted.
 * v13 (9-01, USER): servo duty is slew-rate limited (SERVO_SLEW_STEP_US per
 * ~21 ms pulse frame, ~340 deg/s vs the SG90's ~600 deg/s slam) so scans
 * glide instead of jerking; per-look settle windows budget each slew hop
 * (60/60/120 deg) so pings still fold on a stationary beam - distance
 * accuracy unchanged. The driving sanity scan no longer creeps: it runs
 * at the normal band speed, so an open-field cruise stays at full speed
 * instead of dipping to SLOW every 3 s scan. The scan keeps all three
 * L/C/R looks: mid-scan the band logic's NEAR check is skipped, so the
 * CENTER look is the only forward guard while sweeping (an L/R-only
 * trial build missed a hand placed dead ahead mid-sweep).
 * v13c (9-01, USER): DRIVE_SPEED 60 -> 120 (open-field cruise doubled; SLOW
 * stays 30, TURN/REVERSE stay 140); FAR_BLIND_SCAN_TICKS 30 -> 300 - the
 * counter ticks at the 10 ms control rate, so 30 was 300 ms of FAR, not
 * the documented 3 s, which made the servo sweep ~80% of the time on open
 * ground. 300 ticks restores the intended 3 s cadence.
 * v13d (9-01, USER): tuning - DRIVE_SPEED 150, SLOW band 40 speed over a
 * 25..15 cm strip (slow threshold 40 -> 25 cm: a 10 cm strip is enough
 * once the SLOW speed itself is 40; NEAR stays 15 cm).
 * History: v7/v8 moving arcs had too large a radius for a 15 cm trigger
 * (v7 log: 5.7 cm nose-stand at the wall); v9/v10 trusted the single
 * forward beam at settle time, and a multipath "valid FAR" at a shallow
 * angle walked the car obliquely into the wall.
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

/* 8-30 v7: readings verified after the /100 fix, full logs restored for the
   tabletop run (band transitions, AVOID entry, edge/recover must be visible).
   The dead-branch form keeps variables referenced and format strings
   type-checked under -Werror. */
#define LOG_SONAR_ONLY                    0
#if LOG_SONAR_ONLY
#define LOGF(...)                         do { if (0) { printf(__VA_ARGS__); } } while (0)
#else
#define LOGF(...)                         printf(__VA_ARGS__)
#endif

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
#define SERVO_HOLD_PRIORITY               26  /* above sonar busy-poll: refresh must not starve; a 1.5 ms pulse stretched into a sonar sample is filtered by the median */
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

#define DRIVE_SPEED                       150  /* 9-01 USER v13d: open-field cruise. SLOW 40 gives a visible ~4x step; TURN/REVERSE stay 140 so escape/pivot keeps the torque crown */
#define REVERSE_SPEED                     140  /* keep: escape torque must break static friction */
#define TURN_SPEED                        140  /* keep: in-place pivot has the highest torque demand */

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

/* Forward sonar (HC-SR04) on GPIO7/8; plain GPIO, no conflict with
   UART2 (11/12) or the infrared pair (13/14). */
#define SONAR_TRIG_PIN                    WIFI_IOT_IO_NAME_GPIO_7
#define SONAR_ECHO_PIN                    WIFI_IOT_IO_NAME_GPIO_8
#define ECHO_RISE_TIMEOUT_US              100000U /* trigger handling + flight time */
#define ECHO_FALL_TIMEOUT_US              5000U   /* cap at ~85 cm: beyond SLOW band top it is FAR anyway; keeps the busy-poll window short */
#define SONAR_SAMPLE_TICKS                6U      /* 60 ms: HC-SR04 min cycle time */
#define SONAR_STALE_TICKS                 20U     /* 200 ms silent thread => degrade */
#define SONAR_DEBUG_TICKS                 100U    /* 1 s periodic distance print */
#define SERVO_PIN                         WIFI_IOT_IO_NAME_GPIO_2 /* SG90 gimbal */
#define SERVO_CENTER_DUTY_US              1500U   /* 90 deg: sonar faces forward */
#define SERVO_SCAN_LEFT_DUTY_US           2167U   /* +60 deg look (8-31 v12; L/R verified on the car) */
#define SERVO_SCAN_RIGHT_DUTY_US          833U    /* -60 deg look */
#define SERVO_SLEW_STEP_US                80U     /* v13: max duty step per ~21 ms frame ~ 340 deg/s: scans glide, no slam */
#define OBSTACLE_NEAR_CM_X10              150U    /* 15 cm: enter AVOID (8-30 USER; 10 cm risks contact: median lag ~4-7 cm + arc advance) */
#define OBSTACLE_SLOW_CM_X10              250U    /* 25 cm: slow band (9-01 USER v13d; was 40 cm - a 10 cm strip down to NEAR is enough at SLOW 40) */
#define OBSTACLE_SLOW_SPEED               40   /* 9-01 USER v13d; was 30 - if it stalls, the car parks in the SLOW band (safe failure) */
#define AVOID_PIVOT_BASE_DEG              45U     /* 8-30 v9 USER: 30~60 deg closed-loop pivot slices */
#define AVOID_PIVOT_WIDE_DEG              90U     /* 8-31 v12: escalated slice after a blind (all-echoless) scan */
#define AVOID_PIVOT_JITTER_DEG            15U     /* +/-15 deg around the base */
#define AVOID_SCAN_ECHOLESS_MAX           2U      /* v12: blind scans to endure before "open" is believed */
#define AVOID_PIVOT_TIMEOUT_TICKS         200U    /* 2.0 s fail-safe if STATUS telemetry stalls */
/* FAR_BLIND_SCAN_TICKS: control loop ticks (10 ms each), NOT sonar samples.
   v13c fix: was 30 = 300 ms, so on open ground a fresh sweep started every
   ~0.3 s of FAR (servo sweeping ~80% of the time - the "never stops
   cranking" complaint). 300 ticks = the 3.0 s cadence this was always
   meant to be. */
#define FAR_BLIND_SCAN_TICKS              300U
#define AVOID_EPISODE_MAX_PULSES          (360U * TURN_PULSES_PER_DEG) /* full walled rotation forces NORMAL (livelock guard) */
#define AVOID_PHASE_PIVOT                 0U
#define AVOID_PHASE_SCAN                  1U

/* Scan look order: left, center, right. Both scans (driving sanity check
   and parked AVOID judging) use it: the CENTER look is the only forward
   guard while a sweep is running - the band logic's NEAR check is skipped
   for the whole scan (9-01 field test: a hand dead ahead mid-sweep went
   unseen with L/R-only looks). */
static const uint16_t AVOID_SCAN_DUTY_US[3] = {
    SERVO_SCAN_LEFT_DUTY_US, SERVO_CENTER_DUTY_US, SERVO_SCAN_RIGHT_DUTY_US
};

/* Per-look settle (10 ms ticks), budgeting the v13 slew hops + ring-down:
   start->L is a 60 deg hop (~190 ms slew), L->C another 60 deg, C->R a
   120 deg hop (~360 ms slew) - a flat 280 ms window let the R look fold
   mid-slew. */
static const uint32_t SCAN_LOOK_SETTLE_TICKS[3] = { 28U, 28U, 45U };

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

/* valid=0 means no echo in the sample window: open space, treated as FAR.
   raw_* is the latest single ping, unfiltered: the parked SCAN judging needs
   per-angle readings, which the rolling median would smear across looks. */
typedef struct {
    uint16_t dist_cm_x10;
    uint8_t valid;
    uint32_t updated_tick;
    uint32_t sequence;
    uint16_t raw_cm_x10;
    uint8_t raw_valid;
} SonarSnapshot;

typedef enum {
    BAND_FAR = 0,
    BAND_SLOW,
    BAND_NEAR
} ObstacleBand;

typedef enum {
    PHASE_F_NORMAL = 0,
    PHASE_F_EDGE_LEFT,
    PHASE_F_EDGE_RIGHT,
    PHASE_F_EDGE_BOTH,
    PHASE_F_RECOVER,
    PHASE_F_AVOID,
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
    uint32_t far_blind_ticks;     /* consecutive FAR ticks in NORMAL; triggers sanity scan */
    uint8_t blind_check_active;   /* 1 = sanity scan in progress (from NORMAL, not AVOID) */
    TurnDirection avoid_turn;     /* alternates per episode, then scan-informed */
    uint32_t avoid_target_pulses; /* closed-loop pivot target (differential pulses) */
    uint8_t avoid_episode;        /* 1 while a NEAR episode is in progress */
    uint32_t avoid_episode_pulses; /* cumulative pivoted pulses this episode (360 deg guard) */
    uint8_t avoid_phase;          /* AVOID_PHASE_*: pivoting, or parked scan judging */
    uint32_t avoid_phase_tick;    /* per-phase timer base (pivot timeout / scan settle) */
    uint8_t avoid_scan_idx;       /* 0..looks-1: which look is being taken */
    uint8_t avoid_scan_looks;     /* looks in the active scan: 3 (L/C/R) for both scan kinds */
    uint32_t avoid_scan_settle;   /* v13: per-look settle ticks for the active scan (slew + ring-down) */
    uint8_t avoid_scan_samples;   /* 0 = settling; 1 = collecting; +1 per folded ping */
    uint32_t avoid_scan_seq;      /* sonar sequence of the last folded ping */
    uint16_t avoid_scan_min;      /* running min raw distance at the current look */
    uint16_t avoid_scan_dist[3];  /* per-look result; 0xFFFF = echoless (open) */
    uint8_t avoid_scan_echoless;  /* v12: consecutive blind scans this episode */
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

static osMutexId_t sonar_mutex;
static SonarSnapshot sonar_snapshot;

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
        LOGF("[Safeguard] frame encode failed: cmd=0x%02X len=%u\r\n", cmd, len);
        return -1;
    }

    written = UartWrite(WIFI_IOT_UART_IDX_2, frame, frame_len);
    if (written != frame_len) {
        LOGF("[Safeguard] UART2 write failed: cmd=0x%02X wrote=%d/%u\r\n",
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
            LOGF("[Safeguard] STATUS invalid length: %u\r\n", parser->len);
            return;
        }
        status_publish(parser->payload); /* silent: polled at 10 Hz during RECOVER */
        return;
    }
    if (parser->cmd != PROTOCOL_CMD_ACK) {
        LOGF("[Safeguard] RX ignored: cmd=0x%02X len=%u\r\n", parser->cmd, parser->len);
        return;
    }
    if (parser->len != 2U) {
        LOGF("[Safeguard] ACK invalid length: %u\r\n", parser->len);
        return;
    }

    original_cmd = parser->payload[0];
    status = parser->payload[1];
    LOGF("[Safeguard] ACK %s (0x%02X): %s (%u)\r\n",
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
                LOGF("[Safeguard] RX bad length: %u\r\n", parser->len);
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
                LOGF("[Safeguard] RX checksum error: cmd=0x%02X\r\n", parser->cmd);
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
                LOGF("[Safeguard] UART2 read failed\r\n");
            } else {
                for (i = 0; i < count; i++) {
                    protocol_parse_byte(&parser, buffer[i]);
                }
            }
        } else if (result != WIFI_IOT_SUCCESS) {
            LOGF("[Safeguard] UART2 buffer check failed: %u\r\n", result);
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
                LOGF("[Safeguard] infrared read failed: left=%u right=%u\r\n",
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
                LOGF("[Safeguard] infrared read recovered; debouncing again\r\n");
            }
            was_healthy = 1U;

            if (left.ready != 0U && right.ready != 0U &&
                (last_logged_ready == 0U || left.stable != last_floor_left ||
                 right.stable != last_floor_right)) {
                LOGF("[Safeguard] sensor raw L=%u R=%u, floor L=%u R=%u\r\n",
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

static void sonar_publish(uint16_t dist_cm_x10, uint8_t valid,
    int32_t raw_sample, uint32_t now)
{
    if (osMutexAcquire(sonar_mutex, osWaitForever) != osOK) {
        return;
    }
    sonar_snapshot.dist_cm_x10 = dist_cm_x10;
    sonar_snapshot.valid = valid;
    sonar_snapshot.raw_cm_x10 =
        (raw_sample >= 0 && raw_sample <= 6553) ? (uint16_t)raw_sample : 0U;
    sonar_snapshot.raw_valid = (raw_sample >= 0) ? 1U : 0U;
    sonar_snapshot.updated_tick = now;
    sonar_snapshot.sequence++;
    osMutexRelease(sonar_mutex);
}

static SonarSnapshot sonar_take_snapshot(void)
{
    SonarSnapshot snapshot = {0};

    if (osMutexAcquire(sonar_mutex, osWaitForever) != osOK) {
        return snapshot;
    }
    snapshot = sonar_snapshot;
    osMutexRelease(sonar_mutex);
    return snapshot;
}

/* One HC-SR04 measurement; returns distance in cm x 10, or -1 on timeout
   (task 8 logic, integerized: cm x 10 = echo_us x 0.017 x 10 = echo_us x 17 / 100).
   8-30 v6: was /1000 by mistake - readings came out 10x small, which faked
   a permanent NEAR and looped AVOID. */
static int32_t sonar_measure_cm_x10(void)
{
    WifiIotGpioValue value;
    hi_u64 start, deadline, echo_us;

    GpioSetOutputVal(SONAR_TRIG_PIN, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(SONAR_TRIG_PIN, WIFI_IOT_GPIO_VALUE0);

    deadline = hi_get_us() + ECHO_RISE_TIMEOUT_US;
    do {
        GpioGetInputVal(SONAR_ECHO_PIN, &value);
        if (hi_get_us() > deadline) {
            return -1;
        }
    } while (value == WIFI_IOT_GPIO_VALUE0);
    start = hi_get_us();

    deadline = start + ECHO_FALL_TIMEOUT_US;
    do {
        GpioGetInputVal(SONAR_ECHO_PIN, &value);
        if (hi_get_us() > deadline) {
            return -1;
        }
    } while (value == WIFI_IOT_GPIO_VALUE1);
    echo_us = hi_get_us() - start;

    return (int32_t)((echo_us * 17ULL) / 100ULL);
}

/* Rolling median over the last 3 pings (one per 60 ms cycle). All-valid =>
   median; two valid => lower one (safer = nearer); one valid => itself;
   none valid => publish invalid, which the control side reads as FAR. */
static void phase_f_sonar_thread(void *arg)
{
    int32_t window[3] = {-1, -1, -1};
    uint8_t windex = 0U;
    uint8_t last_reported_valid = 0xFFU;
    uint16_t last_reported_dist = 0U;
    uint32_t last_debug_tick = 0U;

    (void)arg;

    while (1) {
        int32_t sample = sonar_measure_cm_x10();
        int32_t a, b, c;
        int32_t filtered = -1;
        uint32_t now = hi_get_tick();

        window[windex] = sample;
        windex = (uint8_t)((windex + 1U) % 3U);

        a = window[0];
        b = window[1];
        c = window[2];
        if (a >= 0 && b >= 0 && c >= 0) {
            if ((a >= b && b >= c) || (c >= b && b >= a)) {
                filtered = b;
            } else if ((b >= a && a >= c) || (c >= a && a >= b)) {
                filtered = a;
            } else {
                filtered = c;
            }
        } else if (a >= 0 && b >= 0) {
            filtered = (a < b) ? a : b;
        } else if (a >= 0 && c >= 0) {
            filtered = (a < c) ? a : c;
        } else if (b >= 0 && c >= 0) {
            filtered = (b < c) ? b : c;
        } else if (a >= 0) {
            filtered = a;
        } else if (b >= 0) {
            filtered = b;
        } else if (c >= 0) {
            filtered = c;
        }

        if (filtered >= 0) {
            uint16_t dist = (filtered > 6553) ? 6553U : (uint16_t)filtered;
            sonar_publish(dist, 1U, sample, now);
            if (last_reported_valid != 1U) {
                LOGF("[Safeguard] sonar echo acquired: %u.%u cm\r\n",
                    dist / 10U, dist % 10U);
            } else {
                uint16_t diff = (dist > last_reported_dist) ?
                    (uint16_t)(dist - last_reported_dist) :
                    (uint16_t)(last_reported_dist - dist);
                if (diff >= 100U) { /* log jumps >= 10 cm only */
                    LOGF("[Safeguard] sonar %u.%u cm\r\n", dist / 10U, dist % 10U);
                }
            }
            last_reported_valid = 1U;
            last_reported_dist = dist;
        } else {
            sonar_publish(0U, 0U, sample, now);
            if (last_reported_valid != 0U) {
                LOGF("[Safeguard] sonar: no echo (open space)\r\n");
            }
            last_reported_valid = 0U;
        }
        /* 1 Hz debug print: makes hand-wave calibration readable in the log.
           Stays printf even under LOG_SONAR_ONLY: this is the wanted line. */
        if ((uint32_t)(now - last_debug_tick) >= SONAR_DEBUG_TICKS) {
            last_debug_tick = now;
            if (filtered >= 0) {
                printf("[Safeguard] sonar dist=%ld.%ld cm\r\n",
                    (long)(filtered / 10), (long)(filtered % 10));
            } else {
                printf("[Safeguard] sonar dist=-- (no echo)\r\n");
            }
        }
        osDelay(SONAR_SAMPLE_TICKS);
    }
}

/* SG90 hold: the servo goes limp without a continuous pulse train, and a
   limp gimbal lets vibration steer the sonar off-axis.
   8-30 v3 (twitch fix): hardware PWM cannot reach 50 Hz here (u16 period
   count, 24M/160M clock => 366 Hz min), so pulses stay software-made.
   The v2 twitch came from starvation and stretched pulses: the sonar's
   busy polling at equal/higher priority delayed whole periods, and any
   preemption mid-pulse corrupted the 1.5 ms width. Now this thread runs
   above the sonar thread and the high pulse is kernel-locked (no thread
   switch; tick interrupts still run). Low part keeps osDelay: near-zero
   CPU. A pulse occasionally stretching a sonar fall measurement by
   1.5 ms is absorbed by the median-of-3 filter.
   v11: the duty is runtime-settable so a parked AVOID episode can sweep
   the sonar L/C/R. Single writer (control thread), single reader (this
   thread); the 16-bit store is atomic, so no mutex.
   v13 (9-01): the emitted width slews toward the target by at most
   SERVO_SLEW_STEP_US per frame instead of jumping - hops used to slam at
   the SG90's top speed (~600 deg/s), which read as constant cranking
   during the every-3 s sanity scans. At ~340 deg/s a 60 deg hop takes
   ~175 ms, a 120 deg hop ~350 ms; the scan settle windows budget for
   this, so pings still fold on a stationary beam. */
static volatile uint16_t servo_target_duty_us = SERVO_CENTER_DUTY_US;
static volatile uint16_t servo_slew_duty_us = SERVO_CENTER_DUTY_US; /* width being emitted right now */

static void servo_set_duty(uint16_t duty_us)
{
    servo_target_duty_us = duty_us;
}

static void phase_f_servo_thread(void *arg)
{
    (void)arg;

    while (1) {
        uint16_t target = servo_target_duty_us;
        uint16_t width = servo_slew_duty_us;

        if (width != target) {
            /* Saturating step toward the target; unsigned diff keeps the
               -Werror sign-compare check happy. */
            unsigned step = (width < target) ? (unsigned)(target - width)
                                             : (unsigned)(width - target);
            if (step > SERVO_SLEW_STEP_US) {
                step = SERVO_SLEW_STEP_US;
            }
            width = (uint16_t)((width < target) ? (width + step)
                                                : (width - step));
        }
        servo_slew_duty_us = width;

        osKernelLock();
        GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE1);
        hi_udelay(width);
        GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE0);
        osKernelUnlock();
        osDelay(2U); /* ~20 ms low part: refresh ~= 47 Hz, within SG90 tolerance */
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
        case PHASE_F_AVOID:
            return "AVOID";
        case PHASE_F_FAULT_STOP:
            return "FAULT_STOP";
        default:
            return "UNKNOWN";
    }
}

static const char *phase_f_band_name(ObstacleBand band)
{
    switch (band) {
        case BAND_FAR:
            return "FAR";
        case BAND_SLOW:
            return "SLOW";
        case BAND_NEAR:
            return "NEAR";
        default:
            return "UNKNOWN";
    }
}

static uint8_t sonar_is_fresh(const SonarSnapshot *sonar, uint32_t now)
{
    if (sonar->sequence == 0U) {
        return 0U;
    }
    return (uint8_t)((uint32_t)(now - sonar->updated_tick) <= SONAR_STALE_TICKS);
}

/* Fail-open by design: stale thread or no echo both read as FAR; the cliff
   link is independent and keeps its own hard STOP semantics. */
static ObstacleBand obstacle_band(const SonarSnapshot *sonar, uint32_t now)
{
    if (sonar_is_fresh(sonar, now) == 0U || sonar->valid == 0U) {
        return BAND_FAR;
    }
    if (sonar->dist_cm_x10 <= OBSTACLE_NEAR_CM_X10) {
        return BAND_NEAR;
    }
    if (sonar->dist_cm_x10 <= OBSTACLE_SLOW_CM_X10) {
        return BAND_SLOW;
    }
    return BAND_FAR;
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
    LOGF("[Safeguard] state %s -> %s, floor L=%u R=%u\r\n",
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
    LOGF("[Safeguard] recover turn %s, target=%lu pulses\r\n",
        controller->recover_turn == TURN_RIGHT ? "RIGHT" : "LEFT",
        (unsigned long)controller->recover_target_pulses);
    phase_f_enter_state(controller, PHASE_F_RECOVER, sensor, now);
}

/* Shared edge entry from any driving state (NORMAL or AVOID): snapshot the
   safe forward mileage as the blind reverse duration. */
static void phase_f_enter_edge(PhaseFController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    controller->recover_retries = 0U;
    controller->avoid_episode = 0U; /* a cliff interrupt ends any avoid episode */
    controller->avoid_episode_pulses = 0U;
    controller->avoid_scan_echoless = 0U;
    controller->far_blind_ticks = 0U;
    controller->blind_check_active = 0U;
    servo_set_duty(SERVO_CENTER_DUTY_US); /* an edge can preempt mid-scan */
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

/* One pivot step, closed-loop on STM32 odometry (same base-capture pattern
   as RECOVER). Direction is chosen by the caller (scan-informed when scan
   data is available; alternating toggle for the first pivot of a fresh
   episode). base_deg is AVOID_PIVOT_BASE_DEG normally; a blind scan
   escalates to AVOID_PIVOT_WIDE_DEG so the next scan lands far outside the
   shallow specular-blind band. */
static void phase_f_start_pivot(PhaseFController *controller, uint32_t now,
    int32_t base_deg)
{
    int32_t jitter_deg = (int32_t)(phase_f_next_rand() %
        (2U * AVOID_PIVOT_JITTER_DEG + 1U)) - (int32_t)AVOID_PIVOT_JITTER_DEG;
    int32_t deg = base_deg + jitter_deg;

    if (controller->avoid_episode == 0U) {
        controller->avoid_turn = (controller->avoid_turn == TURN_RIGHT) ?
            TURN_LEFT : TURN_RIGHT;
    }
    controller->avoid_episode = 1U;
    controller->avoid_target_pulses =
        (uint32_t)(deg * (int32_t)TURN_PULSES_PER_DEG);
    controller->avoid_episode_pulses += controller->avoid_target_pulses;
    controller->odo_base_valid = 0U;
    controller->odo_base_seq = status_take_snapshot().sequence;
    controller->last_status_poll_tick = now;
    (void)protocol_send_get_status();
    servo_set_duty(SERVO_CENTER_DUTY_US); /* pivot with the beam forward */
    controller->avoid_phase = AVOID_PHASE_PIVOT;
    controller->avoid_phase_tick = now;
    LOGF("[Safeguard] avoid pivot%s %s, target=%lu pulses\r\n",
        base_deg >= (int32_t)AVOID_PIVOT_WIDE_DEG ? " wide" : "",
        controller->avoid_turn == TURN_RIGHT ? "RIGHT" : "LEFT",
        (unsigned long)controller->avoid_target_pulses);
}

static void phase_f_enter_avoid(PhaseFController *controller,
    const SensorSnapshot *sensor, uint32_t now)
{
    phase_f_start_pivot(controller, now, (int32_t)AVOID_PIVOT_BASE_DEG);
    phase_f_enter_state(controller, PHASE_F_AVOID, sensor, now);
}

/* Scan start (v11): first look is LEFT; each look settles its per-look
   window (SCAN_LOOK_SETTLE_TICKS), then folds two fresh raw pings. Both
   the driving sanity scan and the parked AVOID scan take the same three
   L/C/R looks. */
static void phase_f_start_scan(PhaseFController *controller, uint32_t now)
{
    controller->avoid_phase = AVOID_PHASE_SCAN;
    controller->avoid_phase_tick = now;
    controller->avoid_scan_idx = 0U;
    controller->avoid_scan_samples = 0U;
    controller->avoid_scan_seq = 0U;
    controller->avoid_scan_min = 0xFFFFU;
    controller->avoid_scan_looks = 3U;
    controller->avoid_scan_settle = SCAN_LOOK_SETTLE_TICKS[0];
    servo_set_duty(AVOID_SCAN_DUTY_US[0]);
}

static void phase_f_update_controller(PhaseFController *controller,
    const SensorSnapshot *sensor, const SonarSnapshot *sonar, uint32_t now)
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
            controller->far_blind_ticks = 0U;
            controller->blind_check_active = 0U;
            phase_f_enter_state(controller, PHASE_F_NORMAL, sensor, now);
            LOGF("[Safeguard] controller armed\r\n");
        }
        return;
    }

    if (controller->active == 0U) {
        return;
    }

    elapsed = (uint32_t)(now - controller->state_enter_tick);
    switch (controller->state) {
        case PHASE_F_NORMAL:
            /* Cliff wins over obstacle: floor loss is checked first. */
            if (sensor->floor_left == 0U || sensor->floor_right == 0U) {
                phase_f_enter_edge(controller, sensor, now);
            } else if (controller->blind_check_active) {
                /* Sanity scan polling (v13 revised 9-01): three L/C/R
                   looks, same collection as the AVOID scan, but returns
                   to NORMAL instead of pivoting. The CENTER look matters:
                   the band logic's NEAR check is skipped for the whole
                   scan, so C is the only forward guard while sweeping
                   (a hand dead ahead mid-sweep is invisible to L/R). */
                if (controller->avoid_scan_samples == 0U) {
                    if ((uint32_t)(now - controller->avoid_phase_tick) >=
                        controller->avoid_scan_settle) {
                        controller->avoid_scan_seq = sonar->sequence;
                        controller->avoid_scan_min = 0xFFFFU;
                        controller->avoid_scan_samples = 1U;
                    }
                } else if (sonar->sequence != controller->avoid_scan_seq) {
                    controller->avoid_scan_seq = sonar->sequence;
                    if (sonar->raw_valid != 0U &&
                        sonar->raw_cm_x10 < controller->avoid_scan_min) {
                        controller->avoid_scan_min = sonar->raw_cm_x10;
                    }
                    controller->avoid_scan_samples++;
                    if (controller->avoid_scan_samples >= 3U) {
                        controller->avoid_scan_dist[controller->avoid_scan_idx] =
                            controller->avoid_scan_min;
                        if (controller->avoid_scan_idx <
                            (uint8_t)(controller->avoid_scan_looks - 1U)) {
                            controller->avoid_scan_idx++;
                            servo_set_duty(AVOID_SCAN_DUTY_US[controller->avoid_scan_idx]);
                            controller->avoid_scan_settle =
                                SCAN_LOOK_SETTLE_TICKS[controller->avoid_scan_idx];
                            controller->avoid_phase_tick = now;
                            controller->avoid_scan_samples = 0U;
                        } else {
                            uint16_t dl = controller->avoid_scan_dist[0];
                            uint16_t dc = controller->avoid_scan_dist[1];
                            uint16_t dr = controller->avoid_scan_dist[2];
                            uint16_t dmin = (dl < dc) ? dl : dc;
                            uint16_t pl = (dl == 0xFFFFU) ? 8888U : dl;
                            uint16_t pc = (dc == 0xFFFFU) ? 8888U : dc;
                            uint16_t pr = (dr == 0xFFFFU) ? 8888U : dr;

                            dmin = (dmin < dr) ? dmin : dr;
                            LOGF("[Safeguard] blind check scan L=%u.%u C=%u.%u R=%u.%u cm (888.8=echoless)\r\n",
                                pl / 10U, pl % 10U, pc / 10U, pc % 10U,
                                pr / 10U, pr % 10U);
                            servo_set_duty(SERVO_CENTER_DUTY_US);
                            controller->blind_check_active = 0U;
                            if (dmin <= OBSTACLE_NEAR_CM_X10) {
                                if (dl != 0xFFFFU || dr != 0xFFFFU) {
                                    if (dl == 0xFFFFU) {
                                        controller->avoid_turn = TURN_LEFT;
                                    } else if (dr == 0xFFFFU) {
                                        controller->avoid_turn = TURN_RIGHT;
                                    } else if (dl > dr) {
                                        controller->avoid_turn = TURN_LEFT;
                                    } else if (dr > dl) {
                                        controller->avoid_turn = TURN_RIGHT;
                                    }
                                }
                                LOGF("[Safeguard] blind check: wall found, entering AVOID\r\n");
                                controller->avoid_episode = 1U; /* prevent toggle override of scan-informed direction */
                                phase_f_enter_avoid(controller, sensor, now);
                            } else {
                                LOGF("[Safeguard] blind check: open, resuming NORMAL\r\n");
                            }
                        }
                    }
                }
            } else if (obstacle_band(sonar, now) == BAND_NEAR) {
                phase_f_enter_avoid(controller, sensor, now);
            } else {
                ObstacleBand band = obstacle_band(sonar, now);
                if (band == BAND_FAR) {
                    controller->far_blind_ticks++;
                    if (controller->far_blind_ticks >= FAR_BLIND_SCAN_TICKS) {
                        LOGF("[Safeguard] FAR blind for %u ticks, sanity scan\r\n",
                            FAR_BLIND_SCAN_TICKS);
                        controller->blind_check_active = 1U;
                        controller->far_blind_ticks = 0U;
                        phase_f_start_scan(controller, now);
                        break;
                    }
                } else {
                    controller->far_blind_ticks = 0U;
                }
                /* A non-NEAR beat in NORMAL closes the episode: the next
                   NEAR is treated as a new obstacle and may pivot either way. */
                controller->avoid_episode = 0U;
                controller->avoid_episode_pulses = 0U;
                controller->avoid_scan_echoless = 0U;
            }
            break;

        case PHASE_F_AVOID:
            /* Edge preempts the pivot immediately, same entry as from NORMAL. */
            if (sensor->floor_left == 0U || sensor->floor_right == 0U) {
                phase_f_enter_edge(controller, sensor, now);
                break;
            }
            if (controller->avoid_phase == AVOID_PHASE_PIVOT) {
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
                if (controller->odo_base_valid != 0U &&
                    progress >= controller->avoid_target_pulses) {
                    done = 1U;
                }
                /* Fail-safe: a telemetry stall must not spin the car forever. */
                if ((uint32_t)(now - controller->avoid_phase_tick) >=
                    AVOID_PIVOT_TIMEOUT_TICKS) {
                    if (done == 0U) {
                        LOGF("[Safeguard] avoid pivot timeout: achieved=%lu/%lu pulses (base_valid=%u)\r\n",
                            (unsigned long)progress,
                            (unsigned long)controller->avoid_target_pulses,
                            controller->odo_base_valid);
                    }
                    done = 1U;
                }
                if (done != 0U) {
                    phase_f_start_scan(controller, now);
                }
            } else {
                /* SCAN judging (v11/v12): parked, servo sweeps L/C/R. Per
                   look: the scan's settle window for slew + ring-down,
                   then two fresh raw pings, lower one wins. A multipath
                   echo can only inflate a reading, never shrink it, so
                   the sweep minimum is the trustworthy wall distance. All
                   three looks echoless (0xFFFF) is NOT proof of open
                   space - a wall past the specular window returns nothing
                   either - so a blind scan escalates the pivot instead of
                   releasing NORMAL. */
                if (controller->avoid_scan_samples == 0U) {
                    if ((uint32_t)(now - controller->avoid_phase_tick) >=
                        controller->avoid_scan_settle) {
                        controller->avoid_scan_seq = sonar->sequence;
                        controller->avoid_scan_min = 0xFFFFU;
                        controller->avoid_scan_samples = 1U;
                    }
                } else if (sonar->sequence != controller->avoid_scan_seq) {
                    controller->avoid_scan_seq = sonar->sequence;
                    if (sonar->raw_valid != 0U &&
                        sonar->raw_cm_x10 < controller->avoid_scan_min) {
                        controller->avoid_scan_min = sonar->raw_cm_x10;
                    }
                    controller->avoid_scan_samples++;
                    if (controller->avoid_scan_samples >= 3U) {
                        controller->avoid_scan_dist[controller->avoid_scan_idx] =
                            controller->avoid_scan_min;
                        if (controller->avoid_scan_idx <
                            (uint8_t)(controller->avoid_scan_looks - 1U)) {
                            controller->avoid_scan_idx++;
                            servo_set_duty(AVOID_SCAN_DUTY_US[controller->avoid_scan_idx]);
                            controller->avoid_scan_settle =
                                SCAN_LOOK_SETTLE_TICKS[controller->avoid_scan_idx];
                            controller->avoid_phase_tick = now;
                            controller->avoid_scan_samples = 0U;
                        } else {
                            uint16_t dl = controller->avoid_scan_dist[0];
                            uint16_t dc = controller->avoid_scan_dist[1];
                            uint16_t dr = controller->avoid_scan_dist[2];
                            uint16_t dmin = (dl < dc) ? dl : dc;
                            uint16_t pl = (dl == 0xFFFFU) ? 8888U : dl;
                            uint16_t pc = (dc == 0xFFFFU) ? 8888U : dc;
                            uint16_t pr = (dr == 0xFFFFU) ? 8888U : dr;

                            dmin = (dmin < dr) ? dmin : dr;
                            LOGF("[Safeguard] avoid scan L=%u.%u C=%u.%u R=%u.%u cm (888.8=echoless)\r\n",
                                pl / 10U, pl % 10U, pc / 10U, pc % 10U,
                                pr / 10U, pr % 10U);
                            servo_set_duty(SERVO_CENTER_DUTY_US);
                            if (dmin <= OBSTACLE_NEAR_CM_X10) {
                                controller->avoid_scan_echoless = 0U;
                                if (controller->avoid_episode_pulses <
                                    AVOID_EPISODE_MAX_PULSES) {
                                    /* Scan-informed direction: steer toward
                                       the more open side (v12+). */
                                    if (dl != 0xFFFFU || dr != 0xFFFFU) {
                                        if (dl == 0xFFFFU) {
                                            controller->avoid_turn = TURN_LEFT;
                                        } else if (dr == 0xFFFFU) {
                                            controller->avoid_turn = TURN_RIGHT;
                                        } else if (dl > dr) {
                                            controller->avoid_turn = TURN_LEFT;
                                        } else if (dr > dl) {
                                            controller->avoid_turn = TURN_RIGHT;
                                        }
                                        /* dl == dr: keep current direction */
                                    }
                                    phase_f_start_pivot(controller, now,
                                        (int32_t)AVOID_PIVOT_BASE_DEG);
                                } else {
                                    LOGF("[Safeguard] avoid episode: 360 deg still walled; forcing NORMAL\r\n");
                                    phase_f_enter_state(controller,
                                        PHASE_F_NORMAL, sensor, now);
                                }
                            } else if (dmin == 0xFFFFU &&
                                controller->avoid_scan_echoless <
                                AVOID_SCAN_ECHOLESS_MAX) {
                                /* v12: all three beams missed inside a NEAR
                                   episode - the wall can be right there past
                                   the specular window. Distrust "open": step
                                   up to a wide pivot and scan again. */
                                controller->avoid_scan_echoless++;
                                LOGF("[Safeguard] avoid scan blind (%u/%u), escalating\r\n",
                                    controller->avoid_scan_echoless,
                                    AVOID_SCAN_ECHOLESS_MAX);
                                phase_f_start_pivot(controller, now,
                                    (int32_t)AVOID_PIVOT_WIDE_DEG);
                            } else {
                                controller->avoid_scan_echoless = 0U;
                                phase_f_enter_state(controller, PHASE_F_NORMAL,
                                    sensor, now);
                            }
                        }
                    }
                }
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
                    LOGF("[Safeguard] recover timeout: achieved=%lu/%lu pulses (base_valid=%u)\r\n",
                        (unsigned long)progress,
                        (unsigned long)controller->recover_target_pulses,
                        controller->odo_base_valid);
                }
                done = 1U;
            }

            if (done != 0U) {
                controller->recover_motion_done = 1U;
                LOGF("[Safeguard] recover done: target=%lu achieved=%lu pulses\r\n",
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
                    LOGF("[Safeguard] recovery limit reached; latched STOP\r\n");
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
    const SonarSnapshot *sonar, uint32_t now)
{
    MotionCommand command = {0, 0, 1U};
    uint32_t elapsed;

    if (controller->armed == 0U || controller->active == 0U) {
        return command;
    }

    elapsed = (uint32_t)(now - controller->state_enter_tick);
    switch (controller->state) {
        case PHASE_F_NORMAL: {
            int16_t speed;

            /* v13: no creep during the sanity scan. Worst forward-blind
               stretch is the R look + re-center slew + median fill at
               DRIVE_SPEED (~1 s, ~3 cm); the C look guards the middle of
               the sweep, the SLOW/NEAR margins still hold, and the old
               creep halved the open-field cruise every 3 s scan. */
            speed = (obstacle_band(sonar, now) == BAND_SLOW) ?
                OBSTACLE_SLOW_SPEED : DRIVE_SPEED;

            command.left = speed;
            command.right = speed;
            command.stop = 0U;
            break;
        }

        case PHASE_F_AVOID:
            /* Pivot in place: wheels counter-rotate (same mapping as
               RECOVER). SETTLE and the timeout tail hold the default STOP. */
            if (controller->avoid_phase == AVOID_PHASE_PIVOT &&
                (uint32_t)(now - controller->avoid_phase_tick) <
                AVOID_PIVOT_TIMEOUT_TICKS) {
                if (controller->avoid_turn == TURN_RIGHT) {
                    command.left = TURN_SPEED;
                    command.right = -TURN_SPEED;
                } else {
                    command.left = -TURN_SPEED;
                    command.right = TURN_SPEED;
                }
                command.stop = 0U;
            }
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
    uint8_t sonar_stale_logged = 0U;
    uint8_t reset_requested = 0U;
    ObstacleBand last_band = BAND_FAR;
    uint32_t guard_start;

    (void)arg;
    controller.recover_turn = TURN_RIGHT;
    controller.avoid_turn = TURN_RIGHT;

    LOGF("[Safeguard] control started: motion=%d speed=%d near=%u slow=%u (cm x10)\r\n",
        PHASE_F_ENABLE_MOTION, DRIVE_SPEED,
        OBSTACLE_NEAR_CM_X10, OBSTACLE_SLOW_CM_X10);
    protocol_send_ping();
    protocol_send_stop();

    guard_start = hi_get_tick();
    while ((uint32_t)(hi_get_tick() - guard_start) < STARTUP_GUARD_TICKS) {
        phase_f_send_command(&(MotionCommand){0, 0, 1U}, hi_get_tick());
        osDelay(CONTROL_LOOP_TICKS);
    }

#if !PHASE_F_ENABLE_MOTION
    LOGF("[Safeguard] motion disabled; all commands are forced STOP\r\n");
#endif

    while (1) {
        SensorSnapshot sensor = sensor_take_snapshot();
        SonarSnapshot sonar = sonar_take_snapshot();
        uint32_t now = hi_get_tick();
        MotionCommand command = {0, 0, 1U};

        if (phase_f_sensor_is_fresh(&sensor, now) == 0U) {
            if (reset_requested == 0U) {
                sensor_request_reset();
                reset_requested = 1U;
            }
            if (sensor_unavailable_logged == 0U) {
                LOGF("[Safeguard] sensor data unavailable or stale; motors stopped\r\n");
                sensor_unavailable_logged = 1U;
            }
            if (sensor.sequence != 0U &&
                (uint32_t)(now - sensor.updated_tick) > SENSOR_STALE_TICKS &&
                stale_logged == 0U) {
                LOGF("[Safeguard] sensor snapshot stale; forcing local STOP\r\n");
                stale_logged = 1U;
            }
            controller.active = 0U;
            controller.armed = 0U;
            controller.recover_retries = 0U;
            controller.safe_forward_ticks = 0U;
        } else {
            ObstacleBand band = obstacle_band(&sonar, now);

            reset_requested = 0U;
            sensor_unavailable_logged = 0U;
            stale_logged = 0U;
            if (sonar_is_fresh(&sonar, now) == 0U) {
                if (sonar_stale_logged == 0U) {
                    LOGF("[Safeguard] sonar stale; obstacle avoidance degraded to FAR\r\n");
                    sonar_stale_logged = 1U;
                }
            } else {
                sonar_stale_logged = 0U;
            }
            if (band != last_band) {
                LOGF("[Safeguard] obstacle band %s -> %s (dist=%u.%u cm, valid=%u)\r\n",
                    phase_f_band_name(last_band), phase_f_band_name(band),
                    sonar.dist_cm_x10 / 10U, sonar.dist_cm_x10 % 10U, sonar.valid);
                last_band = band;
            }
            phase_f_update_controller(&controller, &sensor, &sonar, now);
            command = phase_f_motion_command(&controller, &sonar, now);
        }

        phase_f_send_command(&command, now);
        if (controller.armed != 0U && controller.active != 0U &&
            (controller.state == PHASE_F_RECOVER ||
             (controller.state == PHASE_F_AVOID &&
              controller.avoid_phase == AVOID_PHASE_PIVOT)) &&
            (uint32_t)(now - controller.last_status_poll_tick) >= STATUS_POLL_TICKS) {
            /* Telemetry cadence for the closed-loop turns; doubles as a second
               heartbeat alongside the 100 ms command resend. */
            controller.last_status_poll_tick = now;
            (void)protocol_send_get_status();
        }
        if (controller.armed != 0U && controller.active != 0U &&
            controller.state == PHASE_F_NORMAL &&
            controller.safe_forward_ticks < EDGE_REVERSE_MAX_TICKS) {
            /* NORMAL always drives forward (full or slow band), so elapsed
               beats measure distance over ground already proven safe. Time
               beats over-count distance in the SLOW band (40 vs the 140
               reverse), biasing the blind reverse long onto previously
               driven ground: the safe side. */
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
        LOGF("[Safeguard] GPIO init failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11,
        WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] GPIO11 UART2 TX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12,
        WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] GPIO12 UART2 RX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(IR_LEFT_PIN, WIFI_IOT_IO_FUNC_GPIO_13_GPIO) != WIFI_IOT_SUCCESS ||
        IoSetFunc(IR_RIGHT_PIN, WIFI_IOT_IO_FUNC_GPIO_14_GPIO) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] infrared GPIO function setup failed\r\n");
        return -1;
    }
    if (GpioSetDir(IR_LEFT_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS ||
        GpioSetDir(IR_RIGHT_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] infrared GPIO direction setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(SONAR_TRIG_PIN, WIFI_IOT_IO_FUNC_GPIO_7_GPIO) != WIFI_IOT_SUCCESS ||
        IoSetFunc(SONAR_ECHO_PIN, WIFI_IOT_IO_FUNC_GPIO_8_GPIO) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] sonar GPIO function setup failed\r\n");
        return -1;
    }
    if (GpioSetDir(SONAR_TRIG_PIN, WIFI_IOT_GPIO_DIR_OUT) != WIFI_IOT_SUCCESS ||
        GpioSetDir(SONAR_ECHO_PIN, WIFI_IOT_GPIO_DIR_IN) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] sonar GPIO direction setup failed\r\n");
        return -1;
    }
    if (GpioSetOutputVal(SONAR_TRIG_PIN, WIFI_IOT_GPIO_VALUE0) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] sonar trigger idle-low setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(SERVO_PIN, WIFI_IOT_IO_FUNC_GPIO_2_GPIO) != WIFI_IOT_SUCCESS ||
        GpioSetDir(SERVO_PIN, WIFI_IOT_GPIO_DIR_OUT) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] servo GPIO setup failed\r\n");
        return -1;
    }
    GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE0);
    /* Only UART2 is initialized: this HAL allows exactly one user UartInit
       (8-29, G.0 BLE experiments), and the sonar needs no UART. */
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        LOGF("[Safeguard] UART2 init failed\r\n");
        return -1;
    }
    return 0;
}

static int phase_f_create_thread(const char *name, osThreadFunc_t function,
    osPriority_t priority)
{
    osThreadAttr_t attr = {0};

    attr.name = name;
    attr.stack_size = PHASE_F_THREAD_STACK_SIZE;
    attr.priority = priority;
    if (osThreadNew(function, NULL, &attr) == NULL) {
        LOGF("[Safeguard] thread create failed: %s\r\n", name);
        return -1;
    }
    return 0;
}

static void TabletopSafeguardEntry(void)
{
    if (phase_f_hardware_init() != 0) {
        return;
    }

    sensor_mutex = osMutexNew(NULL);
    if (sensor_mutex == NULL) {
        LOGF("[Safeguard] sensor mutex create failed\r\n");
        return;
    }
    status_mutex = osMutexNew(NULL);
    if (status_mutex == NULL) {
        LOGF("[Safeguard] status mutex create failed\r\n");
        return;
    }
    sonar_mutex = osMutexNew(NULL);
    if (sonar_mutex == NULL) {
        LOGF("[Safeguard] sonar mutex create failed\r\n");
        return;
    }

    if (phase_f_create_thread("SafeguardAck", (osThreadFunc_t)phase_f_ack_thread,
            (osPriority_t)PHASE_F_THREAD_PRIORITY) != 0) {
        return;
    }
    if (phase_f_create_thread("SafeguardSensor", (osThreadFunc_t)phase_f_sensor_thread,
            (osPriority_t)PHASE_F_THREAD_PRIORITY) != 0) {
        return;
    }
    if (phase_f_create_thread("SafeguardSonar", (osThreadFunc_t)phase_f_sonar_thread,
            (osPriority_t)PHASE_F_THREAD_PRIORITY) != 0) {
        return;
    }
    if (phase_f_create_thread("SafeguardServo", (osThreadFunc_t)phase_f_servo_thread,
            (osPriority_t)SERVO_HOLD_PRIORITY) != 0) {
        return;
    }
    if (phase_f_create_thread("SafeguardControl", (osThreadFunc_t)phase_f_control_thread,
            (osPriority_t)PHASE_F_THREAD_PRIORITY) != 0) {
        return;
    }

    LOGF("[Safeguard] ready v13d: IR GPIO13/14, sonar GPIO7/8, servo GPIO2 slew "
         "80us/frame hold 90deg, UART2 GPIO11/12, 115200 8N1\r\n");
}

APP_FEATURE_INIT(TabletopSafeguardEntry);
