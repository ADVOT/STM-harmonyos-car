/*
 * 9.0_Oled_Uart_Light: STM32 UART2 telemetry on OLED + AP3216C light-gated display
 *
 * Combo of task11 (SSD1306 OLED), the upcoming AP3216C task, and the task24
 * dual-core UART protocol:
 *   - UART2 (GPIO11/12, 115200 8N1) polls GET_STATUS every 200 ms; the STM32
 *     answers with STATUS(0x82): odo L/R i32 LE, speed L/R i16 LE, lease flag.
 *     No motion commands are ever sent (query-only link, motors untouched).
 *   - AP3216C ALS sampled every 300 ms; a hysteresis gate drives the panel:
 *       als > TH + HYST => light => SSD1306_OFF (sleep, <10 uA, GDDRAM kept)
 *       als < TH - HYST => dark  => SSD1306_ON
 *   - OLED and AP3216C share I2C0 (GPIO9/10): every I2C access lives in the
 *     single panel thread, so transactions cannot interleave (no bus mutex).
 *   - GDDRAM writes stay valid while the panel sleeps, so the refresh loop
 *     runs regardless of the ON/OFF state.
 *
 * Caution: this HAL allows exactly one user UartInit (verified 8-29); this
 * application initializes UART2 only.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_uart_ex.h"
#include "hi_time.h"
#include "hal_bsp_ssd1306.h"
#include "hal_bsp_ap3216c.h"

/* The SDK provides this symbol even when older headers omit the declaration. */
extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

#define PROTOCOL_SOF                      0xAAU
#define MAX_PAYLOAD                       16U
#define PROTOCOL_CMD_GET_STATUS           0x04U
#define PROTOCOL_CMD_ACK                  0x81U
#define PROTOCOL_CMD_STATUS               0x82U

/* STATUS payload: odo_left i32 LE | odo_right i32 LE |
   speed_left i16 LE | speed_right i16 LE | flags u8 (bit0 = motion lease). */
#define STATUS_PAYLOAD_LEN                13U

/* Hi3861 runs a 10 ms RTOS tick in this project. */
#define RX_POLL_TICKS                     1U    /* 10 ms */
#define STATUS_POLL_TICKS                 20U   /* 200 ms GET_STATUS cadence */
#define PANEL_LOOP_TICKS                  30U   /* 300 ms: ALS sample + refresh */

/* ALS is a 16-bit raw count, not lux. Gate fixed by Tao on 8-31 desk test:
   below 100 = dark (panel ON), above 100 = lit (panel OFF); exactly 100
   holds the current state (single-count dead band against boundary jitter).
   If the panel's own glow ever oscillates the reading, widen HYSTERESIS. */
#define ALS_LIGHT_THRESHOLD               100U
#define ALS_HYSTERESIS                    0U

#define PANEL_STACK_SIZE                  4096U  /* sprintf users need room */
#define RX_STACK_SIZE                     2048U
#define TX_STACK_SIZE                     1024U
#define TASK_PRIORITY                     25

#define RX_BUFFER_SIZE                    64U
#define OLED_LINE_CHARS                   16U    /* 8x16 font: 16 columns */

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
    int32_t odo_left;
    int32_t odo_right;
    int16_t speed_left;
    int16_t speed_right;
    uint8_t flags;
    uint32_t sequence; /* STATUS frames received: link heartbeat */
} StatusSnapshot;

static osMutexId_t status_mutex;
static StatusSnapshot status_snapshot;

/* ---------------- UART2 protocol ---------------- */

static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t i;
    uint8_t checksum = (uint8_t)(cmd + len);

    for (i = 0U; i < len; i++) {
        checksum = (uint8_t)(checksum + payload[i]);
    }
    return checksum;
}

/* GET_STATUS carries no payload: AA 04 00 04 */
static int protocol_send_get_status(void)
{
    uint8_t frame[4];
    int written;

    frame[0] = PROTOCOL_SOF;
    frame[1] = PROTOCOL_CMD_GET_STATUS;
    frame[2] = 0U;
    frame[3] = protocol_checksum(PROTOCOL_CMD_GET_STATUS, 0U, NULL);
    written = UartWrite(WIFI_IOT_UART_IDX_2, frame, sizeof(frame));
    if (written != (int)sizeof(frame)) {
        printf("[OUL] UART2 write GET_STATUS failed: %d/%u\r\n", written, (unsigned int)sizeof(frame));
        return -1;
    }
    return 0;
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
    status_snapshot.sequence++;
    osMutexRelease(status_mutex);
}

static StatusSnapshot status_take(void)
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
    if (parser->cmd == PROTOCOL_CMD_STATUS) {
        if (parser->len != STATUS_PAYLOAD_LEN) {
            printf("[OUL] STATUS bad length: %u\r\n", parser->len);
            return;
        }
        status_publish(parser->payload); /* silent: polled at 5 Hz */
        return;
    }
    /* Nothing we send should trigger an ACK; log it as a diagnostic. */
    printf("[OUL] RX ignored: cmd=0x%02X len=%u\r\n", parser->cmd, parser->len);
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
                printf("[OUL] RX bad length: %u\r\n", parser->len);
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
                printf("[OUL] RX checksum error: cmd=0x%02X\r\n", parser->cmd);
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

/* ---------------- threads ---------------- */

static void uart_rx_thread(void *arg)
{
    uint8_t buffer[RX_BUFFER_SIZE];
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
                printf("[OUL] UART2 read failed\r\n");
            } else {
                for (i = 0; i < count; i++) {
                    protocol_parse_byte(&parser, buffer[i]);
                }
            }
        } else if (result != WIFI_IOT_SUCCESS) {
            printf("[OUL] UART2 buffer check failed: %u\r\n", result);
        }
        osDelay(RX_POLL_TICKS);
    }
}

static void status_tx_thread(void *arg)
{
    (void)arg;

    while (1) {
        protocol_send_get_status();
        osDelay(STATUS_POLL_TICKS);
    }
}

/* OLED + AP3216C on one I2C bus: owning both from this single thread keeps
   transactions atomic without a mutex. */
static void panel_thread(void *arg)
{
    char line[OLED_LINE_CHARS + 1U];
    uint16_t ir = 0U;
    uint16_t als = 0U;
    uint16_t ps = 0U;
    uint8_t oled_on = 1U; /* SSD1306_Init leaves the panel on */

    (void)arg;

    if (SSD1306_Init() != 0) {
        printf("[OUL] SSD1306 init failed\r\n");
        return;
    }
    if (AP3216C_Init() != 0) {
        printf("[OUL] AP3216C init failed\r\n");
        return;
    }
    SSD1306_CLS();
    SSD1306_ShowStr(0, 0, (uint8_t *)" STM32  LINK  ", 16);
    printf("[OUL] panel ready: SSD1306 + AP3216C on I2C0 (GPIO9/10)\r\n");

    while (1) {
        StatusSnapshot snap;

        if (AP3216C_ReadData(&ir, &als, &ps) != 0) {
            printf("[OUL] AP3216C read failed\r\n");
        } else if (oled_on == 0U && als < (ALS_LIGHT_THRESHOLD - ALS_HYSTERESIS)) {
            SSD1306_ON();
            oled_on = 1U;
        } else if (oled_on != 0U && als > (ALS_LIGHT_THRESHOLD + ALS_HYSTERESIS)) {
            SSD1306_OFF();
            oled_on = 0U;
        }

        snap = status_take();
        snprintf(line, sizeof(line), "L%-7dR%-7d", (int)snap.odo_left, (int)snap.odo_right);
        SSD1306_ShowStr(0, 1, (uint8_t *)line, 16);
        snprintf(line, sizeof(line), "V%-7dV%-7d", (int)snap.speed_left, (int)snap.speed_right);
        SSD1306_ShowStr(0, 2, (uint8_t *)line, 16);
        snprintf(line, sizeof(line), "ALS%-6u%-3s", (unsigned int)als, (oled_on != 0U) ? "ON" : "OFF");
        SSD1306_ShowStr(0, 3, (uint8_t *)line, 16);

        /* Calibration stream: tune ALS_LIGHT_THRESHOLD against these values. */
        printf("[OUL] als=%u oled=%s odo=%d/%d spd=%d/%d lease=%u seq=%u\r\n",
            (unsigned int)als, (oled_on != 0U) ? "ON" : "OFF",
            (int)snap.odo_left, (int)snap.odo_right,
            (int)snap.speed_left, (int)snap.speed_right,
            (unsigned int)(snap.flags & 1U), (unsigned int)snap.sequence);

        osDelay(PANEL_LOOP_TICKS);
    }
}

/* ---------------- entry ---------------- */

static int create_thread(const char *name, osThreadFunc_t function, uint32_t stack_size)
{
    osThreadAttr_t attr = {0};

    attr.name = name;
    attr.stack_size = stack_size;
    attr.priority = (osPriority_t)TASK_PRIORITY;
    if (osThreadNew(function, NULL, &attr) == NULL) {
        printf("[OUL] thread create failed: %s\r\n", name);
        return -1;
    }
    return 0;
}

static void OledUartLightEntry(void)
{
    /* UART2 only: this HAL allows exactly one user UartInit (8-29 verified). */
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[OUL] GPIO init failed\r\n");
        return;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11,
        WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[OUL] GPIO11 UART2 TX setup failed\r\n");
        return;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12,
        WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[OUL] GPIO12 UART2 RX setup failed\r\n");
        return;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[OUL] UART2 init failed\r\n");
        return;
    }

    status_mutex = osMutexNew(NULL);
    if (status_mutex == NULL) {
        printf("[OUL] status mutex create failed\r\n");
        return;
    }

    if (create_thread("OulRx", (osThreadFunc_t)uart_rx_thread, RX_STACK_SIZE) != 0) {
        return;
    }
    if (create_thread("OulTx", (osThreadFunc_t)status_tx_thread, TX_STACK_SIZE) != 0) {
        return;
    }
    if (create_thread("OulPanel", (osThreadFunc_t)panel_thread, PANEL_STACK_SIZE) != 0) {
        return;
    }

    printf("[OUL] ready: UART2 GPIO11/12 115200 8N1 (query-only), OLED+AP3216C I2C0 GPIO9/10\r\n");
}

APP_FEATURE_INIT(OledUartLightEntry);
