/*
 * Task24: Hi3861 UART2 protocol endpoint for the STM32 motor controller.
 *
 * Link: GPIO11 UART2_TXD -> STM32 PA10 USART1_RX
 *       GPIO12 UART2_RXD <- STM32 PA9  USART1_TX
 * UART: 115200 baud, 8 data bits, 1 stop bit, no parity
 *
 * Frame: AA | CMD | LEN | PAYLOAD | CHECK
 * CHECK: low byte of CMD + LEN + PAYLOAD (SOF AA is excluded)
 * SET_SPEED(+280, +280): AA 01 04 18 01 18 01 37
 *
 * Motion is disabled by default. Change TASK24_ENABLE_MOTION_TEST to 1 only
 * for an attended, wheels-off-the-ground test.
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

#define TASK24_ENABLE_MOTION_TEST 0

#define PROTOCOL_SOF           0xAAU
#define MAX_PAYLOAD            16U
#define PROTOCOL_FRAME_MAX     (MAX_PAYLOAD + 4U)

#define PROTOCOL_CMD_SET_SPEED 0x01U
#define PROTOCOL_CMD_STOP      0x02U
#define PROTOCOL_CMD_PING      0x03U
#define PROTOCOL_CMD_ACK       0x81U

#define PROTOCOL_STATUS_OK             0U
#define PROTOCOL_STATUS_BAD_LENGTH     1U
#define PROTOCOL_STATUS_BAD_CHECKSUM   2U
#define PROTOCOL_STATUS_INVALID_PARAM  3U
#define PROTOCOL_STATUS_UNKNOWN_CMD    4U

#define TASK24_THREAD_STACK_SIZE       4096U
#define TASK24_THREAD_PRIORITY         25
#define ACK_READ_BUFFER_SIZE           64U
#define ACK_POLL_DELAY_TICKS           1U
#define SAFE_HEARTBEAT_DELAY_TICKS     100U
#define MOTION_RESEND_DELAY_TICKS      10U
#define MOTION_REPEAT_COUNT            20U
#define MOTION_STOP_DELAY_TICKS        100U

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

static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t i;
    uint8_t checksum = (uint8_t)(cmd + len);

    for (i = 0; i < len; i++) {
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
        return 0;
    }
    if (len != 0U && payload == NULL) {
        return 0;
    }

    frame[0] = PROTOCOL_SOF;
    frame[1] = cmd;
    frame[2] = len;
    for (i = 0; i < len; i++) {
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
        printf("[Task24] Frame encode failed: cmd=0x%02X len=%u\r\n", cmd, len);
        return -1;
    }

    written = UartWrite(WIFI_IOT_UART_IDX_2, frame, frame_len);
    if (written != frame_len) {
        printf("[Task24] UART2 write failed: cmd=0x%02X wrote=%d/%u\r\n",
            cmd, written, frame_len);
        return -1;
    }
    return 0;
}

int protocol_send_ping(void)
{
    return protocol_send_frame(PROTOCOL_CMD_PING, 0U, NULL);
}

int protocol_send_stop(void)
{
    return protocol_send_frame(PROTOCOL_CMD_STOP, 0U, NULL);
}

int protocol_send_speed(int16_t left, int16_t right)
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
        case PROTOCOL_CMD_ACK:
            return "ACK";
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

    if (parser->cmd != PROTOCOL_CMD_ACK) {
        printf("[Task24] RX ignored: cmd=0x%02X len=%u\r\n", parser->cmd, parser->len);
        return;
    }
    if (parser->len != 2U) {
        printf("[Task24] ACK invalid length: %u\r\n", parser->len);
        return;
    }

    original_cmd = parser->payload[0];
    status = parser->payload[1];
    printf("[Task24] ACK %s (0x%02X): %s (%u)\r\n",
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
                printf("[Task24] RX bad length: %u\r\n", parser->len);
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
                printf("[Task24] RX checksum error: cmd=0x%02X\r\n", parser->cmd);
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

static void task24_ack_thread(void *arg)
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
                printf("[Task24] UART2 read failed\r\n");
            } else {
                for (i = 0; i < count; i++) {
                    protocol_parse_byte(&parser, buffer[i]);
                }
            }
        } else if (result != WIFI_IOT_SUCCESS) {
            printf("[Task24] UART2 buffer check failed: %u\r\n", result);
        }
        osDelay(ACK_POLL_DELAY_TICKS);
    }
}

static void task24_run_motion_test(void)
{
    uint8_t i;

    printf("[Task24] Motion test enabled; keep wheels off the ground\r\n");
    while (1) {
        for (i = 0U; i < MOTION_REPEAT_COUNT; i++) {
            protocol_send_speed(140, 140);
            osDelay(MOTION_RESEND_DELAY_TICKS);
        }
        protocol_send_stop();
        osDelay(MOTION_STOP_DELAY_TICKS);

        for (i = 0U; i < MOTION_REPEAT_COUNT; i++) {
            protocol_send_speed(-140, -140);
            osDelay(MOTION_RESEND_DELAY_TICKS);
        }
        protocol_send_stop();
        osDelay(MOTION_STOP_DELAY_TICKS);
    }
}

static void task24_control_thread(void *arg)
{
    (void)arg;

    printf("[Task24] Control started; motion_test=%d\r\n", TASK24_ENABLE_MOTION_TEST);
    protocol_send_ping();

    if (TASK24_ENABLE_MOTION_TEST != 0) {
        task24_run_motion_test();
    }

    while (1) {
        osDelay(SAFE_HEARTBEAT_DELAY_TICKS);
        protocol_send_stop();
    }
}

static int task24_uart2_init(void)
{
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[Task24] GPIO init failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11,
        WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[Task24] GPIO11 UART2 TX setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12,
        WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[Task24] GPIO12 UART2 RX setup failed\r\n");
        return -1;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[Task24] UART2 init failed\r\n");
        return -1;
    }
    return 0;
}

static void Task24ProtocolEntry(void)
{
    osThreadAttr_t attr = {0};

    if (task24_uart2_init() != 0) {
        return;
    }

    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = TASK24_THREAD_STACK_SIZE;
    attr.priority = TASK24_THREAD_PRIORITY;

    attr.name = "Task24Ack";
    if (osThreadNew((osThreadFunc_t)task24_ack_thread, NULL, &attr) == NULL) {
        printf("[Task24] ACK thread create failed\r\n");
        return;
    }

    attr.name = "Task24Control";
    if (osThreadNew((osThreadFunc_t)task24_control_thread, NULL, &attr) == NULL) {
        printf("[Task24] Control thread create failed\r\n");
        return;
    }

    printf("[Task24] UART2 protocol ready: GPIO11 TX, GPIO12 RX, 115200 8N1\r\n");
}

APP_FEATURE_INIT(Task24ProtocolEntry);
