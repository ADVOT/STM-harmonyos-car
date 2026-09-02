/*
 * BLE remote control — dual UART open concurrently, NO time-division switching.
 *
 * Why this works: the Hi3861 UART driver (platform/drivers/uart/uart.c) keeps a
 * fully independent static uart_driver_data_t per port (g_uart_0/1/2), and
 * uart_open()/uart_close() touch only the port's own registers, IRQ and buffers.
 * There is no "only one user UART" restriction. F.1_Tabletop_Safeguard already
 * runs this way in the field: system holds UART0 (log) + UART1 (boot-opened by
 * peripheral_init), user code opens UART2 — all three alive at once.
 *
 * The earlier G.0 time-division version (Deinit UART1 -> Init UART2 per command)
 * is obsolete: it added a ~600ms blind window per command and BLE bytes sent
 * during a motion burst were silently lost.
 *
 * Hardware:
 *   Phone -> JDY-16 BLE -> Hi3861 UART1 (GPIO0/1, 9600 8N1)
 *   Hi3861 UART2 (GPIO11/12, 115200 8N1) -> STM32 (Task24 protocol)
 *
 * STM32 side (motor_pwm, unchanged): frame AA|CMD|LEN|PAYLOAD|CHECK,
 * SET_SPEED=0x01 (i16 LE left, i16 LE right), STOP=0x02, ACK=0x81.
 * A 300ms motion lease auto-stops the car if SET_SPEED stops arriving, so while
 * a burst runs we re-send SET_SPEED every 10ms loop tick.
 *
 * Commands (single ASCII char from the BLE app):
 *   F/B = forward/backward 500ms    L/R = spin left/right 500ms
 *   f/b = forward/backward 2000ms   S   = immediate stop
 *
 * Self-diagnosis: a PING is sent at boot; every second a one-line link stat
 * prints (sent vs ack_ok). ack_ok==0 means the UART2->STM32 leg is dead
 * (wiring / STM32 firmware / serial switch), not a Hi3861 UART problem.
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

#define PROTOCOL_SOF              0xAAU
#define PROTOCOL_CMD_SET_SPEED    0x01U
#define PROTOCOL_CMD_STOP         0x02U
#define PROTOCOL_CMD_PING         0x03U
#define PROTOCOL_CMD_ACK          0x81U

#define BLE_BAUD                  9600U
#define MOTOR_BAUD                115200U
#define DRIVE_SPEED               140
#define BURST_SHORT_TICKS         50U    /*  500ms at 10ms loop tick */
#define BURST_LONG_TICKS          200U   /* 2000ms */
#define SEC_TICKS                 100U   /* 1s at 10ms loop tick */
#define BLE_BUF_SIZE              32U
#define UART2_RX_BUF_SIZE         64U

extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

/* ===== Task24 protocol TX ===== */

static uint8_t checksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint32_t sum = (uint32_t)cmd + len;
    for (uint8_t i = 0; i < len; i++) sum += payload[i];
    return (uint8_t)(sum & 0xFFU);
}

static int send_frame(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t frame[8];
    if (len > 4U) return -1;
    frame[0] = PROTOCOL_SOF; frame[1] = cmd; frame[2] = len;
    for (uint8_t i = 0; i < len; i++) frame[3U + i] = payload[i];
    frame[3U + len] = checksum(cmd, len, payload);
    return UartWrite(WIFI_IOT_UART_IDX_2, frame, 4U + len);
}

static int send_speed(int16_t left, int16_t right)
{
    uint8_t p[4];
    p[0] = (uint8_t)(left & 0xFF);   p[1] = (uint8_t)((left >> 8) & 0xFF);
    p[2] = (uint8_t)(right & 0xFF);  p[3] = (uint8_t)((right >> 8) & 0xFF);
    return send_frame(PROTOCOL_CMD_SET_SPEED, 4U, p);
}

static int send_stop(void) { return send_frame(PROTOCOL_CMD_STOP, 0U, NULL); }

/* ===== UART2 RX: minimal ACK parser (link self-diagnosis) ===== */

static uint32_t g_frames_sent = 0;
static uint32_t g_ack_ok = 0;
static uint32_t g_ack_bad = 0;

static void parse_rx_byte(uint8_t b)
{
    static uint8_t state = 0, cmd = 0, len = 0, chk = 0, idx = 0;
    static uint8_t payload[16];

    switch (state) {
    case 0:
        if (b == PROTOCOL_SOF) state = 1;
        break;
    case 1:
        cmd = b; chk = b; state = 2;
        break;
    case 2:
        len = b; chk = (uint8_t)(chk + b); idx = 0;
        if (len > sizeof(payload)) { state = 0; break; }  /* garbage, resync */
        state = (len == 0U) ? 4 : 3;
        break;
    case 3:
        payload[idx++] = b; chk = (uint8_t)(chk + b);
        if (idx >= len) state = 4;
        break;
    case 4:
        if (b == chk) {
            if (cmd == PROTOCOL_CMD_ACK && len == 2U) {
                if (payload[1] == 0U) {
                    g_ack_ok++;
                } else {
                    g_ack_bad++;
                    printf("[BleRC] NACK cmd=%02X status=%02X\r\n", payload[0], payload[1]);
                }
            }
        } else {
            g_ack_bad++;
            printf("[BleRC] bad frame cmd=%02X len=%u chk=%02X got=%02X\r\n", cmd, len, chk, b);
        }
        state = 0;
        break;
    default:
        state = 0;
        break;
    }
}

/* ===== GPIO / UART setup (once, both ports stay open) ===== */

static int gpio_setup(void)
{
    if (GpioInit() != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD) != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD) != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) return -1;
    return 0;
}

static int uart_setup(void)
{
    WifiIotUartAttribute ble = {BLE_BAUD, WIFI_IOT_UART_DATA_BIT_8, WIFI_IOT_UART_STOP_BIT_1, 0, 0};
    WifiIotUartAttribute mot = {MOTOR_BAUD, WIFI_IOT_UART_DATA_BIT_8, WIFI_IOT_UART_STOP_BIT_1, 0, 0};

    /* peripheral_init() opened UART1 @115200 at boot; drop that stale config once. */
    (void)UartDeinit(WIFI_IOT_UART_IDX_1);
    if (UartInit(WIFI_IOT_UART_IDX_1, &ble, NULL) != WIFI_IOT_SUCCESS) {
        printf("[BleRC] UART1(BLE) init FAILED\r\n");
        return -1;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &mot, NULL) != WIFI_IOT_SUCCESS) {
        printf("[BleRC] UART2(motor) init FAILED\r\n");
        return -1;
    }
    return 0;
}

/* ===== Main thread ===== */

static void ble_remote_thread(void *arg)
{
    (void)arg;
    uint8_t buf[BLE_BUF_SIZE];
    uint8_t rxbuf[UART2_RX_BUF_SIZE];
    int16_t curL = 0, curR = 0;
    uint32_t burst_ticks = 0;
    uint32_t sec_ticks = 0;

    printf("[BleRC] === BLE remote, dual-UART concurrent (v5) ===\r\n");
    printf("[BleRC] F/B/L/R=500ms f/b=2000ms S=stop\r\n");

    if (uart_setup() != 0) return;

    send_frame(PROTOCOL_CMD_PING, 0U, NULL);   /* boot link probe, expect ack_ok=1 */
    g_frames_sent++;

    for (;;) {
        /* 1. drain BLE, latest command wins */
        unsigned char empty = 1U;
        while (UartIsBufEmpty(WIFI_IOT_UART_IDX_1, &empty) == WIFI_IOT_SUCCESS && empty == 0U) {
            int n = UartRead(WIFI_IOT_UART_IDX_1, buf, sizeof(buf));
            if (n <= 0) break;
            for (int i = 0; i < n; i++) {
                uint8_t c = buf[i];
                int16_t L = 0, R = 0; uint32_t t = 0;
                switch (c) {
                case 'F': L =  DRIVE_SPEED; R =  DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'B': L = -DRIVE_SPEED; R = -DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'L': L = -DRIVE_SPEED; R =  DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'R': L =  DRIVE_SPEED; R = -DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'f': L =  DRIVE_SPEED; R =  DRIVE_SPEED; t = BURST_LONG_TICKS;  break;
                case 'b': L = -DRIVE_SPEED; R = -DRIVE_SPEED; t = BURST_LONG_TICKS;  break;
                case 'S':
                    burst_ticks = 0;
                    send_stop(); g_frames_sent++;
                    printf("[BleRC] 'S' stop\r\n");
                    continue;
                default:
                    continue;   /* ignore CRLF / garbage */
                }
                curL = L; curR = R; burst_ticks = t;
                printf("[BleRC] '%c' L=%d R=%d %lums\r\n", c, L, R, (unsigned long)(t * 10U));
            }
        }

        /* 2. motion heartbeat: 10ms << 300ms STM32 lease */
        if (burst_ticks > 0U) {
            send_speed(curL, curR); g_frames_sent++;
            burst_ticks--;
            if (burst_ticks == 0U) {
                send_stop(); g_frames_sent++;
                printf("[BleRC] burst done\r\n");
            }
        }

        /* 3. drain UART2 RX (ACK frames from STM32) */
        empty = 1U;
        while (UartIsBufEmpty(WIFI_IOT_UART_IDX_2, &empty) == WIFI_IOT_SUCCESS && empty == 0U) {
            int n = UartRead(WIFI_IOT_UART_IDX_2, rxbuf, sizeof(rxbuf));
            if (n <= 0) break;
            for (int i = 0; i < n; i++) parse_rx_byte(rxbuf[i]);
        }

        /* 4. 1s link stat: ack_ok not counting up = UART2->STM32 leg dead */
        if (++sec_ticks >= SEC_TICKS) {
            sec_ticks = 0;
            printf("[BleRC] link sent=%lu ack_ok=%lu ack_bad=%lu\r\n",
                   (unsigned long)g_frames_sent, (unsigned long)g_ack_ok, (unsigned long)g_ack_bad);
        }

        osDelay(1);   /* 10ms tick */
    }
}

static void BleRemoteEntry(void)
{
    if (gpio_setup() != 0) return;
    osThreadAttr_t a = {0}; a.name = "BleRC"; a.stack_size = 4096U; a.priority = 25;
    osThreadNew(ble_remote_thread, NULL, &a);
}
APP_FEATURE_INIT(BleRemoteEntry);
