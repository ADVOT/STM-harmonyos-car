/*
 * BLE -> UART2 time-division car control.
 *
 * Strategy: only ONE UART is active at a time to work around the HAL's
 * concurrent-UartInit limitation. On BLE command, Deinit UART1, Init UART2,
 * execute motor burst, Deinit UART2, re-Init UART1 for next command.
 *
 * Hardware:
 *   Phone -> JDY-16 BLE -> Hi3861 UART1 (GPIO0/1, 9600)
 *   Hi3861 UART2 (GPIO11/12, 115200) -> STM32, Task24 protocol
 *
 * Commands:
 *   F = forward 500ms    B = backward 500ms
 *   L = spin left 500ms  R = spin right 500ms
 *   S = stop             f = forward 2000ms   b = backward 2000ms
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

#define DRIVE_SPEED               140
#define MOTION_TICK_MS            10U
#define BLE_POLL_TICKS            1U
#define BLE_READ_BUFFER_SIZE      32U
#define BURST_SHORT_TICKS         50U    /*  500 ms */
#define BURST_LONG_TICKS          200U   /* 2000 ms */

extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

/* ===== Protocol ===== */

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

/* ===== UART switching ===== */

static int ble_init(void)
{
    (void)UartDeinit(WIFI_IOT_UART_IDX_1);
    (void)UartDeinit(WIFI_IOT_UART_IDX_2);
    WifiIotUartAttribute a = {9600, WIFI_IOT_UART_DATA_BIT_8, WIFI_IOT_UART_STOP_BIT_1, 0, 0};
    if (UartInit(WIFI_IOT_UART_IDX_1, &a, NULL) != WIFI_IOT_SUCCESS) {
        printf("[BleCar] BLE init FAILED\r\n"); return -1;
    }
    return 0;
}

static int motor_init(void)
{
    (void)UartDeinit(WIFI_IOT_UART_IDX_1);
    (void)UartDeinit(WIFI_IOT_UART_IDX_2);
    WifiIotUartAttribute a = {115200, WIFI_IOT_UART_DATA_BIT_8, WIFI_IOT_UART_STOP_BIT_1, 0, 0};
    if (UartInit(WIFI_IOT_UART_IDX_2, &a, NULL) != WIFI_IOT_SUCCESS) {
        printf("[BleCar] motor init FAILED\r\n"); return -1;
    }
    return 0;
}

static void motor_run(int16_t left, int16_t right, uint32_t ticks)
{
    for (uint32_t i = 0; i < ticks; i++) { send_speed(left, right); osDelay(1); }
    send_stop(); osDelay(10);
}

/* ===== GPIO ===== */

static int gpio_setup(void)
{
    if (GpioInit() != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD) != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD) != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) return -1;
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) return -1;
    return 0;
}

/* ===== Main ===== */

static void ble_car_thread(void *arg)
{
    (void)arg;
    uint8_t buf[BLE_READ_BUFFER_SIZE];

    printf("[BleCar] === BLE Car Control (time-division) ===\r\n");
    printf("[BleCar] F/B/L/R/S = 500ms, f/b = 2000ms\r\n");

    if (ble_init() != 0) return;

    for (;;) {
        unsigned char empty = 1U;
        if (UartIsBufEmpty(WIFI_IOT_UART_IDX_1, &empty) == WIFI_IOT_SUCCESS && empty == 0U) {
            int n = UartRead(WIFI_IOT_UART_IDX_1, buf, sizeof(buf));
            if (n > 0) {
                printf("[BleCar] rx %d: ", n);
                for (int i = 0; i < n; i++) printf("%c", buf[i]);
                printf("\r\n");

                uint8_t c = buf[0];
                int16_t L = 0, R = 0; uint32_t t = 0;
                switch (c) {
                case 'F': L =  DRIVE_SPEED; R =  DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'B': L = -DRIVE_SPEED; R = -DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'L': L = -DRIVE_SPEED; R =  DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'R': L =  DRIVE_SPEED; R = -DRIVE_SPEED; t = BURST_SHORT_TICKS; break;
                case 'S': L = 0; R = 0; t = 0; break;
                case 'f': L =  DRIVE_SPEED; R =  DRIVE_SPEED; t = BURST_LONG_TICKS;  break;
                case 'b': L = -DRIVE_SPEED; R = -DRIVE_SPEED; t = BURST_LONG_TICKS;  break;
                default:  continue;
                }

                if (motor_init() != 0) { ble_init(); continue; }
                if (t > 0) {
                    printf("[BleCar] motor: L=%d R=%d %lums\r\n", L, R, (unsigned long)(t * MOTION_TICK_MS));
                    motor_run(L, R, t);
                } else {
                    send_stop();
                    printf("[BleCar] motor: STOP\r\n");
                }
                if (ble_init() != 0) return;
                printf("[BleCar] ready\r\n");
            }
        }
        osDelay(BLE_POLL_TICKS);
    }
}

static void BleTurnCalibEntry(void)
{
    if (gpio_setup() != 0) return;
    osThreadAttr_t a = {0}; a.name = "BleCar"; a.stack_size = 4096U; a.priority = 25;
    osThreadNew(ble_car_thread, NULL, &a);
}
APP_FEATURE_INIT(BleTurnCalibEntry);