/*
 * Autonomous timed-spin calibration for turn rate (attended test only).
 *
 * Motor link: Hi3861 UART2 (GPIO11 TX / GPIO12 RX, 115200 8N1) ->
 * STM32 USART1, Task24 protocol: AA | CMD | LEN | PAYLOAD | CHECK.
 * Only ONE user UartInit call is allowed by this SDK's HAL, so this
 * firmware touches nothing but UART2.
 *
 * Timeline after boot (all durations firmware-timed, no human stopwatch):
 *   10.0 s STOP        - placement window: put the car on the target surface
 *   10.0 s SPIN RIGHT  - left wheel +140 / right wheel -140
 *    3.0 s STOP        - measurement window: record total right-turn angle
 *   10.0 s SPIN LEFT   - left wheel -140 / right wheel +140
 *   forever STOP       - done; press reset to repeat
 *
 * The STM32 300 ms motion lease is the deadman switch: any wedge here
 * stops the car. Measure: full turns + residual angle per direction.
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

#define PROTOCOL_SOF                      0xAAU
#define PROTOCOL_CMD_SET_SPEED            0x01U
#define PROTOCOL_CMD_STOP                 0x02U

#define TURN_SPEED                        140
#define COMMAND_RESEND_TICKS              10U   /* 100 ms, below the 300 ms lease */
#define PLACEMENT_SECONDS                 10U
#define SPIN_SECONDS                      10U
#define MEASURE_SECONDS                   3U

/* The SDK provides this symbol even when older headers omit the declaration. */
extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint32_t sum = (uint32_t)cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum & 0xFFU);
}

static int protocol_send_frame(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t frame[8];
    if (len > 4U) {
        return -1;
    }
    frame[0] = PROTOCOL_SOF;
    frame[1] = cmd;
    frame[2] = len;
    for (uint8_t i = 0; i < len; i++) {
        frame[3U + i] = payload[i];
    }
    frame[3U + len] = protocol_checksum(cmd, len, payload);
    return UartWrite(WIFI_IOT_UART_IDX_2, frame, 4U + len);
}

static int protocol_send_speed(int16_t left, int16_t right)
{
    uint8_t payload[4];
    payload[0] = (uint8_t)(left & 0xFF);
    payload[1] = (uint8_t)((left >> 8) & 0xFF);
    payload[2] = (uint8_t)(right & 0xFF);
    payload[3] = (uint8_t)((right >> 8) & 0xFF);
    return protocol_send_frame(PROTOCOL_CMD_SET_SPEED, 4U, payload);
}

static void drain_uart2_rx(void)
{
    uint8_t dump[16];
    for (;;) {
        unsigned char empty = 1U;
        if (UartIsBufEmpty(WIFI_IOT_UART_IDX_2, &empty) != WIFI_IOT_SUCCESS || empty != 0U) {
            break;
        }
        (void)UartRead(WIFI_IOT_UART_IDX_2, dump, sizeof(dump));
    }
}

static void run_phase(const char *name, int16_t left, int16_t right, uint32_t seconds)
{
    uint32_t beats = seconds * 1000U / 10U / COMMAND_RESEND_TICKS;
    printf("[SpinCalib] %s for %lus\r\n", name, (unsigned long)seconds);
    for (uint32_t i = 0; i < beats; i++) {
        if (left == 0 && right == 0) {
            (void)protocol_send_frame(PROTOCOL_CMD_STOP, 0U, NULL);
        } else {
            (void)protocol_send_speed(left, right);
        }
        drain_uart2_rx(); /* drop STM32 ACKs; the lease timeout is the watchdog */
        osDelay(COMMAND_RESEND_TICKS);
    }
}

static void spin_calib_thread(void *arg)
{
    (void)arg;
    printf("[SpinCalib] ready: UART2 115200; %lus placement, then auto spin\r\n",
           (unsigned long)PLACEMENT_SECONDS);
    run_phase("PLACEMENT_STOP", 0, 0, PLACEMENT_SECONDS);
    run_phase("SPIN_RIGHT", TURN_SPEED, -TURN_SPEED, SPIN_SECONDS);
    run_phase("MEASURE_STOP", 0, 0, MEASURE_SECONDS);
    run_phase("SPIN_LEFT", -TURN_SPEED, TURN_SPEED, SPIN_SECONDS);
    printf("[SpinCalib] done; permanent STOP, press reset to repeat\r\n");
    for (;;) {
        run_phase("FINAL_STOP", 0, 0, 60U);
    }
}

static int spin_calib_hardware_init(void)
{
    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[SpinCalib] GpioInit failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[SpinCalib] GPIO11 UART2 TXD setup failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[SpinCalib] GPIO12 UART2 RXD setup failed\r\n");
        return -1;
    }
    WifiIotUartAttribute motor_attr = {
        .baudRate = 115200,
        .dataBits = WIFI_IOT_UART_DATA_BIT_8,
        .stopBits = WIFI_IOT_UART_STOP_BIT_1,
        .parity = 0,
    };
    if (UartInit(WIFI_IOT_UART_IDX_2, &motor_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[SpinCalib] UART2 (motor) init failed\r\n");
        return -1;
    }
    return 0;
}

static void SpinCalibEntry(void)
{
    if (spin_calib_hardware_init() != 0) {
        printf("[SpinCalib] hardware init failed\r\n");
        return;
    }
    osThreadAttr_t attr = {0};
    attr.name = "SpinCalibCtrl";
    attr.stack_size = 4096U;
    attr.priority = 25;
    if (osThreadNew(spin_calib_thread, NULL, &attr) == NULL) {
        printf("[SpinCalib] thread create failed\r\n");
        return;
    }
}

APP_FEATURE_INIT(SpinCalibEntry);
