#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/ip4_addr.h"
#include "lwip/api_shell.h"
#include "lwip/sockets.h"
#include "cmsis_os2.h"
#include "hos_types.h"
#include "wifi_device.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_uart_ex.h"
#include "ohos_init.h"

/* ====== WiFi 配置 ====== */
#define SELECT_WIFI_SSID "ANSWER THE CALL OF"
#define SELECT_WIFI_PASSWORD "D.U.T.Y."
#define SELECT_WIFI_SECURITYTYPE WIFI_SEC_TYPE_PSK
#define SELECT_WLAN_PORT "wlan0"
#define DEF_TIMEOUT 15
#define ONE_SECOND 1

/* ====== TCP 配置 ====== */
#define TCP_SERVER_PORT 8080
#define TCP_RX_BUF_SIZE 128

/* ====== UART2 协议（与 STM32 任务24 协议一致） ====== */
#define PROTOCOL_SOF              0xAAU
#define PROTOCOL_CMD_SET_SPEED    0x01U
#define PROTOCOL_CMD_STOP         0x02U
#define PROTOCOL_CMD_PING         0x03U
#define PROTOCOL_CMD_GET_STATUS   0x04U
#define PROTOCOL_CMD_STATUS       0x82U
#define PROTOCOL_FRAME_MAX        16U
#define PROTOCOL_LEASE_TICKS      1U    /* osDelay 参数是系统 tick，1 tick = 10ms */

/* STATUS payload: odo_left i32 LE | odo_right i32 LE |
   speed_left i16 LE | speed_right i16 LE | flags u8 */
#define STATUS_PAYLOAD_LEN        13U
#define RX_BUF_SIZE               64U

/* ====== 默认速度 ====== */
#define DEFAULT_SPEED 140

/* ====== 外环 P 修正 ====== */
#define ODO_KP                    0.5f   /* odo 差分修正系数（试调起点） */

/* ====== WiFi 全局变量 ====== */
static int g_staScanSuccess = 0;
static int g_ConnectSuccess = 0;
static int ssid_count = 0;
static WifiEvent g_wifiEventHandler = {0};
static WifiErrorCode error;
static struct netif *g_lwip_netif = NULL;

/* ====== Odo 遥测（RX 线程写入，移动循环读取） ====== */
static osMutexId_t odo_mutex;
static volatile int odo_left = 0;
static volatile int odo_right = 0;
static volatile int odo_fresh = 0;   /* 1 = 收到过至少一帧 STATUS */

/* ====== TCP 任务栈 ====== */
#define TCP_STACK_SIZE 10240

/* ====== 函数声明 ====== */
static void WiFiInit(void);
static void WaitSacnResult(void);
static int  WaitConnectResult(void);
static BOOL WifiSTATask(void);

/* WiFi 回调 */
static void OnWifiScanStateChangedHandler(int state, int size);
static void OnWifiConnectionChangedHandler(int state, WifiLinkedInfo *info);
static void OnHotspotStaJoinHandler(StationInfo *info);
static void OnHotspotStateChangedHandler(int state);
static void OnHotspotStaLeaveHandler(StationInfo *info);

/* 协议 */
static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload);
static uint8_t protocol_encode_frame(uint8_t cmd, uint8_t len, const uint8_t *payload,
                                      uint8_t *frame, uint8_t frame_size);
static int  protocol_send_frame(uint8_t cmd, uint8_t len, const uint8_t *payload);
static int  protocol_send_speed(int16_t left, int16_t right);
static int  protocol_send_stop(void);
static void protocol_send_get_status(void);

/* 硬件 */
static int  hardware_init(void);

/* Odo RX 线程 */
static void odo_rx_thread(void *arg);

/* TCP 服务器 */
static void tcp_server_task(void *arg);

/* ==================== WiFi 回调 ==================== */

static void OnWifiScanStateChangedHandler(int state, int size)
{
    (void)state;
    if (size > 0) {
        ssid_count = size;
        g_staScanSuccess = 1;
    }
}

static void OnWifiConnectionChangedHandler(int state, WifiLinkedInfo *info)
{
    (void)info;
    if (state > 0) {
        g_ConnectSuccess = 1;
        printf("callback function for wifi connect\r\n");
    } else {
        printf("connect error,please check password\r\n");
    }
}

static void OnHotspotStaJoinHandler(StationInfo *info)
{
    (void)info;
    printf("STA join AP\n");
}

static void OnHotspotStaLeaveHandler(StationInfo *info)
{
    (void)info;
    printf("HotspotStaLeave:info is null.\n");
}

static void OnHotspotStateChangedHandler(int state)
{
    printf("HotspotStateChanged:state is %d.\n", state);
}

/* ==================== WiFi 初始化 ==================== */

static void WiFiInit(void)
{
    printf("<--Wifi Init-->\r\n");
    g_wifiEventHandler.OnWifiScanStateChanged = OnWifiScanStateChangedHandler;
    g_wifiEventHandler.OnWifiConnectionChanged = OnWifiConnectionChangedHandler;
    g_wifiEventHandler.OnHotspotStaJoin = OnHotspotStaJoinHandler;
    g_wifiEventHandler.OnHotspotStaLeave = OnHotspotStaLeaveHandler;
    g_wifiEventHandler.OnHotspotStateChanged = OnHotspotStateChangedHandler;
    error = RegisterWifiEvent(&g_wifiEventHandler);
    if (error != WIFI_SUCCESS) {
        printf("register wifi event fail!\r\n");
    } else {
        printf("register wifi event succeed!\r\n");
    }
}

static void WaitSacnResult(void)
{
    int scanTimeout = DEF_TIMEOUT;
    while (scanTimeout > 0) {
        sleep(ONE_SECOND);
        scanTimeout--;
        if (g_staScanSuccess == 1) {
            printf("WaitSacnResult:wait success[%d]s\n", (DEF_TIMEOUT - scanTimeout));
            break;
        }
    }
    if (scanTimeout <= 0) {
        printf("WaitSacnResult:timeout!\n");
    }
}

static int WaitConnectResult(void)
{
    int ConnectTimeout = DEF_TIMEOUT;
    while (ConnectTimeout > 0) {
        sleep(1);
        ConnectTimeout--;
        if (g_ConnectSuccess == 1) {
            printf("WaitConnectResult:wait success[%d]s\n", (DEF_TIMEOUT - ConnectTimeout));
            break;
        }
    }
    if (ConnectTimeout <= 0) {
        printf("WaitConnectResult:timeout!\n");
        return 0;
    }
    return 1;
}

static BOOL WifiSTATask(void)
{
    WifiScanInfo *info = NULL;
    unsigned int size = WIFI_SCAN_HOTSPOT_LIMIT;
    WifiDeviceConfig select_ap_config = {0};

    osDelay(200);
    printf("<--System Init-->\r\n");

    WiFiInit();

    if (EnableWifi() != WIFI_SUCCESS) {
        printf("EnableWifi failed, error = %d\r\n", error);
        return -1;
    }

    if (IsWifiActive() == 0) {
        printf("Wifi station is not actived.\r\n");
        return -1;
    }

    info = malloc(sizeof(WifiScanInfo) * WIFI_SCAN_HOTSPOT_LIMIT);
    if (info == NULL) {
        return -1;
    }

    do {
        ssid_count = 0;
        g_staScanSuccess = 0;
        Scan();
        WaitSacnResult();
        error = GetScanInfoList(info, &size);
    } while (g_staScanSuccess != 1);

    printf("********************\r\n");
    for (uint8_t i = 0; i < ssid_count; i++) {
        printf("no:%03d, ssid:%-30s, rssi:%5d\r\n", i + 1, info[i].ssid, info[i].rssi / 100);
    }
    printf("********************\r\n");

    for (uint8_t i = 0; i < ssid_count; i++) {
        if (strcmp(SELECT_WIFI_SSID, info[i].ssid) == 0) {
            int result;
            printf("Select:%3d wireless, Waiting...\r\n", i + 1);
            strcpy(select_ap_config.ssid, info[i].ssid);
            strcpy(select_ap_config.preSharedKey, SELECT_WIFI_PASSWORD);
            select_ap_config.securityType = SELECT_WIFI_SECURITYTYPE;

            if (AddDeviceConfig(&select_ap_config, &result) == WIFI_SUCCESS) {
                if (ConnectTo(result) == WIFI_SUCCESS && WaitConnectResult() == 1) {
                    printf("WiFi connect succeed!\r\n");
                    g_lwip_netif = netifapi_netif_find(SELECT_WLAN_PORT);
                    break;
                }
            }
        }

        if (i == ssid_count - 1) {
            printf("ERROR: No wifi as expected\r\n");
            while (1) osDelay(100);
        }
    }

    if (g_lwip_netif) {
        dhcp_start(g_lwip_netif);
        printf("begain to dhcp\r\n");
    }

    for (;;) {
        if (dhcp_is_bound(g_lwip_netif) == ERR_OK) {
            printf("<-- DHCP state:OK -->\r\n");
            netifapi_netif_common(g_lwip_netif, dhcp_clients_info_show, NULL);
            break;
        }
        printf("<-- DHCP state:Inprogress -->\r\n");
        osDelay(100);
    }

    /* DHCP 完成后启动 TCP 服务器 */
    printf("[TCP] Starting TCP server on port %d...\r\n", TCP_SERVER_PORT);
    tcp_server_task(NULL);

    while (1) osDelay(1000);
    return 0;
}

/* ==================== 协议层 ==================== */

static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t sum = (uint8_t)(cmd + len);
    for (uint8_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + payload[i]);
    }
    return sum;
}

static uint8_t protocol_encode_frame(uint8_t cmd, uint8_t len, const uint8_t *payload,
                                      uint8_t *frame, uint8_t frame_size)
{
    uint8_t total = 4U + len;
    if (total > frame_size) return 0;

    frame[0] = PROTOCOL_SOF;
    frame[1] = cmd;
    frame[2] = len;
    if (len > 0 && payload != NULL) {
        memcpy(&frame[3], payload, len);
    }
    frame[3U + len] = protocol_checksum(cmd, len, payload);
    return total;
}

static int protocol_send_frame(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t frame[PROTOCOL_FRAME_MAX];
    uint8_t frame_len = protocol_encode_frame(cmd, len, payload, frame, sizeof(frame));
    if (frame_len == 0) {
        printf("[TCP] frame encode failed: cmd=0x%02X len=%u\r\n", cmd, len);
        return -1;
    }
    int written = UartWrite(WIFI_IOT_UART_IDX_2, frame, frame_len);
    if (written != (int)frame_len) {
        printf("[TCP] UART2 write failed: cmd=0x%02X wrote=%d/%u\r\n", cmd, written, frame_len);
        return -1;
    }
    return 0;
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

static int protocol_send_stop(void)
{
    return protocol_send_frame(PROTOCOL_CMD_STOP, 0U, NULL);
}

static void protocol_send_get_status(void)
{
    protocol_send_frame(PROTOCOL_CMD_GET_STATUS, 0U, NULL);
}

/* ==================== 硬件初始化 ==================== */

static int hardware_init(void)
{
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[TCP] GPIO init failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11,
        WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[TCP] GPIO11 UART2_TXD func set failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12,
        WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[TCP] GPIO12 UART2_RXD func set failed\r\n");
        return -1;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[TCP] UART2 init failed\r\n");
        return -1;
    }
    printf("[TCP] UART2 init OK (115200 8N1)\r\n");
    return 0;
}

/* ==================== Odo RX 线程 ==================== */

static int32_t decode_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void odo_rx_thread(void *arg)
{
    (void)arg;
    uint8_t buf[RX_BUF_SIZE];
    uint8_t payload[STATUS_PAYLOAD_LEN];
    uint8_t parse_state = 0;  /* 0=WAIT_SOF, 1=WAIT_CMD, 2=WAIT_LEN, 3=WAIT_PAYLOAD, 4=WAIT_CKSUM */
    uint8_t parse_cmd = 0;
    uint8_t parse_len = 0;
    uint8_t parse_idx = 0;
    uint8_t parse_cksum = 0;

    while (1) {
        unsigned char empty = 1U;
        if (UartIsBufEmpty(WIFI_IOT_UART_IDX_2, &empty) != WIFI_IOT_SUCCESS) {
            osDelay(1);
            continue;
        }
        if (empty) {
            osDelay(1);
            continue;
        }

        int count = UartRead(WIFI_IOT_UART_IDX_2, buf, sizeof(buf));
        if (count <= 0) {
            osDelay(1);
            continue;
        }

        for (int i = 0; i < count; i++) {
            uint8_t b = buf[i];

            switch (parse_state) {
            case 0: /* WAIT_SOF */
                if (b == PROTOCOL_SOF) {
                    parse_state = 1;
                }
                break;
            case 1: /* WAIT_CMD */
                parse_cmd = b;
                parse_state = 2;
                break;
            case 2: /* WAIT_LEN */
                parse_len = b;
                parse_idx = 0;
                if (parse_len == 0) {
                    parse_state = 4;
                } else if (parse_len <= STATUS_PAYLOAD_LEN) {
                    parse_state = 3;
                } else {
                    parse_state = 0; /* 长度非法，丢弃 */
                }
                break;
            case 3: /* WAIT_PAYLOAD */
                payload[parse_idx++] = b;
                if (parse_idx >= parse_len) {
                    parse_state = 4;
                }
                break;
            case 4: /* WAIT_CKSUM */
                parse_cksum = b;
                parse_state = 0;
                /* 只处理 STATUS 帧 */
                if (parse_cmd == PROTOCOL_CMD_STATUS && parse_len == STATUS_PAYLOAD_LEN) {
                    uint8_t expected = protocol_checksum(parse_cmd, parse_len, payload);
                    if (parse_cksum == expected) {
                        int32_t ol = decode_i32_le(&payload[0]);
                        int32_t or_ = decode_i32_le(&payload[4]);
                        osMutexAcquire(odo_mutex, osWaitForever);
                        odo_left = ol;
                        odo_right = or_;
                        odo_fresh = 1;
                        osMutexRelease(odo_mutex);
                    }
                }
                break;
            }
        }
    }
}

/* ==================== TCP 服务器 ==================== */

static void tcp_server_task(void *arg)
{
    (void)arg;
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char rx_buf[TCP_RX_BUF_SIZE];
    int recv_len;

    if (hardware_init() != 0) {
        printf("[TCP] Hardware init failed, TCP server aborted\r\n");
        return;
    }

    /* 创建 odo 互斥锁 */
    odo_mutex = osMutexNew(NULL);
    if (odo_mutex == NULL) {
        printf("[TCP] odo mutex create failed\r\n");
    }

    /* 启动 odo RX 线程 */
    osThreadAttr_t rx_attr = {
        .name = "OdoRx",
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .stack_mem = NULL,
        .stack_size = 4096U,
        .priority = 26,
    };
    if (osThreadNew(odo_rx_thread, NULL, &rx_attr) == NULL) {
        printf("[TCP] odo RX thread create failed\r\n");
    } else {
        printf("[TCP] odo RX thread started\r\n");
    }

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        printf("[TCP] socket() failed: %d\r\n", server_fd);
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_SERVER_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("[TCP] bind() failed\r\n");
        lwip_close(server_fd);
        return;
    }

    if (listen(server_fd, 1) < 0) {
        printf("[TCP] listen() failed\r\n");
        lwip_close(server_fd);
        return;
    }

    printf("[TCP] Server listening on port %d\r\n", TCP_SERVER_PORT);
    printf("[TCP] Waiting for client...\r\n");

    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
    if (client_fd < 0) {
        printf("[TCP] accept() failed\r\n");
        lwip_close(server_fd);
        return;
    }

    printf("[TCP] Client connected!\r\n");
    printf("[TCP] Commands: FWD <ms> | BACK <ms> | LEFT <ms> | RIGHT <ms> | STOP\r\n");
    printf("[TCP] Odo correction: Kp=%.1f\r\n", ODO_KP);

    while (1) {
        recv_len = recv(client_fd, rx_buf, sizeof(rx_buf) - 1, 0);
        if (recv_len <= 0) {
            printf("[TCP] Client disconnected\r\n");
            break;
        }
        rx_buf[recv_len] = '\0';

        while (recv_len > 0 && (rx_buf[recv_len - 1] == '\n' || rx_buf[recv_len - 1] == '\r')) {
            rx_buf[--recv_len] = '\0';
        }

        printf("[TCP] RX: '%s'\r\n", rx_buf);

        char *cmd = strtok(rx_buf, " ");
        char *arg_str = strtok(NULL, " ");
        int duration = arg_str ? atoi(arg_str) : 0;
        int16_t base_speed = DEFAULT_SPEED;

        if (cmd == NULL) continue;

        if (strcasecmp(cmd, "STOP") == 0) {
            protocol_send_stop();
            printf("[TCP] -> STOP\r\n");
            continue;
        }

        int16_t left_speed = 0, right_speed = 0;

        if (strcasecmp(cmd, "FWD") == 0 || strcasecmp(cmd, "FORWARD") == 0) {
            left_speed = base_speed;  right_speed = base_speed;
        } else if (strcasecmp(cmd, "BACK") == 0 || strcasecmp(cmd, "BACKWARD") == 0) {
            left_speed = -base_speed; right_speed = -base_speed;
        } else if (strcasecmp(cmd, "LEFT") == 0) {
            left_speed = -base_speed; right_speed = base_speed;
        } else if (strcasecmp(cmd, "RIGHT") == 0) {
            left_speed = base_speed;  right_speed = -base_speed;
        } else {
            printf("[TCP] Unknown command: %s\r\n", cmd);
            continue;
        }

        if (duration <= 0) {
            printf("[TCP] Invalid duration: %d\r\n", duration);
            continue;
        }

        /* 记录 odo 基线 */
        int odo_base_left = 0, odo_base_right = 0;
        int odo_ok = 0;
        if (odo_mutex) {
            osMutexAcquire(odo_mutex, osWaitForever);
            if (odo_fresh) {
                odo_base_left = odo_left;
                odo_base_right = odo_right;
                odo_ok = 1;
            }
            osMutexRelease(odo_mutex);
        }

        printf("[TCP] -> %s %dms (L=%d R=%d) odo_ok=%d base=(%d,%d)\r\n",
               cmd, duration, left_speed, right_speed, odo_ok, odo_base_left, odo_base_right);

        /* 执行循环 */
        int ticks = duration / 10;  /* 10ms per tick */
        int get_status_counter = 0;
        for (int i = 0; i < ticks; i++) {
            /* 每 10 次循环（100ms）发一次 GET_STATUS */
            if (++get_status_counter >= 10) {
                get_status_counter = 0;
                protocol_send_get_status();
            }

            int16_t adj_left = left_speed;
            int16_t adj_right = right_speed;

            /* odo 差分外环：仅直行（FWD/BACK）时修正 */
            if (odo_ok && left_speed == right_speed) {
                int cur_left = 0, cur_right = 0;
                osMutexAcquire(odo_mutex, osWaitForever);
                cur_left = odo_left;
                cur_right = odo_right;
                osMutexRelease(odo_mutex);

                int delta_left = cur_left - odo_base_left;
                int delta_right = cur_right - odo_base_right;
                int diff = delta_right - delta_left;  /* >0: 右轮多 → 车左偏 → 压右轮 */

                int16_t correction = (int16_t)((float)diff * ODO_KP);
                if (left_speed > 0) {
                    /* 前进：右轮多 → 压右轮（减速） */
                    adj_right = (int16_t)(left_speed - correction);
                    adj_left  = (int16_t)(left_speed + correction);
                } else {
                    /* 后退：右轮多 → 压右轮（减速，即更负） */
                    adj_right = (int16_t)(left_speed - correction);
                    adj_left  = (int16_t)(left_speed + correction);
                }

                /* 限幅：不超默认速度 ±50%，不反向 */
                int16_t limit = (int16_t)(base_speed * 3 / 2);
                if (adj_left > limit) adj_left = limit;
                if (adj_left < -limit) adj_left = -limit;
                if (adj_right > limit) adj_right = limit;
                if (adj_right < -limit) adj_right = -limit;
                if (left_speed > 0 && adj_left < 0) adj_left = 0;
                if (left_speed < 0 && adj_left > 0) adj_left = 0;
                if (right_speed > 0 && adj_right < 0) adj_right = 0;
                if (right_speed < 0 && adj_right > 0) adj_right = 0;
            }

            protocol_send_speed(adj_left, adj_right);
            osDelay(PROTOCOL_LEASE_TICKS);  /* 1 tick = 10ms */
        }
        protocol_send_stop();
        printf("[TCP] -> STOP (done)\r\n");
    }

    lwip_close(client_fd);
    lwip_close(server_fd);
    printf("[TCP] Server closed, waiting for new client...\r\n");
    tcp_server_task(NULL);
}

/* ==================== 入口 ==================== */

static void Wifi(void)
{
    osThreadAttr_t attr;
    attr.name = "WifiSTATask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = TCP_STACK_SIZE;
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)WifiSTATask, NULL, &attr) == NULL) {
        printf("Falied to create WifiSTATask!\n");
    }
}

APP_FEATURE_INIT(Wifi);