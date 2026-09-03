#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/ip4_addr.h"
#include "lwip/api_shell.h"
#include "cmsis_os2.h"
#include "hos_types.h"
#include "wifi_device.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_uart_ex.h"
#include "ohos_init.h"
#include "cJSON.h"

#include "cloud_config.h"
#include "oc_mqtt.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ap3216c.h"

/* ====== 通用 ====== */
#define SELECT_WLAN_PORT "wlan0"
#define DEF_TIMEOUT 15
#define ONE_SECOND 1

/* ====== UART2 协议（与 STM32 任务24 / 11.0_TCP_Control 一致）====== */
#define PROTOCOL_SOF              0xAAU
#define PROTOCOL_CMD_SET_SPEED    0x01U
#define PROTOCOL_CMD_STOP         0x02U
#define PROTOCOL_CMD_PING         0x03U
#define PROTOCOL_CMD_GET_STATUS   0x04U
#define PROTOCOL_CMD_STATUS       0x82U
#define PROTOCOL_FRAME_MAX        16U
#define PROTOCOL_LEASE_TICKS      1U     /* osDelay tick，1 tick = 10ms */

/* STATUS payload: odo_left i32 | odo_right i32 | speed_left i16 | speed_right i16 | flags u8 */
#define STATUS_PAYLOAD_LEN        13U
#define RX_BUF_SIZE               64U

/* ====== 运动参数 ====== */
#define DEFAULT_SPEED             140
#define RAMP_STEP                 10     /* 每 10ms 拍速度增量：0→满速约 140ms，换向约 280ms（丝滑斜坡）*/
#define ODO_KP                    0.5f   /* 直行 odo 差分外环纠偏系数 */
#define MOTION_Q_LEN              4

/* ====== 任务栈 ====== */
#define WIFI_STACK_SIZE           10240
#define ODO_RX_STACK_SIZE         4096
#define MOTION_STACK_SIZE         4096
#define SENSOR_STACK_SIZE         4096

/* ====== 遥测上报（需求2：光照+温湿度 → 华为云 Environment service）======
   属性名必须与控制台产品模型逐字一致，否则云端丢弃。FLOAT 类型 value 传 double*。 */
#define ENV_SERVICE_ID            "Environment"
#define ENV_PROP_TEMPERATURE      "Temperature"
#define ENV_PROP_HUMIDITY         "Humidity"
#define ENV_PROP_LIGHT            "Light"
#define SENSOR_REPORT_INTERVAL_MS 5000   /* 5s 上报一次 */

/* ====== WiFi 全局 ====== */
static int g_staScanSuccess = 0;
static int g_ConnectSuccess = 0;
static int ssid_count = 0;
static WifiEvent g_wifiEventHandler = {0};
static WifiErrorCode error;
static struct netif *g_lwip_netif = NULL;

/* ====== Odo 遥测（RX 线程写，运动线程读）====== */
static osMutexId_t odo_mutex;
static volatile int odo_left = 0;
static volatile int odo_right = 0;
static volatile int odo_fresh = 0;

/* ====== 运动命令队列（MQTT 回调写，运动线程读）====== */
typedef struct {
    char dir[8];
    int  duration_ms;
} motion_cmd_t;
static osMessageQueueId_t motion_q;

/* ====== 函数声明 ====== */
static void WiFiInit(void);
static void WaitSacnResult(void);
static int  WaitConnectResult(void);
static BOOL WifiSTATask(void);
static void OnWifiScanStateChangedHandler(int state, int size);
static void OnWifiConnectionChangedHandler(int state, WifiLinkedInfo *info);
static void OnHotspotStaJoinHandler(StationInfo *info);
static void OnHotspotStateChangedHandler(int state);
static void OnHotspotStaLeaveHandler(StationInfo *info);

static uint8_t protocol_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload);
static uint8_t protocol_encode_frame(uint8_t cmd, uint8_t len, const uint8_t *payload,
                                     uint8_t *frame, uint8_t frame_size);
static int  protocol_send_frame(uint8_t cmd, uint8_t len, const uint8_t *payload);
static int  protocol_send_speed(int16_t left, int16_t right);
static int  protocol_send_stop(void);
static void protocol_send_get_status(void);
static int  hardware_init(void);
static int32_t decode_i32_le(const uint8_t *p);
static void odo_rx_thread(void *arg);

static void cmd_handler(uint8_t *recv_data, uint32_t recv_size,
                        uint8_t **resp_data, uint32_t *resp_size);
static void motion_worker(void *arg);
static void sensor_report_thread(void *arg);
static void cloud_start(void);

/* 传感器初始化成功标志（I2C0：SHT20 温湿度 + AP3216C 光照）*/
static int g_sensor_ok = 0;

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

static void OnHotspotStaJoinHandler(StationInfo *info) { (void)info; printf("STA join AP\n"); }
static void OnHotspotStaLeaveHandler(StationInfo *info) { (void)info; printf("HotspotStaLeave:info is null.\n"); }
static void OnHotspotStateChangedHandler(int state) { printf("HotspotStateChanged:state is %d.\n", state); }

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

/* ==================== 协议层（复用 11.0）==================== */
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
        printf("[MQTT] frame encode failed: cmd=0x%02X len=%u\r\n", cmd, len);
        return -1;
    }
    int written = UartWrite(WIFI_IOT_UART_IDX_2, frame, frame_len);
    if (written != (int)frame_len) {
        printf("[MQTT] UART2 write failed: cmd=0x%02X wrote=%d/%u\r\n", cmd, written, frame_len);
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

/* ==================== 硬件初始化（UART2）==================== */
static int hardware_init(void)
{
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    if (GpioInit() != WIFI_IOT_SUCCESS) {
        printf("[MQTT] GPIO init failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD) != WIFI_IOT_SUCCESS) {
        printf("[MQTT] GPIO11 UART2_TXD func set failed\r\n");
        return -1;
    }
    if (IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD) != WIFI_IOT_SUCCESS) {
        printf("[MQTT] GPIO12 UART2_RXD func set failed\r\n");
        return -1;
    }
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("[MQTT] UART2 init failed\r\n");
        return -1;
    }
    printf("[MQTT] UART2 init OK (115200 8N1)\r\n");
    return 0;
}

/* ==================== Odo RX 线程（复用 11.0）==================== */
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
    uint8_t parse_state = 0, parse_cmd = 0, parse_len = 0, parse_idx = 0, parse_cksum = 0;

    while (1) {
        unsigned char empty = 1U;
        if (UartIsBufEmpty(WIFI_IOT_UART_IDX_2, &empty) != WIFI_IOT_SUCCESS) { osDelay(1); continue; }
        if (empty) { osDelay(1); continue; }

        int count = UartRead(WIFI_IOT_UART_IDX_2, buf, sizeof(buf));
        if (count <= 0) { osDelay(1); continue; }

        for (int i = 0; i < count; i++) {
            uint8_t b = buf[i];
            switch (parse_state) {
            case 0: if (b == PROTOCOL_SOF) parse_state = 1; break;
            case 1: parse_cmd = b; parse_state = 2; break;
            case 2:
                parse_len = b; parse_idx = 0;
                if (parse_len == 0) parse_state = 4;
                else if (parse_len <= STATUS_PAYLOAD_LEN) parse_state = 3;
                else parse_state = 0;
                break;
            case 3:
                payload[parse_idx++] = b;
                if (parse_idx >= parse_len) parse_state = 4;
                break;
            case 4:
                parse_cksum = b; parse_state = 0;
                if (parse_cmd == PROTOCOL_CMD_STATUS && parse_len == STATUS_PAYLOAD_LEN) {
                    uint8_t expected = protocol_checksum(parse_cmd, parse_len, payload);
                    if (parse_cksum == expected) {
                        int32_t ol = decode_i32_le(&payload[0]);
                        int32_t orr = decode_i32_le(&payload[4]);
                        osMutexAcquire(odo_mutex, osWaitForever);
                        odo_left = ol; odo_right = orr; odo_fresh = 1;
                        osMutexRelease(odo_mutex);
                    }
                }
                break;
            }
        }
    }
}

/* ==================== MQTT 命令回调（运行在 MQTT 接收线程，勿阻塞）==================== */
static void cmd_handler(uint8_t *recv_data, uint32_t recv_size,
                        uint8_t **resp_data, uint32_t *resp_size)
{
    (void)resp_data; (void)resp_size;
    char buf[192];
    if (recv_data == NULL || recv_size == 0) return;
    if (recv_size >= sizeof(buf)) recv_size = sizeof(buf) - 1;
    memcpy(buf, recv_data, recv_size);
    buf[recv_size] = '\0';
    printf("[MQTT] cmd payload: %s\r\n", buf);

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        printf("[MQTT] json parse fail\r\n");
        return;
    }

    cJSON *cname = cJSON_GetObjectItem(root, "command_name");
    if (cname != NULL && cJSON_IsString(cname) &&
        strcmp(cJSON_GetStringValue(cname), CLOUD_CMD_MOVE) == 0) {
        cJSON *paras = cJSON_GetObjectItem(root, "paras");
        if (paras != NULL) {
            cJSON *dir = cJSON_GetObjectItem(paras, "dir");
            cJSON *dur = cJSON_GetObjectItem(paras, "duration");
            motion_cmd_t m;
            memset(&m, 0, sizeof(m));
            if (dir != NULL && cJSON_IsString(dir)) {
                strncpy(m.dir, cJSON_GetStringValue(dir), sizeof(m.dir) - 1);
            }
            if (dur != NULL && cJSON_IsNumber(dur)) {
                m.duration_ms = dur->valueint;
            }
            if (m.dir[0] != '\0') {
                /* 需求1：切换指令时先排空积压的旧命令，只保留最新一条。
                   长按/连发会把多条命令灌进 motion_q，导致切向后要等旧命令逐条执行完；
                   motion_worker 本就有即时抢占，清空 backlog 后最新指令立刻生效。 */
                motion_cmd_t stale;
                int dropped = 0;
                while (osMessageQueueGet(motion_q, &stale, NULL, 0U) == osOK) {
                    dropped++;
                }
                if (dropped > 0) {
                    printf("[MQTT] flush %d stale cmd(s), latest wins\r\n", dropped);
                }
                if (osMessageQueuePut(motion_q, &m, 0U, 0U) == osOK) {
                    printf("[MQTT] enqueue dir=%s dur=%d\r\n", m.dir, m.duration_ms);
                } else {
                    printf("[MQTT] queue full, drop dir=%s\r\n", m.dir);
                }
            }
        }
    } else {
        printf("[MQTT] not a '%s' command, ignore\r\n", CLOUD_CMD_MOVE);
    }
    cJSON_Delete(root);
}

/* ==================== 运动工作线程（持续/定时 + 速度斜坡 + 即时抢占 + 直行 odo 纠偏）====================
   需求A（赛车丝滑）：duration==0 = 持续模式（按住一直走，每 10ms 续 SET_SPEED 保 STM32 300ms 租约），
   收到 STOP 或新方向才变；duration>0 = 定时模式（走完后斜坡降速停）。
   速度斜坡：cur 每拍向 tgt 逼近 RAMP_STEP，起步/换向/停止都渐变。 */
static int16_t ramp_step(int16_t cur, int16_t tgt)
{
    if (cur < tgt) { cur = (int16_t)(cur + RAMP_STEP); if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur = (int16_t)(cur - RAMP_STEP); if (cur < tgt) cur = tgt; }
    return cur;
}

static void motion_worker(void *arg)
{
    (void)arg;
    motion_cmd_t cmd;
    int active = 0;
    int continuous = 0;
    int16_t tgt_l = 0, tgt_r = 0;   /* 目标轮速 */
    int16_t cur_l = 0, cur_r = 0;   /* 斜坡后的实际输出 */
    int remaining = 0;
    int base_l = 0, base_r = 0, odo_ok = 0;
    int gs_counter = 0;

    while (1) {
        /* 空闲时阻塞等命令；运动中非阻塞轮询以便即时抢占 */
        uint32_t wait = active ? 0U : osWaitForever;
        if (osMessageQueueGet(motion_q, &cmd, NULL, wait) == osOK) {
            if (strcasecmp(cmd.dir, "STOP") == 0) {
                tgt_l = 0; tgt_r = 0; continuous = 0; remaining = 0; odo_ok = 0;
                if (!active) { protocol_send_stop(); cur_l = 0; cur_r = 0; }
                /* 运动中：不立即停，交给下面斜坡降速到 0 再停 */
                printf("[MQTT] -> STOP\r\n");
            } else {
                int16_t b = DEFAULT_SPEED;
                int known = 1;
                if (strcasecmp(cmd.dir, "FWD") == 0)        { tgt_l = b;  tgt_r = b; }
                else if (strcasecmp(cmd.dir, "BACK") == 0)  { tgt_l = -b; tgt_r = -b; }
                else if (strcasecmp(cmd.dir, "LEFT") == 0)  { tgt_l = -b; tgt_r = b; }
                else if (strcasecmp(cmd.dir, "RIGHT") == 0) { tgt_l = b;  tgt_r = -b; }
                else { known = 0; printf("[MQTT] unknown dir '%s'\r\n", cmd.dir); }

                if (known) {
                    if (cmd.duration_ms > 0) { continuous = 0; remaining = cmd.duration_ms / 10; if (remaining <= 0) remaining = 1; }
                    else { continuous = 1; remaining = 0; }   /* duration=0 → 持续（按住一直走）*/
                    odo_ok = 0;
                    if (odo_mutex) {
                        osMutexAcquire(odo_mutex, osWaitForever);
                        if (odo_fresh) { base_l = odo_left; base_r = odo_right; odo_ok = 1; }
                        osMutexRelease(odo_mutex);
                    }
                    gs_counter = 0;
                    active = 1;
                    printf("[MQTT] -> %s %s (tgt L=%d R=%d) odo_ok=%d\r\n",
                           cmd.dir, continuous ? "HOLD" : "timed", tgt_l, tgt_r, odo_ok);
                }
            }
        }

        if (!active) {
            continue;   /* 空闲：上面已用 osWaitForever 阻塞取命令，不会忙等 */
        }

        /* 速度斜坡：实际输出向目标逼近 */
        cur_l = ramp_step(cur_l, tgt_l);
        cur_r = ramp_step(cur_r, tgt_r);

        /* 执行一拍（10ms）*/
        if (++gs_counter >= 10) { gs_counter = 0; protocol_send_get_status(); }

        /* 直行 odo 纠偏（作用于斜坡后的输出）*/
        int16_t out_l = cur_l, out_r = cur_r;
        if (odo_ok && tgt_l == tgt_r && tgt_l != 0 && cur_l == cur_r) {
            int co_l = 0, co_r = 0;
            osMutexAcquire(odo_mutex, osWaitForever);
            co_l = odo_left; co_r = odo_right;
            osMutexRelease(odo_mutex);
            int diff = (co_r - base_r) - (co_l - base_l); /* >0 右轮多→车左偏 */
            int16_t correction = (int16_t)((float)diff * ODO_KP);
            out_l = (int16_t)(cur_l + correction);
            out_r = (int16_t)(cur_r - correction);
            int16_t limit = (int16_t)(DEFAULT_SPEED * 3 / 2);
            if (out_l > limit) out_l = limit;
            if (out_l < -limit) out_l = -limit;
            if (out_r > limit) out_r = limit;
            if (out_r < -limit) out_r = -limit;
            if (cur_l > 0 && out_l < 0) out_l = 0;
            if (cur_l < 0 && out_l > 0) out_l = 0;
        }

        protocol_send_speed(out_l, out_r);

        /* 停止/结束判定 */
        if (tgt_l == 0 && tgt_r == 0) {
            /* 斜坡降速中：到 0 才真正 STOP（松开/停止丝滑）*/
            if (cur_l == 0 && cur_r == 0) {
                protocol_send_stop();
                active = 0;
                printf("[MQTT] -> STOP (ramped)\r\n");
            } else {
                osDelay(PROTOCOL_LEASE_TICKS);
            }
        } else if (!continuous) {
            if (--remaining <= 0) {
                tgt_l = 0; tgt_r = 0; odo_ok = 0;   /* 定时到 → 转斜坡降速 */
                printf("[MQTT] -> timed done, ramping stop\r\n");
            }
            osDelay(PROTOCOL_LEASE_TICKS);
        } else {
            osDelay(PROTOCOL_LEASE_TICKS);   /* 持续模式：一直续租，直到 STOP/新方向 */
        }
    }
}

/* ==================== 遥测上报线程（需求2：5s 读光照+温湿度 → 华为云）====================
   坑：profile_fmtvalue 对 FLOAT 类型读的是 *(double*)value，所以温/湿必须用 double 承接
   （SHT20_ReadData 出 float，转存 double 再挂 kv）。属性名与控制台 Environment 模型逐字一致。 */
static void sensor_report_thread(void *arg)
{
    (void)arg;
    float temp = 0.0f, humi = 0.0f;
    uint16_t ir = 0, als = 0, ps = 0;

    double d_temp = 0.0, d_humi = 0.0;   /* FLOAT kv 的 value 必须指向 double */
    int    i_light = 0;                  /* INT  kv 的 value 指向 int */

    oc_mqtt_profile_kv_t kv_light = {
        .nxt = NULL,        .key = ENV_PROP_LIGHT,
        .type = EN_OC_MQTT_PROFILE_VALUE_INT,   .value = &i_light,
    };
    oc_mqtt_profile_kv_t kv_humi = {
        .nxt = &kv_light,   .key = ENV_PROP_HUMIDITY,
        .type = EN_OC_MQTT_PROFILE_VALUE_FLOAT, .value = &d_humi,
    };
    oc_mqtt_profile_kv_t kv_temp = {
        .nxt = &kv_humi,    .key = ENV_PROP_TEMPERATURE,
        .type = EN_OC_MQTT_PROFILE_VALUE_FLOAT, .value = &d_temp,
    };
    oc_mqtt_profile_service_t service = {
        .nxt = NULL, .service_id = ENV_SERVICE_ID,
        .event_time = NULL, .service_property = &kv_temp,
    };

    while (1) {
        osDelay(SENSOR_REPORT_INTERVAL_MS / 10);   /* osDelay 单位 tick=10ms（与全工程一致）*/
        if (!g_sensor_ok) continue;

        int ok = 1;
        if (SHT20_ReadData(&temp, &humi) != 0) ok = 0;
        if (AP3216C_ReadData(&ir, &als, &ps) != 0) ok = 0;
        if (!ok) { printf("[ENV] sensor read fail\r\n"); continue; }

        d_temp  = (double)temp;
        d_humi  = (double)humi;
        i_light = (int)als;

        int rr = oc_mqtt_profile_propertyreport(CLOUD_DEVICE_ID, &service);
        printf("[ENV] report T=%.1fC H=%.1f%% L=%d rc=%d\r\n", temp, humi, als, rr);
    }
}

/* ==================== 连云启动序列 ==================== */
static void cloud_start(void)
{
    if (hardware_init() != 0) {
        printf("[MQTT] hardware init failed\r\n");
        return;
    }

    odo_mutex = osMutexNew(NULL);
    if (odo_mutex == NULL) printf("[MQTT] odo mutex create failed\r\n");

    motion_q = osMessageQueueNew(MOTION_Q_LEN, sizeof(motion_cmd_t), NULL);
    if (motion_q == NULL) {
        printf("[MQTT] motion queue create failed\r\n");
        return;
    }

    osThreadAttr_t rx_attr = {
        .name = "OdoRx", .attr_bits = 0U, .cb_mem = NULL, .cb_size = 0U,
        .stack_mem = NULL, .stack_size = ODO_RX_STACK_SIZE, .priority = 26,
    };
    if (osThreadNew(odo_rx_thread, NULL, &rx_attr) == NULL) {
        printf("[MQTT] odo rx thread create failed\r\n");
    }

    osThreadAttr_t mo_attr = {
        .name = "Motion", .attr_bits = 0U, .cb_mem = NULL, .cb_size = 0U,
        .stack_mem = NULL, .stack_size = MOTION_STACK_SIZE, .priority = 25,
    };
    if (osThreadNew(motion_worker, NULL, &mo_attr) == NULL) {
        printf("[MQTT] motion worker create failed\r\n");
    }

    device_info_init(CLOUD_CLIENT_ID, CLOUD_USERNAME, CLOUD_PASSWORD);
    oc_set_cmd_rsp_cb(cmd_handler);
    int rc = oc_mqtt_init();
    printf("[MQTT] oc_mqtt_init rc=%d (0=ok)\r\n", rc);

    /* 需求2：初始化 I2C0 传感器（SHT20 温湿度 + AP3216C 光照），起遥测上报线程。
       放在 oc_mqtt_init 之后，保证首次 5s 上报时 MQTT 已连。GPIO9/10(I2C0) 与 UART2(GPIO11/12) 不冲突。 */
    if (SHT20_Init() == 0 && AP3216C_Init() == 0) {
        g_sensor_ok = 1;
        printf("[ENV] sensors init OK (SHT20 + AP3216C on I2C0)\r\n");
    } else {
        printf("[ENV] sensors init FAILED, telemetry disabled\r\n");
    }

    osThreadAttr_t se_attr = {
        .name = "EnvReport", .attr_bits = 0U, .cb_mem = NULL, .cb_size = 0U,
        .stack_mem = NULL, .stack_size = SENSOR_STACK_SIZE, .priority = 24,
    };
    if (osThreadNew(sensor_report_thread, NULL, &se_attr) == NULL) {
        printf("[ENV] sensor report thread create failed\r\n");
    }
}

/* ==================== WiFi STA 任务 ==================== */
static BOOL WifiSTATask(void)
{
    WifiScanInfo *info = NULL;
    unsigned int size = WIFI_SCAN_HOTSPOT_LIMIT;
    WifiDeviceConfig select_ap_config = {0};

    osDelay(200);
    printf("<--System Init-->\r\n");
    WiFiInit();

    if (EnableWifi() != WIFI_SUCCESS) { printf("EnableWifi failed, error = %d\r\n", error); return -1; }
    if (IsWifiActive() == 0) { printf("Wifi station is not actived.\r\n"); return -1; }

    info = malloc(sizeof(WifiScanInfo) * WIFI_SCAN_HOTSPOT_LIMIT);
    if (info == NULL) return -1;

    do {
        ssid_count = 0; g_staScanSuccess = 0;
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
        if (strcmp(CLOUD_WIFI_SSID, info[i].ssid) == 0) {
            int result;
            printf("Select:%3d wireless, Waiting...\r\n", i + 1);
            strcpy(select_ap_config.ssid, info[i].ssid);
            strcpy(select_ap_config.preSharedKey, CLOUD_WIFI_PASSWORD);
            select_ap_config.securityType = WIFI_SEC_TYPE_PSK;
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

    if (g_lwip_netif) { dhcp_start(g_lwip_netif); printf("begain to dhcp\r\n"); }

    for (;;) {
        if (dhcp_is_bound(g_lwip_netif) == ERR_OK) {
            printf("<-- DHCP state:OK -->\r\n");
            netifapi_netif_common(g_lwip_netif, dhcp_clients_info_show, NULL);
            break;
        }
        printf("<-- DHCP state:Inprogress -->\r\n");
        osDelay(100);
    }

    /* DHCP 完成后连云 */
    printf("[MQTT] connecting Huawei Cloud IoTDA ...\r\n");
    cloud_start();

    while (1) osDelay(1000);
    return 0;
}

static void Wifi(void)
{
    osThreadAttr_t attr;
    attr.name = "WifiSTATask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = WIFI_STACK_SIZE;
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)WifiSTATask, NULL, &attr) == NULL) {
        printf("Falied to create WifiSTATask!\n");
    }
}

APP_FEATURE_INIT(Wifi);
