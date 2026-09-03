#ifndef CLOUD_CONFIG_H
#define CLOUD_CONFIG_H

/* ============================================================
 * 华为云 IoTDA + WiFi 接入配置（模板）
 *
 * 用法：把本文件复制为 cloud_config.h，填入真实值。
 *   cloud_config.h 已加入 .gitignore，不会上传；本 .example 可入库。
 * 所有值均可在华为云 IoTDA 控制台获取，切勿把真实密钥提交到仓库。
 * ============================================================ */

/* ===== WiFi（STA 连接的路由/热点）===== */
#define CLOUD_WIFI_SSID            "YOUR_WIFI_SSID"
#define CLOUD_WIFI_PASSWORD        "YOUR_WIFI_PASSWORD"

/* ===== 华为云 IoTDA 设备侧 MQTT 接入 =====
 * 控制台 → 总览 → 接入信息 → "MQTT(1883) 设备接入地址"
 * 形如 xxxxxxxxxx.iot-mqtts.cn-north-4.myhuaweicloud.com（每人不同） */
#define CLOUD_MQTT_HOST            "YOUR_INSTANCE.iot-mqtts.cn-north-4.myhuaweicloud.com"
#define CLOUD_MQTT_PORT            1883

/* 注册设备时生成的设备 ID */
#define CLOUD_DEVICE_ID            "YOUR_DEVICE_ID"

/* ===== MQTT 鉴权三元组 =====
 * 用华为云 iot-tool 由 device_id + device_secret 生成：
 *   https://iot-tool.obs-website.cn-north-4.myhuaweicloud.com/
 * 填 Generate 出来的 ClientId / Username / Password */
#define CLOUD_CLIENT_ID            "YOUR_CLIENT_ID"
#define CLOUD_USERNAME             "YOUR_USERNAME"
#define CLOUD_PASSWORD             "YOUR_PASSWORD"

/* ===== 产品模型标识（与云端一致）=====
 * service_id = 你产品里的服务 ID；命令名 = 新增的 move 命令 */
#define CLOUD_SERVICE_ID           "Industrial_Internet"
#define CLOUD_CMD_MOVE             "move"

#endif /* CLOUD_CONFIG_H */
