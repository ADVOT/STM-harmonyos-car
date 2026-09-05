# 华为云 MQTT 网页控制小车 — 实施记录

> 2026-09-02 起。阶段二任务14-16（WiFi/MQTT/华为云）的落地成果：自建网页 → 华为云 IoTDA → 小车。
> 密钥不写进本文件。

## 0. 目标与结论

在网页上点按钮/填时长 → 经华为云 IoTDA → 下发到小车 Hi3861 → 驱动 STM32 电机（前进/后退/左转/右转/停止）。
**可行，且设备侧几乎全是现成积木。** 固件已在 VM `BUILD SUCCESS`（占位凭据版，验证编译）；待填真凭据重编 + 烧录 + 端到端联调。

## 1. 架构（三端）

```
[网页 public/] --HTTP POST /api/command--> [Node 后端 server.js]
   --AK/SK 签名(SDK-HMAC-SHA256)--> [华为云 IoTDA 应用侧 REST]
   --命令下发(MQTT)--> [Hi3861 设备侧 MQTT 客户端]
   --订阅 $oc/devices/{id}/sys/commands/#--> cJSON 解析 move
   --UART2 协议帧 AA|CMD|LEN|PAYLOAD|CHECK--> [STM32 motor_pwm] → 双电机
```

关键约束（华为云 IoTDA 特性，决定了为什么要有后端）：
- IoTDA 的 **MQTT 是设备侧通道**，每个设备只能收发自己的 `$oc/devices/{自己id}/...`；**给别的设备下发命令是应用侧能力，只能走 REST（需 AK/SK 签名）或 AMQP**。
- 所以浏览器即使能 MQTT-over-WebSocket 连华为云，作为"一个设备"也没法直接给小车发命令 → 必须有个持 AK/SK 的后端代理应用侧 REST。
- 附带好处：小车和电脑**不必同一局域网**（各自上云汇合），不同于 11.0 的同网 TCP 直连。

## 2. 复用的现成资产（没重写）

| 资产 | 来源 | 复用点 |
|---|---|---|
| WiFi+DHCP+UART2 协议+电机差速+odo 纠偏 | `harmony_app/11.0_TCP_Control/TCP_Control.c` | 几乎整份，只把 TCP 服务器换成 MQTT |
| 华为云 MQTT 连接模块 | `supportPack/5_QST_car/src/oc_mqtt.c`+`include/oc_mqtt.h` | `oc_mqtt_init/device_info_init/oc_set_cmd_rsp_cb/oc_mqtt_profile_propertyreport` |
| profile 打包 | `.../oc_mqtt_profile_package.c/.h` | 属性上报/命令响应打包（只依赖 cJSON，不需要 iot_link） |
| paho MQTT | `supportPack/paho_mqtt`（MQTTClient-C + MQTTPacket + liteOS 端口） | 设备侧 MQTT 协议栈 |

## 3. 关键发现 / 坑

- **VM 里原本没有 paho**（`third_party/paho_mqtt`、`third_party/iot_link` 都不存在；只有 lwIP 自带的 `mqtt.h`，API 完全不同）。`usr_config.mk` 的 `CONFIG_MQTT=y` 网关的是 **lwIP 的 mqtt**，与 paho 无关。→ 需一次性把 `supportPack/paho_mqtt` 拷到 VM `third_party/paho_mqtt`（**VM 重建后要重拷**）。
- **`5_QST_car`/`4_Sum` 没有 main 应用入口，也没有任何 `MQTTSubscribe`**（`mq_client` 是 static）。订阅命令主题这层得自己加：在 `oc_mqtt.c` 的 `oc_mqtt_entry()` 里 `MQTTConnect` 成功后补 `MQTTSubscribe(&mq_client, "$oc/devices/{device_id}/sys/commands/#", QOS1, mqtt_callback)`。
- **`transport.c` 不能编进来**：它是 MQTTPacket 的 sample 传输层，我们用 liteOS 端口（`MQTTLiteOS.c` 提供 `NetworkInit/NetworkConnect/linux_read/write`）；BUILD.gn 源清单不含 transport.c。
- **paho 的 `MQTTCLIENT_PLATFORM_HEADER` include 有 `#if defined` 保护**，不定义即跳过，不影响编译。
- **`usr_config.mk` 现状已够用**：`CONFIG_CJSON=y`、`CONFIG_UART2_SUPPORT=y`（电机链路），无需改。
- **oc_mqtt.c 的命令响应 topic 是课程遗留的 `$crsp/`+topic[6:]**，不符 IoTDA 标准。阶段一我们**不发命令响应**（cmd_handler 不填 resp_data），小车照常执行，只是 IoTDA 同步命令会显示"响应超时"。要干净 ACK 再改这里。
- **VM 全量编译会清掉 `out/wifiiot/` 里的非标准命名文件**（我放那的 `allinone.bin.l0_trace_backup` 被删了，导致一次没还原成功）。→ 教训：**备份 bin 别放 out/wifiiot，放 `~/` 或工程目录**。
- **华为云接入地址随实例格式不同，务必以控制台「设备接入」段为准**（本次我一度判断错，纠正）：
  - USER的实例（新格式）：设备接入段里 **MQTT(1883) / MQTTS(8883) / MQTT over WebSocket(443) / HTTPS(443) 共用同一域名** `{id}.st1.iotda-device.{region}.myhuaweicloud.com`，靠**端口**区分协议。→ 固件 `CLOUD_MQTT_HOST` = 这个 `st1.iotda-device` 域名、`CLOUD_MQTT_PORT` = **1883**。（**不是** iot-mqtts！我最初拿任务16 文档的老格式套，误判成"填错了"，其实USER originally 填的就是对的。）
  - 任务16 文档示例 `{id}.iot-mqtts.{region}...:1883` 是**老实例格式**，不同实例不一样，别照搬。
  - CoAP(S) 设备口 = `{id}.st1.iotda-coaps.{region}...`；应用侧 API 口 = `{id}.st1.iotda-app.{region}...`（后端 `.env` `HW_IOTDA_APP_ENDPOINT` 用 HTTPS 443 这个，USER已填对）。
  - 教训：判断端点类型**直接看控制台「设备接入」里 MQTT(1883) 那行对应的域名**，别凭域名关键字猜。自检改用 `strings allinone.bin | grep -c iotda-device`（本实例固件应≥1）。

## 4. 文件清单（本次新增）

设备固件 `harmony_app/12.0_MQTT_Cloud_Control/`：
- `src/MQTT_Cloud_Control.c` — 主应用（WiFi/DHCP 复用 11.0；`cloud_start()` 起 UART2+odo_rx+motion_worker 线程、填鉴权、注册 cmd_handler、`oc_mqtt_init`；`cmd_handler` cJSON 解析 move 入队；`motion_worker` 带即时抢占 + 直行 odo 纠偏）
- `src/oc_mqtt.c` — 拷自 supportPack，改 2 处：接入域名走 cloud_config.h、连接成功后补订阅命令主题（含 NetworkConnect/MQTTConnect 返回值打印）
- `src/oc_mqtt_profile_package.c` — 原样拷入
- `include/oc_mqtt.h` — 改：`OC_SERVER_IP/PORT` = `CLOUD_MQTT_HOST/PORT`
- `include/oc_mqtt_profile_package.h` — 原样
- `include/cloud_config.example.h` — 配置模板（入库）
- `include/cloud_config.h` — 真值（**gitignore，不入库**）
- `BUILD.gn` — static_library，编 3 个本地源 + 12 个 paho 源（绝对路径 `//third_party/paho_mqtt/...`），include 挂 paho/cJSON/lwip/cmsis/wifiiot

后端 + 网页 `web/cloud_car_server/`：
- `huawei.js` — **用华为官方 SDK**（`@huaweicloud/huaweicloud-sdk-iotda`，签名由 SDK 处理，不再手写）：`sendCommand(dir,duration)`→`createCommand`（路径 `POST /v5/iot/{project_id}/devices/{device_id}/commands`，**注意有 `/iot` 段**，漏了会 404 IOTDA.000029）；`getShadow()`→`showDeviceShadow`（阶段二）
- `server.js` — express 托管 public/ + `POST /api/command`（校验 dir、夹取 duration、STOP→0）+ `GET /api/status`
- `public/index.html|app.js|style.css` — 方向按钮 + 时长 + 键盘方向键(空格=停) + 遥测刷新/自动轮询区 + 下发日志
- `.env.example`（入库）/ `.env`（**gitignore**）
- `package.json` — 依赖 express + dotenv + `@huaweicloud/huaweicloud-sdk-iotda` + uuid（SDK 的 peer 依赖，缺了报 Cannot find module 'uuid'）

`.gitignore` 新增：`web/cloud_car_server/.env`、`.../node_modules/`、`harmony_app/12.0_MQTT_Cloud_Control/include/cloud_config.h`

## 5. 云端模型（USER的 IoTDA 产品）

- service_id = `Industrial_Internet`（属性 `Electric_Current`、`Power_consumption`；已有命令 `Power`，均保留不用）
- **新增命令 `move`**：参数 `dir`(string 枚举 FWD/BACK/LEFT/RIGHT/STOP) + `duration`(int, ms)
- 设备收到的下发 payload：`{"object_device_id":..,"service_id":"Industrial_Internet","command_name":"move","paras":{"dir":"FWD","duration":1000}}`
- 后端 REST 下发 body：`{"service_id":"Industrial_Internet","command_name":"move","paras":{"dir":"FWD","duration":1000}}`
- WiFi = 手机热点即可，**必须 2.4GHz**（Hi3861 只支持 2.4G），加密 WPA2-PSK（固件写死 `WIFI_SEC_TYPE_PSK`）

## 6. 可回滚的 VM 编译流程（不破坏USER的 L.0 循线环境）

VM 当前活动目标是 `L.0_Trace_Following`。编译 12.0 会覆盖 `allinone.bin`，故用备份-切换-编译-提取-还原：
1. `cp app/BUILD.gn app/BUILD.gn.mqtt_cloud_backup`
2. `cp out/wifiiot/Hi3861_wifiiot_app_allinone.bin ~/allinone_l0_backup.bin`（**放 home，不放 out/**）
3. 改 app/BUILD.gn features → 只留 `startup` + `12.0_MQTT_Cloud_Control:MQTT_Cloud_Control`
4. `export PATH=...gcc_riscv32/bin:ninja:gn:hc-gen:llvm/bin && python build.py wifiiot`
5. 校验：`strings out/wifiiot/...allinone.bin | grep "connecting Huawei Cloud"`
6. `cp allinone.bin allinone_mqtt_cloud.bin`；scp 到本地 `out/wifiiot/`
7. 还原：`cp BUILD.gn.mqtt_cloud_backup app/BUILD.gn`；`cp ~/allinone_l0_backup.bin out/wifiiot/Hi3861_wifiiot_app_allinone.bin`
8. 清理临时备份

> 编译命令（SETUP §3 定型）：ssh harmony-vm 非交互不加载 .bashrc，必须显式 export 工具链 PATH。

## 7. 联调步骤

1. 云端加 `move` 命令（§5）。
2. 填 `cloud_config.h`（VM 工程内，WiFi+MQTT域名+device_id+iot-tool三元组+service_id）与后端 `.env`（AK/SK+project_id+应用侧API地址+device_id）。
3. 按 §6 用真凭据重编 12.0 → scp bin 到本地。
4. HiBurn 烧录（**勾 Auto burn**，否则 load fail 0xC35A69A5）。
5. 串口（UartAssist@115200）应看到：WiFi connect → DHCP OK → `[OC] NetworkConnect rc=0` → `[OC] MQTTConnect rc=0` → `[OC] MQTTSubscribe $oc/devices/.../sys/commands/# rc=0`。
6. 后端：`cd web/cloud_car_server && npm install && npm start`。
7. 网页 `http://localhost:3000` 点"前进 1000ms" → 小车前进 1s 停；LEFT/RIGHT/BACK/STOP 逐一验；运动中再点验即时抢占。
8. 也可先用 IoTDA 控制台"命令下发"直接测设备侧（不经网页）。

## 8. 待办 / 阶段二

- [x] 填真凭据重编 + 烧录 + 端到端联调（**9-03 全通**，见 §10）
- [x] 设备命令响应（改 oc_mqtt.c：`$crsp` → IoTDA 标准 `.../sys/commands/response/request_id={rid}`，且**异步发**避免接收线程死锁，见 §10.3）——网页命令下发从 21s 超时降到 **744ms / HTTP200 / result_code:0**
- [ ] 阶段二遥测回显：**改为上报 光照(AP3216C)+温湿度(SHT20)**（原计划的 odo/speed 暂缓）；云模型加属性 + 固件 `oc_mqtt_profile_propertyreport` 定时上报；网页 `/api/status`+遥测区已有（进行中，见 §12）
- [ ] 需求1：长按 flooding → 切换指令时清空 motion_q 只执行最新（进行中，见 §12）
- [x] 端到端通过后更新 `SETUP.md` 进度
- [ ] git 提交前先问USER（AGENTS 铁律）；密钥文件已 gitignore

## 9. 后端 401 排查全记录（9-02）—— 已解决

现象：`sendCommand` 一直 `401 IOTDA.000002 Authentication failed`。

排查中我先后给过两个**错误**判断，都更正掉：①一开始怪"USER的 AK/SK 填错 / project_id 是账号ID"；②后又怪"代码只给 endpoint 没设 region"。USER核对后 AK/SK 与 `credentials.csv` 完全一致、`HW_PROJECT_ID` 确为 cn-north-4 项目ID（账号ID 是另一串，两者别混淆）、User Name 是**账号本身**（账号级密钥权限全开）——凭证侧全对。

### 对照实验（同一把有效账号密钥）

| 写法 | 命中域名 | Authorization 算法 | 结果 |
|---|---|---|---|
| A. 只 `withEndpoint` | st1 专属域名 | `SDK-HMAC-SHA256`(标准) | **401** |
| B. `withRegion(IoTDARegion.CN_NORTH_4)` | 被 SDK 换成通用域名 `iotda.cn-north-4...` | 标准 | **403 IOTDA.000021**「未订阅」——签名**过了** |
| C. `new Region('cn-north-4', st1域名)` | st1 专属域名 | 标准 | **401** |
| **D. C + `withDerivedPredicate(getDefaultDerivedPredicate)`** | st1 专属域名 | **`V11-HMAC-SHA256` 派生** | **403 IOTDA.014016「设备不在线」= 鉴权通过！** |

### 真正根因（读 SDK 源码定位）

- `AKSKSigner.js`：标准签名 `hmacSHA256(SK, stringToSign)`，**不含 region/service** → region 是红鲱鱼，A/C 加不加 region 都一样 401。
- `BaseCredentials.isDerivedAuth()`：只有当 `derivedPredicate` 被设置时才可能走派生签名；而**全 SDK 没有任何地方自动设 `derivedPredicate`** → 默认永远标准签名。
- `getDefaultDerivedPredicate(req)` = `!DEFAULT_ENDPOINT_REG.test(hostname)`；该正则只允许「一个可选中段标签」：
  - 通用域名 `iotda.cn-north-4.myhuaweicloud.com` → 匹配 → 用**标准签名**（通用网关接受 → B 的 403）。
  - 专属域名 `xxx.st1.iotda-app.cn-north-4.myhuaweicloud.com`（st1/iotda-app/cn-north-4 三段）→ **不匹配 → 需派生签名**；但 SDK 没开派生 → 用标准签名 → 专属网关拒签 **401**。
- `DerivedAKSKSigner.js:94`：派生 info = `{date}/{credential.getRegionId()}/{derivedAuthServiceName}`，服务名 `iotdm` 由 `IoTDAClient.newBuilder()` 自动设；regionId 由 `ClientBuilder` 从 `withRegion(region).id` 注入 → 派生签名**需要 region**（这才是 region 的真正用处，不是标准签名）。

一句话：**IoTDA 标准版实例的专属 st1 域名要求派生签名 V11-HMAC-SHA256(iotdm+region)，华为官方 Node SDK 默认不开派生谓词，导致退回标准签名被专属网关 401 拒绝。**

### 修复（已固化进 `huawei.js` `buildClient`）

```js
const cred = new core.BasicCredentials({ ak: AK, sk: SK, projectId: PROJECT_ID });
// 关键：开启派生签名谓词（专属域名走 V11-HMAC-SHA256，通用域名走标准）
cred.withDerivedPredicate(
  core.BasicCredentials.getDefaultDerivedPredicate.bind(core.BasicCredentials)
);
const region = new core.Region(process.env.HW_REGION || 'cn-north-4', 'https://' + host);
return iotda.IoTDAClient.newBuilder().withCredential(cred).withRegion(region).build();
```
`.bind(core.BasicCredentials)` 必须加，否则静态方法里的 `this.DEFAULT_ENDPOINT_REG` 丢失。`.env.example` 补了 `HW_REGION=cn-north-4`。

### 验证结果（9-02，后端整条通）

- Authorization 头 = `V11-HMAC-SHA256 Credential=<AK>/20260902/cn-north-4/iotdm` ✓
- `POST /api/command {FWD,1000}` 与 `{STOP,0}` → `403 IOTDA.014016 设备不在线`（命令已被云受理，仅因固件未烧、设备离线）。
- `GET /api/status` → **`200`**，返回设备影子，含历史上报属性 `Electric_Current:47 / Power_consumption:58`（version 3）→ **阶段二遥测查询接口也已通**。

> 教训：401 不一定是密钥/region 错。华为专属实例域名(st1)需**派生签名**，而官方 Node SDK 默认不开 `derivedPredicate`。排查签名类 401 时，先看 `Authorization` 头是 `SDK-HMAC-SHA256`(标准) 还是 `V11-HMAC-SHA256`(派生)，再对着目标域名该用哪种签名去核对——别急着改 region、更别急着怀疑用户密钥。

### 下一步（唯一剩余）

- [x] USER按 §6/§7 烧录 → 设备上线 → 重发命令 `200` 且小车动（**9-03 完成，见 §10**）。

---

## 10. 设备侧联调全记录（9-03）—— 端到端打通

烧录后逐个排掉三个设备侧坑，最终 **网页点按钮 → 小车动 + 云端秒回 ACK** 全通。

### 10.1 设备侧 MQTT 鉴权（MQTTConnect rc=-1 → rc=0）

华为 IoTDA 设备侧 MQTT 三元组规则（**官方文档核实**，与"应用侧 AK/SK"完全两回事）：

- **clientId** = `{deviceId}_0_{connectType}_{timestamp}`
  - `connectType=0`：HMACSHA256，**不校验时间戳新鲜度** → 固定时间戳永久可用（本次用这个）
  - `connectType=1`：HMACSHA256，**校验时间戳**（必须近期，否则拒）
  - `timestamp` = `YYYYMMDDHH`，**UTC 时间**（02 UTC = 北京 10:00）
- **username** = `{deviceId}`（恒等于设备ID，**不是**密钥/密码串）
- **password** = `HMACSHA256(deviceSecret, timestamp)` 的 64 位十六进制；**connectType 不参与 password 计算** → 把 clientId 的 `1` 改 `0` 不影响 password 有效性

本次两个 bug（都在 `cloud_config.h`）：① `CLOUD_USERNAME` 误填成那串十六进制 password → 改成 deviceId；② `CLOUD_CLIENT_ID` 用了 `connectType=1` + 过期 UTC 时间戳（`2026090202`≈14h 前）→ 改 `connectType=0`。password 不动。改完 `MQTTConnect rc=0`、`MQTTSubscribe rc=0`、`oc_mqtt_init rc=0`，控制台显示在线。

> USER两次质疑时间戳解读，澄清：**设备名后缀 `_20260902`（8位，注册时随便填、不透明）≠ clientId 末尾鉴权时间戳 `_2026090202`（10位 YYYYMMDDHH，且是 UTC）**。控制台注册 10:23 GMT+08:00 = 02:23 UTC → 对上 `202609 02 02`。设备曾于 9-02 10:38 激活过 = secret 有效，无需重置密钥。

### 10.2 KERNEL PANIC：MQTTTask 栈溢出（2048 → 8192）

现象：STOP 能收，FWD 一收就 `KERNEL PANIC` 重启。根因：paho 端口 `third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.c:33` `attr.stack_size = 2048` 太小，撑不住 `cmd_handler`（`buf[192]` + `cJSON_Parse` 递归 + printf）。STOP 路径浅勉强活，FWD 路径溢出。改 **2048 → 8192** 后 FWD 干净执行（`-> FWD 1000ms (L=140 R=140)` → `-> STOP (done)`，无 panic）。

> 这是 **VM 系统级改动**（不在 app 工程内），VM 重建后要重打，连同 §3 的 paho 拷贝、`APP_INIT_EVENT_NUM` 等一起记进 VM 环境清单。

### 10.3 命令响应 22s 超时（IOTDA.014111）—— 死锁根因 + 异步修法

现象：命令收到、车也动，但后端 `POST /api/command` **干等 21s** 才回，且 `error_code: IOTDA.014111`（Command request timed out）。即设备执行了命令却没在超时窗口内回应用层响应。

**第一版修法（错，会死锁）**：在 `mqtt_callback` 里同步 `oc_mqtt_profile_cmdresp(...)` 发响应。结果仍 21s 超时。读 paho 源码定位到**必然死锁**：

- paho 的"互斥锁"其实是 `MQTTLiteOS.c:87` `osSemaphoreNew(1,1,NULL)` —— **二值信号量，无持有者概念、不可重入**。
- `MQTTClient.c:376` `MQTTRun` 先 `MutexLock`（acquire 信号量，计数 1→0），再 `cycle()`（379）分发收到的 PUBLISH → 回调 `mqtt_callback`。
- 我在回调里 `MQTTPublish` → `MQTTClient.c:627` 又 `MutexLock` 同一信号量 → **同线程对已为 0 的信号量二次 acquire，`LOS_WAIT_FOREVER` 永久阻塞**。
- 后果：MQTTTask 卡死，响应发不出（云端 22s 超时），keepalive 也停摆 → 设备随后掉线。`cmd_handler` 在 publish 前已把运动入队，所以"车会动但响应超时"完全吻合。

**正确修法（异步响应，标准范式）**：响应绝不在 MQTT 接收线程里发。`oc_mqtt.c` 改为：
- `mqtt_callback` 只从命令 topic 提取 `request_id`，`osMessageQueuePut` 投进 `s_resp_q`（深度 8），**不 publish**。
- 新增独立线程 `oc_resp_worker`（`oc_mqtt_init` 里创建，栈 4096，osPriorityNormal）：`osMessageQueueGet` 取出 rid → `oc_mqtt_profile_cmdresp(CLOUD_DEVICE_ID, {ret_code:0})` → publish `{"result_code":0}` 到 `$oc/devices/{id}/sys/commands/response/request_id={rid}`。
- 该线程不持接收信号量，发布时 `MQTTRun` 早已 `cycle()` 返回并释放信号量 → 干净 acquire，无死锁；且响应延迟与运动执行**解耦**（哪怕下 60s 长指令也能秒回）。

**验证（9-03）**：`POST {FWD,1000}` → **744ms** 返回，`{"ok":true,"status":200,"data":{"command_id":"...","response":{"result_code":0}}}`；串口 `[OC] cmdresp rid=... rc=0`；浏览器点"前进"→ 下发日志 `✔ FWD 1000ms 已下发（HTTP 200）`，小车前进 1s。**只动了 `oc_mqtt.c`，未碰 app 与头文件、未改 paho。** 新 bin 757800B，SHA-256 `04079a1f...`，特征串 `oc_resp`/`cmdresp rid`/`commands/response/request_id` 均在、旧 `$crsp` 已消失。

> 通用教训：**paho embedded 的锁是不可重入二值信号量，任何在消息回调里同步 publish/subscribe 的写法都会自死锁**。命令响应、以及任何"收到消息后要回发"的逻辑，一律交给独立线程异步做。

### 10.4 本地后端常驻（EADDRINUSE）

后台跑 node 反复被工具 kill / `&` 孤儿进程占住 3000 端口。定型：杀 `MSYS_NO_PATHCONV=1 taskkill /PID <pid> /F`；起 `setsid node server.js > server_run.log 2>&1 < /dev/null &` + `disown`。`netstat` 里 `0.0.0.0:3000` 与 `[::]:3000` 两行是**同一进程**的 IPv4/IPv6 双栈监听，kill 一个 PID 即可。

---

## 11. 演示 Runbook（现场按此走）

**A. 硬件/云端前置**：① 手机热点开（SSID `ANSWER THE CALL OF` / 密码 `D.U.T.Y.`，**必须 2.4GHz**，Hi3861 只支持 2.4G）；② 小车 Hi3861+STM32 上电（Hi3861 烧 `mqtt_cloud` 固件，STM32 沿用 motor_pwm 最新版，都不用再烧）；③ 串口(UartAssist@115200) 确认 `[OC] MQTTConnect rc=0` / `MQTTSubscribe ... rc=0` / `oc_mqtt_init rc=0`（= 云端在线）。

**B. 起后端**：`cd web/cloud_car_server && node server.js` → 打印 `cloud-car-server 已启动: http://localhost:3000`。窗口全程别关。报 `EADDRINUSE` = 已有实例在跑，直接用或 `taskkill /PID <netstat查到的> /F` 再启。

**C. 开网页**：`http://localhost:3000`。

**D. 操作**：填运动时长(默认1000ms) → 点 前进/后退/左转/右转（或键盘方向键，空格=停）→ 车动 + 下发日志 `✔ ... HTTP 200`；点停止立即停。投串口可见全链路：`[MQTT] cmd payload` → `[OC] cmdresp rid=... rc=0` → `[MQTT] -> FWD ...`。

**注意**：`.env` 是密钥别投屏；第一次点若卡 ~20s/超时基本是设备没在线（查 A 的热点+串口三行）。

