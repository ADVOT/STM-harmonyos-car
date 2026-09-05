# OLED 串口遥测 + 光照控屏 实施记录（8-31）

> USER布置的组合实验（阶段二任务11 OLED × AP3216C 光照预习 × 任务24 双核协议）：
> ① OLED 显示小车由串口发回的内容（= STM32 经 UART2 应答的 STATUS 遥测）
> ② AP3216C 光照传感器：有光时 OLED 灭，没光时 OLED 开
> 工程 = VM `9.0_Oled_Uart_Light`，本地源 = `backups/harmony_app/9.0_Oled_Uart_Light/`（本地即备份）。

## 1. 需求澄清（开工前问过USER）

- 「串行发回的内容」= **STM32 经 UART2 发回**（任务24 双核协议通道），不是蓝牙/UART0
- 独立工程，**不**并进 F.1 防撞防掉合体
- USER提供的根目录 `hal_bsp_ap3216c.c/.h` 经逐字节比对 = supportPack `3_ap3216c` 同一份官方驱动，直接用 supportPack 版，根目录两份文件未采用

## 2. 现成件盘点（零新驱动）

| 件 | 来源 | 关键点 |
|---|---|---|
| SSD1306 OLED | supportPack `1_ssd1306`（任务11 已实机验证） | 驱动自带 `SSD1306_ON/OFF`（0xAF/0xAE + 电荷泵开关，休眠 <10uA，**显存保留**，灭屏状态写屏有效，恢复即见新内容） |
| AP3216C 光照 | supportPack `3_ap3216c` | `AP3216C_ReadData(&ir,&als,&ps)`，ALS=16bit 原始值；**与 OLED 同挂 I2C0（GPIO9/10，地址 0x3C），接线不动**（任务12 已验证 SHT20+OLED 同总线） |
| UART2 协议 | F.1 工程搬代码 | GPIO11/12、115200 8N1、帧 `AA|CMD|LEN|PAYLOAD|CHECK`（CHECK=CMD+LEN+PAYLOAD 低字节）；`UartIsBufEmpty` SDK 有实现但头文件漏声明，需手动 extern |
| STM32 应答 | `motor_pwm/` 8-29 遥测版，零改动 | 收 GET_STATUS(0x04) 只回 STATUS(0x82) **不回 ACK**；payload 13B = odo L/R i32 LE + speed L/R i16 LE + flags u8(bit0=租约) |

## 3. 设计要点

三线程（全优先级 25）：

1. `OulRx`（2K 栈）：搬 F.1 接收——`UartIsBufEmpty` 查空 → `UartRead` → 字节级帧状态机 → STATUS 更新全局快照（`status_mutex`）
2. `OulTx`（1K 栈）：每 200ms 发 `AA 04 00 04`（GET_STATUS；STM32 被动应答，不轮询不说话）。**只查询，永不发运动命令，电机零风险**
3. `OulPanel`（4K 栈，sprintf 需要）：开头顺序 `SSD1306_Init` → `AP3216C_Init`（两驱动各自 I2cInit 重复调幂等，任务12 实证）；loop 300ms = 读 ALS → 滞回控 ON/OFF → 格式化 4 行上屏 → printf 标定流

**I2C 并发对策 = 单线程收编**：OLED 与 AP3216C 共享 I2C0，两驱动内部各自直接 `I2cWrite/Read`，多线程会交错（花屏/读错）。把全部 I2C 访问放进 panel 一个线程顺序执行，天然互斥，连锁都不用。

**显示布局**（16 号字，y 参数是逻辑行 0~3，驱动内部 ×2 换页；每行定宽 16 字符防残留）：

```
 STM32  LINK       ← 静态标题，init 后写一次
L%-7dR%-7d         ← 双轮累计里程（脉冲）
V%-7dV%-7d         ← 双轮瞬时速度
ALS%-6u%-3s        ← 光照原始值 + ON/OFF 屏态
```

**光照滞回**：`als > TH+HYST` 判有光→OFF；`als < TH-HYST` 判没光→ON；中间保持。**8-31 USER实测定版：TH=100、HYST=0**（>100 灭、<100 开、恰 100 保持=单点死区；初版 300/100 时USER实测 300+ 仍亮屏，据此改 100）。滞回本来的两个理由：阈值边界防抖 + **OLED 自发光可能照亮 AP3216C 造成自反馈振荡**——若实测出现屏闪烁，把 HYST 加大（比如 20~50）重编。

**HAL 坑位遵守**：只 UartInit UART2 一路（8-29 实锤 HAL 只放行一次用户 UartInit）。

## 4. 构建

- 本地写完 scp 推 VM → `app/BUILD.gn` 切 feature（F.1 配置留底 `BUILD.gn.safeguard_backup`）→ 全量编译 **BUILD SUCCESS 一次过**
- strings 校验：`[OUL]` 标记 19 处 + OLED 标题串均在 bin 内
- 产物：`out/wifiiot/Hi3861_wifiiot_app_allinone_oul.bin`（SHA-256 a754054b8e47…）
- 坑：sed 插入 feature 行时原行逗号 + 插入行逗号 → `,,` 语法错，手工修掉（下次用 sed 追加行别带逗号）

## 5. 实测阶梯（待USER执行）

1. HiBurn 烧 `Hi3861_wifiiot_app_allinone_oul.bin`（勾 Auto burn）；串口开关切 Hi3861；UartAssist@115200
2. 期望串口：`[OUL] ready: ...` → 每 300ms `[OUL] als=N oled=ON odo=L/R spd=L/R lease=0 seq=递增`（seq 涨 = 链路活）
3. 静止看屏：ODO 不变、V=0；**手推车轮**（不通电，编码器照样计数）→ ODO 跟随变化
4. 手遮 AP3216C（板载光照传感器）→ OLED 应保持/切到亮；手电照 → 灭
5. 阈值不合适：看串口 als 值在有光/遮光两态的实际读数，改 `ALS_LIGHT_THRESHOLD/ALS_HYSTERESIS` 重编（增量编译很快）

## 5. 实测结果（8-31 ✅ 收官）

- 链路/显示/控屏全跑通。**USER实测 ALS 读数：环境光 ~330、遮光 ~30**——阈值 100 居中裕量充足，切换正常无振荡（自发光反馈量 < 滞回需求，HYST=0 即可）
- 首版阈值 300/100 时 300+ 仍亮屏 → USER定改 100/0，增量编译秒级完成

## 6. 待办

- [x] ~~实测定 ALS 阈值~~ ✅ 8-31 USER定 100/0（实测 330/30），bin SHA-256 853416b1…
- [x] ~~留意 OLED 自发光反馈~~ ✅ 8-31 实测无振荡
- [ ] VM BUILD.gn 回切 safeguard 或保持 9.0（看USER下一步安排）
- [ ] 验证后：VM BUILD.gn 回切 safeguard 或保持 9.0（看USER下一步安排）
