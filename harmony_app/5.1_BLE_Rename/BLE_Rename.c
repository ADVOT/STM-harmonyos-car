/*
 * 一次性工具：给 JDY-16 蓝牙模块改名（JDY-16 → Gamer_0o0）
 *
 * 背景：班里一车 JDY-16 出厂同名，手机搜蓝牙时分不清哪台是自己的。
 * JDY-16 支持 AT 指令配置，关键约束：
 *   ① AT 指令只在【蓝牙未连接】时有效——连上后模块进透传模式，
 *      UART 收到的字节原样转发给手机，根本不解析 AT。所以烧本固件
 *      前手机必须断开/关闭蓝牙。
 *   ② 指令必须以 \r\n 结尾（JDY 手册原话"务必加上\r\n"）。
 *   ③ 改名格式 AT+NAME<名字>：名字紧跟，无等号无空格；【查询 = AT+NAME 无参数】
 *      （踩坑：网帖说查询是 AT+NAME?，实测 ? 会被当成参数——名字被设成 "?"）
 *      参数写模块自身 Flash，掉电不丢，改一次永久生效。
 *      另：printf 里的中文字符串按 UTF-8 编进固件，GBK 终端显示为乱码——
 *      固件打印一律用英文，中文只写在注释里。
 *   ④ Hi3861 的 UartRead 是阻塞读（底层 hi_event_wait，空缓冲挂起线程），
 *      读前必须 UartIsBufEmpty 查空——任务9 的"每秒轮询"实为阻塞等待。
 *
 * 用法：烧录 → 复位 → UartAssist@115200 看四条指令的响应 →
 *       手机开蓝牙应能搜到 Gamer_0o0 → 验证完烧回 5.0 作业固件。
 */
#include <stdio.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"

#define NEW_NAME "Gamer_0o0"

/* SDK 实现了但没在 wifiiot_uart.h 里声明，手动补原型（符号在 libiothardware.a） */
extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

/* 发一条 AT 指令，并把随后 ~1s 内模块的响应打印到调试串口 */
static void AtCmd(const char *cmd)
{
    uint8_t buf[256];

    printf("[AT >>] %s", cmd);   /* cmd 自带 \r\n */
    UartWrite(WIFI_IOT_UART_IDX_1, (unsigned char *)cmd, strlen(cmd));
    for (int i = 0; i < 10; i++){   /* 10 × 100ms = 1s 收集窗口 */
        unsigned char empty = 1;
        /* 教训（8-27 实锤）：UartRead 是【阻塞读】，底层 hi_event_wait，
         * 空缓冲时调用 = 线程永久挂起。必须先 UartIsBufEmpty 查空再读！ */
        if (UartIsBufEmpty(WIFI_IOT_UART_IDX_1, &empty) == WIFI_IOT_SUCCESS && empty == 0){
            int n = UartRead(WIFI_IOT_UART_IDX_1, buf, sizeof(buf)-1);
            if (n > 0){
                buf[n] = '\0';
                printf("[AT <<] %s\n", buf);
            }
        }
        osDelay(10);   /* 100ms */
    }
}

static void RenameTask(void *arg)
{
    (void)arg;
    char cmd[64];

    /* GPIO0/1 复用为 UART1，9600/8/1/N 跟着 JDY-16 出厂配置走 */
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    WifiIotUartAttribute uart_attr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    if (UartInit(WIFI_IOT_UART_IDX_1, &uart_attr, NULL) != WIFI_IOT_SUCCESS){
        printf("Failed to init uart1!\n");
        return;
    }

    osDelay(200);   /* 2s，等模块上电稳定；此间手机别连蓝牙 */
    printf("=== rename JDY-16 to %s ===\n", NEW_NAME);

    AtCmd("AT\r\n");        /* 连通性测试，期望回 OK */
    AtCmd("AT+NAME\r\n");   /* 查当前名（JDY 惯例：不带参数=查询；带参数=设置！） */
    snprintf(cmd, sizeof(cmd), "AT+NAME%s\r\n", NEW_NAME);
    AtCmd(cmd);             /* 改名，期望回显 +NAME=<新名> + OK */
    AtCmd("AT+NAME\r\n");   /* 再查确认，期望 +NAME=Gamer_0o0 */

    printf("=== done. reset and search %s on phone ===\n", NEW_NAME);
    while (1){
        osDelay(1000);
    }
}

static void BLE_RenameEntry(void)
{
    osThreadAttr_t attr;
    attr.name = "RenameTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)RenameTask, NULL, &attr) == NULL){
        printf("Failed to create RenameTask!\n");
    }
}

APP_FEATURE_INIT(BLE_RenameEntry);
