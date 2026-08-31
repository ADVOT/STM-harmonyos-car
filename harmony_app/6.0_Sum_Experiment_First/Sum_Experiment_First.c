/*
 * 任务10 第一阶段综合实验：舵机摆头测距 + 红外寻线 + 15s 后切蓝牙
 *
 * 硬件引脚汇总：
 *   GPIO0/1  = UART1（JDY-16 蓝牙，9600/8/1/N）
 *   GPIO2    = SG90 舵机（软件模拟 PWM，20ms 周期，0.5ms=0° / 2.5ms=180°）
 *   GPIO7/8  = HC-SR04 超声波（Trig 输出 / Echo 输入）
 *   GPIO13/14= TCRT5000 红外对管（左/右，低电平 = 黑色）
 *
 * 结构（1 队列 + 1 互斥锁 + 1 单次定时器 + 4 线程，全 4K 栈、全 prio 25 同级轮转）：
 *   thread1  —— 抢锁 → 舵机 0° → 测距打印 → 持锁 200ms → 放锁 → sleep(5)
 *   thread2  —— 同构，舵机 180°；两者借锁交替 = 左右摆头测距
 *   thread3  —— 启动时建 osTimerOnce(1500 ticks = 15s) 并置 flag=2；
 *               循环非阻塞取队列打印（蓝牙模式下的手机消息从这里出来）
 *   TestTask —— 模式分发器：flag==2 → trace()（红外寻线），flag==3 → bluetooth()
 *   定时器回调（15s 到）—— flag 2→3，打印 "bluetooth start"（唯一切换开关）
 *
 * 模式单向切换：进蓝牙后 bluetooth() 是死循环，TestTask 不再返回，回不去寻线。
 *
 * 与参考答案的区别（沿用各任务已验证的改法）：
 *   ① Peripheral_Init() 修笔误：GPIO2 的 GpioSetDir 参考答案写成了 GPIO_1
 *   ② GetDistance 用任务8 带超时版（参考答案 while(1) 死等 Echo 会卡死线程）
 *   ③ 队列【传值不传指针】：消息结构体自带 char Buf[100]，入队整条拷贝
 *      （参考答案传指向 bluetooth() 栈缓冲的指针，靠 sleep 赌读写不错开）
 *   ④ UartRead 是阻塞读（8-27 实锤底层 hi_event_wait），读前 UartIsBufEmpty 查空
 *   ⑤ 固件打印一律英文（中文按 UTF-8 编进固件，GBK 终端乱码）
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_watchdog.h"
#include "hi_time.h"

/* ===== 引脚与参数 ===== */
#define GPIO_UART1_TX   WIFI_IOT_IO_NAME_GPIO_0    /* UART1_TXD → JDY-16 */
#define GPIO_UART1_RX   WIFI_IOT_IO_NAME_GPIO_1    /* UART1_RXD ← JDY-16 */
#define GPIO_SG90       WIFI_IOT_IO_NAME_GPIO_2    /* 舵机信号线 */
#define GPIO_TRIG       WIFI_IOT_IO_NAME_GPIO_7    /* HC-SR04 Trig */
#define GPIO_ECHO       WIFI_IOT_IO_NAME_GPIO_8    /* HC-SR04 Echo */
#define GPIO_IR_LEFT    WIFI_IOT_IO_NAME_GPIO_13   /* 左红外 */
#define GPIO_IR_RIGHT   WIFI_IOT_IO_NAME_GPIO_14   /* 右红外 */

#define UART_BUFF_SIZE    256
#define MSGQUEUE_OBJECTS  16
#define THREAD_STACK_SIZE (1024 * 4)
#define THREAD_PRIO       25

/* 模式标志：0=上电初始，2=红外寻线，3=蓝牙通信（单向切换，只升不降） */
#define MODE_INIT       0
#define MODE_TRACE      2
#define MODE_BLUETOOTH  3

/* Echo 超时：上升沿 100ms（触发处理+声波往返足够）；高电平最长 23.5ms@400cm，给 60ms */
#define ECHO_RISE_TIMEOUT_US 100000
#define ECHO_FALL_TIMEOUT_US 60000

/* SDK 实现了但没在 wifiiot_uart.h 里声明，手动补原型（符号在 libiothardware.a） */
extern unsigned int UartIsBufEmpty(WifiIotUartIdx id, unsigned char *empty);

/* 队列消息【传值】：Idx=序号，Buf=内容，入队即整条拷贝 */
typedef struct {
    uint8_t Idx;
    char Buf[100];
} MSGQUEUE_OBJ_t;

static osMessageQueueId_t mid_MsgQueue;
static osMutexId_t mutex_id;
static osTimerId_t timer_id;
static volatile uint8_t flag = MODE_INIT;   /* 跨线程 + 定时器回调共享，加 volatile */

/* ===== 外设统一初始化（入口调一次；各驱动函数里不再重复设置） ===== */
static void Peripheral_Init(void)
{
    GpioInit();

    /* UART1 → 蓝牙模块 JDY-16（9600/8/1/N 跟模块出厂配置走） */
    IoSetFunc(GPIO_UART1_TX, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(GPIO_UART1_RX, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    WifiIotUartAttribute uart_attr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    if (UartInit(WIFI_IOT_UART_IDX_1, &uart_attr, NULL) != WIFI_IOT_SUCCESS) {
        printf("Failed to init uart1!\r\n");
    }

    /* 舵机 GPIO2 推挽输出（参考答案这里笔误设了 GPIO_1，本版修正） */
    IoSetFunc(GPIO_SG90, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(GPIO_SG90, WIFI_IOT_GPIO_DIR_OUT);

    /* 超声波：Trig 输出平时拉低，Echo 输入 */
    IoSetFunc(GPIO_TRIG, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(GPIO_ECHO, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(GPIO_TRIG, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(GPIO_ECHO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 红外对管：输入 */
    IoSetFunc(GPIO_IR_LEFT, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(GPIO_IR_RIGHT, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIO_IR_LEFT, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIO_IR_RIGHT, WIFI_IOT_GPIO_DIR_IN);
}

/* ===== 超声波测距（任务8 带超时版）：返回距离 cm，超时返回 -1 ===== */
static float GetDistance(void)
{
    WifiIotGpioValue value;
    hi_u64 start, deadline, echo_us;

    /* Trig 发 20us 高电平触发（协议要求 >=10us） */
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 等 Echo 上升沿 */
    deadline = hi_get_us() + ECHO_RISE_TIMEOUT_US;
    do {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (hi_get_us() > deadline) {
            return -1;
        }
    } while (value == WIFI_IOT_GPIO_VALUE0);
    start = hi_get_us();

    /* 等 Echo 下降沿 */
    deadline = start + ECHO_FALL_TIMEOUT_US;
    do {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (hi_get_us() > deadline) {
            return -1;
        }
    } while (value == WIFI_IOT_GPIO_VALUE1);
    echo_us = hi_get_us() - start;

    /* 距离(cm) = 高电平时间(us) x 0.034 / 2（声速 340m/s，除 2 是往返） */
    return (float)echo_us * 0.034f / 2;
}

/* ===== 舵机（任务7 原样）：20ms 周期，高电平 duty 微秒定角度 ===== */
static void set_angle(unsigned int duty)
{
    GpioSetOutputVal(GPIO_SG90, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(GPIO_SG90, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

static void engine_run_0(void)   { for (int i = 0; i < 10; i++) set_angle(500);  }  /* 0° */
static void engine_run_180(void) { for (int i = 0; i < 10; i++) set_angle(2500); }  /* 180° */

/* ===== 红外寻线：读左右对管打印黑白（低电平 = 反射弱 = 黑色） ===== */
static void trace(void)
{
    WifiIotGpioValue val;

    GpioGetInputVal(GPIO_IR_LEFT, &val);
    printf(val == WIFI_IOT_GPIO_VALUE0 ? "left black\r\n" : "left white\r\n");

    GpioGetInputVal(GPIO_IR_RIGHT, &val);
    printf(val == WIFI_IOT_GPIO_VALUE0 ? "right black\r\n" : "right white\r\n");
}

/* ===== 蓝牙模式：死循环读 UART1，收到消息拷进队列（thread3 取出打印） =====
 * 注意此函数永不返回——TestTask 调进来之后就一直跑这里。
 */
static void bluetooth(void)
{
    uint8_t uart_buff[UART_BUFF_SIZE];
    MSGQUEUE_OBJ_t msg;
    static uint8_t seq = 0;

    osTimerDelete(timer_id);   /* 单次定时器已到点，句柄清掉 */
    printf("uart is running!\r\n");

    while (1) {
        unsigned char empty = 1;
        /* UartRead 是阻塞读，空缓冲会挂起线程——先查空再读 */
        if (UartIsBufEmpty(WIFI_IOT_UART_IDX_1, &empty) == WIFI_IOT_SUCCESS && empty == 0) {
            int rt = UartRead(WIFI_IOT_UART_IDX_1, uart_buff, sizeof(uart_buff) - 1);
            if (rt > 0) {
                uart_buff[rt] = '\0';   /* 先补结束符再当字符串用 */
                printf("Uart1 read %d bytes: %s\r\n", rt, uart_buff);
                msg.Idx = seq++;
                strncpy(msg.Buf, (char *)uart_buff, sizeof(msg.Buf) - 1);
                msg.Buf[sizeof(msg.Buf) - 1] = '\0';
                if (osMessageQueuePut(mid_MsgQueue, &msg, 0U, 0U) != osOK) {
                    printf("Message Queue full, msg dropped\r\n");
                }
            }
        }
        osDelay(10);   /* 100ms 查一拍 */
    }
}

/* ===== 15s 单次定时器回调：寻线 → 蓝牙的唯一切换开关 ===== */
static void Timer1_Callback(void *arg)
{
    (void)arg;
    if (flag == MODE_TRACE) {
        flag = MODE_BLUETOOTH;
    }
    printf("bluetooth start\r\n");
}

/* ===== thread1：舵机 0° 测距（与 thread2 借锁交替 = 左右摆头） ===== */
static void thread1(void *arg)
{
    (void)arg;
    osDelay(100U);   /* 1s 启动延时，等外设稳定 */
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread1 is runing.\r\n");
        engine_run_0();
        float distance = GetDistance();
        if (distance < 0) {
            printf("hcsr04 timeout (no echo)\r\n");
        } else {
            printf("distance is %.1f(CM)\r\n", distance);
        }
        osDelay(20U);          /* 持锁 200ms，让对面等一等 */
        osMutexRelease(mutex_id);
        sleep(5);
    }
}

/* ===== thread2：舵机 180° 测距，其余同 thread1 ===== */
static void thread2(void *arg)
{
    (void)arg;
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread2 is runing.\r\n");
        engine_run_180();
        float distance = GetDistance();
        if (distance < 0) {
            printf("hcsr04 timeout (no echo)\r\n");
        } else {
            printf("distance is %.1f(CM)\r\n", distance);
        }
        osDelay(20U);
        osMutexRelease(mutex_id);
        sleep(5);
    }
}

/* ===== thread3：建 15s 定时器 + 置 flag=2 启动寻线；循环非阻塞取队列打印 ===== */
static void thread3(void *arg)
{
    (void)arg;
    MSGQUEUE_OBJ_t rx;

    /* Hi3861 1 tick = 10ms，1500 ticks = 15s */
    timer_id = osTimerNew(Timer1_Callback, osTimerOnce, NULL, NULL);
    if (timer_id == NULL) {
        printf("create timer failed!\r\n");
    } else {
        flag = MODE_TRACE;
        printf("trace start\r\n");
        if (osTimerStart(timer_id, 1500U) != osOK) {
            printf("Timer could not be started\r\n");
        }
    }

    while (1) {
        /* 非阻塞取（timeout=0）：蓝牙模式下手机消息从这里打印出来 */
        if (osMessageQueueGet(mid_MsgQueue, &rx, NULL, 0U) == osOK) {
            printf("queue msg #%u: %s\r\n", rx.Idx, rx.Buf);
        }
        sleep(1);
    }
}

/* ===== TestTask：模式分发器，switch(flag) 派活 ===== */
static void TestTask(void *arg)
{
    (void)arg;
    sleep(5);   /* 等 thread3 把 flag 置成 2 */
    while (1) {
        switch (flag) {
            case MODE_BLUETOOTH:
                bluetooth();   /* 进来就不返回 */
                break;
            case MODE_TRACE:
                trace();
                break;
            default:
                break;
        }
        sleep(1);
    }
}

/* ===== 入口：关看门狗 → 外设初始化 → 队列 → 互斥锁 → 4 线程 ===== */
static void Sum_ExampleEntry(void)
{
    WatchDogDisable();
    Peripheral_Init();

    mid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);
    if (mid_MsgQueue == NULL) {
        printf("Falied to create Message Queue!\r\n");
        return;
    }

    /* 锁必须建在线程之前（任务7 教训：thread2 无延时可能拿到 NULL 锁） */
    mutex_id = osMutexNew(NULL);
    if (mutex_id == NULL) {
        printf("Falied to create Mutex!\r\n");
        return;
    }

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = THREAD_STACK_SIZE;
    attr.priority = THREAD_PRIO;

    attr.name = "thread1";
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Falied to create thread1!\r\n");
    }
    attr.name = "thread2";
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Falied to create thread2!\r\n");
    }
    attr.name = "thread3";
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL) {
        printf("Falied to create thread3!\r\n");
    }
    attr.name = "TestTask";
    if (osThreadNew((osThreadFunc_t)TestTask, NULL, &attr) == NULL) {
        printf("Falied to create TestTask!\r\n");
    }
    printf("[SumFirst] ENTRY.\r\n");
}

APP_FEATURE_INIT(Sum_ExampleEntry);

/*
 * 预期现象（UartAssist@115200，串口开关拨 3861 端）：
 *   [SumFirst] ENTRY.
 *   trace start
 *   thread2 is runing. / distance is xx.x(CM)     <- 舵机 180° 测距
 *   thread1 is runing. / distance is xx.x(CM)     <- 舵机 0° 测距（两者交替 = 摆头）
 *   left white / right white ...                  <- 前 15s 红外寻线打印
 *   bluetooth start                               <- 15s 到
 *   uart is running!
 *   （手机蓝牙调试器连 Gamer_0o0 发消息）→
 *   Uart1 read N bytes: xxx
 *   queue msg #0: xxx                             <- thread3 从队列取出打印
 */
