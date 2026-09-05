/*
 * 任务8：GPIO 驱动 HC-SR04 超声波测距
 *
 * 硬件：HC-SR04 接 Hi3861——Trig=GPIO7（输出触发）/ Echo=GPIO8（输入回响），量程 2~400cm
 *
 * 测距协议：
 *   1. Trig 发 >=10us 高电平触发模块（这里发 20us）
 *   2. 模块发出一串 40kHz 超声波脉冲，同时把 Echo 拉高
 *   3. Echo 高电平持续时间 = 超声波往返时间
 *   4. 距离(cm) = 高电平时间(us) x 0.034 / 2（声速 340m/s，除以 2 是往返）
 *
 * 与参考答案的区别：等 Echo 的两个循环都加了超时退出——
 * 参考答案是 while(1) 死等，Echo 一旦异常（线松/没接/无反射面）线程直接卡死，
 * 只能靠关看门狗硬扛。加超时后测距失败返回 -1，线程照常活着打下一拍。
 */
#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"

#define GPIO_TRIG 7   /* HC-SR04 Trig，输出触发脉冲 */
#define GPIO_ECHO 8   /* HC-SR04 Echo，输入回响信号 */
#define GPIO_FUNC 0

/* 等上升沿的超时：触发后模块内部处理 + 声波往返，100ms 足够 */
#define ECHO_RISE_TIMEOUT_US 100000
/* 高电平最长约 23.5ms（400cm 往返），给 60ms 裕量 */
#define ECHO_FALL_TIMEOUT_US 60000
static volatile unsigned char Flag=0;
/*
 * 测一次距，返回距离(cm)；任一边沿超时（没收到回响）返回 -1
 */
static float GetDistance(void)
{
    WifiIotGpioValue value;
    hi_u64 start, deadline, echo_us;

    /* Trig 发 20us 高电平触发测距 */
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 等 Echo 上升沿（超时返回 -1） */
    deadline = hi_get_us() + ECHO_RISE_TIMEOUT_US;
    do {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (hi_get_us() > deadline) {
            return -1;
        }
    } while (value == WIFI_IOT_GPIO_VALUE0);
    start = hi_get_us();

    /* 等 Echo 下降沿（超时返回 -1） */
    deadline = start + ECHO_FALL_TIMEOUT_US;
    do {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (hi_get_us() > deadline) {
            return -1;
        }
    } while (value == WIFI_IOT_GPIO_VALUE1);
    echo_us = hi_get_us() - start;

    /* 距离(cm) = 高电平时间(us) x 0.034 / 2 */
    return (float)echo_us * 0.034f / 2;
}
static void DTimercallback(void *arg){
    (void)arg;
    Flag=1;
}
static void TTimercallback(void *arg){
    (void)arg;
    printf("tick=%u\r\n",hi_get_tick());
}
/* 测距线程：看纸条测距（3s 定时器置 Flag，看到就测） */
static void Hcsr04Task(void *arg)
{
    (void)arg;
    printf("start test hcsr04\r\n");

    while (1) {
        if(Flag){
            Flag=0;
            float distance = GetDistance();
            if (distance < 0) {
                printf("hcsr04 timeout (no echo)\r\n");
            } else {
                printf("distance is %.1f (cm)\r\n", distance);
            }
        }
        osDelay(1);
    }
}

/* 任务入口：关看门狗 → 初始化引脚 → 建线程 */
static void Hcsr04(void)
{
    WatchDogDisable();

    /* GPIO7/8 设为普通 GPIO 功能，再定方向 */
    hi_io_set_func(GPIO_TRIG, GPIO_FUNC);
    hi_io_set_func(GPIO_ECHO, GPIO_FUNC);
    GpioSetDir(GPIO_TRIG, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(GPIO_ECHO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE0);  /* Trig 平时保持低 */

    osThreadAttr_t attr;
    attr.name = "Hcsr04";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew(Hcsr04Task, NULL, &attr) == NULL) {
        printf("Failed to create Hcsr04 Task!\n");
    }
    osTimerId_t DTimer = osTimerNew(DTimercallback, osTimerPeriodic, NULL, NULL);
    osTimerId_t TTimer = osTimerNew(TTimercallback, osTimerPeriodic, NULL, NULL);
    if(DTimer==NULL||TTimer==NULL){
        printf("Failed to create Timer!\n");
        return;
    }
    osTimerStart(DTimer, 300U);
    osTimerStart(TTimer, 100U);
}

APP_FEATURE_INIT(Hcsr04);
