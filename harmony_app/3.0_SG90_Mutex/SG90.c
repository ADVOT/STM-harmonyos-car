/*
 * 任务7：GPIO 驱动 SG90 舵机 + 互斥锁三线程（演示优先级翻转）
 *
 * 硬件：SG90 舵机接 Hi3861 GPIO2（原理图确认），纯 GPIO 软件模拟 PWM。
 * 舵机协议：20ms 一个周期的脉冲，高电平宽度决定角度——
 *   0.5ms=0°  1.0ms=45°  1.5ms=90°  2.0ms=135°  2.5ms=180°
 * 每个角度连发 10 次脉冲，保证舵机有足够时间转到位置。
 *
 * 演示点（优先级翻转）：
 *   thread3 优先级最低(24) 但无启动延时，上电先抢到锁 → 舵机先 180°
 *   thread1 优先级最高(26) 延时 1s 后来抢锁，只能阻塞等 thread3 释放
 *   → 高优先级被低优先级"堵"住，这就是优先级翻转现象
 */
#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"

#define SG90_GPIO 2

osMutexId_t mutex_id;  /* 互斥锁 ID（全局，三个线程都要用） */
uint8_t flag;          /* 舵机角度标志位，给 thread2 打印用 */

/* 输出一个脉冲：高电平 duty 微秒 + 低电平 (20000-duty) 微秒 = 20ms 周期 */
void set_angle(unsigned int duty)
{
    GpioSetDir(SG90_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(SG90_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);                 /* hi_udelay = 微秒级延时 */
    GpioSetOutputVal(SG90_GPIO, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/* 五个角度各发 10 次脉冲 */
void engine_run_0(void)   { for (int i = 0; i < 10; i++) set_angle(500);  }
void engine_run_45(void)  { for (int i = 0; i < 10; i++) set_angle(1000); }
void engine_run_90(void)  { for (int i = 0; i < 10; i++) set_angle(1500); }
void engine_run_135(void) { for (int i = 0; i < 10; i++) set_angle(2000); }
void engine_run_180(void) { for (int i = 0; i < 10; i++) set_angle(2500); }

/* 任务1：最高优先级(26)，启动延时 1s——来时锁已被 thread3 占着，只能阻塞等 */
static void thread1(void)
{
    osDelay(100U);  /* 100 ticks = 1s（Hi3861 上 1 tick = 10ms） */
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);  /* 抢锁，抢不到就阻塞 */
        printf("thread1 is runing.\r\n");
        flag = 90;
        engine_run_90();   /* 舵机转 90° */
        osDelay(500U);     /* 持锁 5 秒，期间别人抢不到 */
        osMutexRelease(mutex_id);
    }
}

/* 任务2：中优先级(25)，不碰锁——互斥锁对它毫无影响，照跑照打印 */
void thread2(void)
{
    osDelay(100U);
    while (1) {
        printf("thread2 is runing.\r\n");
        switch (flag) {
            case 90:  printf("SG90 turn 90 du.\r\n");  break;
            case 180: printf("SG90 turn 180 du.\r\n"); break;
            default:  break;
        }
        flag = 0;  /* 清除标志位 */
        osDelay(100U);
    }
}

/* 任务3：最低优先级(24)，但无启动延时——上电第一个抢到锁，舵机先 180° */
void thread3(void)
{
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread3 is runing.\r\n");
        flag = 180;
        engine_run_180();  /* 舵机转 180° */
        osDelay(300U);     /* 持锁 3 秒，高优先级的 thread1 只能等 */
        osMutexRelease(mutex_id);
    }
}

/***** 入口：初始化 + 建锁 + 建三个线程 *****/
static void SG90(void)
{
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);

    /* 注意：锁必须建在线程之前！参考答案把 osMutexNew 放在三个
       osThreadNew 后面，thread3 无延时可能拿到 NULL 锁去 Acquire */
    mutex_id = osMutexNew(NULL);
    if (mutex_id == NULL) {
        printf("Falied to create Mutex!\n");
    }

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    /* 有效优先级区间 9~38，数字越大优先级越高（CMSIS 方向） */
    attr.name = "thread1";
    attr.priority = 26;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Falied to create thread1!\n");
    }
    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Falied to create thread2!\n");
    }
    attr.name = "thread3";
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL) {
        printf("Falied to create thread3!\n");
    }
}

APP_FEATURE_INIT(SG90);
