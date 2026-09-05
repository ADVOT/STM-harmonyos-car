#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_sht20.h"

osSemaphoreId_t sem1;

void thread1(void)
{
    while (1) {
        // 1秒中释放两次sem1信号量，使得thread2和thread3能同步执行
        osSemaphoreRelease(sem1);
        // 此处若只释放一次信号量，则thread2和thread3会交替运行
        osSemaphoreRelease(sem1);
        printf("\n");
        printf("Thread1释放信号量!\n");
        osDelay(300); // 延时3秒
    }
}

void thread2(void)
{
    float temperature = 0, humidity = 0;
    printf("i2c_sht20_demo()!\n");
    SHT20_Init(); // SHT20初始化
    while (1) {
        // 等待sem1信号量
        osSemaphoreAcquire(sem1, osWaitForever);
        SHT20_ReadData(&temperature, &humidity);
        printf("temperature = %.2f humidity = %.2f\r\n", temperature, humidity);
        printf("Thread2 得到信号量!\n");
        osDelay(1); // 延时10ms
    }
}

void thread3(void)
{
    while (1) {
        // 等待sem1信号量
        osSemaphoreAcquire(sem1, osWaitForever);
        printf("Thread3 得到信号量!\n");
        osDelay(1); // 延时10ms
    }
}

static void i2c_sht20_demo(void)
{
    osThreadAttr_t attr;

    // 修正参考答案的竞态隐患：先创建信号量，再创建线程
    sem1 = osSemaphoreNew(4, 0, NULL); // 创建信号量初始值为0，最大值为4
    if (sem1 == NULL) {
        printf("Falied to create Semaphore1!\n");
        return;
    }

    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    attr.name = "thread1";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Falied to create thread1!\n");
    }
    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Falied to create thread2!\n");
    }
    attr.name = "thread3";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL) {
        printf("Falied to create thread3!\n");
    }
}

APP_FEATURE_INIT(i2c_sht20_demo);
