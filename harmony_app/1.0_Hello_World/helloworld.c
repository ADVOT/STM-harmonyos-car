#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

/* 线程1 入口函数：每秒打印一次 */
static void HelloWorldT(void *arg){
    (void)arg;
    while(1){
        printf("Hello World!\n");
        usleep(1000000);
    }
}

/* 线程2 入口函数：先睡 1s 错开节奏，之后每 3s 打印一次 */
static void HelloQstT(void *arg){
    (void)arg;
    usleep(500000);
    while(1){
        printf("Hello QST!\n");
        usleep(3000000);
    }
}

static void HelloWorldE(void){
    /* osThreadAttr_t：线程属性包（名字/栈大小/优先级），={0} 先清零再逐项填 */
    osThreadAttr_t attr={0};
    attr.stack_size=1024;/* 任务栈：本线程的局部变量+函数调用所用的内存 */
    attr.priority=osPriorityAboveNormal;

    /* 创建线程1：osThreadNew(入口函数, 传给入口函数的参数, 属性包) */
    attr.name="HelloWorldT";             /* 调试标签名，可任意起 */
    if((osThreadNew((osThreadFunc_t)HelloWorldT, NULL, &attr)==NULL)){
        printf("[HelloWorld] create task fail!\n");
        return;
    }

    /* 创建线程2：复用同一个 attr，只需换掉 name 和入口函数，再调一次 osThreadNew */
    attr.name="HelloQstT";
    if((osThreadNew((osThreadFunc_t)HelloQstT, NULL, &attr)==NULL)){
        printf("[HelloQst] create task fail!\n");
        return;
    }
    printf("[HelloWorld] ENTRY.\n");
}
SYS_RUN(HelloWorldE);
