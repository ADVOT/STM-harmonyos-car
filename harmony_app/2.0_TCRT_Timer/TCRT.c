/*
 * 任务6 红外对管收发（TCRT5000）—— 软件定时器周期采样
 * 硬件：左=GPIO13 / 右=GPIO14，普通 GPIO 输入；低电平（传感器灯灭）= 识别到黑色
 *
 * 程序结构（谁调用谁）：
 *   开机 -> 系统自动调 TCRTEntry（入口）
 *        -> 建线程 TCRTTask：建软件定时器并启动
 *        -> 之后每 50ms 由系统自动调 TimerCallback（读引脚打印）
 */
#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"      /* GpioInit / GpioSetDir / GpioGetInputVal */
#include "wifiiot_gpio_ex.h"   /* IoSetFunc：引脚功能复用选择 */
uint32_t scanTicks = 50U;
osTimerId_t timerId;
/* ===== 0. 常量：两个传感器接的引脚 ===== */
#define PIN_LEFT   WIFI_IOT_IO_NAME_GPIO_13   /* 左红外接收端 */
#define PIN_RIGHT  WIFI_IOT_IO_NAME_GPIO_14   /* 右红外接收端 */

/* ===== 1. 定时器回调函数：每 50ms 被【系统自动】调用一次 =====
 * 不需要任何人调用它——软件定时器到点后，由内核的定时器任务来调。
 * 工作：读左右两个引脚的电平，打印黑白状态。
 */
static void TimerCallback(void *arg)
{
(void)arg;   /* 本例不用传参，(void) 消除"参数未使用"编译警告 */

    WifiIotGpioValue val;   /* 电平枚举：WIFI_IOT_GPIO_VALUE0=低电平 / VALUE1=高电平 */

    /* 读左传感器：GpioGetInputVal(引脚, &变量) —— 把电平写进 val */
    GpioGetInputVal(PIN_LEFT, &val);
    if(val==WIFI_IOT_GPIO_VALUE0){
        printf("left black\r\n");    /* 低电平 = 红外反射弱 = 黑色（此时传感器灯灭） */
    } else{
        printf("left white\r\n");
    }

    /* 读右传感器，同理 */
    GpioGetInputVal(PIN_RIGHT, &val);
    if(val==WIFI_IOT_GPIO_VALUE0){
        printf("right black\r\n");
    } else{
        printf("right white\r\n");
    }
}
static void HelloQstCallback(void *arg){
    (void)arg;
    printf("hello QST\r\n");
    if(scanTicks>2){
        scanTicks--;
        osTimerStart(timerId, scanTicks);
    }
}
/* ===== 2. 线程：把软件定时器建起来并启动 =====
 * 建完定时器这个线程就没活了——之后回调由系统按周期自动触发。
 */
static void TCRTTask(void *arg)
{
(void)arg;
    printf("start test tcrt5000\r\n");

    /* osTimerNew(回调函数, 类型, 传给回调的参数, 属性) -> 返回定时器 ID
     *   osTimerPeriodic = 周期型（到点自动重新开始计时）
     *   osTimerOnce     = 单次型（只触发一次）
     * 考点：osTimerNew 不能在中断服务函数里调用
     */
    timerId = osTimerNew(TimerCallback, osTimerPeriodic, NULL, NULL);
    if(timerId==NULL){
        printf("create timer failed!\r\n");
        return;
    }
    osTimerId_t timerId2 = osTimerNew(HelloQstCallback, osTimerPeriodic, NULL, NULL);
    if(timerId2==NULL){ printf("create timer2 failed!\r\n"); return; }
    if(osTimerStart(timerId2, 6*scanTicks)!=osOK){   /* 300 ticks = 3 秒 */
        printf("start timer2 failed!\r\n");
    }
    /* osTimerStart(定时器ID, ticks)：启动定时器
     * 嫌串口刷屏太快的话，把 50 改成 100 就是 1 秒打印一次
     */
    if(osTimerStart(timerId, scanTicks)!=osOK){
        printf("start timer failed!\r\n");
    } else{
        printf("timer started!\r\n");
    }
}

/* ===== 3. 入口函数：开机后由系统自动调用（靠文件末尾的 APP_FEATURE_INIT 注册） =====
 * 做两件事：GPIO 初始化 + 建线程
 */
static void TCRTEntry(void)
{
    /* --- GPIO 初始化三步走 --- */
    GpioInit();   /* ① GPIO 外设总初始化，整个程序只调这一次 */

    /* ② 引脚功能复用：Hi3861 的引脚大多身兼数职（I2C/PWM/UART...），
     *    IoSetFunc 告诉芯片"这个引脚这次当普通 GPIO 用" */
    IoSetFunc(PIN_LEFT,  WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(PIN_RIGHT, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);

    /* ③ 方向设置：传感器是往芯片里"送"信号的，所以设为输入 */
    GpioSetDir(PIN_LEFT,  WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(PIN_RIGHT, WIFI_IOT_GPIO_DIR_IN);

    /* --- 建线程：和任务5 同一套写法（栈 10240、优先级 25） --- */
    osThreadAttr_t attr ={0};
    attr.name = "TCRTTask";
    attr.stack_size = 10240;
    attr.priority = 25;
    if(osThreadNew((osThreadFunc_t)TCRTTask, NULL, &attr)==NULL){
        printf("Failed to create TCRTTask!\n");
        return;
    }
    printf("[TCRT] ENTRY.\n");
}
APP_FEATURE_INIT(TCRTEntry);   /* 注册入口：开机后系统自动调 TCRTEntry */
