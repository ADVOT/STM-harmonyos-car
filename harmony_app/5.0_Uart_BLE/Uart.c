/*
 * 任务9：UART1 收发（经 JDY-16 蓝牙模块）+ 消息队列三线程联动
 *
 * 硬件：Hi3861 UART1 —— GPIO0=TX / GPIO1=RX，接车上 JDY-16 蓝牙模块。
 *       模块出厂固定 9600/8/1/N，两端必须一致（波特率是约定，见学习记录 8-25 第6条）。
 *       手机装"蓝牙调试器"APP 搜 JDY-16 连接，发的字符串就进 UART1。
 *
 * 结构（1 队列+3 线程，全 prio 25 同级轮转）：
 *   thread2   —— 每秒轮询 UartRead，收到数据拷进消息 → osMessageQueuePut 入队
 *   UART_Task —— 启动时发一次 "Hello,I'm ready!"；之后阻塞等队列消息、收到打印，
 *                只有收到手机发来 "ready" 开头的消息时才再回一次（问一句答一句）
 *   thread3   —— 作业：每 3s 往队列连发 5 条消息（#100~#104），验证 FIFO 依次读出
 *
 * 与参考答案的四处区别（都是修隐患，机制不变）：
 *   ① 队列【传值不传指针】：消息结构体里直接带 char Buf[100]，
 *      osMessageQueuePut 会把整条消息【拷进】队列节点。参考答案传的是
 *      指向 thread2 栈缓冲的指针，靠 sleep(1) 赌读写不错开——栈缓冲一被
 *      下次接收覆盖，队列里那条消息内容就变了。
 *   ② 先补 '\0' 再打印（参考答案先打印后补，打印的是没结尾的缓冲）。
 *   ③ UartRead 读到 0 字节（没人发）不入队，不发空消息。
 *   ④ Idx 当递增序号用，烧录后从打印能直接看出 FIFO 顺序对不对。
 */
#include <stdio.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"

#define MSGQUEUE_OBJECTS 16        /* 队列最大消息数 */
#define UART_BUFF_SIZE   256       /* 单次接收缓冲（蓝牙字符串很短，256 足够） */
#define UART_TASK_STACK_SIZE 4096
#define UART_TASK_PRIO   25

/* 队列里流动的消息：Idx=序号，Buf=内容（值语义，入队即拷贝） */
typedef struct{
    uint8_t Idx;
    char Buf[100];
} MSGQUEUE_OBJ_t;

static osMessageQueueId_t mid_MsgQueue;
static const char *data="Hello,I'm ready!\r\n";  /* 往手机发的字符串 */

/* 线程1：UART1 初始化+阻塞等队列消息；启动发一次 ready，之后收到 "ready" 开头的消息才回复 */
static void UART_Task(void *arg)
{
    (void)arg;
    MSGQUEUE_OBJ_t rx;

    /* GPIO0/1 复用为 UART1_TXD/RXD */
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0,WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1,WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);

    /* 9600/8/1/N —— 跟着 JDY-16 出厂配置走 */
    WifiIotUartAttribute uart_attr={
        .baudRate=9600,
        .dataBits=8,
        .stopBits=1,
        .parity=0,
    };
    if (UartInit(WIFI_IOT_UART_IDX_1,&uart_attr,NULL) != WIFI_IOT_SUCCESS){
        printf("Failed to init uart1!\n");
        return;
    }
    printf("UART Test Start\n");
    /* 启动先发一次，之后只在收到 "ready?" 时回复 */
    UartWrite(WIFI_IOT_UART_IDX_1,(unsigned char *)data,strlen(data));

    while (1){
        /* 阻塞等消息：没消息时线程挂起让出 CPU，来消息被唤醒 */
        if (osMessageQueueGet(mid_MsgQueue,&rx,NULL,osWaitForever) == osOK){
            printf("queue msg #%u: %s\n",rx.Idx,rx.Buf);
            /* 前缀匹配 "ready"：兼容全角？（UTF-8 三字节）、半角?、APP 尾部带 \r\n */
            if (strncmp(rx.Buf,"ready",5) == 0){
                UartWrite(WIFI_IOT_UART_IDX_1,(unsigned char *)data,strlen(data));
            }
        }
    }
}

/* 线程2：每秒轮询 UART1，有数据就拷进消息入队 */
static void thread2(void *arg)
{
    (void)arg;
    uint8_t uart_buff[UART_BUFF_SIZE];
    MSGQUEUE_OBJ_t msg;
    static uint8_t seq=0;  /* 消息序号，验证 FIFO 用 */

    osDelay(100); /* 等 UART_Task 先把串口初始化完 */
    while (1){
        int rt=UartRead(WIFI_IOT_UART_IDX_1,uart_buff,sizeof(uart_buff)-1);
        if (rt > 0){
            uart_buff[rt]='\0';  /* 先补结束符，后面才能当字符串用 */
            printf("Uart1 read %d bytes: %s\n",rt,uart_buff);
            msg.Idx=seq++;
            strncpy(msg.Buf,(char *)uart_buff,sizeof(msg.Buf)-1);
            msg.Buf[sizeof(msg.Buf)-1]='\0';
            /* 传的是 &msg，但 Put 会把整个结构体拷进队列节点，指针本身不驻留 */
            if (osMessageQueuePut(mid_MsgQueue,&msg,0U,0U) != osOK){
                printf("Message Queue full,msg dropped\n");
            }
        }
        osDelay(100); /* 1s 轮询一拍 */
    }
}

/* 线程3：空转，每 3s 报个到（演示同级时间片轮转互不耽误） */
static void thread3(void *arg)
{
    (void)arg;
    static const char *demoMsgs[]={"first","second","third","fourth","fifth"};
    MSGQUEUE_OBJ_t msg;
    osDelay(100);
    while (1){
        for (int i=0;i<5;i++){
            msg.Idx=100+i;
            strncpy(msg.Buf,demoMsgs[i],sizeof(msg.Buf)-1);
            msg.Buf[sizeof(msg.Buf)-1]='\0';
            if (osMessageQueuePut(mid_MsgQueue,&msg,0U,0U) != osOK){
                printf("Message Queue full,msg %d dropped\n",i);
            }
        }
        printf("thread3: 5 messages posted\n");
        osDelay(300);
    }
}

static void UART_ExampleEntry(void)
{
    /* 队列：16 条 × sizeof(MSGQUEUE_OBJ_t)，入队即整条拷贝 */
    mid_MsgQueue=osMessageQueueNew(MSGQUEUE_OBJECTS,sizeof(MSGQUEUE_OBJ_t),NULL);
    if (mid_MsgQueue == NULL){
        printf("Failed to create Message Queue!\n");
        return;
    }

    osThreadAttr_t attr;
    attr.attr_bits=0U;
    attr.cb_mem=NULL;
    attr.cb_size=0U;
    attr.stack_mem=NULL;
    attr.priority=UART_TASK_PRIO;

    attr.name="UART_Task";
    attr.stack_size=UART_TASK_STACK_SIZE;
    if (osThreadNew((osThreadFunc_t)UART_Task,NULL,&attr) == NULL){
        printf("Failed to create UART_Task!\n");
    }
    attr.name="thread2";
    attr.stack_size=UART_TASK_STACK_SIZE;
    if (osThreadNew((osThreadFunc_t)thread2,NULL,&attr) == NULL){
        printf("Failed to create thread2!\n");
    }
    attr.name="thread3";
    attr.stack_size=1024;
    if (osThreadNew((osThreadFunc_t)thread3,NULL,&attr) == NULL){
        printf("Failed to create thread3!\n");
    }
}

APP_FEATURE_INIT(UART_ExampleEntry);
/*
 * 预期现象（UartAssist@115200）：
 *   UART Test Start
 *   （约 1s 后）
 *   thread3: 5 messages posted
 *   queue msg #100: first
 *   queue msg #101: second
 *   queue msg #102: third
 *   queue msg #103: fourth
 *   queue msg #104: fifth     <- 一口气连发，严格按序取出
 *   （期间手机发任意内容都会入队打印：queue msg #0: abc，各源序号各自递增）
 *   （手机发 "ready" 开头（ready?/ready？都行）时板上回一次 "Hello,I'm ready!"；发别的只打印不回复）
 *   （3s 后新一轮 #100~#104）
 */