/*
 * 任务11 主线：I2C 驱动 SSD1306 OLED —— 标题 + 日期 + 走时时钟
 * 参考官方流程自写；栈给到 4K（官方 1024 配 sprintf 偏小）
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"

#define OLED_TASK_STACK_SIZE (1024 * 4)
#define OLED_TASK_PRIORITY 25

static void OledClockTask(void)
{
    uint8_t displayBuff[20] = {0};
    uint8_t hour = 16, min = 0, sec = 0;

    if (SSD1306_Init() != 0) {
        printf("[Task11] SSD1306 init failed!\r\n");
        return;
    }
    SSD1306_CLS();                                                      // 清屏
    SSD1306_ShowStr(0, 0, (uint8_t *)" QST CAR ", 16);                  // 标题：页0-1
    SSD1306_ShowStr(0, 3, (uint8_t *)"2026:08:28", 16);                 // 日期：页6-7
    printf("[Task11] OLED clock demo start.\r\n");

    while (1) {
        sec++;
        if (sec > 59) {
            sec = 0;
            min++;
        }
        if (min > 59) {
            min = 0;
            hour++;
        }
        if (hour > 23) {
            hour = 0;
        }
        memset(displayBuff, 0, sizeof(displayBuff));
        sprintf((char *)displayBuff, "%02d:%02d:%02d", hour, min, sec);
        SSD1306_ShowStr(0, 2, displayBuff, 16);                         // 时钟：页4-5
        sleep(1);
    }
}

static void I2cSsd1306Demo(void)
{
    osThreadAttr_t attr = {0};
    attr.name = "oled_task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = OLED_TASK_STACK_SIZE;
    attr.priority = OLED_TASK_PRIORITY;
    if (osThreadNew((osThreadFunc_t)OledClockTask, NULL, &attr) == NULL) {
        printf("[Task11] Failed to create oled_task!\r\n");
    }
}

APP_FEATURE_INIT(I2cSsd1306Demo);
