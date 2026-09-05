/*
 * 任务11 作业：OLED 显示「鸿蒙先锋号」
 * 驱动 ShowStr 只有 ASCII 字库，汉字走 DrawBMP 位图；
 * 字模 qst_glyphs.h 由 gen_glyph.py 生成（SimSun 16x16，页寻址列字节格式）
 */
#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"
#include "qst_glyphs.h"

#define OLED_TASK_STACK_SIZE (1024 * 4)
#define OLED_TASK_PRIORITY 25

/* 5 字 x 16 列 = 80 列，水平居中 x0=(128-80)/2=24；垂直占页 2~3（像素行 16-31） */
static void OledHomeworkTask(void)
{
    if (SSD1306_Init() != 0) {
        printf("[Task11-hw] SSD1306 init failed!\r\n");
        return;
    }
    SSD1306_CLS();
    SSD1306_ShowStr(0, 0, (uint8_t *)"  HARMONY-OS  ", 16);   // 顶部 ASCII 标题：页0-1
    SSD1306_DrawBMP(24, 2, 24 + 80, 4, BMP_HM_XFH);          // 鸿蒙先锋号：页2-3
    printf("[Task11-hw] draw BMP done.\r\n");

    while (1) {
        sleep(1);
    }
}

static void I2cSsd1306Homework(void)
{
    osThreadAttr_t attr = {0};
    attr.name = "oled_hw_task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = OLED_TASK_STACK_SIZE;
    attr.priority = OLED_TASK_PRIORITY;
    if (osThreadNew((osThreadFunc_t)OledHomeworkTask, NULL, &attr) == NULL) {
        printf("[Task11-hw] Failed to create oled_hw_task!\r\n");
    }
}

APP_FEATURE_INIT(I2cSsd1306Homework);
