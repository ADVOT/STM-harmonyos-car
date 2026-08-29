#ifndef __NFC_H
#define __NFC_H

#include "sys.h"

extern u8 NFC_WakeUp_Ok;   //PN532 唤醒成功标志
extern u8 NFC_find_Card;   //命中已登记卡号标志

void NFC_Init(void);       //UART2 初始化 + 唤醒 PN532（阻塞等应答，最多 2s）
void NFC_Handler(void);    //主循环调用：周期寻卡 + 帧解析 + 命中处理

#endif
