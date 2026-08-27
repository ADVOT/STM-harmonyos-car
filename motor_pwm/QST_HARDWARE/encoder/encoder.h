#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"

/********************* 电机霍尔编码器接口定义 *********************/
/* 左电机编码器 A/B 相：PA0/PA1 = TIM2_CH1/CH2
   右电机编码器 A/B 相：PA6/PA7 = TIM3_CH1/CH2
   编码器模式 TI12：A/B 两相上升沿下降沿都计，精度×4
   PSC=0（编码器模式分的是外部脉冲时钟，0 = 每个脉冲都计）
   ARR=65535（计满回卷）                                        */
#define ENCODER_TIM_PERIOD 65535   //自动重装载值

void Encoder_Init_TIM2(void);   //左编码器初始化（TIM2）
void Encoder_Init_TIM3(void);   //右编码器初始化（TIM3）
int  Read_Encoder(u8 TIMX);     //读 TIMX(2/3) 本周期脉冲数并清零；带符号，正转正/反转负

#endif
