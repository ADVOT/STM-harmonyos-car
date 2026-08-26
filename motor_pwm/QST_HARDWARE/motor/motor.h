#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/********************* L9110S 电机驱动引脚定义 *********************/
/* 方向脚（普通推挽输出）：
   AIN = PB13 = 右电机方向
   BIN = PB14 = 左电机方向
   PWM 脚（TIM4 复用输出）：
   PWMA = TIM4_CH1 = PB6 = 右电机速度
   PWMB = TIM4_CH2 = PB7 = 左电机速度        */
#define AIN  PBout(13)
#define BIN  PBout(14)
#define PWMA TIM4->CCR1
#define PWMB TIM4->CCR2

void Motor_Init(void);          //电机方向脚初始化
void PWM_Init(u16 arr, u16 psc);//TIM4 PWM 初始化（arr=7199,psc=9 → 1kHz）
void Set_Pwm(int moto1, int moto2); //moto1=左轮, moto2=右轮；符号=方向，绝对值=占空比(≤7199)

#endif
