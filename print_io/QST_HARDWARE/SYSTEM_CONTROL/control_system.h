#ifndef __SYSTEM_CONTROL_H
#define __SYSTEM_CONTROL_H
#include "stm32f10x.h"
#include "sys.h"

void Set_Pwm(int motor_a,int motor_b);
void Key(void);
void Xianfu_Pwm(int amplitude);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_B_Mode2(int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
int Incremental_PI_C (int Encoder,int Target);
int Position_PID_A (int Encoder,int Target);
int Position_PID_B (int Encoder,int Target);
int Position_PID_C (int Encoder,int Target);
void Sport_Analysis(float VC,int ds);
//void System_Control(void);

#endif
