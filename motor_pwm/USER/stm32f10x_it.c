#include "stm32f10x_it.h"
#include "protocol.h"

#define CONTROL_PERIOD_MS 100U

volatile u32 millis = 0;
volatile u8 control_due = 0;
static u8 control_elapsed_ms = 0;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    millis++;
    Protocol_Tick1ms();

    control_elapsed_ms++;
    if (control_elapsed_ms >= CONTROL_PERIOD_MS)
    {
        control_elapsed_ms = 0;
        control_due = 1;
    }
}
