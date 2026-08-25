#include "stm32f10x.h"
#include "sys.h"
#include "colorful_led.h"

int main(void)
  { 
		Stm32_Clock_Init(9);						//外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====中断优先级分组		
		uart_init(115200);	            //=====串口初始化为
		JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

		colorful_led_Init();            //=====炫彩灯初始化
		//SysTick_Config(72000000/1000);		//滴答定时器，每1ms触发一次中断
    
		//L_runingled();                  //前灯特效
		R_runingled();                //后灯特效
		/**主要程序**/
	while(1)
	{
		
	}
}
	

