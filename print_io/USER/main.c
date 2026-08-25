#include "stm32f10x.h"
#include "sys.h"

int main(void)
  {
		u8 led_mode=0;      //=====当前灯效模式（0=无，编号见colorful_led.h）

		Stm32_Clock_Init(9);						//外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====中断优先级分组
		uart_init(115200);	            //=====串口初始化为115200
		JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

		colorful_led_Init();            //=====炫彩灯初始化

		printf("QST青软\r\n");
		/**主要程序：串口命令切换灯效（HELLO/ALLON/BLINK/RAIN/FLOW/OFF）**/
	while(1)
	{
		if(USART_RX_STA==1)  //收到完整命令：更新灯效模式
		{
			led_mode=LED_CMD;
			USART_RX_STA=0;  //清标志，准备接收下一条命令
		}

		switch(led_mode)     //按当前模式跑一帧灯效（每个函数内部自带动画延时）
		{
			case LED_MODE_HELLO: fx_hello_frame(); break;  //前灯跑马灯
			case LED_MODE_ALLON: led_all_on(WS_WHITE); delay_ms(200); break;  //12颗全亮
			case LED_MODE_BLINK: fx_blink_frame(); break;  //12颗同步闪烁
			case LED_MODE_RAIN:  fx_rain_frame();  break;  //彩虹换色
			case LED_MODE_FLOW:  fx_flow_frame();  break;  //流水扫过12颗
			case LED_MODE_OFF:   led_all_off();    delay_ms(200); break;  //全灭
			default: delay_ms(100); break;                 //无命令时空转
		}
	}
}
	

