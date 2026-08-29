#include "stm32f10x.h"
#include "sys.h"
#include "nfc.h"

//任务20：UART2 驱动 PN532 读 NFC 卡，命中登记卡号执行用户函数
//调试打印走 USART1(PA9/115200)，NFC 数据走 USART2(PA2/PA3/115200)
int main(void)
{
	Stm32_Clock_Init(9);            //外部时钟8Mhz 9倍频 = 72MHz
	MY_NVIC_PriorityGroupConfig(2); //中断优先级分组
	uart_init(115200);              //USART1 调试串口
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG
	JTAG_Set(SWD_ENABLE);           //打开SWD调试

	colorful_led_Init();            //炫彩灯初始化

	printf("QST青软 Task20 NFC\r\n");

	NFC_Init();                     //UART2 初始化 + 唤醒 PN532

	/**主要程序：循环寻卡，命中执行用户函数**/
	while(1)
	{
		NFC_Handler();
	}
}
