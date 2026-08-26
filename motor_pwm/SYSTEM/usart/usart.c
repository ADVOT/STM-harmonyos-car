#include "sys.h"
#include "usart.h"
#include "string.h"	  

//****************串口命令表（灯效切换）*****************//
//命令字必须唯一互为前缀无关，主循环按 LED_CMD 分发灯效
const char *LED_CMD_TABLE[]={"HELLO","ALLON","BLINK","RAIN","FLOW","OFF"};

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


#if EN_USART1_RX   //如果使能了接收

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_RX_STA=0;       //接收状态标记
u8 LED_CMD=0;            //最新命令编号（0=无，1~6对应LED_CMD_TABLE）
u8 count=0;
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
	u8 k;
	u8 matched;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)   //判断中断类型
		{
			Res =USART_ReceiveData(USART1);	//读取接收到的数据
			if(USART_RX_STA==0)             //上一条命令没被主循环取走前，不收新数据
			{
				USART_RX_BUF[count++]=Res;
				matched=0;
				for(k=0;k<6;k++)            //判断当前缓冲是否仍是某条命令的前缀
				{
					if(strlen(LED_CMD_TABLE[k])>=count &&
					   strncmp((char*)USART_RX_BUF,LED_CMD_TABLE[k],count)==0)
					{
						matched=1;
						if(strlen(LED_CMD_TABLE[k])==count)   //完整命中一条命令
						{
							LED_CMD=k+1;                     //命令编号1~6
							USART_RX_STA=1;                  //表示接收到完整数据
						}
						break;
					}
				}
				if(matched==0)              //前缀失配：丢弃重来
				{
					count=0;
					for(k=0;k<6;k++)        //当前字节可能是新命令的首字符
					{
						if(LED_CMD_TABLE[k][0]==Res)
						{
							USART_RX_BUF[count++]=Res;
							break;
						}
					}
				}
			}
    }
    USART_ClearFlag(USART1, USART_FLAG_RXNE); //清除接收表示位
}

#endif	

