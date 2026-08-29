#include "nfc.h"
#include "usart.h"
#include "delay.h"
#include "colorful_led.h"
#include "string.h"
#include "stdio.h"

//**************** PN532 指令（任务20 官方文档字节流） *****************//
//唤醒：16 字节唤醒前导 + SAMConfiguration(D4 14 01)
static const u8 NFC_CMD_WAKEUP[]={
	0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00};
//寻卡：InListPassiveTarget(D4 4A)，寻 1 张 Type A
static const u8 NFC_CMD_FIND[]={
	0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};

//已登记卡号（8-28 手机 App 读：UID=63 1F 49 06，卡号 105455459 为其倒序十进制）
static const u8 CARD_UID[4]={0x63,0x1F,0x49,0x06};

#define NFC_FRAME_MAX  32   //payload 缓冲上限（寻卡应答 payload=12 字节）

static u8 nfc_payload[NFC_FRAME_MAX]; //一帧的 payload（LEN 段内容）
static volatile u8 nfc_frame_ready=0; //收到完整普通帧
static volatile u8 nfc_ack_ready=0;   //收到 ACK 帧(00 00 FF 00 FF 00)

u8 NFC_WakeUp_Ok=0;
u8 NFC_find_Card=0;
static u8 card_present=0;      //卡在场上标志（命中改为边沿触发，避免 200ms 刷屏）
static u8 card_lost_ticks=0;   //无卡帧计数，用于卡离场判定
static u8 last_uid[4]={0};     //最近一次读到的卡号

//---------------- UART2（PA2=TX / PA3=RX，115200 8N1） ----------------
//注意：USART2 挂在 APB1，GPIOA 挂在 APB2，两个时钟都要开
static void NFC_UART2_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //USART2 时钟(APB1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //GPIOA 时钟(APB2)

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               //USART2_TX = PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;               //USART2_RX = PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

static void NFC_SendBytes(const u8 *buf, u16 len)
{
	u16 i;
	for(i=0;i<len;i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
		USART_SendData(USART2, buf[i]);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET); //等最后一字节出栈
}

//---------------- 接收状态机（USART2 中断内） ----------------
//官方参考答案按固定帧长收（唤醒15/寻卡25），粘包错位就乱；
//这里按 PN532 帧结构解析：ACK 帧 = 00 00 FF 00 FF 00；
//普通帧 = 00 00 FF LEN LCS payload[LEN] DCS 00，带 LCS/DCS 校验。
void USART2_IRQHandler(void)
{
	static u8 state=0, len=0, idx=0, sum=0;
	u8 res;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) == RESET)
	{
		return;
	}
	res = USART_ReceiveData(USART2);

	switch(state)
	{
	case 0:                       //等帧头第 1 个 00
		if(res==0x00) state=1;
		break;
	case 1:                       //等帧头第 2 个 00
		if(res==0x00) state=2;
		else state=0;
		break;
	case 2:                       //等 FF
		if(res==0xFF) state=3;
		else if(res==0x00) state=1;  //多出来的 00 可能是新帧头
		else state=0;
		break;
	case 3:                       //LEN：0 = ACK 帧路径
		if(res==0x00) state=10;
		else { len=res; idx=0; sum=0; state=4; }
		break;
	case 4:                       //LCS 校验：LEN+LCS=0x100
		if((u8)(len+res)==0x00) state=5;
		else state=0;
		break;
	case 5:                       //收 payload
		if(nfc_frame_ready==0 && idx<NFC_FRAME_MAX)  //主循环没取走就丢弃本帧
		{
			nfc_payload[idx]=res;
			sum=(u8)(sum+res);
			idx++;
		}
		else
		{
			idx++;
		}
		if(idx>=len) state=6;
		break;
	case 6:                       //DCS 校验：sum(payload)+DCS=0x100
		if((u8)(sum+res)==0x00) state=7;
		else state=0;
		break;
	case 7:                       //报尾 00
		if(res==0x00 && nfc_frame_ready==0)
		{
			nfc_frame_ready=1;
		}
		state=0;
		break;
	case 10:                      //ACK 帧第 5 字节应为 FF
		if(res==0xFF) state=11;
		else state=0;
		break;
	case 11:                      //ACK 帧第 6 字节应为 00
		if(res==0x00) nfc_ack_ready=1;
		state=0;
		break;
	default:
		state=0;
		break;
	}
}

//---------------- 帧解析（主循环上下文） ----------------
static void NFC_ParseFrame(void)
{
	u8 i;
	u8 plen=0;

	//payload 长度不由返回值带出来，这里按指令码还原（本任务只用两种应答）
	if(nfc_payload[0]==0xD5 && nfc_payload[1]==0x15)         //SAMConfiguration 应答
	{
		plen=2;
		NFC_WakeUp_Ok=1;
	}
	else if(nfc_payload[0]==0xD5 && nfc_payload[1]==0x4B)    //寻卡应答
	{
		plen=12;
		if(nfc_payload[2]>=1 && nfc_payload[7]==4)           //有卡且 UID 4 字节
		{
			card_lost_ticks=0;
			for(i=0;i<4;i++) last_uid[i]=nfc_payload[8+i];
			printf("[NFC] card UID: %02X %02X %02X %02X\r\n",
				last_uid[0],last_uid[1],last_uid[2],last_uid[3]);
			if(card_present==0)                              //新到卡边沿才触发
			{
				card_present=1;
				if(memcmp(last_uid, CARD_UID, 4)==0)
				{
					NFC_find_Card=1;
				}
				else
				{
					printf("[NFC] unknown card\r\n");
				}
			}
		}
		else                                                 //无卡应答
		{
			card_present=0;
		}
	}
	for(i=0;i<plen;i++)                                      //调试用：HEX 回放
	{
		printf("%02X ", nfc_payload[i]);
	}
	if(plen>0) printf("\r\n");
}

//---------------- 对外接口 ----------------
void NFC_Init(void)
{
	u16 i;

	NFC_UART2_Init(115200);
	delay_ms(50);
	printf("[NFC] send wakeup...\r\n");
	NFC_SendBytes(NFC_CMD_WAKEUP, sizeof(NFC_CMD_WAKEUP));

	for(i=0;i<40;i++)                 //最多等 2s 唤醒应答
	{
		delay_ms(50);
		if(nfc_frame_ready)
		{
			nfc_frame_ready=0;
			NFC_ParseFrame();
		}
		if(NFC_WakeUp_Ok) break;
	}
	if(NFC_WakeUp_Ok) printf("[NFC] wakeup OK\r\n");
	else              printf("[NFC] wakeup TIMEOUT, check wiring\r\n");
}

static void FoundCard_Handler(void)
{
	u8 i;
	NFC_find_Card=0;
	printf("[NFC] card HIT! run user handler\r\n");
	for(i=0;i<24;i++)                 //流水灯扫一轮作为命中反馈
	{
		fx_flow_frame();
	}
}

void NFC_Handler(void)
{
	if(NFC_WakeUp_Ok==0)              //未唤醒不寻卡
	{
		delay_ms(200);
		return;
	}
	if(NFC_find_Card)                 //命中先跑用户函数
	{
		FoundCard_Handler();
		return;
	}

	nfc_frame_ready=0;
	NFC_SendBytes(NFC_CMD_FIND, sizeof(NFC_CMD_FIND));
	delay_ms(200);
	if(nfc_frame_ready)
	{
		nfc_frame_ready=0;
		NFC_ParseFrame();
	}

	if(card_present)                  //卡离场兜底：连续 5 拍(约1s)无卡帧则清标志
	{
		card_lost_ticks++;
		if(card_lost_ticks>5)
		{
			card_present=0;
			card_lost_ticks=0;
			printf("[NFC] card removed\r\n");
		}
	}
}
