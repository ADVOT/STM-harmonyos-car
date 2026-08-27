#include "encoder.h"

/**************************************************************************
函数功能：把 TIM2 初始化为编码器接口模式（左电机，PA0/PA1）
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);     //使能TIM2时钟（APB1）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    //使能GPIOA时钟（APB2）

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;    //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;               //预分频0：每个脉冲都计
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
		TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);       //编码器模式3：两相四沿计数（×4）
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;                   //输入滤波，滤毛刺
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);                    //清除更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM2, 0);                                 //计数器清零
	TIM_Cmd(TIM2, ENABLE);                                   //使能TIM2
}

/**************************************************************************
函数功能：把 TIM3 初始化为编码器接口模式（右电机，PA6/PA7）
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);     //使能TIM3时钟（APB1）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    //使能GPIOA时钟（APB2）

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;    //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;               //预分频0：每个脉冲都计
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
		TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);       //编码器模式3：两相四沿计数（×4）
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;                   //输入滤波，滤毛刺
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);                    //清除更新标志位
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM3, 0);                                 //计数器清零
	TIM_Cmd(TIM3, ENABLE);                                   //使能TIM3
}

/**************************************************************************
函数功能：读取单位时间内的编码器计数（读后立即清零，供下一周期用）
入口参数：TIMX = 2（左/TIM2）或 3（右/TIM3）
返回  值：带符号脉冲数，正转为正、反转为负
说    明：(short) 强转的讲究——CNT 是 16 位无符号，反转时从 0 往下减
          回卷成大数，强转 short 后直接变成带符号"位移量"，方向自动正确。
          前提：两次读取间隔内脉冲数 < 32768（100ms 周期远够）
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;

	switch(TIMX)
	{
		case 2:
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
			Encoder_TIM = (short)TIM2->CNT;                  //取本周期脉冲数
			TIM_SetCounter(TIM2, 0);                         //清零，下一周期重新计
			break;
		case 3:
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			Encoder_TIM = (short)TIM3->CNT;
			TIM_SetCounter(TIM3, 0);
			break;
		default:
			Encoder_TIM = 0;
	}
	return Encoder_TIM;
}
