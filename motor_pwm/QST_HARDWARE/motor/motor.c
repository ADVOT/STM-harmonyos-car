#include "motor.h"

/**************************************************************************
函数功能：初始化电机方向脚（L9110S 的 AIN/BIN）
入口参数：无
返回  值：无
**************************************************************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能PB端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_13; //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	AIN=0;
	BIN=0;
}

/**************************************************************************
函数功能：初始化 TIM4 两路 PWM（CH1=PB6 右轮 / CH2=PB7 左轮）
入口参数：arr=自动重装载值 psc=预分频系数（7199,9 → 72M/10/7200=1000Hz）
返回  值：无
**************************************************************************/
void PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	Motor_Init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //使能TIM4时钟（APB1）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能GPIOB时钟（APB2）

	//PB6/PB7 复用推挽输出，输出 TIM4_CH1/CH2 的 PWM 波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;                 //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;              //预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;       //PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                      //初始占空比0（电机不动）
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);                //CH1 → PB6
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);                //CH2 → PB7

	TIM_CtrlPWMOutputs(TIM4, ENABLE);                       //主输出使能
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);       //CH1预装载使能
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);       //CH2预装载使能
	TIM_ARRPreloadConfig(TIM4, ENABLE);                     //ARR预装载使能
	TIM_Cmd(TIM4, ENABLE);                                  //使能TIM4
}

/**************************************************************************
函数功能：取绝对值
**************************************************************************/
u32 myabs(long int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else    temp=a;
	return temp;
}

/**************************************************************************
函数功能：设置左右电机 PWM
入口参数：moto1=左轮 moto2=右轮；≥0 正转占空比=值，<0 反转且比较值反相
          （占空比 = 比较值/(ARR+1)，值不能超过 7199）
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1, int moto2)
{
	//右电机（A组）
	if(moto2>=0){ AIN=0; PWMA=myabs(moto2);       }
	else        { AIN=1; PWMA=7199-myabs(moto2);  }
	//左电机（B组）
	if(moto1>=0){ BIN=0; PWMB=myabs(moto1);       }
	else        { BIN=1; PWMB=7199-myabs(moto1);  }
}
