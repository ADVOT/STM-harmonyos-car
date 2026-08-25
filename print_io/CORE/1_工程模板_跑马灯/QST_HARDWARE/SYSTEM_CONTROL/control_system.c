#include "control_system.h"	

#define K1    1     // 角度转动系数   
extern int a1,a2,b1,b2;
float  K2;					//	速度转动系数
float Pwm_Max=7199;		
float Angle_X,Angle_Y;	//最终倾角X.Y
/* 加速度计的返回值暂存变量 */
float JSDx = 0;		
float JSDy = 0;
float JSDz = 0.0;

extern UARTFrameTypeDef  UART2Frame; 
extern u8 dir_rec;                                                //  小车方向
extern int Target_MotorA,Target_MotorB;    						//电机目标值
extern float  Target_angle;  
u8 t=0; //  计数 用来控制反转速度
/****************************角度获取**************************/
void Angle_Calcu(void)	
{
		
//加速度(角度)
	
	if(mpu_dmp_get_data(&JSDx,&JSDy,&JSDz)==0)
	{
	  
	//z角度
			put_shuzu(USART3,JSDz);
				
					
////y角度
//			if(JSDy>=0)
//			{
//				    OLED_ShowString(48,0,"+",12);
//						OLED_FLOAT(50,0,JSDy, 5,12);
//			}
//			else if(JSDy<0)
//				{   dir=-JSDy;
//			      OLED_ShowString(48,0,"-",12);
//						OLED_FLOAT(50,0,dir, 5,12);
//			  }				
////x角度					
//			if(JSDx>=0)
//			{
//				    OLED_ShowString(48,1,"+",12);
//						OLED_FLOAT(50,1,JSDx, 5,12);
//			}
//			else if(JSDx<0)
//				{   i=-JSDx;
//			      OLED_ShowString(48,1,"-",12);
//						OLED_FLOAT(50,1,i, 5,12);
//			  }
   }
	
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	
	    if(moto1>0)			               AIN=0;			        //接线得接对      正向调速
	
			else if(moto1==0) 	          {moto1=7199;AIN=1;}	                 //刹车
      else if(moto1<0)	            {AIN=1;moto1=0;}                     //倒车
			PWMA=myabs(moto1);
			
		  if(moto2>0)	                  BIN=0;		
			else if(moto2==0)             {moto2=7199;BIN=1;}			
			else if(moto2<0)	            {BIN=1;moto2=0;}
			PWMB=myabs(moto2);	

}


/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

int Incremental_PI_A (int Encoders_A,int Target_A)
{ 	
	 static int Bias_A,Pwm_A,Last_bias_A;                   
//	 Target_A=Target_A;
	 Bias_A=Target_A-(Encoders_A*50);                //计算偏差
	 Pwm_A+=Velocity_KP*Bias_A+Velocity_KD*(Bias_A-Last_bias_A);   //增量式PI控制器
	if(Pwm_A>7199)Pwm_A=7199;
	if(Pwm_A<0)Pwm_A=0;
	 Last_bias_A=Bias_A;	                   //保存上一次偏差 
	 return Pwm_A;                         //增量输出
}
int Incremental_PI_B (int Encoders_B,int Target_B)
{ 	
	 static int Bias_B,Pwm_B,Last_bias_B;
//	 Target_B=Target_B/71;
	 Bias_B=Target_B-(Encoders_B*50);                //计算偏差
	 Pwm_B+=Velocity_KP*Bias_B+Velocity_KD*(Bias_B-Last_bias_B);   //增量式PI控制器
	if(Pwm_B>7199)Pwm_B=7199;
	if(Pwm_B<0)Pwm_B=0;
	 Last_bias_B=Bias_B;	                   //保存上一次偏差 
	 return Pwm_B;                         //增量输出
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_A (int Encoder,int Target)                  //200 全速
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>1000)Integral_bias=1000;
	 if(Integral_bias<-1000)Integral_bias=-1000;
	 Pwm=Position_KP_A*Bias+Position_KI_A/100*Integral_bias+Position_KD_A*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>1000)Integral_bias=1000;
	 if(Integral_bias<-1000)Integral_bias=-1000;
	 Pwm=Position_KP_B*Bias+Position_KI_B/100*Integral_bias+Position_KD_B*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：系统控制函数
入口参数：
返回  值：
**************************************************************************/
void System_Control(void)
{
//put_string(USART3,"erorr ");
	
//put_shuzu(USART3,i);

	if(uart_rec_flag)                //收到一帧数据
	{
		if(CAR_buff[3]==0)                                            //不转角
		{
			if(CAR_buff[0]==0||CAR_buff[0]==1)             //  0原地  1前进  
			{	
				Target_MotorA=CAR_buff[1];
				Target_MotorB=CAR_buff[2];
				Key_mode=1;
				uart_rec_flag=0;
			}
			else if(CAR_buff[0]==2)                       //  2倒转
			{
				Key_mode=3;
			 	uart_rec_flag=0;
			}			
		}
    else                                                              //转角
    {
		   Target_angle=(float)CAR_buff[3];                                 
			 Key_mode=2;
			 uart_rec_flag=0;
		}	
    memset(CAR_buff,0,4);
	}
	   
		
					
				if((1 == 	Key_mode))              //速度闭环******************************************************************
				{
           R_led_CLC();					
					 Encoder_A=Read_Encoder(2);                                          //===读取编码器的值 	转角电机	/2.5转化成360°  转一圈900个脉冲	  435									
			     Encoder_B=Read_Encoder(3);                                          //===读取编码器的值   转速电机  /2.72转化成360  转一圈980个脉冲		570
//*****速度编码对比   64%    124      100%   142      40%  a：93  b：  87				 

//					  put_string(USART3,"EA: ");
//					  put_shuzu(USART3,Encoder_A);
//					  put_string(USART3,"EB: ");
//            put_shuzu(USART3,Encoder_B);	
					
					  Motor_A=Incremental_PI_A(Encoder_A,Target_MotorA);                         //===速度闭环控制计算电机B最终PWM 角度电机转动
					  Motor_B=Incremental_PI_B(Encoder_B,Target_MotorB);                          //===速度闭环控制计算电机B最终PWM 角度电机转动
					
//					  put_string(USART3,"MA: ");
//					  put_shuzu(USART3,Motor_A);
//					  put_string(USART3,"MB: ");
//            put_shuzu(USART3,Motor_B);

//					 Motor_A=Target_MotorA;
//					 Motor_B=Target_MotorB;
//					 Motor_A=7199;
//					 Motor_B=7199;
	
				}
				
				else if((3 == Key_mode))               //倒转倒车***********************************************************************
				{
							Read_Encoder(0);				  //清除编码器计数
              R_led_mode();					
							Motor_A = -1;                                               //===速度闭环控制计算电机B最终PWM 角度电机转动
							Motor_B = -1;				
				}
				
				else if(2 == Key_mode)                  //转弯****************************************************************************
				{
					Read_Encoder(0);            //清除编码器计数
/******************************角度闭环************************************************/
//					Angle_Calcu();        //角度获取

//					if(JSDz<Target_angle+3.5&&JSDz>Target_angle-3.5)
//					{
//						Motor_A = 0;
//				    Motor_B = 0;
//					}
//						
//					else if(JSDz>Target_angle+3.5)
//					{
//						Motor_A = 2300;
////						if(t==0)
////				    {
////							Motor_B = -1;
////							t=8;
////						}
////						else
//							Motor_B = 0;
//						
////						t--;
//					}
//					else if(JSDz<Target_angle-3.5)
//					{
//						Motor_B = 2300;
////						if(t==0)
////				    {
////							Motor_A = -1;
////							t=8;
////						}
////						 else
//							Motor_A = 0;
//						
////						 t--;
//						 
//					}
//					

					
/******************正常转弯***************************/					
					if(Target_angle>=0)
					{
						Motor_A = -1;
				    Motor_B = 7100;
					}
					else
					{
						Motor_A = 7100;
				    Motor_B = -1;
					}
					
				}
				
				else 	                                  //停车*********************************************************************
		   {	
//				LED1 = 1;
				Motor_A = 0;
				Motor_B = 0;
				Read_Encoder(0);            //清除编码器计数
				 
		   }
		
		   Set_Pwm(Motor_B,Motor_A);     //===赋值给PWM寄存器 
  
		
}


/**
  * @brief  系统滴答定时器中断服务函数
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
	if(0 != Jiaodu)
		
	System_Control();
}





