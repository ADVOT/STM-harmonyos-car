#include "colorful_led.h"

u8 L_ws_data[ws_num];
u8 R_ws_data[ws_num];
/**************************************************************************
函数功能：colorful_led接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void colorful_led_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;	          //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
	
}

void L_send_0(void)
{
    DIL=1;
    Wait400ns;
    DIL=0;
    Wait850ns;
}
void L_send_1(void)
{
    DIL=1;
    Wait850ns;
    DIL=0;
    Wait400ns;
}

void R_send_0(void)
{
    DIR=1;
    Wait400ns;
    DIR=0;
    Wait850ns;
}

void R_send_1(void)
{
    DIR=1;
    Wait850ns;
    DIR=0;
    Wait400ns;
}

void L_ws2812_reset(void)
{
	DIL=0;
	delay_us(66);
}

void R_ws2812_reset(void)
{
	DIR=0;
	delay_us(66);
}

void L_ws2812_rgb(u8 L_ws_num,u8 ws_r,u8 ws_g,u8 ws_b)    //将颜色数据发送到数组中
{
    L_ws_data[(L_ws_num-1)*3]=ws_g;
    L_ws_data[(L_ws_num-1)*3+1]=ws_r;
    L_ws_data[(L_ws_num-1)*3+2]=ws_b;
}

void L_ws2812_refresh(u8 ws_count)          //根据数组数据对对应的灯进行点亮
{
    u8 L_ws_ri=0;
    
    for(;L_ws_ri<ws_count*3;L_ws_ri++)
    {
        if((L_ws_data[L_ws_ri]&0x80)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x40)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x20)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x10)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x08)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x04)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x02)==0) L_send_0(); else L_send_1();
        if((L_ws_data[L_ws_ri]&0x01)==0) L_send_0(); else L_send_1();
    }
    
    //延时一段时间
    L_ws2812_reset();
}

void R_ws2812_rgb(u8 R_ws_num,u8 ws_r,u8 ws_g,u8 ws_b)     //将颜色数据发送到数组中
{
    R_ws_data[(R_ws_num-1)*3]=ws_g;
    R_ws_data[(R_ws_num-1)*3+1]=ws_r;
    R_ws_data[(R_ws_num-1)*3+2]=ws_b;
}

void R_ws2812_refresh(u8 ws_count)            //根据数组数据对对应的灯进行点亮
{
    u8 R_ws_ri=0;
    
    for(;R_ws_ri<ws_count*3;R_ws_ri++)
    {
        if((R_ws_data[R_ws_ri]&0x80)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x40)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x20)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x10)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x08)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x04)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x02)==0) R_send_0(); else R_send_1();
        if((R_ws_data[R_ws_ri]&0x01)==0) R_send_0(); else R_send_1();
    }
    
    //延时一段时间
    R_ws2812_reset();
}
/***前灯彩色炫彩灯***/   
void L_led_mode(void)
{
	u8 times;
	 while(1) 
    {  
        times++; 

        if(times > 17)
            times = 0;
        
        switch(times)
        {
            case 0:
                L_ws2812_rgb(1, WS_RED);
                L_ws2812_rgb(2, WS_GREEN);
                L_ws2812_rgb(3, WS_BLUE);
                L_ws2812_rgb(4, WS_WHITE);
                L_ws2812_rgb(5, WS_PURPLE);
                L_ws2812_rgb(6, WS_YELLOW);
                L_ws2812_rgb(7, WS_BROWN);
                L_ws2812_rgb(8, WS_BLUE);
                L_ws2812_refresh(led_num);
                break;
            case 1:
                L_ws2812_rgb(1, AliceBlue);
                L_ws2812_rgb(2, AntiqueWhite);
                L_ws2812_rgb(3, Aqua);
                L_ws2812_rgb(4, Aquamarine);
                L_ws2812_rgb(5, Azure);
                L_ws2812_rgb(6, Beige);
                L_ws2812_rgb(7, Bisque);
                L_ws2812_rgb(8, BlanchedAlmond);
                L_ws2812_refresh(led_num);
                break;
            case 2:
                L_ws2812_rgb(1, Blue);
                L_ws2812_rgb(2, BlueViolet);
                L_ws2812_rgb(3, Brown);
                L_ws2812_rgb(4, BurlyWood);
                L_ws2812_rgb(5, CadetBlue);
                L_ws2812_rgb(6, Chartreuse);
                L_ws2812_rgb(7, Chocolate);
                L_ws2812_rgb(8, Coral);
                L_ws2812_refresh(led_num);
                break;
            case 3:
                L_ws2812_rgb(1, CornflowerBlue);
                L_ws2812_rgb(2, Cornsilk);
                L_ws2812_rgb(3, Crimson);
                L_ws2812_rgb(4, Cyan);
                L_ws2812_rgb(5, DarkBlue);
                L_ws2812_rgb(6, DarkCyan);
                L_ws2812_rgb(7, DarkGoldenRod);
                L_ws2812_rgb(8, DarkGray);
                L_ws2812_refresh(led_num);
                break;
            case 4:
                L_ws2812_rgb(1, DarkGreen);
                L_ws2812_rgb(2, DarkKhaki);
                L_ws2812_rgb(3, DarkMagenta);
                L_ws2812_rgb(4, DarkOliveGreen);
                L_ws2812_rgb(5, DarkOrange);
                L_ws2812_rgb(6, DarkOrchid);
                L_ws2812_rgb(7, DarkRed);
                L_ws2812_rgb(8, DarkSalmon);
                L_ws2812_refresh(led_num);
                break;
            case 5:
                L_ws2812_rgb(1, DarkSeaGreen);
                L_ws2812_rgb(2, DarkSlateBlue);
                L_ws2812_rgb(3, DarkSlateGray);
                L_ws2812_rgb(4, DarkTurquoise);
                L_ws2812_rgb(5, DarkViolet);
                L_ws2812_rgb(6, DeepPink);
                L_ws2812_rgb(7, DeepSkyBlue);
                L_ws2812_rgb(8, DimGray);
                L_ws2812_refresh(led_num);
                break;
            case 6:
                L_ws2812_rgb(1, DodgerBlue);
                L_ws2812_rgb(2, FireBrick);
                L_ws2812_rgb(3, FloralWhite);
                L_ws2812_rgb(4, ForestGreen);
                L_ws2812_rgb(5, Fuchsia);
                L_ws2812_rgb(6, Gainsboro);
                L_ws2812_rgb(7, GhostWhite);
                L_ws2812_rgb(8, Gold);
                L_ws2812_refresh(led_num);
                break;
            case 7:
                L_ws2812_rgb(1, GoldenRod);
                L_ws2812_rgb(2, Gray);
                L_ws2812_rgb(3, Green);
                L_ws2812_rgb(4, GreenYellow);
                L_ws2812_rgb(5, HoneyDew);
                L_ws2812_rgb(6, HotPink);
                L_ws2812_rgb(7, IndianRed);
                L_ws2812_rgb(8, Indigo);
                L_ws2812_refresh(led_num);
                break;
         case 8:
                L_ws2812_rgb(1, Ivory);
                L_ws2812_rgb(2, Khaki);
                L_ws2812_rgb(3, Lavender);
                L_ws2812_rgb(4, LavenderBlush);
                L_ws2812_rgb(5, LawnGreen);
                L_ws2812_rgb(6, LemonChiffon);
                L_ws2812_rgb(7, LightBlue);
                L_ws2812_rgb(8, LightCoral);
                L_ws2812_refresh(led_num);
                break;
				case 9:
                L_ws2812_rgb(1, LightCyan);
                L_ws2812_rgb(2, LightGoldenRodYellow);
                L_ws2812_rgb(3, LightGray);
                L_ws2812_rgb(4, LightGreen);
                L_ws2812_rgb(5, LightPink);
                L_ws2812_rgb(6, LightSalmon);
                L_ws2812_rgb(7, LightSeaGreen);
                L_ws2812_rgb(8, LightSkyBlue);
                L_ws2812_refresh(led_num);
                break;
				case 10:
                L_ws2812_rgb(1, LightSlateGray);
                L_ws2812_rgb(2, LightSteelBlue);
                L_ws2812_rgb(3, LightYellow);
                L_ws2812_rgb(4, Lime);
                L_ws2812_rgb(5, LimeGreen);
                L_ws2812_rgb(6, Linen);
                L_ws2812_rgb(7, Magenta);
                L_ws2812_rgb(8, Maroon);
                L_ws2812_refresh(led_num);
                break;
				case 11:
                L_ws2812_rgb(1, MediumAquaMarine);
                L_ws2812_rgb(2, MediumBlue);
                L_ws2812_rgb(3, MediumOrchid);
                L_ws2812_rgb(4, MediumPurple);
                L_ws2812_rgb(5, MediumSeaGreen);
                L_ws2812_rgb(6, MediumSlateBlue);
                L_ws2812_rgb(7, MediumSpringGreen);
                L_ws2812_rgb(8, MediumTurquoise);
                L_ws2812_refresh(led_num);
                break;
				case 12:
                L_ws2812_rgb(1, MediumVioletRed);
                L_ws2812_rgb(2, MidnightBlue);
                L_ws2812_rgb(3, MintCream);
                L_ws2812_rgb(4, MistyRose);
                L_ws2812_rgb(5, Moccasin);
                L_ws2812_rgb(6, NavajoWhite);
                L_ws2812_rgb(7, Navy);
                L_ws2812_rgb(8, OldLace);
                L_ws2812_refresh(led_num);
                break;
				case 13:
                L_ws2812_rgb(1, Olive);
                L_ws2812_rgb(2, OliveDrab);
                L_ws2812_rgb(3, Orange);
                L_ws2812_rgb(4, OrangeRed);
                L_ws2812_rgb(5, Orchid);
                L_ws2812_rgb(6, PaleGoldenRod);
                L_ws2812_rgb(7, PaleGreen);
                L_ws2812_rgb(8, PaleTurquoise);
                L_ws2812_refresh(led_num);
                break;
				case 14:
                L_ws2812_rgb(1, PaleVioletRed);
                L_ws2812_rgb(2, PapayaWhip);
                L_ws2812_rgb(3, PeachPuff);
                L_ws2812_rgb(4, Peru);
                L_ws2812_rgb(5, Pink);
                L_ws2812_rgb(6, Plum);
                L_ws2812_rgb(7, PowderBlue);
                L_ws2812_rgb(8, Purple);
                L_ws2812_refresh(led_num);
                break;
				case 15:
                L_ws2812_rgb(1, Red);
                L_ws2812_rgb(2, RosyBrown);
                L_ws2812_rgb(3, RoyalBlue);
                L_ws2812_rgb(4, SaddleBrown);
                L_ws2812_rgb(5, Salmon);
                L_ws2812_rgb(6, SandyBrown);
                L_ws2812_rgb(7, SeaGreen);
                L_ws2812_rgb(8, SeaShell);
                L_ws2812_refresh(led_num);
                break;
				case 16:
                L_ws2812_rgb(1, Sienna);
                L_ws2812_rgb(2, Silver);
                L_ws2812_rgb(3, SkyBlue);
                L_ws2812_rgb(4, SlateBlue);
                L_ws2812_rgb(5, SlateGray);
                L_ws2812_rgb(6, Snow);
                L_ws2812_rgb(7, SpringGreen);
                L_ws2812_rgb(8, SteelBlue);
                L_ws2812_refresh(led_num);
                break;
				case 17:
                L_ws2812_rgb(1, Tan);
                L_ws2812_rgb(2, Teal);
                L_ws2812_rgb(3, Thistle);
                L_ws2812_rgb(4, Tomato);
                L_ws2812_rgb(5, Turquoise);
                L_ws2812_rgb(6, Violet);
                L_ws2812_rgb(7, Wheat);
                L_ws2812_rgb(8, White);
                L_ws2812_refresh(led_num);
                break;	
        }
       
        delay_ms(1000);        
    }  
}
/***后灯尾灯表示倒车***/
void R_led_mode(void)
{
	
	              R_ws2812_rgb(1, Red);
                R_ws2812_rgb(2, WhiteSmoke);
                R_ws2812_rgb(3, WhiteSmoke);
                R_ws2812_rgb(4, WhiteSmoke);
                R_ws2812_rgb(5, WhiteSmoke);
                R_ws2812_rgb(6, Red);
                R_ws2812_refresh(led_num);
		           
}
/*****后灯关闭*****/
void R_led_CLC(void)
{
	              R_ws2812_rgb(1, WS_DARK);
                R_ws2812_rgb(2, WS_DARK);
                R_ws2812_rgb(3, WS_DARK);
                R_ws2812_rgb(4, WS_DARK);
                R_ws2812_rgb(5, WS_DARK);
                R_ws2812_rgb(6, WS_DARK);
                R_ws2812_refresh(led_num);
		           
}

/*****前灯跑马灯效果*****/
void L_runingled(void)    //前灯跑马灯
{
	u8 i,j;
/*流光*/	
	for(j=1;j<7;j++)    
	 { 
		 for(i=1;i<7;i++)    //把灯的颜色写在每个灯的数组中
		{
			if(i==j)
				L_ws2812_rgb(i, WS_WHITE);
			else
				L_ws2812_rgb(i, WS_DARK);
		}	
		 L_ws2812_refresh(led_num);  //更新灯颜色
		 delay_ms(100);
	 }
/*反流光*/	
	for(j=6;j>=1;j--)    
	 { 
		 for(i=6;i>=1;i--)    //把灯的颜色写在每个灯的数组中
		{
			if(i==j)
				L_ws2812_rgb(i, WS_WHITE);
			else
				L_ws2812_rgb(i, WS_DARK);
		}	
		 L_ws2812_refresh(led_num);  //更新灯颜色
		 delay_ms(100);
	 }	 

/***回响车灯***/
	 while(1) 
    {  
			for(j=1;j<6;j++)        //回
		{	 
				for(i=1;i<7;i++)    //把灯的颜色写在每个灯的数组中
				{
					if(j==1)    // * _ _ _ _ *
					{	
						if((i==6||i==1))
						L_ws2812_rgb(i, GhostWhite);
					  else
						L_ws2812_rgb(i, WS_DARK);
				  }
					if(j==2)    // * * _ _ * *
					{	
					if((i==5||i==2||i==6||i==1))
						L_ws2812_rgb(i, GhostWhite);
					else
						L_ws2812_rgb(i, WS_DARK);
				  }
					if(j==3)    // * * * * * *
					{	
					if((i==3||i==4||i==5||i==2||i==6||i==1))
						L_ws2812_rgb(i, GhostWhite);
					else
						L_ws2812_rgb(i, WS_DARK);
				  }
					if(j==4)    // * * _ _ * *
					{	
					if((i==5||i==2||i==6||i==1))
						L_ws2812_rgb(i, GhostWhite);
					else
						L_ws2812_rgb(i, WS_DARK);
				  }
					if(j==5)   // * _ _ _ _ *
					{	
						if((i==6||i==1))
						L_ws2812_rgb(i, GhostWhite);
					  else
						L_ws2812_rgb(i, WS_DARK);
				  }
					
				}	
				L_ws2812_refresh(led_num);  //更新灯颜色
				if(j==3)
				delay_ms(200);
				else
				delay_ms(100);
	 }	    
	
    delay_ms(50);        
    }
}

/*****后灯跑马灯效果*****/
void R_runingled(void)
{
	u8 i,j;
/*流光*/	
	for(j=1;j<7;j++)    
	 { 
		 for(i=1;i<7;i++)    //把灯的颜色写在每个灯的数组中
		{
			if(i==j)
				R_ws2812_rgb(i, WS_WHITE);
			else
				R_ws2812_rgb(i, WS_DARK);
		}	
		 R_ws2812_refresh(led_num);  //更新灯颜色
		 delay_ms(100);
	 }
/*反流光*/	
	for(j=6;j>=1;j--)    
	 { 
		 for(i=6;i>=1;i--)    //把灯的颜色写在每个灯的数组中
		{
			if(i==j)
				R_ws2812_rgb(i, WS_WHITE);
			else
				R_ws2812_rgb(i, WS_DARK);
		}	
		 R_ws2812_refresh(led_num);  //更新灯颜色
		 delay_ms(100);
	 }	 

/***回响车灯***/
	 while(1) 
    {  
			for(j=1;j<6;j++)        //回
		{	 
				for(i=1;i<7;i++)    //把灯的颜色写在每个灯的数组中
				{
					if(j==1)    // * _ _ _ _ *
					{	
						if((i==6||i==1))
						R_ws2812_rgb(i, GhostWhite);
					  else
						R_ws2812_rgb(i, WS_DARK);
				  }
					if(j==2)    // * * _ _ * *
					{	
					if((i==5||i==2||i==6||i==1))
						R_ws2812_rgb(i, GhostWhite);
					else
						R_ws2812_rgb(i, WS_DARK);
				  }
					if(j==3)    // * * * * * *
					{	
					if((i==3||i==4||i==5||i==2||i==6||i==1))
						R_ws2812_rgb(i, GhostWhite);
					else
						R_ws2812_rgb(i, WS_DARK);
				  }
					if(j==4)    // * * _ _ * *
					{	
					if((i==5||i==2||i==6||i==1))
						R_ws2812_rgb(i, GhostWhite);
					else
						R_ws2812_rgb(i, WS_DARK);
				  }
					if(j==5)   // * _ _ _ _ *
					{	
						if((i==6||i==1))
						R_ws2812_rgb(i, GhostWhite);
					  else
						R_ws2812_rgb(i, WS_DARK);
				  }
					
				}	
				R_ws2812_refresh(led_num);  //更新灯颜色
				if(j==3)
				delay_ms(200);
				else
				delay_ms(100);
		}
	
    delay_ms(50);        
    }
}
