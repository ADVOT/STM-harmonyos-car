#include "sys.h"

#define ws_num 24
#define led_num 6
#define DIL PCout(13) 
#define DIR PCout(14) 

//****************RGB灯带颜色表*****************//
#define WS_DARK  0,0,0
#define WS_WHITE  255,255,255
#define WS_RED   255,0,0
#define WS_GREEN  0,255,0
#define WS_BLUE  0,0,255
#define WS_YELLOW  255,255,0
#define WS_PURPLE   255,0,255
#define WS_CYAN  0,255,255
#define WS_BROWN    165,42,42

#define AliceBlue	240,248,255
#define AntiqueWhite	250,235,215
#define Aqua	0,255,255
#define Aquamarine	127,255,212
#define Azure	240,255,255
#define Beige	245,245,220
#define Bisque	255,228,196
#define BlanchedAlmond	255,235,205

#define Blue		0,0,255
#define BlueViolet	 	138,43,226
#define Brown	 	165,42,42
#define BurlyWood	 	222,184,135
#define CadetBlue	 	95,158,160
#define Chartreuse	 	127,255,0
#define Chocolate	 	210,105,30
#define Coral	 	255,127,80

#define CornflowerBlue	 	100,149,237
#define Cornsilk	 	255,248,220
#define Crimson	 	220,20,60
#define Cyan	 	0,255,255
#define DarkBlue	 	0,0,139
#define DarkCyan	 	0,139,139
#define DarkGoldenRod	 	184,134,11
#define DarkGray	 	169,169,169

#define DarkGreen	 	0,100,0
#define DarkKhaki	 	189,183,107
#define DarkMagenta	 	139,0,139
#define DarkOliveGreen	 	85,107,47
#define DarkOrange	 	255,140,0
#define DarkOrchid	 	153,50,204
#define DarkRed	 	139,0,0
#define DarkSalmon	 	233,150,122

#define DarkSeaGreen	 	143,188,143
#define DarkSlateBlue	 	72,61,139
#define DarkSlateGray	 	47,79,79
#define DarkTurquoise	 	0,206,209
#define DarkViolet	 	148,0,211
#define DeepPink	 	255,20,147
#define DeepSkyBlue	 	0,191,255
#define DimGray	 	105,105,105

#define DodgerBlue	 	30,144,255
#define FireBrick	 	178,34,34
#define FloralWhite	 	255,250,240
#define ForestGreen	 	34,139,34
#define Fuchsia	 	255,0,255
#define Gainsboro	 	220,220,220
#define GhostWhite	 	248,248,255
#define Gold	 	255,215,0

#define GoldenRod	 	218,165,32
#define Gray	 	128,128,128
#define Green	 	0,128,0
#define GreenYellow	 	173,255,47
#define HoneyDew	 	240,255,240
#define HotPink	 	255,105,180
#define IndianRed	 	205,92,92
#define Indigo	 	75,0,130

#define Ivory	 	255,255,240
#define Khaki	 	240,230,140
#define Lavender	 	230,230,250
#define LavenderBlush	 	255,240,245
#define LawnGreen	 	124,252,0
#define LemonChiffon	 	255,250,205
#define LightBlue	 	173,216,230
#define LightCoral	 	240,128,128

#define LightCyan	 	224,255,255
#define LightGoldenRodYellow	 	250,250,210
#define LightGray	 	211,211,211
#define LightGreen	 	144,238,144
#define LightPink	 	255,182,193
#define LightSalmon	 	255,160,122
#define LightSeaGreen	 	32,178,170
#define LightSkyBlue	 	135,206,250

#define LightSlateGray	 	119,136,153
#define LightSteelBlue	 	176,196,222
#define LightYellow	 	255,255,224
#define Lime	 	0,255,0
#define LimeGreen	 	50,205,50
#define Linen	 	250,240,230
#define Magenta	 	255,0,255
#define Maroon	 	128,0,0

#define MediumAquaMarine	 	102,205,170
#define MediumBlue	 	0,0,205
#define MediumOrchid	 	186,85,211
#define MediumPurple	 	147,112,219
#define MediumSeaGreen	 	60,179,113
#define MediumSlateBlue	 	123,104,238
#define MediumSpringGreen	 	0,250,154
#define MediumTurquoise	 	72,209,204

#define MediumVioletRed	 	199,21,133
#define MidnightBlue	 	25,25,112
#define MintCream	 	245,255,250
#define MistyRose	 	255,228,225
#define Moccasin	 	255,228,181
#define NavajoWhite	 	255,222,173
#define Navy	 	0,0,128
#define OldLace	 	253,245,230

#define Olive	 	128,128,0
#define OliveDrab	 	107,142,35
#define Orange	 	255,165,0
#define OrangeRed	 	255,69,0
#define Orchid	 	218,112,214
#define PaleGoldenRod	 	238,232,170
#define PaleGreen	 	152,251,152
#define PaleTurquoise	 	175,238,238

#define PaleVioletRed	 	219,112,147
#define PapayaWhip	 	255,239,213
#define PeachPuff	 	255,218,185
#define Peru	 	205,133,63
#define Pink	 	255,192,203
#define Plum	 	221,160,221
#define PowderBlue	 	176,224,230
#define Purple	 	128,0,128

#define Red	 	255,0,0
#define RosyBrown	 	188,143,143
#define RoyalBlue	 	65,105,225
#define SaddleBrown	 	139,69,19
#define Salmon	 	250,128,114
#define SandyBrown	 	244,164,96
#define SeaGreen	 	46,139,87
#define SeaShell	255,245,238

#define Sienna	 	160,82,45
#define Silver	 	192,192,192
#define SkyBlue	 	135,206,235
#define SlateBlue	 	106,90,205
#define SlateGray	 	112,128,144
#define Snow	 	255,250,250
#define SpringGreen	 	0,255,127
#define SteelBlue	 	70,130,180

#define Tan	 	210,180,140
#define Teal	 	0,128,128
#define Thistle	 	216,191,216
#define Tomato	 	255,99,71
#define Turquoise	 	64,224,208
#define Violet	 	238,130,238
#define Wheat	 	245,222,179
#define White	 	255,255,255

#define WhiteSmoke	 	245,245,245
#define Yellow	 	255,255,0
#define YellowGreen	 	154,205,50

#define    Wait10nop        {__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();}
#define    Wait250ns        {__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();}
#define    Wait400ns        {Wait250ns;Wait10nop;}    //388
#define    Wait850ns        {Wait250ns;Wait10nop;Wait10nop;Wait10nop;Wait10nop;__NOP();__NOP();__NOP();__NOP();__NOP();}    //860



void colorful_led_Init(void);
void L_ws2812_rgb(u8 L_ws_num,u8 ws_r,u8 ws_g,u8 ws_b);
void R_ws2812_rgb(u8 L_ws_num,u8 ws_r,u8 ws_g,u8 ws_b); 
void L_ws2812_refresh(u8 ws_count);
void R_ws2812_refresh(u8 ws_count);
void R_led_mode(void);
void L_led_mode(void);
void R_led_CLC(void);
void L_runingled(void);    //前灯跑马灯

//****************灯效模式（串口命令编号1~6）*****************//
#define LED_MODE_HELLO  1   //HELLO 前灯跑马灯
#define LED_MODE_ALLON  2   //ALLON 12颗全亮
#define LED_MODE_BLINK  3   //BLINK 12颗同步闪烁
#define LED_MODE_RAIN   4   //RAIN  彩虹换色
#define LED_MODE_FLOW   5   //FLOW  流水扫过12颗
#define LED_MODE_OFF    6   //OFF   全部熄灭

void led_all_on(u8 r,u8 g,u8 b);   //12颗全亮同色
void led_all_off(void);            //12颗全灭
void fx_hello_frame(void);         //跑马灯单帧（非阻塞版L_runingled）
void fx_blink_frame(void);         //同步闪烁单帧
void fx_rain_frame(void);          //彩虹换色单帧
void fx_flow_frame(void);          //流水扫过单帧

