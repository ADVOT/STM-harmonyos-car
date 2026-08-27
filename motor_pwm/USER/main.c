#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "delay.h"

/*==================== 任务23：速度闭环 + 外环纠偏 → 前进1米倒退1米回起点 ====================
   任务模式（8-27 桃定）：固定速度直行 1 米 → 倒退 1 米 → 回到出发点停车。
   - 内环：目标速度 → 编码器实测 → 增量式 PID 修 PWM，两轮各一套
   - 外环（YAW_CORRECT=1）：里程差纠偏。两编码器累计差 = 朝向的表（学习记录#11），
     差速叠加到两轮目标把里程差追平（#12）。倒车时符号自动适配，公式不变。
     前进段攒下的 diff 在倒退段被压回 0 → 朝向回归；但注意：外环只保航向不保横线，
     S 形纠偏留下的横向偏移消不掉（学习记录 8-27#5），消偏移靠下面的软启动少歪
   - 软启动爬坡：目标每拍只动 RAMP_STEP。起步猛给油时两轮挣脱静摩擦有先后 =
     车头先歪（落地首跑实测右轮慢、偏右斜线、回程偏移一个车身位）；
     爬坡让挣脱对称 + 外环低速段就有纠偏余地；换向时目标扫过 0 = 自动软刹车
   - 判距用两轮平均里程 odo_avg，1米 = 2800/0.1425 ≈ 19649 脉冲
   - 局限：无陀螺仪，只能保持起步那一刻的朝向；打滑会让脉冲数≠实际距离，
     终点看串口 odo/diff 回零程度，别用尺量

   参考答案三个坑（8-27 分析，本工程已规避，勿照抄官方）：
   1) 官方积分项累加了但没接入输出（死代码）；本版增量结构输出=累加器本身，天然含积分
   2) 官方 KP/KD 角色与标准公式对调：增量结构下 KP×误差累加=积分作用（管消静差），
      KD×误差增量累积≈比例作用（管响应快慢）
   3) 官方 TageB=Rs_To_CPR(-1.0) 是别人车右编码器相序反的情况；
      本车实测两轮同转向同号为正（任务22作业），目标符号按行程方向取 ±

   历史版本留底 backups/：fixed280 / gearhomework / yaw / mission_v1（无软启动）
==========================================================================*/
#define PULSES_PER_REV   2800     /* 一圈脉冲数：官方标称 ppr700×4倍频，右轮实测2739印证 */
#define WHEEL_CIRC_M     0.1425f  /* 车轮周长（米）= 桃实测 14.25cm，用于换算下面的一米脉冲 */
#define TARGET_PULSES    280      /* 巡航速度：1转/s = 280 脉冲/100ms ≈ 0.1425 m/s */
#define ONE_METER_PULSES 19649L   /* 1米 = 2800 / 0.1425 ≈ 19649 脉冲 */
#define RAMP_STEP        20       /* 软启动：目标每拍(100ms)只动 20 → 0到280 爬 1.4 秒 */

/*==================== 外环：里程差纠偏（保持起步朝向走直线） ====================
   离散稳定性：diff(k+1) ≈ diff(k)×(1-2×KP_YAW)，0<KP_YAW<1 才收敛；
   内环有响应延迟，KP_YAW 取小（外环要比内环慢 3~5 倍），画龙就减半、纠不动就加倍。
==========================================================================*/
#define YAW_CORRECT      1
#define KP_YAW           0.1f     /* 每 1 脉冲里程差 → 0.1 脉冲目标修正 */
#define CORR_MAX         40       /* 修正限幅：纠太猛会来回摆（振荡） */

/* 行程状态机：FWD 前进1米 → BWD 倒退1米回起点 → DONE 停车 */
#define LEG_FWD   0
#define LEG_BWD   1
#define LEG_DONE  2

int L_speed = 0;   //左轮速度（100ms 内的脉冲数）
int R_speed = 0;   //右轮速度（100ms 内的脉冲数）
int Motor_L = 0;   //左轮 PID 输出（写进 Set_Pwm 的占空比值）
int Motor_R = 0;   //右轮 PID 输出
long Odo_L = 0;    //左轮里程累计（脉冲，上电清零）
long Odo_R = 0;    //右轮里程累计

/**************************************************************************
增量式 PID 状态（每轮一份）。结构体化：官方双函数 A/B 重复代码合一。
角色对调警告：kp 实为积分作用（消静差），kd 实为比例作用（管响应）。
**************************************************************************/
typedef struct {
	float kp, kd;      /* 见上警告，勿按标准公式字面理解 */
	int   pwm;         /* 累计输出（增量式必须记忆历史） */
	float err_prev;    /* 上一次误差 */
} PID_Inc_t;

PID_Inc_t pidL = {7.0f, 0.003f, 0, 0};   //官方初值起步，实测已稳，未调
PID_Inc_t pidR = {7.0f, 0.003f, 0, 0};   //左右电机阻力不同，闭环自动补偿

/**************************************************************************
函数功能：增量式 PID 计算
入口参数：p=该轮PID状态，encoder=实测脉冲（100ms），target=目标脉冲
返回  值：累计后的 PWM 输出（已限幅 ±7199 = TIM4 ARR 满量程）
**************************************************************************/
int Incremental_PID(PID_Inc_t *p, int encoder, int target)
{
	float err = (float)(target - encoder);
	p->pwm += (int)(p->kp * err + p->kd * (err - p->err_prev));
	if (p->pwm > 7199) p->pwm = 7199;          //输出限幅：PWM 满量程
	else if (p->pwm < -7199) p->pwm = -7199;
	p->err_prev = err;
	return p->pwm;
}

/**************************************************************************
函数功能：系统控制函数，由 SysTick 中断每 100ms 调用一次
          读编码器 → 里程累计 → 行程状态机 → 软启动爬坡 → 外环纠偏 → 双 PID → 打印
入口参数：无
返回  值：无
**************************************************************************/
void System_Control(void)
{
	static int leg = LEG_FWD;        //行程状态：去 → 回 → 停
	static int ramp = 0;             //软启动爬坡后的实际目标（带符号）
	long odo_avg, odo_diff;
	int target, targetL, targetR, corr = 0;
	char leg_ch;

	L_speed = Read_Encoder(2);       //左轮：TIM2
	R_speed = Read_Encoder(3);       //右轮：TIM3
	Odo_L += L_speed;
	Odo_R += R_speed;
	odo_avg  = (Odo_L + Odo_R) / 2;  //平均里程 = 车心位置
	odo_diff = Odo_L - Odo_R;        //里程差 = 朝向的表

	/*---- 行程状态机：到1米换向、回原点停车 ----
	   换向不清 PID 累加器：目标随爬坡平滑扫过 0，累加器自己退火，全程无硬切 */
	if (leg == LEG_FWD && odo_avg >= ONE_METER_PULSES)
	{
		leg = LEG_BWD;
	}
	else if (leg == LEG_BWD && odo_avg <= 0)
	{
		leg = LEG_DONE;
	}

	if (leg == LEG_DONE)
	{
		Set_Pwm(0,0);
		printf("[S] stop | odo %6ld diff %5ld\r\n", odo_avg, odo_diff);
		return;
	}

	target = (leg == LEG_FWD) ? TARGET_PULSES : -TARGET_PULSES;
	/* 软启动爬坡：ramp 向 target 每拍靠拢 RAMP_STEP，起步/换向都不猛给油 */
	if (ramp < target) { ramp += RAMP_STEP; if (ramp > target) ramp = target; }
	else if (ramp > target) { ramp -= RAMP_STEP; if (ramp < target) ramp = target; }
	target = ramp;
	leg_ch = (leg == LEG_FWD) ? 'F' : 'B';

#if YAW_CORRECT
	/* 外环：左比右多走 → diff>0 → corr>0 → 压左抬右（倒车时各量反号，公式自动适配） */
	corr = (int)(KP_YAW * (float)odo_diff);
	if (corr > CORR_MAX) corr = CORR_MAX;
	else if (corr < -CORR_MAX) corr = -CORR_MAX;
#endif

	targetL = target - corr;
	targetR = target + corr;
	Motor_L = Incremental_PID(&pidL, L_speed, targetL);
	Motor_R = Incremental_PID(&pidR, R_speed, targetR);
	Set_Pwm(Motor_L, Motor_R);

	printf("[%c] L %4d/%+d pwm %5d | R %4d/%+d pwm %5d | odo %6ld diff %5ld corr %3d\r\n",
	       leg_ch, L_speed, targetL, Motor_L, R_speed, targetR, Motor_R,
	       odo_avg, odo_diff, corr);
}

int main(void)
{
	Stm32_Clock_Init(9);            //外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
	MY_NVIC_PriorityGroupConfig(2); //=====中断优先级分组
	uart_init(115200);              //=====串口初始化为115200
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

	Encoder_Init_TIM2();            //=====左电机编码器（PA0/PA1）
	Encoder_Init_TIM3();            //=====右电机编码器（PA6/PA7）
	PWM_Init(7199,9);               //=====TIM4 两路PWM 频率1000Hz（电机驱动）
	SysTick_Config(72000000/1000);  //=====滴答定时器，每1ms触发一次中断

	printf("ALL-ready\r\n");
	/**主要程序：状态机+双环全在 SysTick 100ms 节拍的 System_Control() 里，主循环清空。
	   注意：上电即开始爬坡起步，烧录前架空轮子、先拔ST-Link再开电池；
	   正式跑时车头摆正再松手（外环保的是起步朝向）**/
	while(1)
	{
	}
}
