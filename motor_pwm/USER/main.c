#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "delay.h"
#include "usart.h"
#include "protocol.h"
#include "colorful_led.h"

#define MAX_TARGET_SPEED       420
#define MOTION_LEASE_MS        300U
#define RAMP_STEP              20
#define PWM_LIMIT              7199

volatile int target_speed_left = 0;
volatile int target_speed_right = 0;
volatile u32 motion_lease_active = 0;
volatile u32 last_motion_ms = 0;

static int ramp_left = 0;
static int ramp_right = 0;

/* Cumulative encoder odometry (pulses since power-on). Read_Encoder returns
   the per-control-period count and clears the timer counter, so System_Control
   integrates it here. StopMotion never resets these: masters take differences. */
volatile int odo_left = 0;
volatile int odo_right = 0;
static int speed_left_now = 0;
static int speed_right_now = 0;

/* The PID state is retained across normal SET_SPEED updates. */
typedef struct
{
    float kp;
    float kd;
    int pwm;
    float err_prev;
} PID_Inc_t;

static PID_Inc_t pidL = {7.0f, 0.003f, 0, 0};
static PID_Inc_t pidR = {7.0f, 0.003f, 0, 0};

extern volatile u32 millis;
extern volatile u8 control_due;

static u32 Main_GetMillis(void)
{
    u32 now;

    __disable_irq();
    now = millis;
    __enable_irq();
    return now;
}

static u8 Main_TakeControlDue(void)
{
    u8 due;

    __disable_irq();
    due = control_due;
    control_due = 0;
    __enable_irq();
    return due;
}

static void ResetPid(PID_Inc_t *pid)
{
    pid->pwm = 0;
    pid->err_prev = 0;
}

static void StopMotion(void)
{
    target_speed_left = 0;
    target_speed_right = 0;
    motion_lease_active = 0;
    ramp_left = 0;
    ramp_right = 0;
    ResetPid(&pidL);
    ResetPid(&pidR);
    Set_Pwm(0, 0);
}

static void ServiceMotionLease(void)
{
    u32 now;

    if (motion_lease_active == 0U)
    {
        return;
    }

    now = Main_GetMillis();
    if ((u32)(now - last_motion_ms) >= MOTION_LEASE_MS)
    {
        StopMotion();
    }
}

/* ---- Turn-signal corner lamps (WS2812, front strip PC13 / rear strip PC14)
   Pure chassis feature: derives the lit corner from the commanded wheel
   speeds, so STOP and lease expiry clear the lamps automatically.
   diff = right-left > 0 means CCW rotation (nose swings left); when
   reversing (sum < 0) the rear swings to the opposite side, which is why
   the rear corners are mirrored against the front ones. Pivot in place
   (sum ~ 0) is treated as a forward turn: only the front lamp lights. */

#define TURN_LIGHT_DZ         10     /* deadband; targets are 0/+-30/+-60/+-140 */
#define TURN_LIGHT_RESEND_MS  500U   /* re-push period: self-heal glitched frames */

/* Corner LED indices (1..6) on each strip; each corner lights 2 adjacent
   LEDs. 9-01 on-car test: initial 1-2/5-6 guess came out mirrored, so the
   L/R pairs below are the swapped (verified) assignment. */
#define FRONT_LEFT_A    5U
#define FRONT_LEFT_B    6U
#define FRONT_RIGHT_A   1U
#define FRONT_RIGHT_B   2U
#define REAR_LEFT_A     5U
#define REAR_LEFT_B     6U
#define REAR_RIGHT_A    1U
#define REAR_RIGHT_B    2U

typedef enum
{
    CORNER_NONE = 0,
    CORNER_FRONT_LEFT,
    CORNER_FRONT_RIGHT,
    CORNER_REAR_LEFT,
    CORNER_REAR_RIGHT
} TurnLightCorner;

static TurnLightCorner ResolveTurnCorner(void)
{
    int diff = target_speed_right - target_speed_left;
    int sum = target_speed_right + target_speed_left;

    if (diff > TURN_LIGHT_DZ)            /* CCW: nose left */
    {
        return (sum < -TURN_LIGHT_DZ) ? CORNER_REAR_RIGHT : CORNER_FRONT_LEFT;
    }
    if (diff < -TURN_LIGHT_DZ)           /* CW: nose right */
    {
        return (sum < -TURN_LIGHT_DZ) ? CORNER_REAR_LEFT : CORNER_FRONT_RIGHT;
    }
    return CORNER_NONE;
}

static void RenderTurnCorner(TurnLightCorner corner)
{
    u8 i;

    for (i = 1; i <= led_num; i++)
    {
        L_ws2812_rgb(i, WS_DARK);
        R_ws2812_rgb(i, WS_DARK);
    }

    switch (corner)
    {
        case CORNER_FRONT_LEFT:
            L_ws2812_rgb(FRONT_LEFT_A, WS_YELLOW);
            L_ws2812_rgb(FRONT_LEFT_B, WS_YELLOW);
            break;
        case CORNER_FRONT_RIGHT:
            L_ws2812_rgb(FRONT_RIGHT_A, WS_YELLOW);
            L_ws2812_rgb(FRONT_RIGHT_B, WS_YELLOW);
            break;
        case CORNER_REAR_LEFT:
            R_ws2812_rgb(REAR_LEFT_A, WS_YELLOW);
            R_ws2812_rgb(REAR_LEFT_B, WS_YELLOW);
            break;
        case CORNER_REAR_RIGHT:
            R_ws2812_rgb(REAR_RIGHT_A, WS_YELLOW);
            R_ws2812_rgb(REAR_RIGHT_B, WS_YELLOW);
            break;
        default:
            break;
    }

    L_ws2812_refresh(led_num);
    R_ws2812_refresh(led_num);
}

static void TurnLightService(u32 now)
{
    static TurnLightCorner active = CORNER_NONE;
    static u8 initialized = 0U;
    static u32 last_refresh_ms = 0U;
    TurnLightCorner wanted = ResolveTurnCorner();

    if ((initialized == 0U) || (wanted != active) ||
        ((u32)(now - last_refresh_ms) >= TURN_LIGHT_RESEND_MS))
    {
        RenderTurnCorner(wanted);
        active = wanted;
        last_refresh_ms = now;
        initialized = 1U;
    }
}

static int MoveToward(int current, int target)
{
    if (current < target)
    {
        current += RAMP_STEP;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= RAMP_STEP;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

static int Incremental_PID(PID_Inc_t *pid, int encoder, int target)
{
    float err = (float)(target - encoder);

    pid->pwm += (int)(pid->kp * err + pid->kd * (err - pid->err_prev));
    if (pid->pwm > PWM_LIMIT)
    {
        pid->pwm = PWM_LIMIT;
    }
    else if (pid->pwm < -PWM_LIMIT)
    {
        pid->pwm = -PWM_LIMIT;
    }
    pid->err_prev = err;
    return pid->pwm;
}

void System_Control(void)
{
    int speed_left;
    int speed_right;
    int target_left;
    int target_right;

    speed_left = Read_Encoder(2);
    speed_right = Read_Encoder(3);

    odo_left += speed_left;
    odo_right += speed_right;
    speed_left_now = speed_left;
    speed_right_now = speed_right;

    target_left = target_speed_left;
    target_right = target_speed_right;
    ramp_left = MoveToward(ramp_left, target_left);
    ramp_right = MoveToward(ramp_right, target_right);

    if (motion_lease_active == 0U)
    {
        Set_Pwm(0, 0);
        return;
    }

    target_left = Incremental_PID(&pidL, speed_left, ramp_left);
    target_right = Incremental_PID(&pidR, speed_right, ramp_right);
    Set_Pwm(target_left, target_right);
}

static int DecodeInt16(const u8 *data)
{
    int value;

    value = (int)((u16)data[0] | ((u16)data[1] << 8));
    if (value & 0x8000)
    {
        value -= 0x10000;
    }
    return value;
}

static u8 SpeedIsValid(int speed)
{
    return (speed >= -MAX_TARGET_SPEED && speed <= MAX_TARGET_SPEED);
}

static void SendAck(u8 original_cmd, ProtocolStatus status)
{
    u8 payload[2];

    payload[0] = original_cmd;
    payload[1] = (u8)status;
    USART1_SendFrame(PROTOCOL_CMD_ACK, 2, payload);
}

static void SendStatus(void)
{
    u8 payload[PROTOCOL_STATUS_PAYLOAD_LEN];
    u32 left;
    u32 right;
    u16 speed_left;
    u16 speed_right;

    /* Same context as System_Control (main loop): 32-bit reads are consistent. */
    left = (u32)odo_left;
    right = (u32)odo_right;
    speed_left = (u16)speed_left_now;
    speed_right = (u16)speed_right_now;

    payload[0] = (u8)(left & 0xFFU);
    payload[1] = (u8)((left >> 8) & 0xFFU);
    payload[2] = (u8)((left >> 16) & 0xFFU);
    payload[3] = (u8)((left >> 24) & 0xFFU);
    payload[4] = (u8)(right & 0xFFU);
    payload[5] = (u8)((right >> 8) & 0xFFU);
    payload[6] = (u8)((right >> 16) & 0xFFU);
    payload[7] = (u8)((right >> 24) & 0xFFU);
    payload[8] = (u8)(speed_left & 0xFFU);
    payload[9] = (u8)((speed_left >> 8) & 0xFFU);
    payload[10] = (u8)(speed_right & 0xFFU);
    payload[11] = (u8)((speed_right >> 8) & 0xFFU);
    payload[12] = (u8)((motion_lease_active != 0U) ? 1U : 0U);
    USART1_SendFrame(PROTOCOL_CMD_STATUS, PROTOCOL_STATUS_PAYLOAD_LEN, payload);
}

static void HandleEvent(const ProtocolEvent *event)
{
    int left;
    int right;

    if (event->cmd == PROTOCOL_CMD_ACK)
    {
        return;
    }

    if (event->status != PROTOCOL_STATUS_OK)
    {
        SendAck(event->cmd, event->status);
        return;
    }

    switch (event->cmd)
    {
        case PROTOCOL_CMD_SET_SPEED:
            left = DecodeInt16(event->payload);
            right = DecodeInt16(&event->payload[2]);
            if (!SpeedIsValid(left) || !SpeedIsValid(right))
            {
                SendAck(event->cmd, PROTOCOL_STATUS_INVALID_PARAM);
                return;
            }
            target_speed_left = left;
            target_speed_right = right;
            last_motion_ms = Main_GetMillis();
            motion_lease_active = 1;
            SendAck(event->cmd, PROTOCOL_STATUS_OK);
            break;

        case PROTOCOL_CMD_STOP:
            StopMotion();
            SendAck(event->cmd, PROTOCOL_STATUS_OK);
            break;

        case PROTOCOL_CMD_PING:
            SendAck(event->cmd, PROTOCOL_STATUS_OK);
            break;

        case PROTOCOL_CMD_GET_STATUS:
            SendStatus();
            break;

        default:
            SendAck(event->cmd, PROTOCOL_STATUS_UNKNOWN_CMD);
            break;
    }
}

int main(void)
{
    ProtocolEvent event;

    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    Protocol_Init();
    uart_init(115200);
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    PWM_Init(7199, 9);
    colorful_led_Init();
    Set_Pwm(0, 0);
    SysTick_Config(72000000 / 1000);

    while (1)
    {
        if (Protocol_TakeEvent(&event) != 0U)
        {
            HandleEvent(&event);
        }
        ServiceMotionLease();
        TurnLightService(Main_GetMillis());
        if (Main_TakeControlDue() != 0U)
        {
            System_Control();
        }
    }
}
