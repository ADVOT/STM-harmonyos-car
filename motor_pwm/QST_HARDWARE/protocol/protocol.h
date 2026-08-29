#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "sys.h"

#define PROTOCOL_SOF              0xAAU
#define MAX_PAYLOAD              16U
#define PROTOCOL_MAX_PAYLOAD      MAX_PAYLOAD
#define PROTOCOL_MAX_FRAME_SIZE   (PROTOCOL_MAX_PAYLOAD + 4U)
#define PROTOCOL_RX_TIMEOUT_MS    50U

/* Frame: SOF | CMD | LEN | PAYLOAD | CHECK.
   CHECK is the low byte of CMD + LEN + PAYLOAD; SOF is excluded.
   The parser publishes only events and never operates motors. */
#define PROTOCOL_CMD_SET_SPEED    0x01U
#define PROTOCOL_CMD_STOP         0x02U
#define PROTOCOL_CMD_PING         0x03U
#define PROTOCOL_CMD_GET_STATUS   0x04U
#define PROTOCOL_CMD_ACK          0x81U
#define PROTOCOL_CMD_STATUS       0x82U

/* STATUS payload: odo_left i32 LE | odo_right i32 LE |
   speed_left i16 LE | speed_right i16 LE | flags u8 (bit0 = motion lease). */
#define PROTOCOL_STATUS_PAYLOAD_LEN  13U

typedef enum
{
    PROTOCOL_STATUS_OK = 0,
    PROTOCOL_STATUS_BAD_LENGTH = 1,
    PROTOCOL_STATUS_BAD_CHECKSUM = 2,
    PROTOCOL_STATUS_INVALID_PARAM = 3,
    PROTOCOL_STATUS_UNKNOWN_CMD = 4
} ProtocolStatus;

typedef struct
{
    u8 cmd;
    u8 len;
    u8 payload[PROTOCOL_MAX_PAYLOAD];
    ProtocolStatus status;
} ProtocolEvent;

void Protocol_Init(void);
void Protocol_ParseByte(u8 byte);
void Protocol_Tick1ms(void);
u8 Protocol_TakeEvent(ProtocolEvent *event);
u8 Protocol_Checksum(u8 cmd, u8 len, const u8 *payload);
u8 Protocol_EncodeFrame(u8 cmd, u8 len, const u8 *payload,
                        u8 *frame, u8 capacity);

#endif
