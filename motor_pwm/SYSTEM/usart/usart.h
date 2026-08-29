#ifndef __USART_H
#define __USART_H

#include "sys.h"

void uart_init(u32 bound);
void USART1_SendBytes(const u8 *data, u8 len);
u8 USART1_SendFrame(u8 cmd, u8 len, const u8 *payload);

#endif
