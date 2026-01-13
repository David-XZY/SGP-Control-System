#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"
#include <stdio.h>

/* ?????? */
extern char rx_buffer[20];
extern uint8_t rx_data;
extern uint8_t new_cmd_flag;

/* ???? */
void UART_Init_Receive(void);

#endif /* __BSP_UART_H */
