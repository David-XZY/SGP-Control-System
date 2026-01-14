#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"
#include <stdio.h>

// UART 接收相关变量
extern char rx_buffer[64];
extern uint8_t rx_data;
extern volatile uint8_t new_cmd_flag;

extern float target_pos_all[6];

// 初始化 UART 接收中断
void UART_Init_Receive(void);
void UART_ProcessCommand(void);

#endif /* __BSP_UART_H */
