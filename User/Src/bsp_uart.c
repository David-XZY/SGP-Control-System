#include "bsp_uart.h"
#include <string.h>
//
uint8_t rx_data;
char rx_buffer[64];
static uint8_t rx_index = 0;
volatile uint8_t new_cmd_flag = 0;

// 初始化 UART 接收中断（在main.c中调用）
void UART_Init_Receive(void) {
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

// UART 接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_data == '\n' || rx_data == '\r') {
            rx_buffer[rx_index] = '\0';
            rx_index = 0;
            new_cmd_flag = 1;
        } else if (rx_index < 63) {
            rx_buffer[rx_index++] = rx_data;
        }
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
