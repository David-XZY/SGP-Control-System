#include "bsp_uart.h"
#include <string.h>

uint8_t rx_data;
char rx_buffer[20];
static uint8_t rx_index = 0;
uint8_t new_cmd_flag = 0;

/* printf ???? UART1 */
int fputc(int ch, FILE *f) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

void UART_Init_Receive(void) {
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

/* ???????? */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_data == '\n' || rx_data == '\r') {
            rx_buffer[rx_index] = '\0';
            rx_index = 0;
            new_cmd_flag = 1;
        } else if (rx_index < 19) {
            rx_buffer[rx_index++] = rx_data;
        }
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
