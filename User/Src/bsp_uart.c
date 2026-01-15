#include "bsp_uart.h"
#include "Actuator.h"
#include <string.h>
#include <stdlib.h>

uint8_t rx_data;
char rx_buffer[64];
static uint8_t rx_index = 0;
volatile uint8_t new_cmd_flag = 0;

// 存储 6 轴目标的数组
float target_pos_all[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void UART_Init_Receive(void) {
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

// 串口中断逻辑优化
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_data == '\n' || rx_data == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                new_cmd_flag = 1; // 标记有新指令待处理
                rx_index = 0;
            }
        } else if (rx_index < 63) {
            rx_buffer[rx_index++] = rx_data;
        }
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

/**
 * @brief 解析指令格式：
 * 1. 单轴： "1=25.5" (第1轴移动到25.5mm)
 * 2. 全轴： "all=10,10,10,10,10,10" (6轴同步目标)
 */
void UART_ProcessCommand(void) {
    if (!new_cmd_flag) return;

    // if (strncmp(rx_buffer, "all=", 4) == 0) {
    //     // 解析 6 轴数据
    //     sscanf(rx_buffer + 4, "%f,%f,%f,%f,%f,%f", 
    //            &target_pos_all[0], &target_pos_all[1], &target_pos_all[2],
    //            &target_pos_all[3], &target_pos_all[4], &target_pos_all[5]);
    // } 
    // else if (rx_buffer[1] == '=') {
    //     // 解析单轴数据，例如 "1=20.0"
    //     int id = rx_buffer[0] - '1'; // 转换 1-6 为 0-5
    //     if (id >= 0 && id < 6) {
    //         target_pos_all[id] = atof(rx_buffer + 2);
    //     }
    // }

    if (strncmp(rx_buffer, "kp=", 3) == 0) {
        float kp_val = atof(rx_buffer + 3);
        acts[0].kp_vel = kp_val;
        printf("Updated kp_vel: %.2f\r\n", acts[0].kp_vel);
    } else if (strncmp(rx_buffer, "ki=", 3) == 0) {
        float ki_val = atof(rx_buffer + 3);
        acts[0].ki_vel = ki_val;
        printf("Updated ki_vel: %.2f\r\n", acts[0].ki_vel);
    }

    new_cmd_flag = 0; // 处理完毕，清除标志
    // printf("Target Updated: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\r\n",
    //        target_pos_all[0], target_pos_all[1], target_pos_all[2],
    //        target_pos_all[3], target_pos_all[4], target_pos_all[5]);
}
