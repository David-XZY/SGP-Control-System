#include "bsp_uart.h"
#include "Actuator.h"
#include "control_manager.h"
#include "encoder.h"
#include "safety_manager.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USART1 中断接收的当前字节 */
uint8_t rx_data;
/* 命令行缓存（以 \n 或 \r 结尾） */
char rx_buffer[96];
/* 写入索引 */
static uint8_t rx_index = 0u;
/* 新命令就绪标志 */
volatile uint8_t new_cmd_flag = 0u;

/* 解析 6 个逗号分隔浮点数，成功返回 6 */
static int parse_six_floats(const char *s, float out[6]) {
    return sscanf(s, "%f,%f,%f,%f,%f,%f", &out[0], &out[1], &out[2], &out[3], &out[4], &out[5]);
}

/* 执行 TEST 命令 */
static void handle_test_command(uint8_t axis, uint8_t test_mode, uint16_t pwm, int32_t stop_count) {
    int16_t cnt;
    uint8_t dir_in1;
    uint8_t dir_in2;
    const char *mode_name;

    if (axis >= 6u) {
        printf("ERR,TEST_AXIS\r\n");
        return;
    }

    if (pwm > (uint16_t)PWM_MAX) {
        pwm = (uint16_t)PWM_MAX;
    }

    if (test_mode == 0u) {
        printf("ACK,TEST,HOME,axis=%u\r\n", (unsigned)axis);

        /* 阻塞回零前停掉周期中断，避免控制管理器覆盖输出 */
        HAL_TIM_Base_Stop_IT(&htim6);
        Actuator_ManualHome(&acts[axis], axis);
        Encoder_ResetPos(axis);
        HAL_TIM_Base_Start_IT(&htim6);

        ControlMgr_EnterManualTest();
        ControlMgr_ManualBrakeAll();
        printf("TEST,HOME_DONE,axis=%u,cnt=%d\r\n", (unsigned)axis, (int)Encoder_GetRawCount(axis));
        return;
    }

    if ((test_mode != 1u) && (test_mode != 2u)) {
        printf("ERR,TEST_MODE\r\n");
        return;
    }

    if (test_mode == 1u) {
        /* 伸长方向：IN1=1, IN2=0 */
        dir_in1 = 1u;
        dir_in2 = 0u;
        mode_name = "EXT";
    } else {
        /* 缩回方向：IN1=0, IN2=1 */
        dir_in1 = 0u;
        dir_in2 = 1u;
        mode_name = "RET";
    }

    ControlMgr_EnterManualTest();
    ControlMgr_ManualBrakeAll();

    ControlMgr_ManualSetAxisOutput(axis, dir_in1, dir_in2, pwm);
    printf("ACK,TEST,%s,axis=%u,pwm=%u,target=%ld\r\n", mode_name, (unsigned)axis, (unsigned)pwm, (long)stop_count);

    if (stop_count == 0) {
        /* 清掉当前命令标志，允许后续新命令打断该循环 */
        new_cmd_flag = 0u;
        while (1) {
            if (ControlMgr_IsEstopLatched()) {
                printf("TEST,ESTOP\r\n");
                break;
            }

            cnt = Encoder_GetRawCount(axis);
            printf("TEST,axis=%u,cnt=%d\r\n", (unsigned)axis, (int)cnt);

            /* 持续打印模式下，收到新命令则退出，让主循环处理新命令 */
            if (new_cmd_flag) {
                break;
            }

            /* 周期重发手动输出，保证占空比保持 */
            ControlMgr_ManualSetAxisOutput(axis, dir_in1, dir_in2, pwm);
            HAL_Delay(100);
        }

        ControlMgr_ManualBrakeAll();
        printf("TEST,STOP,axis=%u,cnt=%d\r\n", (unsigned)axis, (int)Encoder_GetRawCount(axis));
        return;
    }

    while (1) {
        if (ControlMgr_IsEstopLatched()) {
            printf("TEST,ESTOP\r\n");
            break;
        }

        cnt = Encoder_GetRawCount(axis);

        if (((stop_count >= 0) && (cnt >= (int16_t)stop_count)) || ((stop_count < 0) && (cnt <= (int16_t)stop_count))) {
            break;
        }

        if (new_cmd_flag) {
            break;
        }

        ControlMgr_ManualSetAxisOutput(axis, dir_in1, dir_in2, pwm);
        HAL_Delay(5);
    }

    ControlMgr_ManualBrakeAll();
    HAL_Delay(50);
    printf("TEST,REACHED,axis=%u,cnt=%d\r\n", (unsigned)axis, (int)Encoder_GetRawCount(axis));
}

/* 初始化中断接收 */
void UART_Init_Receive(void) {
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

/* UART 接收完成回调：按行组包 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if ((rx_data == '\n') || (rx_data == '\r')) {
            if (rx_index > 0u) {
                rx_buffer[rx_index] = '\0';
                new_cmd_flag = 1u;
                rx_index = 0u;
            }
        } else if (rx_index < (sizeof(rx_buffer) - 1u)) {
            rx_buffer[rx_index++] = (char)rx_data;
        }

        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

/* 解析并执行命令 */
void UART_ProcessCommand(void) {
    char cmd[96];
    float v[6];

    if (!new_cmd_flag) {
        return;
    }

    /* 先取走当前命令并清标志，避免长处理过程覆盖后续新命令 */
    strncpy(cmd, rx_buffer, sizeof(cmd) - 1u);
    cmd[sizeof(cmd) - 1u] = '\0';
    new_cmd_flag = 0u;

    if (strcmp(cmd, "RUN") == 0) {
        ControlMgr_SetRunMode();
        printf("ACK,RUN\r\n");
    } else if (strcmp(cmd, "IDLE") == 0) {
        ControlMgr_SetIdleMode();
        printf("ACK,IDLE\r\n");
    } else if (strcmp(cmd, "HOME") == 0) {
        ControlMgr_StartHoming();
        printf("ACK,HOME\r\n");
    } else if (strcmp(cmd, "ABORT_HOME") == 0) {
        ControlMgr_AbortHoming();
        printf("ACK,ABORT_HOME\r\n");
    } else if (strcmp(cmd, "ESTOP") == 0) {
        Safety_ForceEstopLatch();
        printf("ACK,ESTOP\r\n");
    } else if (strcmp(cmd, "RESET") == 0) {
        printf("ACK,RESET\r\n");
        HAL_Delay(10);
        ControlMgr_RequestReset();
    } else if (strncmp(cmd, "SETPOS,", 7) == 0) {
        if (parse_six_floats(cmd + 7, v) == 6) {
            ControlMgr_SetTargetPosAll(v);
            printf("ACK,SETPOS\r\n");
        } else {
            printf("ERR,SETPOS\r\n");
        }
    } else if (strncmp(cmd, "PIDALL,", 7) == 0) {
        float kp_pos, kp_vel, ki_vel, kd_vel, vel_lim;
        if (sscanf(cmd + 7, "%f,%f,%f,%f,%f", &kp_pos, &kp_vel, &ki_vel, &kd_vel, &vel_lim) == 5) {
            ControlMgr_SetPidAll(kp_pos, kp_vel, ki_vel, kd_vel, vel_lim);
            printf("ACK,PIDALL\r\n");
        } else {
            printf("ERR,PIDALL\r\n");
        }
    } else if (strncmp(cmd, "PID,", 4) == 0) {
        unsigned int axis;
        float kp_pos, kp_vel, ki_vel, kd_vel, vel_lim;
        if (sscanf(cmd + 4, "%u,%f,%f,%f,%f,%f", &axis, &kp_pos, &kp_vel, &ki_vel, &kd_vel, &vel_lim) == 6) {
            if ((axis >= 1u) && (axis <= 6u)) {
                ControlMgr_SetPidAxis((uint8_t)(axis - 1u), kp_pos, kp_vel, ki_vel, kd_vel, vel_lim);
                printf("ACK,PID,%u\r\n", axis);
            } else {
                printf("ERR,PID_AXIS\r\n");
            }
        } else {
            printf("ERR,PID\r\n");
        }
    } else if (strncmp(cmd, "SETPOS1,", 8) == 0) {
        ControlMgr_SetTargetPosAxis(0u, (float)atof(cmd + 8));
        printf("ACK,SETPOS1\r\n");
    } else if (strncmp(cmd, "TEST,", 5) == 0) {
        unsigned int axis;
        unsigned int mode;
        unsigned int pwm;
        long stop_count;

        if (sscanf(cmd + 5, "%u,%u,%u,%ld", &axis, &mode, &pwm, &stop_count) == 4) {
            handle_test_command((uint8_t)axis, (uint8_t)mode, (uint16_t)pwm, (int32_t)stop_count);
        } else {
            printf("ERR,TEST\r\n");
        }
    } else if (strcmp(cmd, "STATUS") == 0) {
        printf("ACK,STATUS,mode=%d,estop=%d\r\n", (int)ControlMgr_GetMode(), ControlMgr_IsEstopLatched() ? 1 : 0);
    } else {
        printf("ERR,UNKNOWN,%s\r\n", cmd);
    }
}
