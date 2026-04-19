#include "bsp_uart.h"
#include "Actuator.h"
#include "control_manager.h"
#include "encoder.h"
#include "safety_manager.h"
#include "tim.h"
#include <stdbool.h>
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

typedef enum {
    FW_MODE_OFF = 0,
    FW_MODE_VEL,
    FW_MODE_POS,
    FW_MODE_SYNC
} FireWaterMode_e;

static FireWaterMode_e g_fw_mode = FW_MODE_OFF;
static uint32_t g_fw_last_tick = 0u;
static uint16_t g_fw_period_ms = 100u;
static uint8_t g_fw_axis = 0u;
static bool g_fw_header_pending = false;

/* 解析 6 个逗号分隔浮点数，成功返回 6 */
static int parse_six_floats(const char *s, float out[6]) {
    return sscanf(s, "%f,%f,%f,%f,%f,%f", &out[0], &out[1], &out[2], &out[3], &out[4], &out[5]);
}

/* 打印单轴当前 PID 参数 */
static void print_axis_pid(uint8_t axis) {
    printf(
        "PIDCFG,axis=%u,kp_pos=%.4f,kp_vel=%.4f,ki_vel=%.4f,kd_vel=%.4f,vel_limit=%.4f,ff_bias=%.2f,ff_gain=%.3f\r\n",
        (unsigned)(axis + 1u), acts[axis].kp_pos, acts[axis].kp_vel, acts[axis].ki_vel, acts[axis].kd_vel,
        acts[axis].vel_limit, acts[axis].ff_bias_pwm, acts[axis].ff_gain_pwm_per_vel);
}

/* 参数改动后重置速度环内部状态 */
static void reset_axis_pid_state(uint8_t axis) {
    acts[axis].integral_vel = 0.0f;
    acts[axis].last_vel_error = 0.0f;
    acts[axis].d_term_filt = 0.0f;
}

/* 启用 FireWater 连续输出 */
static void fw_enable(FireWaterMode_e mode, uint16_t period_ms) {
    g_fw_mode = mode;
    g_fw_period_ms = (period_ms < 10u) ? 10u : period_ms;
    g_fw_last_tick = 0u;
    g_fw_header_pending = true;
}

/* 关闭 FireWater 连续输出 */
static void fw_disable(void) {
    g_fw_mode = FW_MODE_OFF;
    g_fw_header_pending = false;
}

/* 按配置周期输出 FireWater 数据 */
static void fw_stream_tick(void) {
    uint32_t now;
    SystemMode_e mode;
    uint8_t axis;

    if (g_fw_mode == FW_MODE_OFF) {
        return;
    }

    mode = ControlMgr_GetMode();
    if (((g_fw_mode == FW_MODE_VEL) && (mode != SYS_TUNE_VEL)) || ((g_fw_mode == FW_MODE_POS) && (mode != SYS_TUNE_POS)) ||
        ((g_fw_mode == FW_MODE_SYNC) && !ControlMgr_IsSyncTestActive())) {
        return;
    }

    axis = g_fw_axis;
    if ((g_fw_mode != FW_MODE_SYNC) && (axis >= 6u)) {
        return;
    }

    now = HAL_GetTick();
    if ((g_fw_last_tick != 0u) && ((now - g_fw_last_tick) < g_fw_period_ms)) {
        return;
    }
    g_fw_last_tick = now;

    if (g_fw_header_pending) {
        if (g_fw_mode == FW_MODE_VEL) {
            printf("target_cnt,actual_cnt,target_vel,actual_vel,ff_pwm,pid_u,output_pwm\r\n");
        } else if (g_fw_mode == FW_MODE_POS) {
            printf("target_vel,actual_vel,target_pos,actual_pos\r\n");
        } else {
            printf("t_ms,stage,L_ref,L1,L2,L3,L4,L5,L6,V1,V2,V3,V4,V5,V6,Emax,Spread\r\n");
        }
        g_fw_header_pending = false;
    }

    if (g_fw_mode == FW_MODE_VEL) {
        float target_cnt = Encoder_PosMmToCount(axis, acts[axis].target_pos);
        int16_t actual_cnt = Encoder_GetRawCount(axis);
        printf("%.3f,%d,%.3f,%.3f,%.3f,%.3f,%u\r\n", target_cnt, (int)actual_cnt, acts[axis].target_vel,
               acts[axis].current_vel, acts[axis].dbg_u_ff, acts[axis].dbg_u_pid, (unsigned)acts[axis].cmd_pwm);
    } else if (g_fw_mode == FW_MODE_POS) {
        printf("%.3f,%.3f,%.3f,%.3f\r\n", acts[axis].target_vel, acts[axis].current_vel, acts[axis].target_pos,
               acts[axis].current_pos);
    } else {
        printf("%lu,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
               (unsigned long)now, (unsigned)ControlMgr_GetSyncState(), ControlMgr_GetSyncRefLen(),
               ControlMgr_GetAxisLenMm(0u), ControlMgr_GetAxisLenMm(1u), ControlMgr_GetAxisLenMm(2u),
               ControlMgr_GetAxisLenMm(3u), ControlMgr_GetAxisLenMm(4u), ControlMgr_GetAxisLenMm(5u),
               acts[0].current_vel, acts[1].current_vel, acts[2].current_vel, acts[3].current_vel, acts[4].current_vel,
               acts[5].current_vel, ControlMgr_GetSyncMaxErr(), ControlMgr_GetSyncSpread());
    }
}

/* 执行旧流程阻塞回零（单轴） */
static void handle_old_home(uint8_t axis) {
    if (axis >= 6u) {
        printf("ERR,HOME_OLD_AXIS\r\n");
        return;
    }

    ControlMgr_StopTune();

    printf("ACK,HOME_OLD,axis=%u\r\n", (unsigned)(axis + 1u));

    /* 阻塞回零前停掉周期中断，避免控制管理器覆盖输出 */
    HAL_TIM_Base_Stop_IT(&htim6);
    Actuator_ManualHome(&acts[axis], axis);
    Encoder_ResetPos(axis);
    HAL_TIM_Base_Start_IT(&htim6);

    ControlMgr_SetIdleMode();
    printf("HOME_OLD_DONE,axis=%u,cnt=%d\r\n", (unsigned)(axis + 1u), (int)Encoder_GetRawCount(axis));
}

/* 执行 TEST 命令（保留兼容） */
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
        dir_in1 = 1u;
        dir_in2 = 0u;
        mode_name = "EXT";
    } else {
        dir_in1 = 0u;
        dir_in2 = 1u;
        mode_name = "RET";
    }

    ControlMgr_EnterManualTest();
    ControlMgr_ManualBrakeAll();

    ControlMgr_ManualSetAxisOutput(axis, dir_in1, dir_in2, pwm);
    printf("ACK,TEST,%s,axis=%u,pwm=%u,target=%ld\r\n", mode_name, (unsigned)axis, (unsigned)pwm, (long)stop_count);

    if (stop_count == 0) {
        new_cmd_flag = 0u;
        while (1) {
            if (ControlMgr_IsEstopLatched()) {
                printf("TEST,ESTOP\r\n");
                break;
            }

            cnt = Encoder_GetRawCount(axis);
            printf("TEST,axis=%u,cnt=%d\r\n", (unsigned)axis, (int)cnt);

            if (new_cmd_flag) {
                break;
            }

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

    fw_stream_tick();

    if (!new_cmd_flag) {
        return;
    }

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
    } else if (strncmp(cmd, "HOME_OLD,", 9) == 0) {
        unsigned int axis;
        if (sscanf(cmd + 9, "%u", &axis) == 1 && axis >= 1u && axis <= 6u) {
            handle_old_home((uint8_t)(axis - 1u));
        } else {
            printf("ERR,HOME_OLD\r\n");
        }
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
                print_axis_pid((uint8_t)(axis - 1u));
            } else {
                printf("ERR,PID_AXIS\r\n");
            }
        } else {
            printf("ERR,PID\r\n");
        }
    } else if (strncmp(cmd, "KPP,", 4) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 4, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u) {
            acts[axis - 1u].kp_pos = value;
            reset_axis_pid_state((uint8_t)(axis - 1u));
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,KPP\r\n");
        }
    } else if (strncmp(cmd, "KPV,", 4) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 4, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u) {
            acts[axis - 1u].kp_vel = value;
            reset_axis_pid_state((uint8_t)(axis - 1u));
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,KPV\r\n");
        }
    } else if (strncmp(cmd, "KIV,", 4) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 4, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u) {
            acts[axis - 1u].ki_vel = value;
            reset_axis_pid_state((uint8_t)(axis - 1u));
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,KIV\r\n");
        }
    } else if (strncmp(cmd, "KDV,", 4) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 4, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u) {
            acts[axis - 1u].kd_vel = value;
            reset_axis_pid_state((uint8_t)(axis - 1u));
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,KDV\r\n");
        }
    } else if (strncmp(cmd, "VLIM,", 5) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 5, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u) {
            acts[axis - 1u].vel_limit = value;
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,VLIM\r\n");
        }
    } else if (strncmp(cmd, "FFBIAS,", 7) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 7, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u && value >= 0.0f &&
            value <= PWM_MAX) {
            acts[axis - 1u].ff_bias_pwm = value;
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,FFBIAS\r\n");
        }
    } else if (strncmp(cmd, "FFGAIN,", 7) == 0) {
        unsigned int axis;
        float value;
        if (sscanf(cmd + 7, "%u,%f", &axis, &value) == 2 && axis >= 1u && axis <= 6u && value >= 0.0f) {
            acts[axis - 1u].ff_gain_pwm_per_vel = value;
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,FFGAIN\r\n");
        }
    } else if (strncmp(cmd, "FF,", 3) == 0) {
        unsigned int axis;
        float bias;
        float gain;
        if (sscanf(cmd + 3, "%u,%f,%f", &axis, &bias, &gain) == 3 && axis >= 1u && axis <= 6u && bias >= 0.0f &&
            bias <= PWM_MAX && gain >= 0.0f) {
            acts[axis - 1u].ff_bias_pwm = bias;
            acts[axis - 1u].ff_gain_pwm_per_vel = gain;
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,FF\r\n");
        }
    } else if (strncmp(cmd, "SHOWPID,", 8) == 0) {
        unsigned int axis;
        if (sscanf(cmd + 8, "%u", &axis) == 1 && axis >= 1u && axis <= 6u) {
            print_axis_pid((uint8_t)(axis - 1u));
        } else {
            printf("ERR,SHOWPID\r\n");
        }
    } else if (strncmp(cmd, "TUNEVEL,", 8) == 0) {
        unsigned int axis;
        float target_vel;
        if (sscanf(cmd + 8, "%u,%f", &axis, &target_vel) == 2 && axis >= 1u && axis <= 6u) {
            ControlMgr_StartTuneVel((uint8_t)(axis - 1u), target_vel);
            g_fw_axis = (uint8_t)(axis - 1u);
            if (g_fw_mode == FW_MODE_VEL) {
                g_fw_header_pending = true;
                g_fw_last_tick = 0u;
            }
            printf("ACK,TUNEVEL,axis=%u,target_vel=%.3f\r\n", axis, target_vel);
        } else {
            printf("ERR,TUNEVEL\r\n");
        }
    } else if (strncmp(cmd, "TUNEPOS,", 8) == 0) {
        unsigned int axis;
        float target_pos;
        if (sscanf(cmd + 8, "%u,%f", &axis, &target_pos) == 2 && axis >= 1u && axis <= 6u &&
            target_pos >= 0.0f && target_pos <= 300.0f) {
            ControlMgr_StartTunePos((uint8_t)(axis - 1u), target_pos);
            g_fw_axis = (uint8_t)(axis - 1u);
            if (g_fw_mode == FW_MODE_POS) {
                g_fw_header_pending = true;
                g_fw_last_tick = 0u;
            }
            printf("ACK,TUNEPOS,axis=%u,target_pos=%.3f\r\n", axis, target_pos);
        } else {
            printf("ERR,TUNEPOS\r\n");
        }
    } else if (strcmp(cmd, "TUNESTOP") == 0) {
        ControlMgr_StopTune();
        printf("ACK,TUNESTOP\r\n");
    } else if (strcmp(cmd, "SYNC,HOME40") == 0) {
        if (ControlMgr_StartSyncHome40()) {
            if (g_fw_mode == FW_MODE_SYNC) {
                g_fw_header_pending = true;
                g_fw_last_tick = 0u;
            }
            printf("ACK,SYNC,HOME40\r\n");
        } else {
            printf("ERR,SYNC_HOME40\r\n");
        }
    } else if (strncmp(cmd, "SYNC,EXT,", 9) == 0) {
        float start_len;
        float end_len;
        float speed;
        if (sscanf(cmd + 9, "%f,%f,%f", &start_len, &end_len, &speed) == 3) {
            if (ControlMgr_StartSyncExtend(start_len, end_len, speed)) {
                if (g_fw_mode == FW_MODE_SYNC) {
                    g_fw_header_pending = true;
                    g_fw_last_tick = 0u;
                }
                printf("ACK,SYNC,EXT,start=%.3f,end=%.3f,speed=%.3f\r\n", start_len, end_len, speed);
            } else {
                printf("ERR,SYNC_EXT,range=0..290,speed=0..40,end_gt_start\r\n");
            }
        } else {
            printf("ERR,SYNC_EXT\r\n");
        }
    } else if (strncmp(cmd, "SYNC,SINE,", 10) == 0) {
        float center;
        float amp;
        float freq;
        float cycles;
        if (sscanf(cmd + 10, "%f,%f,%f,%f", &center, &amp, &freq, &cycles) == 4) {
            if (ControlMgr_StartSyncSine(center, amp, freq, cycles)) {
                if (g_fw_mode == FW_MODE_SYNC) {
                    g_fw_header_pending = true;
                    g_fw_last_tick = 0u;
                }
                printf("ACK,SYNC,SINE,center=%.3f,amp=%.3f,freq=%.3f,cycles=%.3f\r\n", center, amp, freq, cycles);
            } else {
                printf("ERR,SYNC_SINE,range=0..290,max_ref_speed=40\r\n");
            }
        } else {
            printf("ERR,SYNC_SINE\r\n");
        }
    } else if (strcmp(cmd, "SYNCSTOP") == 0) {
        ControlMgr_StopSyncTest();
        printf("ACK,SYNCSTOP\r\n");
    } else if (strcmp(cmd, "SYNCSTATUS") == 0) {
        printf("ACK,SYNCSTATUS,mode=%d,state=%u,ref=%.3f,err=%.3f,spread=%.3f\r\n", (int)ControlMgr_GetMode(),
               (unsigned)ControlMgr_GetSyncState(), ControlMgr_GetSyncRefLen(), ControlMgr_GetSyncMaxErr(),
               ControlMgr_GetSyncSpread());
    } else if (strncmp(cmd, "FWVEL,ON,", 9) == 0) {
        unsigned int period_ms;
        if (sscanf(cmd + 9, "%u", &period_ms) == 1) {
            fw_enable(FW_MODE_VEL, (uint16_t)period_ms);
            printf("ACK,FWVEL,ON,%u\r\n", (unsigned)g_fw_period_ms);
        } else {
            printf("ERR,FWVEL\r\n");
        }
    } else if (strncmp(cmd, "FWPOS,ON,", 9) == 0) {
        unsigned int period_ms;
        if (sscanf(cmd + 9, "%u", &period_ms) == 1) {
            fw_enable(FW_MODE_POS, (uint16_t)period_ms);
            printf("ACK,FWPOS,ON,%u\r\n", (unsigned)g_fw_period_ms);
        } else {
            printf("ERR,FWPOS\r\n");
        }
    } else if (strncmp(cmd, "FWSYNC,ON,", 10) == 0) {
        unsigned int period_ms;
        if (sscanf(cmd + 10, "%u", &period_ms) == 1) {
            fw_enable(FW_MODE_SYNC, (uint16_t)period_ms);
            printf("ACK,FWSYNC,ON,%u\r\n", (unsigned)g_fw_period_ms);
        } else {
            printf("ERR,FWSYNC\r\n");
        }
    } else if (strcmp(cmd, "FWOFF") == 0) {
        fw_disable();
        printf("ACK,FWOFF\r\n");
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
