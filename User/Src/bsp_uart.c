#include "bsp_uart.h"
#include "Actuator.h"
#include "control_manager.h"
#include "encoder.h"
#include "homing_manager.h"
#include "hwt901b.h"
#include "safety_manager.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*
 * 串口命令说明
 * - 每条命令以 \r\n 或 \n 结尾。
 * - 当前仅保留统一的新命令格式；旧版兼容命令已删除。
 * - 轴号统一为 1~6，位置单位为 mm，速度单位为 mm/s，周期单位为 ms。
 * - 返回格式统一为：OK,<类别>,... / ERR,<类别>,<原因> / DATA,<类别>,...
 * - MOVE、TUNE POS、SYS RUN 需要先完成回零，否则返回 NOT_HOMED。
 *
 * =========================
 * 新命令列表（当前可用）
 * =========================
 *
 * 1. 系统命令 SYS
 * SYS RUN
 *   进入闭环运行模式。要求已回零且未急停。
 * SYS IDLE
 *   进入空闲停车模式，控制管理器会下发刹车/零 PWM 命令。
 * SYS ESTOP
 *   软件触发急停锁存。锁存后只能通过复位恢复。
 * SYS RESET
 *   软件复位 MCU。
 * SYS STATUS?
 *   查询系统状态，返回 mode、estop、homed。
 *
 * 2. 回零命令 HOME
 * HOME ALL
 *   启动 6 轴并行回零。回零完成后 homed 标志置位。
 * HOME STOP
 *   中止当前回零流程，并进入空闲/失败相关状态。
 * HOME STATUS?
 *   查询 6 个轴的回零状态，以及 busy、done、failed 标志。
 *
 * 3. 运动命令 MOVE
 * MOVE <axis>,<pos_mm>
 *   设置单轴目标位置，例如 MOVE 1,25.0 表示 1 轴目标位置 25.0mm。
 * MOVE ALL,<p1>,<p2>,<p3>,<p4>,<p5>,<p6>
 *   设置 6 轴目标位置，例如 MOVE ALL,20,20,20,20,20,20。
 * MOVE LEN,<len_mm>
 *   设置 6 轴统一物理长度，会自动扣除各轴回零后的零点长度。
 * MOVE STOP
 *   停止运动并进入 IDLE。
 * MOVE STATUS?
 *   查询 6 轴目标位置、当前位置和当前速度。
 * 注意：MOVE 只设置目标位置，不直接输出 PWM；实际运动需要 SYS RUN 进入闭环运行，
 *       闭环运行中由位置环/速度环 PID 计算输出。
 *
 * 4. 调参命令 TUNE
 * TUNE VEL,<axis>,<vel_mm_s>
 *   单轴速度环调参，例如 TUNE VEL,1,10。
 * TUNE POS,<axis>,<pos_mm>
 *   单轴位置环调参，例如 TUNE POS,1,30。
 * TUNE STOP
 *   停止单轴调参并刹车。
 *
 * 5. 参数配置命令 CFG
 * CFG PID,<axis>,<kpp>,<kpv>,<kiv>,<kdv>,<vlim>
 *   设置单轴完整 PID 和速度限幅。
 * CFG PIDALL,<kpp>,<kpv>,<kiv>,<kdv>,<vlim>
 *   设置全部 6 轴完整 PID 和速度限幅。
 * CFG KPP,<axis>,<value>
 *   设置单轴位置环 P。
 * CFG KPV,<axis>,<value>
 *   设置单轴速度环 P。
 * CFG KIV,<axis>,<value>
 *   设置单轴速度环 I。
 * CFG KDV,<axis>,<value>
 *   设置单轴速度环 D。
 * CFG VLIM,<axis>,<value>
 *   设置单轴速度限幅。内部会走统一限幅逻辑。
 * CFG FF,<axis>,<bias>,<gain>
 *   设置单轴速度前馈死区偏置和前馈增益。
 * CFG PID?,<axis>
 *   查询单轴 PID、速度限幅和前馈参数。
 * CFG ALL?
 *   查询全部 6 轴 PID、速度限幅和前馈参数。
 *
 * 6. 监控输出命令 MON
 * MON OFF
 *   关闭连续监控输出。
 * MON VEL,<axis>,<period_ms>
 *   输出单轴速度调参数据，例如 MON VEL,1,100。
 * MON POS,<axis>,<period_ms>
 *   输出单轴位置调参数据，例如 MON POS,1,100。
 * MON STATUS,<period_ms>
 *   周期输出系统状态，例如 MON STATUS,500。
 *
 * 7. 手动测试命令 TEST
 * TEST OUT,<axis>,EXT,<pwm>,<stop_count>
 *   单轴手动伸出。stop_count=0 表示持续运行，直到收到新命令/急停。
 * TEST OUT,<axis>,RET,<pwm>,<stop_count>
 *   单轴手动缩回。stop_count 为停止计数阈值。
 * TEST STOP
 *   手动测试停车并退出手动测试模式。
 *
 */

#define CMD_POS_MIN_MM 0.0f
#define CMD_POS_MAX_MM 290.0f
#define CMD_AXIS_COUNT 6u

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
    FW_MODE_STATUS
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

static bool axis_from_1based(unsigned int axis, uint8_t *axis_idx) {
    if ((axis < 1u) || (axis > CMD_AXIS_COUNT)) {
        return false;
    }
    *axis_idx = (uint8_t)(axis - 1u);
    return true;
}

static bool pos_in_range(float pos) {
    return (pos >= CMD_POS_MIN_MM) && (pos <= CMD_POS_MAX_MM);
}

static bool parse_six_positions(const char *s, float out[6]) {
    uint8_t i;

    if (parse_six_floats(s, out) != 6) {
        return false;
    }

    for (i = 0u; i < CMD_AXIS_COUNT; i++) {
        if (!pos_in_range(out[i])) {
            return false;
        }
    }
    return true;
}

static void print_axis_pid_data(uint8_t axis) {
    printf("DATA,PID,axis=%u,kp_pos=%.4f,kp_vel=%.4f,ki_vel=%.4f,kd_vel=%.4f,vel_limit=%.4f,ff_bias=%.2f,ff_gain=%.3f\r\n",
           (unsigned)(axis + 1u), acts[axis].kp_pos, acts[axis].kp_vel, acts[axis].ki_vel, acts[axis].kd_vel,
           acts[axis].vel_limit, acts[axis].ff_bias_pwm, acts[axis].ff_gain_pwm_per_vel);
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
    if (((g_fw_mode == FW_MODE_VEL) && (mode != SYS_TUNE_VEL)) || ((g_fw_mode == FW_MODE_POS) && (mode != SYS_TUNE_POS))) {
        return;
    }

    axis = g_fw_axis;
    if (((g_fw_mode == FW_MODE_VEL) || (g_fw_mode == FW_MODE_POS)) && (axis >= CMD_AXIS_COUNT)) {
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
            printf("t_ms,mode,estop,homed\r\n");
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
        printf("%lu,%d,%d,%d\r\n", (unsigned long)now, (int)mode, ControlMgr_IsEstopLatched() ? 1 : 0,
               ControlMgr_IsHomed() ? 1 : 0);
    }
}

/* 执行 TEST 命令 */
static void handle_test_command(uint8_t axis, uint8_t test_mode, uint16_t pwm, int32_t stop_count) {
    int16_t cnt;
    uint8_t dir_in1;
    uint8_t dir_in2;
    const char *mode_name;

    if (axis >= 6u) {
        printf("ERR,TEST,AXIS_RANGE\r\n");
        return;
    }

    if (pwm > (uint16_t)PWM_MAX) {
        pwm = (uint16_t)PWM_MAX;
    }

    if ((test_mode != 1u) && (test_mode != 2u)) {
        printf("ERR,TEST,MODE\r\n");
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
    printf("OK,TEST,%s,axis=%u,pwm=%u,target=%ld\r\n", mode_name, (unsigned)(axis + 1u), (unsigned)pwm,
           (long)stop_count);

    if (stop_count == 0) {
        new_cmd_flag = 0u;
        while (1) {
            if (ControlMgr_IsEstopLatched()) {
                printf("ERR,TEST,ESTOP\r\n");
                break;
            }

            cnt = Encoder_GetRawCount(axis);
            printf("DATA,TEST,axis=%u,cnt=%d\r\n", (unsigned)(axis + 1u), (int)cnt);

            if (new_cmd_flag) {
                break;
            }

            ControlMgr_ManualSetAxisOutput(axis, dir_in1, dir_in2, pwm);
            HAL_Delay(100);
        }

        ControlMgr_ManualBrakeAll();
        printf("DATA,TEST,STOP,axis=%u,cnt=%d\r\n", (unsigned)(axis + 1u), (int)Encoder_GetRawCount(axis));
        return;
    }

    while (1) {
        if (ControlMgr_IsEstopLatched()) {
            printf("ERR,TEST,ESTOP\r\n");
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
    printf("DATA,TEST,REACHED,axis=%u,cnt=%d\r\n", (unsigned)(axis + 1u), (int)Encoder_GetRawCount(axis));
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
    } else if (huart->Instance == USART2) {
        HWT901B_RxCpltCallback(huart);
    }
}

static uint16_t clamp_period_ms(unsigned int period_ms) {
    if (period_ms > 60000u) {
        return 60000u;
    }
    return (uint16_t)period_ms;
}

static void print_home_status(void) {
    const HomeAxisState_e *states = Homing_GetStates();
    printf("DATA,HOME,state=%d,%d,%d,%d,%d,%d,busy=%d,done=%d,failed=%d\r\n", (int)states[0], (int)states[1],
           (int)states[2], (int)states[3], (int)states[4], (int)states[5], Homing_IsBusy() ? 1 : 0,
           Homing_IsDone() ? 1 : 0, Homing_IsFailed() ? 1 : 0);
}

static void print_move_status(void) {
    printf("DATA,MOVE,a1=%.3f/%.3f/%.3f,a2=%.3f/%.3f/%.3f,a3=%.3f/%.3f/%.3f,a4=%.3f/%.3f/%.3f,a5=%.3f/%.3f/%.3f,a6=%.3f/%.3f/%.3f\r\n",
           acts[0].target_pos, acts[0].current_pos, acts[0].current_vel, acts[1].target_pos, acts[1].current_pos,
           acts[1].current_vel, acts[2].target_pos, acts[2].current_pos, acts[2].current_vel, acts[3].target_pos,
           acts[3].current_pos, acts[3].current_vel, acts[4].target_pos, acts[4].current_pos, acts[4].current_vel,
           acts[5].target_pos, acts[5].current_pos, acts[5].current_vel);
}

static void print_all_pid_data(void) {
    uint8_t i;
    for (i = 0u; i < CMD_AXIS_COUNT; i++) {
        print_axis_pid_data(i);
    }
}

static void print_imu_data(void) {
    HWT901B_Data_t imu;

    HWT901B_GetData(&imu);
    printf("DATA,IMU,online=%u,age=%lu,roll=%.3f,pitch=%.3f,yaw=%.3f,gx=%.3f,gy=%.3f,gz=%.3f,ax=%.3f,ay=%.3f,az=%.3f,temp=%.2f,mag=%d/%d/%d,flags=0x%X\r\n",
           (unsigned)imu.online, (unsigned long)(HAL_GetTick() - imu.last_update_ms), imu.angle_deg[0],
           imu.angle_deg[1], imu.angle_deg[2], imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2], imu.acc_g[0],
           imu.acc_g[1], imu.acc_g[2], imu.temperature_c, (int)imu.mag[0], (int)imu.mag[1], (int)imu.mag[2],
           (unsigned)imu.update_flags);
}

static void print_imu_status(void) {
    HWT901B_Data_t imu;

    HWT901B_GetData(&imu);
    printf("DATA,IMU_STATUS,online=%u,last_ms=%lu,age=%lu,flags=0x%X\r\n", (unsigned)imu.online,
           (unsigned long)imu.last_update_ms, (unsigned long)(HAL_GetTick() - imu.last_update_ms),
           (unsigned)imu.update_flags);
}

static void print_imu_result(const char *op, int32_t ret) {
    if (ret == 0) {
        printf("OK,IMU,%s\r\n", op);
    } else {
        printf("ERR,IMU,%s,%ld\r\n", op, (long)ret);
    }
}

static bool process_new_command(const char *cmd) {
    unsigned int axis;
    unsigned int period_ms;
    unsigned int pwm;
    uint8_t axis_idx;
    float v[6];
    float value;
    float kp_pos;
    float kp_vel;
    float ki_vel;
    float kd_vel;
    float vel_lim;

    if (strcmp(cmd, "SYS RUN") == 0) {
        if (ControlMgr_IsEstopLatched()) {
            printf("ERR,SYS,ESTOP\r\n");
        } else if (!ControlMgr_IsHomed()) {
            printf("ERR,SYS,NOT_HOMED\r\n");
        } else {
            ControlMgr_SetRunMode();
            printf("OK,SYS,RUN\r\n");
        }
        return true;
    }
    if (strcmp(cmd, "SYS IDLE") == 0) {
        ControlMgr_SetIdleMode();
        printf("OK,SYS,IDLE\r\n");
        return true;
    }
    if (strcmp(cmd, "SYS ESTOP") == 0) {
        Safety_ForceEstopLatch();
        printf("OK,SYS,ESTOP\r\n");
        return true;
    }
    if (strcmp(cmd, "SYS RESET") == 0) {
        printf("OK,SYS,RESET\r\n");
        HAL_Delay(10);
        ControlMgr_RequestReset();
        return true;
    }
    if (strcmp(cmd, "SYS STATUS?") == 0) {
        printf("DATA,STATUS,mode=%d,estop=%d,homed=%d\r\n", (int)ControlMgr_GetMode(),
               ControlMgr_IsEstopLatched() ? 1 : 0, ControlMgr_IsHomed() ? 1 : 0);
        return true;
    }

    if (strcmp(cmd, "IMU STATUS?") == 0) {
        print_imu_status();
        return true;
    }
    if (strcmp(cmd, "IMU DATA?") == 0) {
        print_imu_data();
        return true;
    }
    if (strcmp(cmd, "IMU CFG BASIC") == 0) {
        print_imu_result("CFG_BASIC", HWT901B_SetBasicOutput());
        return true;
    }
    if (strncmp(cmd, "IMU RATE,", 9) == 0) {
        unsigned int rate_hz;

        if (sscanf(cmd + 9, "%u", &rate_hz) == 1) {
            print_imu_result("RATE", HWT901B_SetOutputRateByHz((uint16_t)rate_hz));
        } else {
            printf("ERR,IMU,RATE_PARAM\r\n");
        }
        return true;
    }
    if (strcmp(cmd, "IMU SAVE") == 0) {
        print_imu_result("SAVE", HWT901B_SaveConfig());
        return true;
    }
    if (strcmp(cmd, "IMU READ ACC") == 0) {
        print_imu_result("READ_ACC", HWT901B_ReadAccReg());
        return true;
    }
    if (strcmp(cmd, "IMU CAL ACC") == 0) {
        print_imu_result("CAL_ACC", HWT901B_StartAccCali());
        return true;
    }
    if (strcmp(cmd, "IMU CAL MAG START") == 0) {
        print_imu_result("CAL_MAG_START", HWT901B_StartMagCali());
        return true;
    }
    if (strcmp(cmd, "IMU CAL MAG STOP") == 0) {
        print_imu_result("CAL_MAG_STOP", HWT901B_StopMagCali());
        return true;
    }

    if (strcmp(cmd, "HOME ALL") == 0) {
        if (ControlMgr_IsEstopLatched()) {
            printf("ERR,HOME,ESTOP\r\n");
        } else {
            ControlMgr_StartHoming();
            printf("OK,HOME,ALL\r\n");
        }
        return true;
    }
    if (strcmp(cmd, "HOME STOP") == 0) {
        ControlMgr_AbortHoming();
        printf("OK,HOME,STOP\r\n");
        return true;
    }
    if (strcmp(cmd, "HOME STATUS?") == 0) {
        print_home_status();
        return true;
    }
    if (strcmp(cmd, "MOVE STOP") == 0) {
        ControlMgr_SetIdleMode();
        printf("OK,MOVE,STOP\r\n");
        return true;
    }
    if (strcmp(cmd, "MOVE STATUS?") == 0) {
        print_move_status();
        return true;
    }
    if (strncmp(cmd, "MOVE LEN,", 9) == 0) {
        if (!ControlMgr_IsHomed()) {
            printf("ERR,MOVE,NOT_HOMED\r\n");
        } else if ((sscanf(cmd + 9, "%f", &value) == 1) && ControlMgr_SetTargetLenAll(value)) {
            printf("OK,MOVE,LEN,%.3f\r\n", value);
        } else {
            printf("ERR,MOVE,RANGE\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "MOVE ALL,", 9) == 0) {
        if (!ControlMgr_IsHomed()) {
            printf("ERR,MOVE,NOT_HOMED\r\n");
        } else if (parse_six_positions(cmd + 9, v)) {
            ControlMgr_SetTargetPosAll(v);
            printf("OK,MOVE,ALL\r\n");
        } else {
            printf("ERR,MOVE,RANGE\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "MOVE ", 5) == 0) {
        if (!ControlMgr_IsHomed()) {
            printf("ERR,MOVE,NOT_HOMED\r\n");
        } else if ((sscanf(cmd + 5, "%u,%f", &axis, &value) == 2) && axis_from_1based(axis, &axis_idx) &&
                   pos_in_range(value)) {
            ControlMgr_SetTargetPosAxis(axis_idx, value);
            printf("OK,MOVE,%u,%.3f\r\n", axis, value);
        } else {
            printf("ERR,MOVE,PARAM\r\n");
        }
        return true;
    }

    if (strncmp(cmd, "TUNE VEL,", 9) == 0) {
        if (ControlMgr_IsEstopLatched()) {
            printf("ERR,TUNE,ESTOP\r\n");
        } else if ((sscanf(cmd + 9, "%u,%f", &axis, &value) == 2) && axis_from_1based(axis, &axis_idx)) {
            ControlMgr_StartTuneVel(axis_idx, value);
            g_fw_axis = axis_idx;
            printf("OK,TUNE,VEL,%u,%.3f\r\n", axis, value);
        } else {
            printf("ERR,TUNE,PARAM\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "TUNE POS,", 9) == 0) {
        if (ControlMgr_IsEstopLatched()) {
            printf("ERR,TUNE,ESTOP\r\n");
        } else if (!ControlMgr_IsHomed()) {
            printf("ERR,TUNE,NOT_HOMED\r\n");
        } else if ((sscanf(cmd + 9, "%u,%f", &axis, &value) == 2) && axis_from_1based(axis, &axis_idx) &&
                   pos_in_range(value)) {
            ControlMgr_StartTunePos(axis_idx, value);
            g_fw_axis = axis_idx;
            printf("OK,TUNE,POS,%u,%.3f\r\n", axis, value);
        } else {
            printf("ERR,TUNE,PARAM\r\n");
        }
        return true;
    }
    if (strcmp(cmd, "TUNE STOP") == 0) {
        ControlMgr_StopTune();
        printf("OK,TUNE,STOP\r\n");
        return true;
    }

    if (strncmp(cmd, "CFG PID?,", 9) == 0) {
        if ((sscanf(cmd + 9, "%u", &axis) == 1) && axis_from_1based(axis, &axis_idx)) {
            print_axis_pid_data(axis_idx);
        } else {
            printf("ERR,CFG,AXIS_RANGE\r\n");
        }
        return true;
    }
    if (strcmp(cmd, "CFG ALL?") == 0) {
        print_all_pid_data();
        return true;
    }
    if (strncmp(cmd, "CFG PIDALL,", 11) == 0) {
        if (sscanf(cmd + 11, "%f,%f,%f,%f,%f", &kp_pos, &kp_vel, &ki_vel, &kd_vel, &vel_lim) == 5) {
            ControlMgr_SetPidAll(kp_pos, kp_vel, ki_vel, kd_vel, vel_lim);
            printf("OK,CFG,PIDALL\r\n");
        } else {
            printf("ERR,CFG,PARAM\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "CFG PID,", 8) == 0) {
        if ((sscanf(cmd + 8, "%u,%f,%f,%f,%f,%f", &axis, &kp_pos, &kp_vel, &ki_vel, &kd_vel, &vel_lim) == 6) &&
            axis_from_1based(axis, &axis_idx)) {
            ControlMgr_SetPidAxis(axis_idx, kp_pos, kp_vel, ki_vel, kd_vel, vel_lim);
            printf("OK,CFG,PID,%u\r\n", axis);
            print_axis_pid_data(axis_idx);
        } else {
            printf("ERR,CFG,PARAM\r\n");
        }
        return true;
    }
    if ((strncmp(cmd, "CFG KPP,", 8) == 0) || (strncmp(cmd, "CFG KPV,", 8) == 0) ||
        (strncmp(cmd, "CFG KIV,", 8) == 0) || (strncmp(cmd, "CFG KDV,", 8) == 0) ||
        (strncmp(cmd, "CFG VLIM,", 9) == 0)) {
        const char *args = (strncmp(cmd, "CFG VLIM,", 9) == 0) ? (cmd + 9) : (cmd + 8);

        if ((sscanf(args, "%u,%f", &axis, &value) == 2) && axis_from_1based(axis, &axis_idx)) {
            kp_pos = acts[axis_idx].kp_pos;
            kp_vel = acts[axis_idx].kp_vel;
            ki_vel = acts[axis_idx].ki_vel;
            kd_vel = acts[axis_idx].kd_vel;
            vel_lim = acts[axis_idx].vel_limit;

            if (strncmp(cmd, "CFG KPP,", 8) == 0) {
                kp_pos = value;
            } else if (strncmp(cmd, "CFG KPV,", 8) == 0) {
                kp_vel = value;
            } else if (strncmp(cmd, "CFG KIV,", 8) == 0) {
                ki_vel = value;
            } else if (strncmp(cmd, "CFG KDV,", 8) == 0) {
                kd_vel = value;
            } else {
                vel_lim = value;
            }

            ControlMgr_SetPidAxis(axis_idx, kp_pos, kp_vel, ki_vel, kd_vel, vel_lim);
            printf("OK,CFG,%u\r\n", axis);
            print_axis_pid_data(axis_idx);
        } else {
            printf("ERR,CFG,PARAM\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "CFG FF,", 7) == 0) {
        float bias;
        float gain;

        if ((sscanf(cmd + 7, "%u,%f,%f", &axis, &bias, &gain) == 3) && axis_from_1based(axis, &axis_idx) &&
            (bias >= 0.0f) && (bias <= PWM_MAX) && (gain >= 0.0f)) {
            acts[axis_idx].ff_bias_pwm = bias;
            acts[axis_idx].ff_gain_pwm_per_vel = gain;
            printf("OK,CFG,FF,%u\r\n", axis);
            print_axis_pid_data(axis_idx);
        } else {
            printf("ERR,CFG,PARAM\r\n");
        }
        return true;
    }

    if (strcmp(cmd, "MON OFF") == 0) {
        fw_disable();
        printf("OK,MON,OFF\r\n");
        return true;
    }
    if (strncmp(cmd, "MON VEL,", 8) == 0) {
        if ((sscanf(cmd + 8, "%u,%u", &axis, &period_ms) == 2) && axis_from_1based(axis, &axis_idx)) {
            g_fw_axis = axis_idx;
            fw_enable(FW_MODE_VEL, clamp_period_ms(period_ms));
            printf("OK,MON,VEL,%u,%u\r\n", axis, (unsigned)g_fw_period_ms);
        } else {
            printf("ERR,MON,PARAM\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "MON POS,", 8) == 0) {
        if ((sscanf(cmd + 8, "%u,%u", &axis, &period_ms) == 2) && axis_from_1based(axis, &axis_idx)) {
            g_fw_axis = axis_idx;
            fw_enable(FW_MODE_POS, clamp_period_ms(period_ms));
            printf("OK,MON,POS,%u,%u\r\n", axis, (unsigned)g_fw_period_ms);
        } else {
            printf("ERR,MON,PARAM\r\n");
        }
        return true;
    }
    if (strncmp(cmd, "MON STATUS,", 11) == 0) {
        if (sscanf(cmd + 11, "%u", &period_ms) == 1) {
            fw_enable(FW_MODE_STATUS, clamp_period_ms(period_ms));
            printf("OK,MON,STATUS,%u\r\n", (unsigned)g_fw_period_ms);
        } else {
            printf("ERR,MON,PARAM\r\n");
        }
        return true;
    }

    if (strcmp(cmd, "TEST STOP") == 0) {
        ControlMgr_ManualBrakeAll();
        ControlMgr_ExitManualTest();
        printf("OK,TEST,STOP\r\n");
        return true;
    }
    if (strncmp(cmd, "TEST OUT,", 9) == 0) {
        char dir[4];
        long stop_count;
        uint8_t test_mode;

        if (ControlMgr_IsEstopLatched()) {
            printf("ERR,TEST,ESTOP\r\n");
        } else if ((sscanf(cmd + 9, "%u,%3[^,],%u,%ld", &axis, dir, &pwm, &stop_count) == 4) &&
                   axis_from_1based(axis, &axis_idx) && ((strcmp(dir, "EXT") == 0) || (strcmp(dir, "RET") == 0))) {
            test_mode = (strcmp(dir, "EXT") == 0) ? 1u : 2u;
            handle_test_command(axis_idx, test_mode, (uint16_t)pwm, (int32_t)stop_count);
        } else {
            printf("ERR,TEST,PARAM\r\n");
        }
        return true;
    }

    return false;
}

/* 解析并执行命令 */
void UART_ProcessCommand(void) {
    char cmd[96];

    fw_stream_tick();

    if (!new_cmd_flag) {
        return;
    }

    strncpy(cmd, rx_buffer, sizeof(cmd) - 1u);
    cmd[sizeof(cmd) - 1u] = '\0';
    new_cmd_flag = 0u;

    if (process_new_command(cmd)) {
        return;
    }

    printf("ERR,UNKNOWN,%s\r\n", cmd);
}
