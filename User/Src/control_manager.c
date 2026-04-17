#include "control_manager.h"

#include "Actuator.h"
#include "encoder.h"
#include "homing_manager.h"
#include "safety_manager.h"
#include "tim.h"
#include <stdio.h>

#define CTRL_TICK_DT_S 0.01f

typedef struct {
    TIM_HandleTypeDef *tim; /* 该轴 PWM 定时器 */
    uint32_t channel;       /* 该轴 PWM 通道 */
    GPIO_TypeDef *in1_port; /* 该轴方向 IN1 端口 */
    uint16_t in1_pin;       /* 该轴方向 IN1 引脚 */
    GPIO_TypeDef *in2_port; /* 该轴方向 IN2 端口 */
    uint16_t in2_pin;       /* 该轴方向 IN2 引脚 */
} AxisHwMap_t;

/* 6 轴执行器全局对象（定义在 main.c） */
extern Actuator_t acts[CTRL_AXIS_NUM];

/* 轴号到硬件资源映射表 */
static const AxisHwMap_t g_axis_hw[CTRL_AXIS_NUM] = {
    {&htim9, TIM_CHANNEL_1, M1_IN1_GPIO_Port, M1_IN1_Pin, M1_IN2_GPIO_Port, M1_IN2_Pin},
    {&htim9, TIM_CHANNEL_2, M2_IN1_GPIO_Port, M2_IN1_Pin, M2_IN2_GPIO_Port, M2_IN2_Pin},
    {&htim10, TIM_CHANNEL_1, M3_IN1_GPIO_Port, M3_IN1_Pin, M3_IN2_GPIO_Port, M3_IN2_Pin},
    {&htim11, TIM_CHANNEL_1, M4_IN1_GPIO_Port, M4_IN1_Pin, M4_IN2_GPIO_Port, M4_IN2_Pin},
    {&htim12, TIM_CHANNEL_1, M5_IN1_GPIO_Port, M5_IN1_Pin, M5_IN2_GPIO_Port, M5_IN2_Pin},
    {&htim12, TIM_CHANNEL_2, M6_IN1_GPIO_Port, M6_IN1_Pin, M6_IN2_GPIO_Port, M6_IN2_Pin}};

/* 当前系统模式 */
static SystemMode_e g_mode = SYS_BOOT;
/* 状态打印节流时间戳（ms） */
static uint32_t g_last_print_tick = 0u;
/* 各轴回零后对应的物理长度（mm，来自测试数据“初始时实际长度”） */
static const float g_zero_len_mm[CTRL_AXIS_NUM] = {30.0f, 32.0f, 31.0f, 30.0f, 30.0f, 29.5f};
/* 回零完成后的统一目标物理长度（mm） */
#define UNIFIED_START_LEN_MM 40.0f
/* 单轴调参上下文 */
static uint8_t g_tune_axis = 0u;
static float g_tune_target_vel = 0.0f;
static float g_tune_target_pos = 0.0f;

/* 将每轴命令经过安全门控后写入硬件 */
static void apply_outputs_with_safety_gate(void) {
    uint8_t i;
    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        uint8_t out_in1;
        uint8_t out_in2;
        uint16_t out_pwm;

        Safety_GateOutput(acts[i].cmd_in1, acts[i].cmd_in2, acts[i].cmd_pwm, &out_in1, &out_in2, &out_pwm);
        Actuator_WriteOutput(&acts[i], out_in1, out_in2, out_pwm);
    }
}

/* 将所有轴命令置为刹车 */
static void set_all_brake_commands(void) {
    uint8_t i;
    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        Actuator_SetBrakeCommand(&acts[i]);
    }
}

/* 控制管理器初始化 */
void ControlMgr_Init(void) {
    uint8_t i;

    Safety_Init();
    Homing_Init();

    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        Actuator_Init(&acts[i], g_axis_hw[i].tim, g_axis_hw[i].channel, g_axis_hw[i].in1_port, g_axis_hw[i].in1_pin,
                      g_axis_hw[i].in2_port, g_axis_hw[i].in2_pin);
        HAL_TIM_PWM_Start(g_axis_hw[i].tim, g_axis_hw[i].channel);
    }

    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        acts[i].target_pos = Encoder_GetPos_mm(i);
    }

    g_mode = SYS_IDLE;
}

/* 10ms 周期任务：读取反馈、执行控制、统一输出 */
void ControlMgr_Tick10ms(void) {
    uint8_t i;

    Safety_PollKeys_AndUpdate();
    if (Safety_IsEstopLatched()) {
        g_mode = SYS_ESTOP_LATCHED;
    }

    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        Actuator_UpdateStatus(&acts[i], Encoder_GetPos_mm(i));
    }

    switch (g_mode) {
    case SYS_RUN_CLOSED_LOOP:
        for (i = 0; i < CTRL_AXIS_NUM; i++) {
            Actuator_PositionControl(&acts[i]);
            Actuator_VelocityControl(&acts[i]);
        }
        break;

    case SYS_TUNE_VEL:
        for (i = 0; i < CTRL_AXIS_NUM; i++) {
            if (i == g_tune_axis) {
                acts[i].target_vel = g_tune_target_vel;
                acts[i].target_pos += (g_tune_target_vel * CTRL_TICK_DT_S);
                Actuator_VelocityControl(&acts[i]);
            } else {
                Actuator_SetBrakeCommand(&acts[i]);
            }
        }
        break;

    case SYS_TUNE_POS:
        for (i = 0; i < CTRL_AXIS_NUM; i++) {
            if (i == g_tune_axis) {
                acts[i].target_pos = g_tune_target_pos;
                Actuator_PositionControl(&acts[i]);
                Actuator_VelocityControl(&acts[i]);
            } else {
                Actuator_SetBrakeCommand(&acts[i]);
            }
        }
        break;

    case SYS_HOMING:
        Homing_Tick10ms(acts);
        if (Homing_IsFailed()) {
            g_mode = SYS_FAULT;
        } else if (Homing_IsDone()) {
            for (i = 0; i < CTRL_AXIS_NUM; i++) {
                /* 统一到 4cm：目标位移 = 40mm - 各轴回零物理长度 */
                acts[i].target_pos = UNIFIED_START_LEN_MM - g_zero_len_mm[i];
            }
            g_mode = SYS_RUN_CLOSED_LOOP;
        }
        break;

    case SYS_MANUAL_TEST:
        /* 手动测试模式下保持当前 cmd_in/cmd_pwm，不进行闭环覆盖 */
        break;

    case SYS_ESTOP_LATCHED:
    case SYS_FAULT:
    case SYS_IDLE:
    case SYS_BOOT:
    default:
        set_all_brake_commands();
        break;
    }

    apply_outputs_with_safety_gate();
}

/* 主循环低频任务：打印系统状态 */
void ControlMgr_MainTask(void) {
    uint32_t now = HAL_GetTick();

    if ((now - g_last_print_tick) >= 200u) {
        g_last_print_tick = now;
        printf("STAT,%lu,mode=%d,estop=%d,a1(%.2f,%.2f,%.2f,%.2f)\r\n", (unsigned long)now, (int)g_mode,
               Safety_IsEstopLatched() ? 1 : 0, acts[0].target_pos, acts[0].current_pos, acts[0].target_vel,
               acts[0].current_vel);
    }
}

/* 请求切换到闭环运行模式 */
void ControlMgr_SetRunMode(void) {
    if (Safety_IsEstopLatched()) {
        return;
    }
    if ((g_mode == SYS_IDLE) || (g_mode == SYS_RUN_CLOSED_LOOP) || (g_mode == SYS_MANUAL_TEST) ||
        (g_mode == SYS_TUNE_VEL) || (g_mode == SYS_TUNE_POS)) {
        g_mode = SYS_RUN_CLOSED_LOOP;
    }
}

/* 请求切换到空闲模式 */
void ControlMgr_SetIdleMode(void) {
    if (Safety_IsEstopLatched()) {
        return;
    }
    if ((g_mode == SYS_RUN_CLOSED_LOOP) || (g_mode == SYS_HOMING) || (g_mode == SYS_IDLE) || (g_mode == SYS_MANUAL_TEST) ||
        (g_mode == SYS_TUNE_VEL) || (g_mode == SYS_TUNE_POS)) {
        g_mode = SYS_IDLE;
    }
}

/* 启动并行回零 */
void ControlMgr_StartHoming(void) {
    if (Safety_IsEstopLatched()) {
        return;
    }
    Homing_StartParallel();
    g_mode = SYS_HOMING;
}

/* 中止回零 */
void ControlMgr_AbortHoming(void) {
    Homing_Abort();
    g_mode = SYS_IDLE;
}

/* 软件复位 */
void ControlMgr_RequestReset(void) { NVIC_SystemReset(); }

/* 设置所有轴目标位置 */
void ControlMgr_SetTargetPosAll(const float pos[CTRL_AXIS_NUM]) {
    uint8_t i;
    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        acts[i].target_pos = pos[i];
    }
}

/* 设置单轴目标位置 */
void ControlMgr_SetTargetPosAxis(uint8_t axis, float pos) {
    if (axis >= CTRL_AXIS_NUM) {
        return;
    }
    acts[axis].target_pos = pos;
}

/* 设置单轴 PID 参数 */
void ControlMgr_SetPidAxis(uint8_t axis, float kp_pos, float kp_vel, float ki_vel, float kd_vel, float vel_limit) {
    if (axis >= CTRL_AXIS_NUM) {
        return;
    }

    acts[axis].kp_pos = kp_pos;
    acts[axis].kp_vel = kp_vel;
    acts[axis].ki_vel = ki_vel;
    acts[axis].kd_vel = kd_vel;
    acts[axis].vel_limit = vel_limit;
    acts[axis].integral_vel = 0.0f;
    acts[axis].last_vel_error = 0.0f;
    acts[axis].d_term_filt = 0.0f;
}

/* 对全部轴统一设置 PID 参数 */
void ControlMgr_SetPidAll(float kp_pos, float kp_vel, float ki_vel, float kd_vel, float vel_limit) {
    uint8_t i;
    for (i = 0; i < CTRL_AXIS_NUM; i++) {
        ControlMgr_SetPidAxis(i, kp_pos, kp_vel, ki_vel, kd_vel, vel_limit);
    }
}

/* 进入手动测试模式 */
void ControlMgr_EnterManualTest(void) {
    if (Safety_IsEstopLatched()) {
        return;
    }
    set_all_brake_commands();
    g_mode = SYS_MANUAL_TEST;
}

/* 退出手动测试模式 */
void ControlMgr_ExitManualTest(void) {
    if (Safety_IsEstopLatched()) {
        return;
    }
    set_all_brake_commands();
    g_mode = SYS_IDLE;
}

/* 手动设置单轴输出 */
void ControlMgr_ManualSetAxisOutput(uint8_t axis, uint8_t in1, uint8_t in2, uint16_t pwm) {
    if ((axis >= CTRL_AXIS_NUM) || (g_mode != SYS_MANUAL_TEST)) {
        return;
    }
    acts[axis].cmd_in1 = (in1 != 0u) ? 1u : 0u;
    acts[axis].cmd_in2 = (in2 != 0u) ? 1u : 0u;
    acts[axis].cmd_pwm = pwm;
}

/* 手动将全部轴置为刹车 */
void ControlMgr_ManualBrakeAll(void) {
    set_all_brake_commands();
}

/* 进入单轴速度环调参模式 */
void ControlMgr_StartTuneVel(uint8_t axis, float target_vel) {
    if ((axis >= CTRL_AXIS_NUM) || Safety_IsEstopLatched()) {
        return;
    }

    g_tune_axis = axis;
    g_tune_target_vel = target_vel;
    g_tune_target_pos = acts[axis].current_pos;
    acts[axis].target_pos = acts[axis].current_pos;
    acts[axis].target_vel = 0.0f;
    acts[axis].integral_vel = 0.0f;
    acts[axis].last_vel_error = 0.0f;
    acts[axis].d_term_filt = 0.0f;
    set_all_brake_commands();
    g_mode = SYS_TUNE_VEL;
}

/* 进入单轴位置环调参模式 */
void ControlMgr_StartTunePos(uint8_t axis, float target_pos) {
    if ((axis >= CTRL_AXIS_NUM) || Safety_IsEstopLatched()) {
        return;
    }
    if (target_pos < 0.0f) {
        target_pos = 0.0f;
    }
    if (target_pos > 300.0f) {
        target_pos = 300.0f;
    }

    g_tune_axis = axis;
    g_tune_target_pos = target_pos;
    g_tune_target_vel = 0.0f;
    acts[axis].target_pos = target_pos;
    acts[axis].integral_vel = 0.0f;
    acts[axis].last_vel_error = 0.0f;
    acts[axis].d_term_filt = 0.0f;
    set_all_brake_commands();
    g_mode = SYS_TUNE_POS;
}

/* 停止单轴调参模式并刹车 */
void ControlMgr_StopTune(void) {
    set_all_brake_commands();
    if (g_mode == SYS_TUNE_VEL || g_mode == SYS_TUNE_POS) {
        g_mode = SYS_IDLE;
    }
}

/* 获取当前调参轴（0~5） */
uint8_t ControlMgr_GetTuneAxis(void) { return g_tune_axis; }

/* 读取当前系统模式 */
SystemMode_e ControlMgr_GetMode(void) { return g_mode; }

/* 查询急停锁存状态 */
bool ControlMgr_IsEstopLatched(void) { return Safety_IsEstopLatched(); }
