#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "main.h"

/* PWM 最大输出（与当前定时器 ARR=4199 对齐，预留 1 个计数） */
#define PWM_MAX 4198.0f
/* 速度滑动平均窗口长度（用于抑制编码器速度抖动） */
#define VEL_WINDOW_SIZE 10

typedef struct {
    /* --- 硬件映射参数 --- */
    TIM_HandleTypeDef *htim_pwm;   /* PWM 输出使用的定时器句柄 */
    uint32_t pwm_channel;          /* PWM 输出通道号 */
    GPIO_TypeDef *dir_port_in1;    /* 方向 IN1 的 GPIO 端口 */
    uint16_t dir_pin_in1;          /* 方向 IN1 的 GPIO 引脚 */
    GPIO_TypeDef *dir_port_in2;    /* 方向 IN2 的 GPIO 端口 */
    uint16_t dir_pin_in2;          /* 方向 IN2 的 GPIO 引脚 */

    /* --- 状态量/目标量（单位：mm, mm/s） --- */
    float target_pos;              /* 目标位置（位置环输入） */
    float target_vel;              /* 目标速度（位置环输出/速度环输入） */
    float current_pos;             /* 当前反馈位置 */
    float current_vel;             /* 当前反馈速度 */
    float last_pos;                /* 上一周期位置（用于差分算速度） */

    /* --- 速度滤波缓存 --- */
    float vel_buffer[VEL_WINDOW_SIZE]; /* 速度滑动窗口缓存 */
    uint8_t vel_idx;                   /* 缓存写入索引（环形） */

    /* --- 控制参数与内部状态 --- */
    float kp_pos;                  /* 位置环 P */
    float kp_vel;                  /* 速度环 P */
    float ki_vel;                  /* 速度环 I */
    float vel_limit;               /* 速度目标限幅（绝对值） */
    float integral_vel;            /* 速度环积分累积 */
    float kd_vel;                  /* 速度环 D */
    float last_vel_error;          /* 上一周期速度误差 */
    float d_term_filt;             /* D 项低通滤波状态 */
    float ff_bias_pwm;             /* 速度前馈偏置（克服死区） */
    float ff_gain_pwm_per_vel;     /* 速度前馈增益（PWM / (mm/s)） */

    /* --- 调试观测量（便于 FireWater 输出） --- */
    float dbg_u_pid;               /* PID 输出（有符号，PWM 等效量） */
    float dbg_u_ff;                /* 前馈输出（有符号，PWM 等效量） */
    float dbg_u_total;             /* 前馈+PID 合成输出（有符号） */

    /* --- 输出命令缓存（先计算，再统一写硬件） --- */
    uint8_t cmd_in1;               /* 方向命令 IN1（0/1） */
    uint8_t cmd_in2;               /* 方向命令 IN2（0/1） */
    uint16_t cmd_pwm;              /* PWM 命令值 */
} Actuator_t;

/* 6 轴执行器全局对象（在 main.c 定义） */
extern Actuator_t acts[6];

/**
 * @brief 初始化单个执行器对象并绑定硬件资源
 * @param act  [in/out] 执行器对象指针
 * @param htim [in] PWM 定时器句柄
 * @param ch   [in] PWM 通道号
 * @param p1   [in] 方向 IN1 端口
 * @param n1   [in] 方向 IN1 引脚
 * @param p2   [in] 方向 IN2 端口
 * @param n2   [in] 方向 IN2 引脚
 * @return 无
 */
void Actuator_Init(Actuator_t *act, TIM_HandleTypeDef *htim, uint32_t ch,
                   GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2);

/**
 * @brief 根据位置反馈更新执行器状态（位置与速度）
 * @param act          [in/out] 执行器对象指针
 * @param feedback_pos [in] 反馈位置（mm）
 * @return 无
 */
void Actuator_UpdateStatus(Actuator_t *act, float feedback_pos);

/**
 * @brief 位置环控制：由位置误差计算目标速度
 * @param act [in/out] 执行器对象指针
 * @return 无
 */
void Actuator_PositionControl(Actuator_t *act);

/**
 * @brief 速度环控制：由速度误差计算方向与 PWM 命令
 * @param act [in/out] 执行器对象指针
 * @return 无
 * @note 本函数只写 cmd_in1/cmd_in2/cmd_pwm，不直接写硬件
 */
void Actuator_VelocityControl(Actuator_t *act);

/**
 * @brief 将方向与 PWM 命令实际写入硬件
 * @param act [in] 执行器对象指针
 * @param in1 [in] IN1 输出（0/1）
 * @param in2 [in] IN2 输出（0/1）
 * @param pwm [in] PWM 比较值
 * @return 无
 */
void Actuator_WriteOutput(Actuator_t *act, uint8_t in1, uint8_t in2, uint16_t pwm);

/**
 * @brief 将执行器命令置为刹车（00 + PWM=0）
 * @param act [in/out] 执行器对象指针
 * @return 无
 */
void Actuator_SetBrakeCommand(Actuator_t *act);

/**
 * @brief 阻塞式单轴手动回零（兼容旧流程）
 * @param act      [in/out] 执行器对象指针
 * @param axis_idx [in] 轴号（0~5）
 * @return 无
 * @warning 含阻塞循环和 HAL_Delay，不建议在新流程中使用
 */
void Actuator_ManualHome(Actuator_t *act, uint8_t axis_idx);

#endif
