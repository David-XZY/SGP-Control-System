#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "main.h"

#define PWM_MIN 1200.0f  // 电机启动死区
#define PWM_MAX 4198.0f  // PWM 最大值
#define VEL_WINDOW_SIZE  10

typedef struct {
    // 1. 硬件句柄
    TIM_HandleTypeDef *htim_pwm;
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port_in1;
    uint16_t dir_pin_in1;
    GPIO_TypeDef *dir_port_in2;
    uint16_t dir_pin_in2;

    // 2. 物理状态 (单位: mm, mm/s)
    float target_pos;
    float target_vel;
    float current_pos;
    float current_vel;
    float last_pos;

    // 速度平滑相关
    float vel_buffer[VEL_WINDOW_SIZE]; // 速度缓冲区
    uint8_t vel_idx;

    // 3. 串级 PID 参数
    float kp_pos;             // 位置环 P
    float kp_vel, ki_vel;     // 速度环 PI
    float vel_limit;          // 最大允许速度
    float integral_vel;       // 速度环积分累加
    float kd_vel;           // 速度环 D 参数
    float last_vel_error;   // 上一次的速度误差
    float d_term_filt;      // 滤波后的 D 项输出
} Actuator_t;

extern Actuator_t acts[6];

// 函数声明
void Actuator_Init(Actuator_t *act, TIM_HandleTypeDef *htim, uint32_t ch, 
                  GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2);
void Actuator_UpdateStatus(Actuator_t *act, float feedback_pos);
void Actuator_PositionControl(Actuator_t *act);
void Actuator_VelocityControl(Actuator_t *act);
void Actuator_ManualHome(Actuator_t *act, uint8_t axis_idx);

#endif
