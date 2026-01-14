#include "actuator.h"
#include "encoder.h"  
#include <stdio.h>
#include <math.h>

void Actuator_Init(Actuator_t *act, TIM_HandleTypeDef *htim, uint32_t ch, 
                  GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2) {
    act->htim_pwm = htim;
    act->pwm_channel = ch;
    act->dir_port_in1 = p1; act->dir_pin_in1 = n1;
    act->dir_port_in2 = p2; act->dir_pin_in2 = n2;
    
    // 初始化 PID 默认参数 (需根据 Stewart 平台实际负载调试)
    act->kp_pos = 0.5f; 
    act->kp_vel = 5.0f; act->ki_vel = 0.01f;
    act->vel_limit = 10.0f; // 限制最大速度 50mm/s
}

void Actuator_ControlLoop(Actuator_t *act, float feedback_pos) {
    // 1. 更新物理状态
    act->current_pos = feedback_pos;
    act->current_vel = (act->current_pos - act->last_pos) / 0.01f; // 10ms 采样
    act->last_pos = act->current_pos;

    // 2. 位置环 (P 控制) -> 输出目标速度
    float pos_error = act->target_pos - act->current_pos;
    float target_vel = pos_error * act->kp_pos;
    
    // 速度限幅
    if (target_vel > act->vel_limit) target_vel = act->vel_limit;
    if (target_vel < -act->vel_limit) target_vel = -act->vel_limit;

    // 3. 速度环 (PI 控制) -> 输出控制量 U
    float vel_error = target_vel - act->current_vel;
    act->integral_vel += vel_error * 0.01f;
    // 积分限幅 (防止过冲)
    if (act->integral_vel > 100.0f) act->integral_vel = 100.0f;
    
    float control_u = (vel_error * act->kp_vel) + (act->integral_vel * act->ki_vel);

    // 4. 方向判定与 15AS 映射
    uint16_t final_pwm = 0;
    if (fabs(control_u) > 0.1f) { // 设定微小死区
        if (control_u > 0) { // 伸出
            HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_SET);
        } else { // 缩回
            HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_RESET);
        }
        
        // 核心：1200-4198 映射计算
        float abs_u = fabs(control_u);
        if (abs_u > 100.0f) abs_u = 100.0f; // 假设 100 为满额控制量
        final_pwm = (uint16_t)(PWM_MIN + (abs_u * (PWM_MAX - PWM_MIN) / 100.0f));
    } else {
        // 停止
        HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_RESET);
        final_pwm = 0;
    }

    // 5. 写入硬件寄存器
    __HAL_TIM_SET_COMPARE(act->htim_pwm, act->pwm_channel, final_pwm);
}

/**
 * @brief 单轴回零函数（阻塞式）
 * @param act: 执行器结构体指针
 * @param axis_idx: 轴索引 (仅用于打印和读取编码器)
 */
void Actuator_ManualHome(Actuator_t *act, uint8_t axis_idx) {
    int16_t last_pulse = 0;
    uint16_t stall_count = 0;

    printf("Axis %d Homing start...\r\n", axis_idx + 1);

    // 1. 设置方向为“缩回” 
    HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_RESET);

    // 2. 输出一个低速安全占空比
    __HAL_TIM_SET_COMPARE(act->htim_pwm, act->pwm_channel, 2200);

    // 3. 循环监测
    while (1) {
        int16_t current_pulse = Encoder_GetRawCount(axis_idx);

        if (current_pulse != last_pulse) {
            stall_count = 0;
            last_pulse = current_pulse;
        } else {
            stall_count++;
        }

        if (stall_count > 20) break; // 约 200ms 不动则停止

        HAL_Delay(10); 
    }

    // 4. 停止电机
    __HAL_TIM_SET_COMPARE(act->htim_pwm, act->pwm_channel, 0);
    HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_RESET);

    // 5. 清空硬件编码器
    Encoder_ResetPos(axis_idx);

    printf("Axis %d Homing Done!\r\n", axis_idx + 1);
}
