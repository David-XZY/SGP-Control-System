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

    // 清空速度缓冲区
    for (int i = 0; i < VEL_WINDOW_SIZE; i++) {
        act->vel_buffer[i] = 0.0f;
    }
    act->vel_idx = 0;
    act->integral_vel = 0.0f;
    act->last_vel_error = 0.0f;
    act->d_term_filt = 0.0f;

    // 初始化 PID 默认参数 (需根据 Stewart 平台实际负载调试)
    act->kp_pos = 6.5f; 
    act->kp_vel = 100.0f; act->ki_vel = 18.0f;
    act->vel_limit = 22.0f; // 限制最大速度 50mm/s
    act->kd_vel = 5.0f;
}

/**
 * @brief 更新执行器的物理状态，并对速度进行窗口平滑处理
 */
void Actuator_UpdateStatus(Actuator_t *act, float feedback_pos) {
    // 1. 更新位置
    act->current_pos = feedback_pos;
    
    // 2. 计算原始瞬时速度 (Raw Velocity)
    float raw_vel = (act->current_pos - act->last_pos) / 0.01f;
    act->last_pos = act->current_pos;

    // 3. 滑动窗口滤波逻辑
    act->vel_buffer[act->vel_idx] = raw_vel;      // 将新速度存入缓冲区
    act->vel_idx = (act->vel_idx + 1) % VEL_WINDOW_SIZE; // 移动索引（环形）

    // 4. 计算窗口内所有样本的平均值
    float vel_sum = 0;
    for (int i = 0; i < VEL_WINDOW_SIZE; i++) {
        vel_sum += act->vel_buffer[i];
    }
    
    // 最终输出平滑后的速度
    act->current_vel = vel_sum / (float)VEL_WINDOW_SIZE;
}
/**
 * @brief 位置闭环控制
 */
void Actuator_PositionControl(Actuator_t *act) {
    float pos_error = act->target_pos - act->current_pos;
    
    // 计算目标速度
    act->target_vel = pos_error * act->kp_pos;
    
    // 速度限幅
    if (act->target_vel > act->vel_limit) act->target_vel = act->vel_limit;
    if (act->target_vel < -act->vel_limit) act->target_vel = -act->vel_limit;

    if (fabs(pos_error) < 0.1f) { 
        act->target_vel = 0.0f;
        act->integral_vel = 0.0f; // 关键：进死区后清空速度环积分，防止“憋大招”
        return;
    }
}

/**
 * @brief 速度闭环控制并驱动硬件
 */
void Actuator_VelocityControl(Actuator_t *act) {
    // 1. 计算速度误差
    float vel_error = act->target_vel - act->current_vel;
    
    // 2. 积分计算与抗饱和限幅
    act->integral_vel += vel_error * 1.0f;
    if (act->integral_vel > 200.0f) act->integral_vel = 200.0f;
    if (act->integral_vel < -200.0f) act->integral_vel = -200.0f;
    
    // 3. 计算控制量 U
    // 1. P 项
    float p_term = vel_error * act->kp_vel;

    // 2. I 项
    float i_term = act->integral_vel * act->ki_vel;

    // 3. D 项 (带简单低通滤波)
    float d_term_raw = (vel_error - act->last_vel_error) / 0.01f;
    act->last_vel_error = vel_error;

    // 低通滤波：新的 D = 0.8 * 旧的 D + 0.2 * 原始计算 D
    // 目的是滤掉编码器量化跳变产生的毛刺
    act->d_term_filt = (act->d_term_filt * 0.8f) + (d_term_raw * 0.2f);
    float d_term = act->d_term_filt * act->kd_vel;

    // 最终输出
    float control_u = p_term + i_term + d_term;

    // 4. 硬件映射逻辑
    uint16_t final_pwm = 0;
    
    // 设定微小控制死区，防止电机在静止时发出滋滋声
    if (fabs(control_u) > 100.0f) {
        // 方向判定
        if (control_u > 0) { // 伸出
            HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_SET);
        } else { // 缩回
            HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_RESET);
        }
        
        // 1200 - 4198 PWM 映射
        float abs_u = fabs(control_u);
        if (abs_u > 2990.0f) abs_u = 2990.0f; 
        final_pwm = (uint16_t)(PWM_MIN + abs_u);
    } else {
        // 停止输出
        HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, GPIO_PIN_RESET);
        final_pwm = 0;
    }

    // 5. 写入定时器
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
