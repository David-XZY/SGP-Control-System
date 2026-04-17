#include "Actuator.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>

/* 控制周期（秒），与 TIM6 的 10ms 中断一致 */
#define CONTROL_DT_S 0.01f

/* 初始化执行器对象并绑定硬件资源 */
void Actuator_Init(Actuator_t *act, TIM_HandleTypeDef *htim, uint32_t ch,
                   GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2) {
    int i;

    act->htim_pwm = htim;
    act->pwm_channel = ch;
    act->dir_port_in1 = p1;
    act->dir_pin_in1 = n1;
    act->dir_port_in2 = p2;
    act->dir_pin_in2 = n2;

    act->target_pos = 0.0f;
    act->target_vel = 0.0f;
    act->current_pos = 0.0f;
    act->current_vel = 0.0f;
    act->last_pos = 0.0f;

    for (i = 0; i < VEL_WINDOW_SIZE; i++) {
        act->vel_buffer[i] = 0.0f;
    }
    act->vel_idx = 0;

    act->kp_pos = 0.0f;
    act->kp_vel = 0.0f;
    act->ki_vel = 0.0f;
    act->kd_vel = 0.0f;
    act->vel_limit = 0.0f;
    act->integral_vel = 0.0f;
    act->last_vel_error = 0.0f;
    act->d_term_filt = 0.0f;

    act->cmd_in1 = 0u;
    act->cmd_in2 = 0u;
    act->cmd_pwm = 0u;
}

/* 根据位置反馈更新状态量（位置、速度） */
void Actuator_UpdateStatus(Actuator_t *act, float feedback_pos) {
    int i;
    float vel_sum;
    float raw_vel;

    act->current_pos = feedback_pos;

    raw_vel = (act->current_pos - act->last_pos) / CONTROL_DT_S;
    act->last_pos = act->current_pos;

    act->vel_buffer[act->vel_idx] = raw_vel;
    act->vel_idx = (uint8_t)((act->vel_idx + 1u) % VEL_WINDOW_SIZE);

    vel_sum = 0.0f;
    for (i = 0; i < VEL_WINDOW_SIZE; i++) {
        vel_sum += act->vel_buffer[i];
    }

    act->current_vel = vel_sum / (float)VEL_WINDOW_SIZE;
}

/* 位置环：由位置误差生成目标速度并限幅 */
void Actuator_PositionControl(Actuator_t *act) {
    float pos_error = act->target_pos - act->current_pos;

    act->target_vel = pos_error * act->kp_pos;

    if (act->target_vel > act->vel_limit) {
        act->target_vel = act->vel_limit;
    }
    if (act->target_vel < -act->vel_limit) {
        act->target_vel = -act->vel_limit;
    }

    if (fabsf(pos_error) < 0.1f) {
        act->target_vel = 0.0f;
        act->integral_vel = 0.0f;
    }
}

/* 速度环：计算方向与 PWM 命令（仅写命令缓存） */
void Actuator_VelocityControl(Actuator_t *act) {
    float vel_error;
    float p_term;
    float i_term;
    float d_term_raw;
    float d_term;
    float control_u;
    float abs_u;

    vel_error = act->target_vel - act->current_vel;

    act->integral_vel += vel_error * CONTROL_DT_S;
    if (act->integral_vel > 200.0f) {
        act->integral_vel = 200.0f;
    }
    if (act->integral_vel < -200.0f) {
        act->integral_vel = -200.0f;
    }

    p_term = vel_error * act->kp_vel;
    i_term = act->integral_vel * act->ki_vel;

    d_term_raw = (vel_error - act->last_vel_error) / CONTROL_DT_S;
    act->last_vel_error = vel_error;

    act->d_term_filt = (act->d_term_filt * 0.8f) + (d_term_raw * 0.2f);
    d_term = act->d_term_filt * act->kd_vel;

    control_u = p_term + i_term + d_term;

    if (fabsf(control_u) > 100.0f) {
        if (control_u > 0.0f) {
            act->cmd_in1 = 1u;
            act->cmd_in2 = 0u;
        } else {
            act->cmd_in1 = 0u;
            act->cmd_in2 = 1u;
        }

        abs_u = fabsf(control_u);
        if (abs_u > (PWM_MAX - PWM_MIN)) {
            abs_u = (PWM_MAX - PWM_MIN);
        }
        act->cmd_pwm = (uint16_t)(PWM_MIN + abs_u);
    } else {
        Actuator_SetBrakeCommand(act);
    }
}

/* 将命令真正写入硬件 */
void Actuator_WriteOutput(Actuator_t *act, uint8_t in1, uint8_t in2, uint16_t pwm) {
    HAL_GPIO_WritePin(act->dir_port_in1, act->dir_pin_in1, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(act->dir_port_in2, act->dir_pin_in2, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(act->htim_pwm, act->pwm_channel, pwm);
}

/* 将命令设置为停车/刹车（00 + PWM=0） */
void Actuator_SetBrakeCommand(Actuator_t *act) {
    act->cmd_in1 = 0u;
    act->cmd_in2 = 0u;
    act->cmd_pwm = 0u;
}

/* 阻塞式单轴回零（保留旧接口） */
void Actuator_ManualHome(Actuator_t *act, uint8_t axis_idx) {
    int16_t last_pulse = 0;
    uint16_t stall_count = 0;

    printf("Axis %d Homing start...\r\n", axis_idx + 1u);

    Actuator_WriteOutput(act, 0u, 1u, 3500u);

    while (1) {
        int16_t current_pulse = Encoder_GetRawCount(axis_idx);

        if (current_pulse != last_pulse) {
            stall_count = 0;
            last_pulse = current_pulse;
        } else {
            stall_count++;
        }

        if (stall_count > 20u) {
            break;
        }

        HAL_Delay(10);
    }

    Actuator_WriteOutput(act, 0u, 0u, 0u);
    Encoder_ResetPos(axis_idx);

    printf("Axis %d Homing Done!\r\n", axis_idx + 1u);
}
