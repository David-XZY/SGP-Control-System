#include "actuator.h"
#include "encoder.h"   
#include <math.h>      
#include <stdio.h>     
#include <stdlib.h>

#define DIR_WAIT_TIME 100   // 换向死区
#define V_SAMPLE_TIME 10    // 10ms 采样周期
#define PWM_MIN 1200        
#define PWM_MAX 4198        

// 1. 位置环：输入 mm，输出 mm/s
static float Position_Loop(Actuator_t *act) {
    float error = act->target_pos_mm - act->current_pos_mm;
    act->pos_integral += error;
    
    // 积分限幅 (单位: mm*s)
    if (act->pos_integral > act->pos_i_limit) act->pos_integral = act->pos_i_limit;
    else if (act->pos_integral < -act->pos_i_limit) act->pos_integral = -act->pos_i_limit;

    float out_vel = (act->pos_kp * error) + (act->pos_ki * act->pos_integral);

    return out_vel;
}

// 2. 速度环：输入 mm/s，输出 PWM 
static float Velocity_Loop(Actuator_t *act, float target_v) {
    float error = target_v - act->current_vel_mms;
    act->vel_integral += error;

    if (act->vel_integral > act->vel_i_limit) act->vel_integral = act->vel_i_limit;
    else if (act->vel_integral < -act->vel_i_limit) act->vel_integral = -act->vel_i_limit;

    return (act->vel_kp * error) + (act->vel_ki * act->vel_integral);
}

void Actuator_Update(Actuator_t *act) {
    uint32_t now = HAL_GetTick();
    static float pwm_val = 0.0f;

    // --- 物理单位转换层 ---
    // 将脉冲数转换为 mm
    act->current_pos_mm = (float)(*(act->p_count)) / act->pulses_per_mm;

    // 计算速度 (mm/s): (脉冲增量 / 比例) / (时间s)
    if (now - act->last_v_tick >= V_SAMPLE_TIME) {
        float delta_mm = (float)(*(act->p_count) - act->last_count) / act->pulses_per_mm;
        act->current_vel_mms = delta_mm * (1000.0f / (float)V_SAMPLE_TIME);
        act->last_count = *(act->p_count);
        act->last_v_tick = now;
    

        // --- 串级 PID 计算 ---
        act->target_vel_mms = Position_Loop(act);
        pwm_val = Velocity_Loop(act, act->target_vel_mms);
    }

    // --- 智能换向逻辑 (15AS 驱动保护) ---
    uint16_t pwm_out = 0;
    uint8_t req_dir = 0; // 1:伸, 2:缩
    if (act->target_vel_mms > 0.5f) req_dir = 1;
    else if (act->target_vel_mms < -0.5f) req_dir = 2;

    switch (act->state) {
        case ACT_STATE_IDLE:
            if (req_dir != 0) {
                // 如果当前请求方向与上一次运动方向相反，且上一次不是停，则进入换向等待
                if (act->last_move_dir != 0 && req_dir != act->last_move_dir) {
                    act->state = (req_dir == 1) ? ACT_STATE_PRE_EXT : ACT_STATE_PRE_RET;
                    act->last_state_tick = now;
                } else {
                    // 同向或初次启动，直接运动
                    act->state = (req_dir == 1) ? ACT_STATE_EXTENDING : ACT_STATE_RETRACTING;
                }
            }
            break;

        case ACT_STATE_PRE_EXT:
        case ACT_STATE_PRE_RET:
            act->pos_integral = 0; // 等待换向时，不应累积位置积分
            act->vel_integral = 0; // 等待换向时，不应累积速度积分
            if (now - act->last_state_tick >= DIR_WAIT_TIME) {
                act->state = (act->state == ACT_STATE_PRE_EXT) ? ACT_STATE_EXTENDING : ACT_STATE_RETRACTING;
            }
            break;

        case ACT_STATE_EXTENDING:
            if (req_dir != 1) { act->state = ACT_STATE_IDLE; act->last_move_dir = 1; }
            pwm_out = PWM_MIN + (uint16_t)fabs(pwm_val);
            break;

        case ACT_STATE_RETRACTING:
            if (req_dir != 2) { act->state = ACT_STATE_IDLE; act->last_move_dir = 2; }
            pwm_out = PWM_MIN + (uint16_t)fabs(pwm_val);
            break;
    }

    // --- 硬件输出驱动 ---
    if (act->state == ACT_STATE_EXTENDING) {
        HAL_GPIO_WritePin(act->dir1_port, act->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(act->dir2_port, act->dir2_pin, GPIO_PIN_SET);
    } else if (act->state == ACT_STATE_RETRACTING) {
        HAL_GPIO_WritePin(act->dir1_port, act->dir1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(act->dir2_port, act->dir2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(act->dir1_port, act->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(act->dir2_port, act->dir2_pin, GPIO_PIN_RESET);
        pwm_out = 0; // 等待期间禁止 PWM
    }

    if (pwm_out > PWM_MAX) pwm_out = PWM_MAX;
    __HAL_TIM_SET_COMPARE(act->htim, act->channel, pwm_out);
}

void Actuator_Init(Actuator_t *act, 
                  TIM_HandleTypeDef *htim, uint32_t channel,
                  GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                  GPIO_TypeDef *dir2_port, uint16_t dir2_pin,
                  volatile long *counter,
                  float p_kp, float p_ki,
                  float v_kp, float v_ki,
                  float ratio) 
{
    // 1. 硬件资源绑定
    act->htim = htim;
    act->channel = channel;
    act->dir1_port = dir1_port;
    act->dir1_pin = dir1_pin;
    act->dir2_port = dir2_port;
    act->dir2_pin = dir2_pin;
    act->p_count = counter;

    // 2. 物理单位比例
    act->pulses_per_mm = ratio;

    // 3. PID 参数设置（每根杆子可独立传参）
    act->pos_kp = p_kp;
    act->pos_ki = p_ki;
    act->vel_kp = v_kp;
    act->vel_ki = v_ki;
    
    // 4. 积分限幅（通用设置，防止 Stewart 平台机构撞击）
    act->pos_i_limit = 10.0f;  // 允许最大 10mm 的误差积分
    act->vel_i_limit = 500.0f; // 允许最大 500 PWM 的速度修正

    // 5. 状态与时间戳初始化
    act->state = ACT_STATE_IDLE;
    act->last_move_dir = 0;
    act->last_state_tick = 0;
    act->last_v_tick = 0;
    act->last_count = *counter;

    // 6. 目标量初始化
    act->target_pos_mm = (float)(*counter) / ratio; // 初始目标设为当前位置
    act->pos_integral = 0.0f;
    act->vel_integral = 0.0f;
    act->current_vel_mms = 0.0f;

    // 7. 确保初始 PWM 为 0
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

void Actuator_SetTarget(Actuator_t *act, float new_target_mm) {
    act->target_pos_mm = new_target_mm;
    act->pos_integral = 0; // 清空位置环积分，防止目标突变导致的过冲
    act->vel_integral = 0; // 清空速度环积分
}

void Actuator_Home(Actuator_t *act) {
    printf("Homing sequence started...\r\n");
    
    // 1. 强制进入 IDLE 并等待 100ms，确保 15AS 驱动器安全
    act->state = ACT_STATE_IDLE;
    HAL_Delay(100); 

    // 2. 切换到手动缩回模式
    HAL_GPIO_WritePin(act->dir1_port, act->dir1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(act->dir2_port, act->dir2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(act->htim, act->channel, 1600); // 慢速回零占空比

    // 3. 阻塞式或非阻塞式检测（这里以阻塞式示范，方便理解）
    uint32_t start_time = HAL_GetTick();
    long last_c = *(act->p_count);
    
    while(1) {
        HAL_Delay(100);
        // 如果 100ms 内脉冲没有变化，认为撞到了物理底端
        if (*(act->p_count) == last_c) {
            break; 
        }
        last_c = *(act->p_count);
        
        // 超时保护：防止传感器故障导致一直撞击
        if (HAL_GetTick() - start_time > 5000) break; 
    }

    // 4. 关键：坐标重置
    __HAL_TIM_SET_COMPARE(act->htim, act->channel, 0); // 停止
    Encoder_Reset();          // 全局计数清零
    act->last_count = 0;      // 结构体计数同步
    act->target_pos_mm = 0;   // 目标设为 0
    act->pos_integral = 0;    // 清空积分
    act->last_move_dir = 2;   // 标记最后一次是缩回
    
    printf("Homing complete. Encoder reset to 0mm.\r\n");
}
