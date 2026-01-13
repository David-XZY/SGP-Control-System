#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "main.h"

// 电机运动杠状态定义
typedef enum {
    ACT_STATE_IDLE = 0,    // 停止
    ACT_STATE_PRE_EXT,     // 准备伸出（换向刹车）
    ACT_STATE_EXTENDING,   // 伸出中
    ACT_STATE_PRE_RET,     // 准备收回（换向刹车）
    ACT_STATE_RETRACTING,  // 收回中
    ACT_STATE_EMERGENCY    // 紧急停止
} ActuatorState_t;

typedef struct {
    // 硬件通道定义
    TIM_HandleTypeDef *htim;  // PWM定时器
    uint32_t channel;         // PWM通道    
    GPIO_TypeDef *dir1_port;  // 方向控制引脚1端口    
    uint16_t dir1_pin;        // 方向控制引脚1引脚    
    GPIO_TypeDef *dir2_port;  // 方向控制引脚2端口    
    uint16_t dir2_pin;        // 方向控制引脚2引脚    
    
    // 反馈相关参数
    volatile long *p_count;   // 编码器计数指针    
    
    // 状态变量    
    ActuatorState_t state;    // 当前状态    
    uint32_t last_state_tick; // 状态切换时的时间（用于非阻塞延迟）    
    float target_pos;         // 目标位置    
    float current_v;          // 当前速度    
    long last_count;          // 上次计数值（速度计算）    
    uint32_t last_v_tick;    

    // 单位转换比例 (1700.0f / 50.0f = 34.0f pulses/mm)
    float pulses_per_mm;      

    // 物理量状态 (使用 mm 和 mm/s)
    float target_pos_mm;      // 目标位置 (mm)
    float current_pos_mm;     // 当前位置 (mm)
    float target_vel_mms;     // 目标速度 (mm/s)
    float current_vel_mms;    // 当前速度 (mm/s)
    
    // 方向记录
    uint8_t last_move_dir;    // 0:停止, 1:伸出, 2:缩回
    
    // 控制参数
    // 位置环 PI
    float pos_kp, pos_ki, pos_integral, pos_i_limit;
    
    // 速度环 PI 
    float vel_kp, vel_ki, vel_integral, vel_i_limit;

    float target_vel;         // 位置环的输出，速度环的目标
} Actuator_t;

// 函数声明
void Actuator_Init(Actuator_t *act, 
                  TIM_HandleTypeDef *htim, uint32_t channel,
                  GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                  GPIO_TypeDef *dir2_port, uint16_t dir2_pin,
                  volatile long *counter,
                  float p_kp, float p_ki,
                  float v_kp, float v_ki,
                  float ratio);

void Actuator_Update(Actuator_t *act); 
void Actuator_SetTarget(Actuator_t *act, float new_target_mm);
void Actuator_Home(Actuator_t *act); 

#endif
