#include "encoder.h"
#include "tim.h"

// 每根杆子的脉冲/毫米比例数组
// 1700脉冲 = 50mm -> 34.0
static const float PULSE_RATIO[ACTUATOR_NUM] = {
    33.8f, 1700.0f/50.0f, 1700.0f/50.0f, 
    1700.0f/50.0f, 1700.0f/50.0f, 1700.0f/50.0f
};

// 硬件定时器映射数组
static TIM_HandleTypeDef* const ENCODER_TIMS[ACTUATOR_NUM] = {
    &htim1, &htim2, &htim3, &htim4, &htim5, &htim8
};

void Encoders_Init(void) {
    for(int i = 0; i < ACTUATOR_NUM; i++) {
        // 开启硬件编码器接口
        HAL_TIM_Encoder_Start(ENCODER_TIMS[i], TIM_CHANNEL_ALL);
    }
}

int16_t Encoder_GetRawCount(uint8_t axis_idx) {
    if (axis_idx >= ACTUATOR_NUM) return 0;
    
    // 读取 CNT 寄存器并强制转换为 int16_t 以处理 0/65535 翻转
    return (int16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIMS[axis_idx]);
}

float Encoder_GetPos_mm(uint8_t axis_idx) {
    if (axis_idx >= ACTUATOR_NUM) return 0.0f;
    
    // 直接基于原始脉冲和对应比例计算物理量
    return (float)Encoder_GetRawCount(axis_idx) / PULSE_RATIO[axis_idx];
}

void Encoder_ResetPos(uint8_t axis_idx) {
    if (axis_idx < ACTUATOR_NUM) {
        __HAL_TIM_SET_COUNTER(ENCODER_TIMS[axis_idx], 0);
    }
}
