#include "encoder.h"
#include "tim.h"

/* 每轴线性标定参数：count = k * pos_mm + b */
static const float PULSE_PER_MM[ACTUATOR_NUM] = {
    16.512f, 16.341f, 16.559f, 16.700f, 16.529f, 16.476f
};
static const float COUNT_BIAS[ACTUATOR_NUM] = {
    -14.88f, 8.22f, 16.73f, -54.13f, -2.87f, 2.17f
};

/* 轴号到编码器定时器句柄映射表 */
static TIM_HandleTypeDef *const ENCODER_TIMS[ACTUATOR_NUM] = {
    &htim1, &htim2, &htim3, &htim4, &htim5, &htim8
};

/* 启动 6 路编码器接口 */
void Encoders_Init(void) {
    for (int i = 0; i < ACTUATOR_NUM; i++) {
        HAL_TIM_Encoder_Start(ENCODER_TIMS[i], TIM_CHANNEL_ALL);
    }
}

/* 读取原始编码器计数 */
int16_t Encoder_GetRawCount(uint8_t axis_idx) {
    if (axis_idx >= ACTUATOR_NUM) {
        return 0;
    }
    return (int16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIMS[axis_idx]);
}

/* 读取位置（mm） */
float Encoder_GetPos_mm(uint8_t axis_idx) {
    float count;

    if (axis_idx >= ACTUATOR_NUM) {
        return 0.0f;
    }
    count = (float)Encoder_GetRawCount(axis_idx);
    return (count - COUNT_BIAS[axis_idx]) / PULSE_PER_MM[axis_idx];
}

/* 将物理位置（mm）换算为目标脉冲计数 */
float Encoder_PosMmToCount(uint8_t axis_idx, float pos_mm) {
    if (axis_idx >= ACTUATOR_NUM) {
        return 0.0f;
    }
    return (pos_mm * PULSE_PER_MM[axis_idx]) + COUNT_BIAS[axis_idx];
}

/* 清零指定轴编码器计数 */
void Encoder_ResetPos(uint8_t axis_idx) {
    if (axis_idx < ACTUATOR_NUM) {
        __HAL_TIM_SET_COUNTER(ENCODER_TIMS[axis_idx], 0);
    }
}
