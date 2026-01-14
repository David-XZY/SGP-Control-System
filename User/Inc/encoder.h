#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#define ACTUATOR_NUM 6

// 1. 初始化 6 路硬件编码器接口
void Encoders_Init(void);

// 2. 获取原始脉冲计数 (int16_t 处理过零翻转)
int16_t Encoder_GetRawCount(uint8_t axis_idx);

// 3. 获取物理位置 (mm)
float Encoder_GetPos_mm(uint8_t axis_idx);

// 4. 重置特定轴的零点
void Encoder_ResetPos(uint8_t axis_idx);

#endif
