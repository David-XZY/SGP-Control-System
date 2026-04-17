#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 执行器/编码器轴数量 */
#define ACTUATOR_NUM 6

/**
 * @brief 初始化 6 路编码器接口并启动编码器模式
 * @param 无
 * @return 无
 */
void Encoders_Init(void);

/**
 * @brief 读取指定轴编码器原始计数
 * @param axis_idx [in] 轴号（0~5）
 * @return int16_t 原始计数（非法轴号返回 0）
 */
int16_t Encoder_GetRawCount(uint8_t axis_idx);

/**
 * @brief 读取指定轴当前位置（mm）
 * @param axis_idx [in] 轴号（0~5）
 * @return float 位置值（非法轴号返回 0.0）
 */
float Encoder_GetPos_mm(uint8_t axis_idx);
float Encoder_PosMmToCount(uint8_t axis_idx, float pos_mm);

/**
 * @brief 清零指定轴编码器计数
 * @param axis_idx [in] 轴号（0~5）
 * @return 无
 */
void Encoder_ResetPos(uint8_t axis_idx);

#endif
