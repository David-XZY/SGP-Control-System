#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

// 外部变量声明
extern volatile long actuator_count;

// 重置编码器计数值
void Encoder_Reset(void);

#endif /* __ENCODER_H */
