#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 声明全局变量，供其他文件访问 */
extern volatile long actuator_count;

/* 函数声明 */
void Encoder_Reset(void);

#endif /* __ENCODER_H */
