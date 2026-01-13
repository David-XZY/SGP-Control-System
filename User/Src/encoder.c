#include "encoder.h"

// 全局变量定义
volatile long actuator_count = 0;
static uint8_t encode_state = 0;

// 重置编码器计数值
void Encoder_Reset(void) {
    actuator_count = 0;
}

// 外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Hall_A_Pin || GPIO_Pin == Hall_B_Pin) {
        uint8_t current_A = (HAL_GPIO_ReadPin(Hall_A_GPIO_Port, Hall_A_Pin) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t current_B = (HAL_GPIO_ReadPin(Hall_B_GPIO_Port, Hall_B_Pin) == GPIO_PIN_SET) ? 1 : 0;
        
        uint8_t new_state = (current_A << 1) | current_B;
        uint8_t state_trans = (encode_state << 2) | new_state;
        
        switch (state_trans) {
            case 0x1: case 0x7: case 0xE: case 0x8: 
                actuator_count--; 
                break;
            case 0x2: case 0xB: case 0xD: case 0x4: 
                actuator_count++; 
                break;
            default: 
                break;
        }
        encode_state = new_state;
    }
}
