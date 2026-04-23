#ifndef __HWT901B_H
#define __HWT901B_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#define HWT901B_UPDATE_ACC   0x01u
#define HWT901B_UPDATE_GYRO  0x02u
#define HWT901B_UPDATE_ANGLE 0x04u
#define HWT901B_UPDATE_MAG   0x08u
#define HWT901B_UPDATE_READ  0x80u

typedef struct {
    float acc_g[3];
    float gyro_dps[3];
    float angle_deg[3];
    int16_t mag[3];
    float temperature_c;
    uint32_t last_update_ms;
    uint16_t update_flags;
    uint8_t online;
} HWT901B_Data_t;

void HWT901B_Init(UART_HandleTypeDef *huart);
void HWT901B_Process(void);
void HWT901B_RxCpltCallback(UART_HandleTypeDef *huart);
void HWT901B_GetData(HWT901B_Data_t *out);
bool HWT901B_IsOnline(void);

int32_t HWT901B_SetBasicOutput(void);
int32_t HWT901B_SetOutputRateByHz(uint16_t hz);
int32_t HWT901B_SaveConfig(void);
int32_t HWT901B_ReadAccReg(void);
int32_t HWT901B_StartAccCali(void);
int32_t HWT901B_StartMagCali(void);
int32_t HWT901B_StopMagCali(void);

#endif
