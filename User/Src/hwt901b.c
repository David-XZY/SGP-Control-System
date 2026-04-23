#include "hwt901b.h"

#include "wit_c_sdk.h"
#include <string.h>

#define HWT901B_ONLINE_TIMEOUT_MS 500u

static UART_HandleTypeDef *s_hwt_uart = NULL;
static uint8_t s_hwt_rx_byte = 0u;
static volatile uint16_t s_hwt_update_flags = 0u;
static HWT901B_Data_t s_hwt_data;

static void HWT901B_Send(uint8_t *data, uint32_t len) {
    if ((s_hwt_uart == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }

    (void)HAL_UART_Transmit(s_hwt_uart, data, (uint16_t)len, 20);
}

static void HWT901B_DelayMs(uint16_t ms) {
    HAL_Delay(ms);
}

static void HWT901B_MarkUpdated(uint32_t reg, uint32_t reg_num) {
    uint32_t i;

    for (i = 0u; i < reg_num; i++) {
        switch (reg) {
        case AZ:
            s_hwt_update_flags |= HWT901B_UPDATE_ACC;
            break;
        case GZ:
            s_hwt_update_flags |= HWT901B_UPDATE_GYRO;
            break;
        case HZ:
            s_hwt_update_flags |= HWT901B_UPDATE_MAG;
            break;
        case Yaw:
            s_hwt_update_flags |= HWT901B_UPDATE_ANGLE;
            break;
        default:
            s_hwt_update_flags |= HWT901B_UPDATE_READ;
            break;
        }
        reg++;
    }
}

static void HWT901B_UpdateSnapshot(uint16_t flags) {
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        s_hwt_data.acc_g[i] = (float)sReg[AX + i] / 32768.0f * 16.0f;
        s_hwt_data.gyro_dps[i] = (float)sReg[GX + i] / 32768.0f * 2000.0f;
        s_hwt_data.angle_deg[i] = (float)sReg[Roll + i] / 32768.0f * 180.0f;
        s_hwt_data.mag[i] = sReg[HX + i];
    }

    s_hwt_data.temperature_c = (float)sReg[TEMP] / 100.0f;
    s_hwt_data.update_flags |= flags;
    s_hwt_data.last_update_ms = HAL_GetTick();
    s_hwt_data.online = 1u;
}

void HWT901B_Init(UART_HandleTypeDef *huart) {
    s_hwt_uart = huart;
    memset(&s_hwt_data, 0, sizeof(s_hwt_data));
    s_hwt_update_flags = 0u;

    (void)WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    (void)WitSerialWriteRegister(HWT901B_Send);
    (void)WitRegisterCallBack(HWT901B_MarkUpdated);
    (void)WitDelayMsRegister(HWT901B_DelayMs);

    if (s_hwt_uart != NULL) {
        (void)HAL_UART_Receive_IT(s_hwt_uart, &s_hwt_rx_byte, 1u);
    }
}

void HWT901B_Process(void) {
    uint16_t flags;
    uint32_t now;

    __disable_irq();
    flags = s_hwt_update_flags;
    s_hwt_update_flags = 0u;
    __enable_irq();

    if (flags != 0u) {
        HWT901B_UpdateSnapshot(flags);
    }

    now = HAL_GetTick();
    if ((s_hwt_data.last_update_ms == 0u) || ((now - s_hwt_data.last_update_ms) > HWT901B_ONLINE_TIMEOUT_MS)) {
        s_hwt_data.online = 0u;
    }
}

void HWT901B_RxCpltCallback(UART_HandleTypeDef *huart) {
    if ((s_hwt_uart == NULL) || (huart->Instance != s_hwt_uart->Instance)) {
        return;
    }

    WitSerialDataIn(s_hwt_rx_byte);
    (void)HAL_UART_Receive_IT(s_hwt_uart, &s_hwt_rx_byte, 1u);
}

void HWT901B_GetData(HWT901B_Data_t *out) {
    if (out == NULL) {
        return;
    }

    __disable_irq();
    *out = s_hwt_data;
    __enable_irq();
}

bool HWT901B_IsOnline(void) {
    return s_hwt_data.online != 0u;
}

int32_t HWT901B_SetBasicOutput(void) {
    return WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
}

int32_t HWT901B_SetOutputRateByHz(uint16_t hz) {
    int32_t rate;

    switch (hz) {
    case 1u:
        rate = RRATE_1HZ;
        break;
    case 2u:
        rate = RRATE_2HZ;
        break;
    case 5u:
        rate = RRATE_5HZ;
        break;
    case 10u:
        rate = RRATE_10HZ;
        break;
    case 20u:
        rate = RRATE_20HZ;
        break;
    case 50u:
        rate = RRATE_50HZ;
        break;
    case 100u:
        rate = RRATE_100HZ;
        break;
    case 200u:
        rate = RRATE_200HZ;
        break;
    default:
        return WIT_HAL_INVAL;
    }

    return WitSetOutputRate(rate);
}

int32_t HWT901B_SaveConfig(void) {
    int32_t ret;

    ret = WitWriteReg(KEY, KEY_UNLOCK);
    if (ret != WIT_HAL_OK) {
        return ret;
    }
    HAL_Delay(1u);
    return WitWriteReg(SAVE, SAVE_PARAM);
}

int32_t HWT901B_ReadAccReg(void) {
    return WitReadReg(AX, 3u);
}

int32_t HWT901B_StartAccCali(void) {
    return WitStartAccCali();
}

int32_t HWT901B_StartMagCali(void) {
    return WitStartMagCali();
}

int32_t HWT901B_StopMagCali(void) {
    return WitStopMagCali();
}
