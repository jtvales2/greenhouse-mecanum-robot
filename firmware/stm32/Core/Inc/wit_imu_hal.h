#ifndef __WIT_IMU_HAL_H
#define __WIT_IMU_HAL_H

#include "main.h"
#include <stdint.h>
#include "wit_c_sdk.h"
#include "REG.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * WT901C-TTL/232 使用普通串口主动输出协议。
 * 不要用 WIT_PROTOCOL_MODBUS。
 */
#define WIT_IMU_PROTOCOL          WIT_PROTOCOL_NORMAL
#define WIT_IMU_ADDR              0x50

#define WIT_IMU_ACC_UPDATE        0x01
#define WIT_IMU_GYRO_UPDATE       0x02
#define WIT_IMU_ANGLE_UPDATE      0x04
#define WIT_IMU_MAG_UPDATE        0x08
#define WIT_IMU_READ_UPDATE       0x80

#define WIT_IMU_ACC_G_PER_LSB          (16.0f / 32768.0f)
#define WIT_IMU_GYRO_DPS_PER_LSB       (2000.0f / 32768.0f)
#define WIT_IMU_ANGLE_DEG_PER_LSB      (180.0f / 32768.0f)
#define WIT_IMU_MAG_MGAUSS_PER_LSB     (0.667f)

typedef struct
{
    float acc_g[3];          /* ax ay az, 单位 g */
    float gyro_dps[3];       /* gx gy gz, 单位 °/s */
    float angle_deg[3];      /* roll pitch yaw, 单位 ° */

    int16_t mag_raw[3];      /* hx hy hz 原始值 */
    float mag_mgauss[3];     /* 磁场估算值，单位 mGauss */

    uint8_t update_flags;
    uint32_t last_update_ms;
} WitImu_Data_t;

void WitImu_HAL_Init(UART_HandleTypeDef *huart);
void WitImu_HAL_RxCpltCallback(UART_HandleTypeDef *huart);
void WitImu_HAL_ErrorCallback(UART_HandleTypeDef *huart);

uint8_t WitImu_GetData(WitImu_Data_t *out);

float WitImu_GetGyroXDps(void);
float WitImu_GetGyroYDps(void);
float WitImu_GetGyroZDps(void);

float WitImu_GetRollDeg(void);
float WitImu_GetPitchDeg(void);
float WitImu_GetYawDeg(void);

uint8_t WitImu_IsOnline(uint32_t timeout_ms);
uint32_t WitImu_GetLastUpdateMs(void);

/* 校准接口 */
int32_t WitImu_StartGyroAccCali(void);
int32_t WitImu_StopGyroAccCali(void);
int32_t WitImu_StartMagCali(void);
int32_t WitImu_StopMagCali(void);
int32_t WitImu_CaliRefAngle(void);
int32_t WitImu_SaveParameter(void);

/*
 * 可选：不用上位机时，单片机写一次配置。
 * 不建议每次开机都调用保存参数。
 */
int32_t WitImu_ConfigModuleOnce(uint8_t save_to_flash);

#ifdef __cplusplus
}
#endif

#endif
