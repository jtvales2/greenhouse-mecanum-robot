#ifndef IMU_YAW_H
#define IMU_YAW_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t online;
    uint8_t fresh;
    uint8_t ready;
    uint8_t bias_ok;

    uint16_t bias_sample_count;

    float yaw_deg;
    float gz_deg_s;
    float gz_raw_deg_s;
    float bias_deg_s;

    uint32_t last_gyro_ms;
} ImuYaw_State_t;

/*
 * WIT gyro-only yaw source:
 *   USART2 -> WIT SDK -> gyro Z
 *   gyro Z -> startup bias removal -> local yaw integration
 *
 * WIT fused yaw and magnetometer are not used by this module.
 */
void ImuYaw_Init(UART_HandleTypeDef *huart);

void ImuYaw_Update10ms(float dt_s, uint32_t now_ms);

uint8_t ImuYaw_IsOnline(void);
uint8_t ImuYaw_IsFresh(void);
uint8_t ImuYaw_IsReady(void);

void ImuYaw_ResetYaw(void);
void ImuYaw_GetState(ImuYaw_State_t *state);

void ImuYaw_UartRxCpltCallback(UART_HandleTypeDef *huart);
void ImuYaw_UartErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* IMU_YAW_H */
