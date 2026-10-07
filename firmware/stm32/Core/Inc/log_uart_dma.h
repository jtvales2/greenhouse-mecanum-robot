#ifndef LOG_UART_DMA_H
#define LOG_UART_DMA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LogUart_Init(UART_HandleTypeDef *huart);
void LogUart_Write(const uint8_t *data, uint16_t len);
void LogUart_Service(void);
void LogUart_TxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
