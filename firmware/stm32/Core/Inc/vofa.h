#pragma once
#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void FW_Init(UART_HandleTypeDef *huart);
void FW_SendLineTag(const char *tag, const float *v, int n); 
void FW_SendLine(const float *v, int n);

#ifdef __cplusplus
}
#endif
