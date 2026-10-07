#pragma once
#include "tim.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t prev;
    int32_t  accum;
    int32_t  last_delta;
    int8_t   sign;
} EncoderHAL_t;

void    EncoderHAL_Init(EncoderHAL_t *enc, TIM_HandleTypeDef *htim);
void    EncoderHAL_InitEx(EncoderHAL_t *enc, TIM_HandleTypeDef *htim, int8_t sign);
void    EncoderHAL_SetSign(EncoderHAL_t *enc, int8_t sign);
void    EncoderHAL_Update(EncoderHAL_t *enc);
int32_t EncoderHAL_GetPosition(const EncoderHAL_t *enc);
int32_t EncoderHAL_GetDeltaCached(const EncoderHAL_t *enc);
float   EncoderHAL_GetSpeed(const EncoderHAL_t *enc, float cpr, float dt_s);
