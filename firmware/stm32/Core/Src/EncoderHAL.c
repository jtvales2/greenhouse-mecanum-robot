#include "EncoderHAL.h"

void EncoderHAL_InitEx(EncoderHAL_t *enc, TIM_HandleTypeDef *htim, int8_t sign)
{
    if (enc == 0 || htim == 0) return;

    enc->htim       = htim;
    enc->prev       = 0;
    enc->accum      = 0;
    enc->last_delta = 0;
    enc->sign       = (sign >= 0) ? +1 : -1;

    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void EncoderHAL_Init(EncoderHAL_t *enc, TIM_HandleTypeDef *htim)
{
    EncoderHAL_InitEx(enc, htim, +1);
}

void EncoderHAL_SetSign(EncoderHAL_t *enc, int8_t sign)
{
    if (enc == 0) return;
    enc->sign = (sign >= 0) ? +1 : -1;
}

void EncoderHAL_Update(EncoderHAL_t *enc)
{
    if (enc == 0 || enc->htim == 0) return;

    uint32_t now = __HAL_TIM_GET_COUNTER(enc->htim);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(enc->htim);
    int32_t d;

    if (arr <= 0xFFFFU) {
        uint16_t now16  = (uint16_t)now;
        uint16_t prev16 = (uint16_t)enc->prev;
        d = (int32_t)(int16_t)(now16 - prev16);
        enc->prev = (uint32_t)now16;
    } else {
        d = (int32_t)(now - enc->prev);
        enc->prev = now;
    }

    d *= (enc->sign >= 0) ? +1 : -1;
    enc->last_delta = d;
    enc->accum     += d;
}

int32_t EncoderHAL_GetPosition(const EncoderHAL_t *enc)
{
    if (enc == 0) return 0;
    return enc->accum;
}

int32_t EncoderHAL_GetDeltaCached(const EncoderHAL_t *enc)
{
    if (enc == 0) return 0;
    return enc->last_delta;
}

float EncoderHAL_GetSpeed(const EncoderHAL_t *enc, float cpr, float dt_s)
{
    if (enc == 0) return 0.0f;
    if (cpr <= 0.0f || dt_s <= 0.0f) return 0.0f;

    return ((float)enc->last_delta) / cpr / dt_s;
}
