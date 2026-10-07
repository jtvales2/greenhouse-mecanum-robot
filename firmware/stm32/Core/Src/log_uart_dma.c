#include "log_uart_dma.h"
#include <string.h>

#ifndef LOG_UART_TX_BUF_SIZE
#define LOG_UART_TX_BUF_SIZE 2048u
#endif

static UART_HandleTypeDef *s_huart = NULL;

static volatile uint8_t  s_tx_buf[LOG_UART_TX_BUF_SIZE];
static volatile uint16_t s_w = 0u;
static volatile uint16_t s_r = 0u;

static volatile uint8_t  s_dma_busy = 0u;
static uint8_t           s_dma_chunk[128];
static uint16_t          s_dma_len = 0u;

static uint16_t _next_idx(uint16_t x)
{
    return (uint16_t)((x + 1u) % LOG_UART_TX_BUF_SIZE);
}

static uint8_t _is_empty(void)
{
    return (s_w == s_r) ? 1u : 0u;
}

static void _push_byte(uint8_t b)
{
    uint16_t next = _next_idx(s_w);

    if (next == s_r)
    {
        /* 满了就丢最老的一字节，绝不阻塞 */
        s_r = _next_idx(s_r);
    }

    s_tx_buf[s_w] = b;
    s_w = next;
}

static uint16_t _pop_chunk(uint8_t *dst, uint16_t max_len)
{
    uint16_t n = 0u;

    while ((n < max_len) && (s_r != s_w))
    {
        dst[n++] = s_tx_buf[s_r];
        s_r = _next_idx(s_r);
    }

    return n;
}

void LogUart_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_w = 0u;
    s_r = 0u;
    s_dma_busy = 0u;
    s_dma_len = 0u;
}

void LogUart_Write(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((s_huart == NULL) || (data == NULL) || (len == 0u))
        return;

    __disable_irq();
    for (i = 0; i < len; i++)
    {
        _push_byte(data[i]);
    }
    __enable_irq();

    LogUart_Service();
}

void LogUart_Service(void)
{
    HAL_StatusTypeDef st;

    if (s_huart == NULL)
        return;

    if (s_dma_busy)
        return;

    if (_is_empty())
        return;

    __disable_irq();
    if (s_dma_busy || _is_empty())
    {
        __enable_irq();
        return;
    }

    s_dma_len = _pop_chunk(s_dma_chunk, sizeof(s_dma_chunk));
    if (s_dma_len == 0u)
    {
        __enable_irq();
        return;
    }

    s_dma_busy = 1u;
    __enable_irq();

    st = HAL_UART_Transmit_DMA(s_huart, s_dma_chunk, s_dma_len);
    if (st != HAL_OK)
    {
        __disable_irq();
        s_dma_busy = 0u;
        __enable_irq();
    }
}

void LogUart_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart))
        return;

    s_dma_busy = 0u;
    LogUart_Service();
}
