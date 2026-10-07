#include "vofa.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *s_fw_uart = NULL;


static int append_f3(char *buf, int cap, float x) {
    int neg = (x < 0.0f);
    if (neg) x = -x;
    int32_t iv = (int32_t)(x * 1000.0f + 0.5f);
    int32_t ip = iv / 1000;
    int32_t fp = iv % 1000;
    int n = 0;
    if (neg) { if (n < cap) buf[n++] = '-'; }
    n += snprintf(buf + n, (n < cap) ? cap - n : 0, "%ld", (long)ip);
    if (n < cap) buf[n++] = '.';
    // 补齐三位小数
    n += snprintf(buf + n, (n < cap) ? cap - n : 0, "%03ld", (long)fp);
    return n;
}

static void fw_tx(const uint8_t *data, uint16_t len) {
    if (!s_fw_uart || !data || !len) return;
    HAL_UART_Transmit(s_fw_uart, (uint8_t*)data, len, 1000);
}

void FW_Init(UART_HandleTypeDef *huart) { s_fw_uart = huart; }

void FW_SendLineTag(const char *tag, const float *v, int n) {
    if (!s_fw_uart || !v || n <= 0) return;
    char line[256];
    int pos = 0;

    if (tag && *tag) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%s:", tag);
    }

    for (int i = 0; i < n; ++i) {
        pos += append_f3(line + pos, (int)sizeof(line) - pos, v[i]);
        if (i != n - 1) {
            if (pos < (int)sizeof(line)) line[pos++] = ',';
        }
    }
    if (pos < (int)sizeof(line)) line[pos++] = '\n';   //必须换行

    fw_tx((uint8_t*)line, (uint16_t)pos);
}

void FW_SendLine(const float *v, int n) { FW_SendLineTag(NULL, v, n); }
