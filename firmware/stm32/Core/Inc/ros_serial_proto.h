#ifndef ROS_SERIAL_PROTO_H
#define ROS_SERIAL_PROTO_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROS_SERIAL_CMD_TIMEOUT_MS      300u
#define ROS_SERIAL_STATUS_PERIOD_MS    100u

void RosSerial_Init(UART_HandleTypeDef *huart);
void RosSerial_Service(uint32_t now_ms);
void RosSerial_RxCpltCallback(UART_HandleTypeDef *huart);
void RosSerial_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t RosSerial_IsOnline(void);
uint32_t RosSerial_GetRxLineCount(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS_SERIAL_PROTO_H */
