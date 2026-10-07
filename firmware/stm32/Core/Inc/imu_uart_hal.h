#ifndef IMU_UART_HAL_H
#define IMU_UART_HAL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_UART_RX_BUF_SIZE
#define IMU_UART_RX_BUF_SIZE 1024U
#endif

#define FRAME_HEAD1 0x7E
#define FRAME_HEAD2 0x23

#define IMU_FUNC_VERSION        0x01
#define IMU_FUNC_RAW_ACCEL      0x04
#define IMU_FUNC_RAW_GYRO       0x0A
#define IMU_FUNC_RAW_MAG        0x10
#define IMU_FUNC_QUAT           0x16
#define IMU_FUNC_EULER          0x26
#define IMU_FUNC_BARO           0x32
#define IMU_FUNC_CALIB_IMU      0x70
#define IMU_FUNC_CALIB_MAG      0x71
#define IMU_FUNC_CALIB_BARO     0x72
#define IMU_FUNC_CALIB_TEMP     0x73
#define IMU_FUNC_REQUEST_DATA   0x80
#define IMU_FUNC_RETURN_STATE   0x81
#define IMU_FUNC_RESET_FLASH    0xA0

/*
 * 欧拉角单位选择：
 * 1 = 模块发来的 roll/pitch/yaw 是 rad，驱动转成 deg
 * 0 = 模块发来的 roll/pitch/yaw 已经是 deg，驱动直接输出
 *
 * 先保持 1。后面手动转车 90° 验证。
 * 如果打印 yaw 变化接近 90°，保持 1。
 * 如果 yaw 变化接近 5000°，改成 0。
 */
#ifndef IMU_EULER_PAYLOAD_IS_RAD
#define IMU_EULER_PAYLOAD_IS_RAD 1
#endif

typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
    float euler[3];
    float baro[4];
    char  version[8];
} imu_measurement_t;

void IMU_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef IMU_UART_StartReceiveIT(void);
void IMU_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void IMU_UART_ErrorCallback(UART_HandleTypeDef *huart);
void IMU_UART_Process(void);

HAL_StatusTypeDef IMU_UART_SendBytes(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
int  IMU_UART_SendCommand(uint8_t function, const uint8_t *params, uint8_t param_len);

void IMU_UART_ClearAutoReportData(void);
int  IMU_UART_GetAccelerometer(float out[3]);
int  IMU_UART_GetGyroscope(float out[3]);
int  IMU_UART_GetMagnetometer(float out[3]);
int  IMU_UART_GetQuaternion(float out[4]);
int  IMU_UART_GetEuler(float out[3]);
int  IMU_UART_GetBarometer(float out[4]);
int  IMU_UART_GetAll(imu_measurement_t *out);

uint32_t IMU_UART_GetMotionSeq(void);
uint32_t IMU_UART_GetMotionTickMs(void);
uint32_t IMU_UART_GetEulerSeq(void);
uint32_t IMU_UART_GetEulerTickMs(void);
uint32_t IMU_UART_GetRxBytes(void);

int  IMU_UART_GetVersion(char *buf, uint16_t buf_len, uint32_t timeout_ms);
int  IMU_UART_CalibrationImu(void);
int  IMU_UART_CalibrationMag(void);
int  IMU_UART_CalibrationTemp(float now_temperature);
int  IMU_UART_ResetUserData(void);
int  IMU_UART_WaitCalibration(uint8_t function, uint32_t timeout_ms);

void    IMU_UART_ClearVersionCache(void);
uint8_t IMU_UART_VersionReady(void);
int     IMU_UART_GetVersionCached(char *buf, uint16_t buf_len);

#ifdef __cplusplus
}
#endif

#endif
