#include "imu_uart_hal.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_it_byte = 0;

static volatile uint8_t  s_rx_buffer[IMU_UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_write_index = 0;
static volatile uint16_t s_rx_read_index  = 0;
static volatile uint32_t s_rx_bytes = 0;

static volatile float s_ax = 0.0f, s_ay = 0.0f, s_az = 0.0f;
static volatile float s_gx = 0.0f, s_gy = 0.0f, s_gz = 0.0f;
static volatile float s_mx = 0.0f, s_my = 0.0f, s_mz = 0.0f;
static volatile float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;
static volatile float s_q0 = 0.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;
static volatile float s_height = 0.0f, s_temperature = 0.0f, s_pressure = 0.0f, s_pressure_contrast = 0.0f;
static volatile int   s_version_high = -1, s_version_mid = 0, s_version_low = 0;
static volatile uint8_t s_last_rx_function = 0;
static volatile int16_t s_last_rx_state = 0;

static volatile uint32_t s_motion_seq = 0;
static volatile uint32_t s_motion_tick_ms = 0;

static volatile uint32_t s_euler_seq = 0;
static volatile uint32_t s_euler_tick_ms = 0;

static inline uint16_t _rxbuf_next(uint16_t index)
{
    return (uint16_t)((index + 1u) % IMU_UART_RX_BUF_SIZE);
}

static inline int _rxbuf_is_empty(void)
{
    return s_rx_write_index == s_rx_read_index;
}

static inline void _rxbuf_push(uint8_t byte_value)
{
    uint16_t next_index = _rxbuf_next(s_rx_write_index);
    if (next_index == s_rx_read_index) {
        s_rx_read_index = _rxbuf_next(s_rx_read_index);
    }
    s_rx_buffer[s_rx_write_index] = byte_value;
    s_rx_write_index = next_index;
}

static inline int _rxbuf_pop(uint8_t *out_byte)
{
    if (_rxbuf_is_empty()) {
        return -1;
    }
    *out_byte = s_rx_buffer[s_rx_read_index];
    s_rx_read_index = _rxbuf_next(s_rx_read_index);
    return 0;
}

static int16_t to_int16(const uint8_t *bytes)
{
    return (int16_t)((((uint16_t)bytes[1]) << 8) | bytes[0]);
}

static float to_float(const uint8_t *bytes)
{
    float v;
    memcpy(&v, bytes, sizeof(float));
    return v;
}

static void _parse_frame_data(uint8_t frame_function, const uint8_t *frame_data, uint16_t payload_len)
{
    if (frame_function == IMU_FUNC_RAW_ACCEL)
    {
        if (payload_len >= 18U)
        {
            float accel_ratio = 16.0f / 32767.0f;
            float deg_to_rad  = 3.14159265358979323846f / 180.0f;
            float gyro_ratio  = (2000.0f / 32767.0f) * deg_to_rad;
            float mag_ratio   = 800.0f / 32767.0f;

            s_ax = to_int16(&frame_data[0])  * accel_ratio;
            s_ay = to_int16(&frame_data[2])  * accel_ratio;
            s_az = to_int16(&frame_data[4])  * accel_ratio;

            s_gx = to_int16(&frame_data[6])  * gyro_ratio;
            s_gy = to_int16(&frame_data[8])  * gyro_ratio;
            s_gz = to_int16(&frame_data[10]) * gyro_ratio;

            s_mx = to_int16(&frame_data[12]) * mag_ratio;
            s_my = to_int16(&frame_data[14]) * mag_ratio;
            s_mz = to_int16(&frame_data[16]) * mag_ratio;
        }
        else if (payload_len >= 6U)
        {
            float accel_ratio = 16.0f / 32767.0f;
            s_ax = to_int16(&frame_data[0]) * accel_ratio;
            s_ay = to_int16(&frame_data[2]) * accel_ratio;
            s_az = to_int16(&frame_data[4]) * accel_ratio;
        }

        s_motion_seq++;
        s_motion_tick_ms = HAL_GetTick();
    }
    else if (frame_function == IMU_FUNC_RAW_GYRO)
    {
        if (payload_len >= 6U)
        {
            float deg_to_rad = 3.14159265358979323846f / 180.0f;
            float gyro_ratio = (2000.0f / 32767.0f) * deg_to_rad;

            s_gx = to_int16(&frame_data[0]) * gyro_ratio;
            s_gy = to_int16(&frame_data[2]) * gyro_ratio;
            s_gz = to_int16(&frame_data[4]) * gyro_ratio;
        }

        s_motion_seq++;
        s_motion_tick_ms = HAL_GetTick();
    }
    else if (frame_function == IMU_FUNC_RAW_MAG)
    {
        if (payload_len >= 6U)
        {
            float mag_ratio = 800.0f / 32767.0f;

            s_mx = to_int16(&frame_data[0]) * mag_ratio;
            s_my = to_int16(&frame_data[2]) * mag_ratio;
            s_mz = to_int16(&frame_data[4]) * mag_ratio;
        }
    }
    else if (frame_function == IMU_FUNC_EULER)
    {
        if (payload_len >= 12U)
        {
            s_roll  = to_float(&frame_data[0]);
            s_pitch = to_float(&frame_data[4]);
            s_yaw   = to_float(&frame_data[8]);
        }

        s_euler_seq++;
        s_euler_tick_ms = HAL_GetTick();
    }
    else if (frame_function == IMU_FUNC_QUAT)
    {
        if (payload_len >= 16U)
        {
            s_q0 = to_float(&frame_data[0]);
            s_q1 = to_float(&frame_data[4]);
            s_q2 = to_float(&frame_data[8]);
            s_q3 = to_float(&frame_data[12]);
        }
    }
    else if (frame_function == IMU_FUNC_BARO)
    {
        if (payload_len >= 16U)
        {
            s_height            = to_float(&frame_data[0]);
            s_temperature       = to_float(&frame_data[4]);
            s_pressure          = to_float(&frame_data[8]);
            s_pressure_contrast = to_float(&frame_data[12]);
        }
    }
    else if (frame_function == IMU_FUNC_VERSION)
    {
        if (payload_len >= 3U)
        {
            s_version_high = frame_data[0];
            s_version_mid  = frame_data[1];
            s_version_low  = frame_data[2];
        }
    }
    else if (frame_function == IMU_FUNC_RETURN_STATE)
    {
        if (payload_len >= 2U)
        {
            s_last_rx_function = frame_data[0];
            s_last_rx_state    = (int16_t)frame_data[1];
        }
    }
}

void IMU_UART_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_write_index = 0;
    s_rx_read_index  = 0;
    s_version_high   = -1;
	s_rx_bytes       = 0;
	
    s_last_rx_function = 0;
    s_last_rx_state    = 0;

    s_motion_seq = 0;
    s_motion_tick_ms = 0;
    s_euler_seq = 0;
    s_euler_tick_ms = 0;

    s_ax = s_ay = s_az = 0.0f;
    s_gx = s_gy = s_gz = 0.0f;
    s_mx = s_my = s_mz = 0.0f;
    s_roll = s_pitch = s_yaw = 0.0f;
    s_q0 = s_q1 = s_q2 = s_q3 = 0.0f;
    s_height = s_temperature = s_pressure = s_pressure_contrast = 0.0f;

    (void)IMU_UART_StartReceiveIT();
}

HAL_StatusTypeDef IMU_UART_StartReceiveIT(void)
{
    if (s_huart == NULL) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(s_huart, &s_rx_it_byte, 1);
}

void IMU_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart)) {
        return;
    }

    s_rx_bytes++;
    _rxbuf_push(s_rx_it_byte);
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_it_byte, 1);
}

void IMU_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart != s_huart)) {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_it_byte, 1);
}

HAL_StatusTypeDef IMU_UART_SendBytes(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if ((s_huart == NULL) || (data == NULL) || (len == 0U)) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(s_huart, (uint8_t *)data, len, timeout_ms);
}

int IMU_UART_SendCommand(uint8_t function, const uint8_t *params, uint8_t param_len)
{
    if (param_len > 3U || (param_len > 0U && params == NULL)) {
        return -1;
    }

    uint8_t frame[8] = {FRAME_HEAD1, FRAME_HEAD2, 0, function, 0, 0, 0, 0};
    for (uint8_t i = 0; i < param_len; ++i) {
        frame[4 + i] = params[i];
    }

    uint8_t frame_len = (uint8_t)(4U + param_len + 1U);
    frame[2] = frame_len;

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < (uint8_t)(frame_len - 1U); ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[frame_len - 1U] = checksum;
		
		if (IMU_UART_SendBytes(frame, frame_len, 100U) == HAL_OK) {
      return 0;
    }
  return -1;
}

void IMU_UART_Process(void)
{
    enum {
        RX_STATE_EXPECT_HEAD1 = 0,
        RX_STATE_EXPECT_HEAD2,
        RX_STATE_EXPECT_LENGTH,
        RX_STATE_EXPECT_FUNCTION,
        RX_STATE_COLLECT_DATA
    };

    static uint8_t  rx_state = RX_STATE_EXPECT_HEAD1;
    static uint8_t  frame_length = 0;
    static uint8_t  frame_function = 0;
    static uint8_t  frame_buffer[64];
    static uint16_t frame_index = 0;

    uint8_t current_byte = 0;

    while (_rxbuf_pop(&current_byte) == 0) {
        switch (rx_state) {
        case RX_STATE_EXPECT_HEAD1:
            rx_state = (current_byte == FRAME_HEAD1) ? RX_STATE_EXPECT_HEAD2 : RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_HEAD2:
            rx_state = (current_byte == FRAME_HEAD2) ? RX_STATE_EXPECT_LENGTH : RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_LENGTH:
            frame_length = current_byte;
            rx_state = RX_STATE_EXPECT_FUNCTION;
            break;

        case RX_STATE_EXPECT_FUNCTION:
            frame_function = current_byte;
            frame_index = 0;
            rx_state = RX_STATE_COLLECT_DATA;
            break;

        case RX_STATE_COLLECT_DATA: {
            uint16_t data_length = (frame_length >= 4U) ? (uint16_t)(frame_length - 4U) : 0U;
            if ((data_length == 0U) || (data_length > sizeof(frame_buffer))) {
                rx_state = RX_STATE_EXPECT_HEAD1;
                break;
            }

            frame_buffer[frame_index++] = current_byte;
            if (frame_index >= data_length) {
                uint8_t calculated_checksum = (uint8_t)(FRAME_HEAD1 + FRAME_HEAD2 + frame_length + frame_function);
                for (uint16_t i = 0; i < (uint16_t)(data_length - 1U); ++i) {
                    calculated_checksum = (uint8_t)(calculated_checksum + frame_buffer[i]);
                }
								
								if (calculated_checksum == frame_buffer[data_length - 1U]) {
									_parse_frame_data(frame_function, frame_buffer, (uint16_t)(data_length - 1U));
								}

                rx_state = RX_STATE_EXPECT_HEAD1;
            }
        } break;

        default:
            rx_state = RX_STATE_EXPECT_HEAD1;
            break;
        }
    }
}

int IMU_UART_GetAccelerometer(float out[3])
{
    if (!out) return -1;
    out[0] = s_ax; out[1] = s_ay; out[2] = s_az;
    return 0;
}

int IMU_UART_GetGyroscope(float out[3])
{
    if (!out) return -1;
    out[0] = s_gx; out[1] = s_gy; out[2] = s_gz;
    return 0;
}

int IMU_UART_GetMagnetometer(float out[3])
{
    if (!out) return -1;
    out[0] = s_mx; out[1] = s_my; out[2] = s_mz;
    return 0;
}

int IMU_UART_GetQuaternion(float out[4])
{
    if (!out) return -1;
    out[0] = s_q0; out[1] = s_q1; out[2] = s_q2; out[3] = s_q3;
    return 0;
}

int IMU_UART_GetEuler(float out[3])
{
    if (!out) return -1;

#if IMU_EULER_PAYLOAD_IS_RAD
    {
        const float RAD2DEG = 57.2957795f;
        out[0] = s_roll  * RAD2DEG;
        out[1] = s_pitch * RAD2DEG;
        out[2] = s_yaw   * RAD2DEG;
    }
#else
    out[0] = s_roll;
    out[1] = s_pitch;
    out[2] = s_yaw;
#endif

    return 0;
}

int IMU_UART_GetBarometer(float out[4])
{
    if (!out) return -1;
    out[0] = s_height;
    out[1] = s_temperature;
    out[2] = s_pressure;
    out[3] = s_pressure_contrast;
    return 0;
}

int IMU_UART_GetAll(imu_measurement_t *out)
{
    if (!out) return -1;
    IMU_UART_GetAccelerometer(out->accel);
    IMU_UART_GetGyroscope(out->gyro);
    IMU_UART_GetMagnetometer(out->mag);
    IMU_UART_GetQuaternion(out->quat);
    IMU_UART_GetEuler(out->euler);
    IMU_UART_GetBarometer(out->baro);

    if (s_version_high >= 0) {
        snprintf(out->version, sizeof(out->version), "%d.%d.%d", s_version_high, s_version_mid, s_version_low);
    } else {
        snprintf(out->version, sizeof(out->version), "-1");
    }
    return 0;
}

uint32_t IMU_UART_GetMotionSeq(void)
{
    return s_motion_seq;
}

uint32_t IMU_UART_GetMotionTickMs(void)
{
    return s_motion_tick_ms;
}

uint32_t IMU_UART_GetEulerSeq(void)
{
    return s_euler_seq;
}

uint32_t IMU_UART_GetEulerTickMs(void)
{
    return s_euler_tick_ms;
}

uint32_t IMU_UART_GetRxBytes(void)
{
    return s_rx_bytes;
}

int IMU_UART_GetVersion(char *buf, uint16_t buf_len, uint32_t timeout_ms)
{
    uint8_t payload[2] = {IMU_FUNC_VERSION, 0x00};
    uint32_t tick_start;

    if (IMU_UART_SendCommand(IMU_FUNC_REQUEST_DATA, payload, (uint8_t)sizeof(payload)) != 0) {
        return -1;
    }

    tick_start = HAL_GetTick();
    while ((HAL_GetTick() - tick_start) < timeout_ms) {
        IMU_UART_Process();
        if (s_version_high >= 0) {
            if ((buf != NULL) && (buf_len > 0U)) {
                snprintf(buf, buf_len, "%d.%d.%d", s_version_high, s_version_mid, s_version_low);
            }
            return 0;
        }
        HAL_Delay(1);
    }

    if ((buf != NULL) && (buf_len > 0U)) {
        snprintf(buf, buf_len, "-1");
    }
    return -1;
}

void IMU_UART_ClearAutoReportData(void)
{
    s_ax = s_ay = s_az = 0.0f;
    s_gx = s_gy = s_gz = 0.0f;
    s_mx = s_my = s_mz = 0.0f;
    s_roll = s_pitch = s_yaw = 0.0f;
    s_q0 = s_q1 = s_q2 = s_q3 = 0.0f;
    s_height = s_temperature = s_pressure = s_pressure_contrast = 0.0f;
}

static int _calibration_with_wait(uint8_t function, const uint8_t *payload, uint8_t payload_len,
                                  const char *label, uint32_t timeout_ms)
{
    s_last_rx_function = 0;
    s_last_rx_state = -1;

    if (IMU_UART_SendCommand(function, payload, payload_len) != 0) {
        return -1;
    }

    int result = IMU_UART_WaitCalibration(function, timeout_ms);
    if (label != NULL) {
        if (result == -1) {
            printf("[IMU] Calibration %s timeout\r\n", label);
        } else if (result == 1) {
            printf("[IMU] Calibration %s success\r\n", label);
        } else {
            printf("[IMU] Calibration %s failed (code=%d)\r\n", label, result);
        }
    }
    return result;
}

int IMU_UART_CalibrationImu(void)
{
    uint8_t payload[2] = {0x01, 0x5F};
    return _calibration_with_wait(IMU_FUNC_CALIB_IMU, payload, (uint8_t)sizeof(payload), "imu", 7000U);
}

int IMU_UART_CalibrationMag(void)
{
    uint8_t payload[2] = {0x01, 0x5F};

    /*
     * 不要无限等待。
     * 原版 timeout=0 会导致没有返回状态时卡死。
     * 磁力计校准需要人工转动车体，先给 30 秒。
     */
    return _calibration_with_wait(IMU_FUNC_CALIB_MAG,
                                  payload,
                                  (uint8_t)sizeof(payload),
                                  "mag",
                                  30000U);
}

int IMU_UART_CalibrationTemp(float now_temperature)
{
    if ((now_temperature > 50.0f) || (now_temperature < -50.0f)) {
        return -1;
    }

    int16_t temperature_raw = (int16_t)(now_temperature * 100.0f);
    uint8_t payload[3] = {
        (uint8_t)(temperature_raw & 0xFF),
        (uint8_t)((temperature_raw >> 8) & 0xFF),
        0x5F
    };
    return _calibration_with_wait(IMU_FUNC_CALIB_TEMP, payload, (uint8_t)sizeof(payload), "temp", 2000U);
}

int IMU_UART_ResetUserData(void)
{
    uint8_t payload[2] = {0x01, 0x5F};
    return IMU_UART_SendCommand(IMU_FUNC_RESET_FLASH, payload, (uint8_t)sizeof(payload));
}

int IMU_UART_WaitCalibration(uint8_t function, uint32_t timeout_ms)
{
    uint32_t tick_start = HAL_GetTick();

    while (1) {
        IMU_UART_Process();

        if (s_last_rx_function == function) {
            return s_last_rx_state;
        }

        if ((timeout_ms != 0U) && ((HAL_GetTick() - tick_start) >= timeout_ms)) {
            return -1;
        }
        HAL_Delay(1);
    }
}

void IMU_UART_ClearVersionCache(void)
{
    s_version_high = -1;
    s_version_mid  = 0;
    s_version_low  = 0;
}

uint8_t IMU_UART_VersionReady(void)
{
    return (s_version_high >= 0) ? 1u : 0u;
}

int IMU_UART_GetVersionCached(char *buf, uint16_t buf_len)
{
    if (s_version_high < 0)
    {
        if (buf && buf_len) snprintf(buf, buf_len, "-1");
        return -1;
    }

    if (buf && buf_len)
    {
        snprintf(buf, buf_len, "%d.%d.%d",
                 s_version_high, s_version_mid, s_version_low);
    }
    return 0;
}
