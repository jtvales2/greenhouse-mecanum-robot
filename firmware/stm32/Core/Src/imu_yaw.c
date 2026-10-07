#include "imu_yaw.h"

#include "robot_config.h"
#include "wit_imu_hal.h"

#include <stdio.h>
#include <string.h>

static float s_yaw_deg;
static float s_gz_raw_deg_s;
static float s_gz_bias_deg_s;
static float s_gz_deg_s;

static float s_bias_sum;
static uint16_t s_bias_sample_count;
static uint32_t s_bias_start_ms;
static uint8_t s_bias_ok;

static uint32_t s_warmup_start_ms;
static uint8_t s_warmup_started;

static uint32_t s_last_gyro_ms;

static float ImuYaw_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float ImuYaw_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void ImuYaw_ResetBiasCalibration(uint32_t now_ms)
{
    s_bias_sum = 0.0f;
    s_bias_sample_count = 0u;
    s_bias_start_ms = now_ms;
}

void ImuYaw_Init(UART_HandleTypeDef *huart)
{
    s_yaw_deg = 0.0f;
    s_gz_raw_deg_s = 0.0f;
    s_gz_bias_deg_s = 0.0f;
    s_gz_deg_s = 0.0f;

    s_bias_sum = 0.0f;
    s_bias_sample_count = 0u;
    s_bias_start_ms = 0u;
    s_bias_ok = 0u;
	
	  s_warmup_start_ms = 0u;
    s_warmup_started = 0u;
	
    s_last_gyro_ms = 0u;

#if ROBOT_USE_WIT_IMU
    WitImu_HAL_Init(huart);
    printf("[IMU_YAW] WIT gyro-only init\r\n");
    printf("[IMU_YAW] keep robot still for gyro bias\r\n");
#else
    (void)huart;
    printf("[IMU_YAW] disabled\r\n");
#endif
}

void ImuYaw_Update10ms(float dt_s, uint32_t now_ms)
{
#if ROBOT_USE_WIT_IMU
    WitImu_Data_t wit;
    uint8_t flags;
    float bias_candidate;

    if (dt_s < 0.001f)
    {
        dt_s = 0.01f;
    }

    if (dt_s > 0.030f)
    {
        dt_s = 0.030f;
    }

    memset(&wit, 0, sizeof(wit));
    flags = WitImu_GetData(&wit);
		
		if ((flags & WIT_IMU_GYRO_UPDATE) != 0u)
{
    s_gz_raw_deg_s =
        ROBOT_IMU_GZ_SIGN * wit.gyro_dps[2];

    s_last_gyro_ms = now_ms;

    /*
     * Let the WIT gyro warm up before estimating zero-rate bias.
     * Samples during this period are intentionally discarded.
     */
    if (s_warmup_started == 0u)
    {
        s_warmup_start_ms = now_ms;
        s_warmup_started = 1u;
    }

    if ((s_bias_ok == 0u) &&
        ((uint32_t)(now_ms - s_warmup_start_ms) <
         ROBOT_IMU_STARTUP_WAIT_MS))
    {
        s_gz_deg_s = 0.0f;
        return;
    }

    if (s_bias_ok == 0u)
    {
            if (s_bias_start_ms == 0u)
            {
                ImuYaw_ResetBiasCalibration(now_ms);
            }

            s_bias_sum += s_gz_raw_deg_s;

            if (s_bias_sample_count < 65535u)
            {
                s_bias_sample_count++;
            }

            if (((uint32_t)(now_ms - s_bias_start_ms) >=
                 ROBOT_GYRO_BIAS_CAL_MS) &&
                (s_bias_sample_count >=
                 ROBOT_GYRO_BIAS_MIN_SAMPLES))
            {
                bias_candidate =
                    s_bias_sum /
                    (float)s_bias_sample_count;

                if (ImuYaw_Abs(bias_candidate) <=
                    ROBOT_GYRO_BIAS_MAX_ABS_DPS)
                {
                    s_gz_bias_deg_s = bias_candidate;
                    s_gz_deg_s = 0.0f;
                    s_yaw_deg = 0.0f;
                    s_bias_ok = 1u;

                    printf("[IMU_YAW] GYRO_BIAS_OK "
                           "bias=%.3f cnt=%u\r\n",
                           s_gz_bias_deg_s,
                           (unsigned)s_bias_sample_count);
                }
                else
                {
                    printf("[IMU_YAW] GYRO_BIAS_REJECT "
                           "bias=%.3f max=%.1f; keep still\r\n",
                           bias_candidate,
                           ROBOT_GYRO_BIAS_MAX_ABS_DPS);

                    ImuYaw_ResetBiasCalibration(now_ms);
                }
            }
        }
        else
        {
            s_gz_deg_s =
                s_gz_raw_deg_s - s_gz_bias_deg_s;

            if (ImuYaw_Abs(s_gz_deg_s) <
                ROBOT_GYRO_ZERO_DEADBAND_DPS)
            {
                s_gz_deg_s = 0.0f;
            }
        }
    }

    if (s_bias_ok == 0u)
    {
        s_gz_deg_s = 0.0f;
        return;
    }

    if (ImuYaw_IsFresh() == 0u)
    {
        s_gz_deg_s = 0.0f;
        return;
    }

    /*
     * Preserve the proven competition control principle:
     * local relative yaw = integral of bias-corrected gyro Z.
     *
     * No magnetometer and no WIT fused yaw are used here.
     */
    s_yaw_deg = ImuYaw_WrapDeg(
        s_yaw_deg + s_gz_deg_s * dt_s);
#else
    (void)dt_s;
    (void)now_ms;
#endif
}

uint8_t ImuYaw_IsOnline(void)
{
#if ROBOT_USE_WIT_IMU
    if (s_last_gyro_ms == 0u)
    {
        return 0u;
    }

    return (((uint32_t)(HAL_GetTick() - s_last_gyro_ms) <=
             ROBOT_IMU_ONLINE_TIMEOUT_MS) ?
            1u : 0u);
#else
    return 0u;
#endif
}

uint8_t ImuYaw_IsFresh(void)
{
    return ImuYaw_IsOnline();
}

uint8_t ImuYaw_IsReady(void)
{
    if (s_bias_ok == 0u)
    {
        return 0u;
    }

    return ImuYaw_IsFresh();
}

void ImuYaw_ResetYaw(void)
{
    s_yaw_deg = 0.0f;
}

void ImuYaw_GetState(ImuYaw_State_t *state)
{
    if (state == 0)
    {
        return;
    }

    state->online = ImuYaw_IsOnline();
    state->fresh = ImuYaw_IsFresh();
    state->ready = ImuYaw_IsReady();
    state->bias_ok = s_bias_ok;

    state->bias_sample_count = s_bias_sample_count;

    state->yaw_deg = s_yaw_deg;
    state->gz_deg_s = s_gz_deg_s;
    state->gz_raw_deg_s = s_gz_raw_deg_s;
    state->bias_deg_s = s_gz_bias_deg_s;

    state->last_gyro_ms = s_last_gyro_ms;
}

void ImuYaw_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
#if ROBOT_USE_WIT_IMU
    WitImu_HAL_RxCpltCallback(huart);
#else
    (void)huart;
#endif
}

void ImuYaw_UartErrorCallback(UART_HandleTypeDef *huart)
{
#if ROBOT_USE_WIT_IMU
    WitImu_HAL_ErrorCallback(huart);
#else
    (void)huart;
#endif
}
