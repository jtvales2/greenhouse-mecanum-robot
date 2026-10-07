#include "wit_imu_hal.h"

static UART_HandleTypeDef *s_wit_huart = 0;
static uint8_t s_wit_rx_byte = 0;

static volatile uint8_t s_wit_update_flags = 0;
static volatile uint32_t s_wit_last_update_ms = 0;

static void WitImu_UartSend(uint8_t *p_data, uint32_t len)
{
    if (s_wit_huart == 0 || p_data == 0 || len == 0)
    {
        return;
    }

    HAL_UART_Transmit(s_wit_huart, p_data, len, 100);
}

static void WitImu_DelayMs(uint16_t ms)
{
    HAL_Delay(ms);
}

static void WitImu_RegUpdateCallback(uint32_t uiReg, uint32_t uiRegNum)
{
    uint32_t i;
    uint32_t reg;

    for (i = 0; i < uiRegNum; i++)
    {
        reg = uiReg + i;

        switch (reg)
        {
            case AZ:
                s_wit_update_flags |= WIT_IMU_ACC_UPDATE;
                break;

            case GZ:
                s_wit_update_flags |= WIT_IMU_GYRO_UPDATE;
                break;

            case Yaw:
                s_wit_update_flags |= WIT_IMU_ANGLE_UPDATE;
                break;

            case HZ:
                s_wit_update_flags |= WIT_IMU_MAG_UPDATE;
                break;

            default:
                s_wit_update_flags |= WIT_IMU_READ_UPDATE;
                break;
        }
    }

    s_wit_last_update_ms = HAL_GetTick();
}

void WitImu_HAL_Init(UART_HandleTypeDef *huart)
{
    s_wit_huart = huart;
    s_wit_update_flags = 0;
    s_wit_last_update_ms = 0;

    WitInit(WIT_IMU_PROTOCOL, WIT_IMU_ADDR);
    WitSerialWriteRegister(WitImu_UartSend);
    WitRegisterCallBack(WitImu_RegUpdateCallback);
    WitDelayMsRegister(WitImu_DelayMs);

    HAL_UART_Receive_IT(s_wit_huart, &s_wit_rx_byte, 1);
}

void WitImu_HAL_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_wit_huart == 0)
    {
        return;
    }

    if (huart->Instance == s_wit_huart->Instance)
    {
        /*
         * 关键：每收到 1 个字节，送给官方 SDK 解析。
         */
        WitSerialDataIn(s_wit_rx_byte);

        /*
         * 继续接收下一字节。
         */
        HAL_UART_Receive_IT(s_wit_huart, &s_wit_rx_byte, 1);
    }
}

void WitImu_HAL_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_wit_huart == 0)
    {
        return;
    }

    if (huart->Instance == s_wit_huart->Instance)
    {
        HAL_UART_AbortReceive_IT(s_wit_huart);

#ifdef __HAL_UART_CLEAR_OREFLAG
        __HAL_UART_CLEAR_OREFLAG(s_wit_huart);
#endif

#ifdef __HAL_UART_CLEAR_FEFLAG
        __HAL_UART_CLEAR_FEFLAG(s_wit_huart);
#endif

#ifdef __HAL_UART_CLEAR_NEFLAG
        __HAL_UART_CLEAR_NEFLAG(s_wit_huart);
#endif

        HAL_UART_Receive_IT(s_wit_huart, &s_wit_rx_byte, 1);
    }
}

uint8_t WitImu_GetData(WitImu_Data_t *out)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t roll, pitch, yaw;
    int16_t hx, hy, hz;
    uint8_t flags;
    uint32_t last_ms;

    if (out == 0)
    {
        return 0;
    }

    __disable_irq();

    ax = sReg[AX];
    ay = sReg[AY];
    az = sReg[AZ];

    gx = sReg[GX];
    gy = sReg[GY];
    gz = sReg[GZ];

    hx = sReg[HX];
    hy = sReg[HY];
    hz = sReg[HZ];

    roll  = sReg[Roll];
    pitch = sReg[Pitch];
    yaw   = sReg[Yaw];

    flags = s_wit_update_flags;
    s_wit_update_flags = 0;

    last_ms = s_wit_last_update_ms;

    __enable_irq();

    out->acc_g[0] = ax * WIT_IMU_ACC_G_PER_LSB;
    out->acc_g[1] = ay * WIT_IMU_ACC_G_PER_LSB;
    out->acc_g[2] = az * WIT_IMU_ACC_G_PER_LSB;

    out->gyro_dps[0] = gx * WIT_IMU_GYRO_DPS_PER_LSB;
    out->gyro_dps[1] = gy * WIT_IMU_GYRO_DPS_PER_LSB;
    out->gyro_dps[2] = gz * WIT_IMU_GYRO_DPS_PER_LSB;

    out->angle_deg[0] = roll  * WIT_IMU_ANGLE_DEG_PER_LSB;
    out->angle_deg[1] = pitch * WIT_IMU_ANGLE_DEG_PER_LSB;
    out->angle_deg[2] = yaw   * WIT_IMU_ANGLE_DEG_PER_LSB;

    out->mag_raw[0] = hx;
    out->mag_raw[1] = hy;
    out->mag_raw[2] = hz;

    out->mag_mgauss[0] = hx * WIT_IMU_MAG_MGAUSS_PER_LSB;
    out->mag_mgauss[1] = hy * WIT_IMU_MAG_MGAUSS_PER_LSB;
    out->mag_mgauss[2] = hz * WIT_IMU_MAG_MGAUSS_PER_LSB;

    out->update_flags = flags;
    out->last_update_ms = last_ms;

    return flags;
}

float WitImu_GetGyroXDps(void)
{
    int16_t raw;

    __disable_irq();
    raw = sReg[GX];
    __enable_irq();

    return raw * WIT_IMU_GYRO_DPS_PER_LSB;
}

float WitImu_GetGyroYDps(void)
{
    int16_t raw;

    __disable_irq();
    raw = sReg[GY];
    __enable_irq();

    return raw * WIT_IMU_GYRO_DPS_PER_LSB;
}

float WitImu_GetGyroZDps(void)
{
    int16_t raw;

    __disable_irq();
    raw = sReg[GZ];
    __enable_irq();

    return raw * WIT_IMU_GYRO_DPS_PER_LSB;
}

float WitImu_GetRollDeg(void)
{
    int16_t raw;

    __disable_irq();
    raw = sReg[Roll];
    __enable_irq();

    return raw * WIT_IMU_ANGLE_DEG_PER_LSB;
}

float WitImu_GetPitchDeg(void)
{
    int16_t raw;

    __disable_irq();
    raw = sReg[Pitch];
    __enable_irq();

    return raw * WIT_IMU_ANGLE_DEG_PER_LSB;
}

float WitImu_GetYawDeg(void)
{
    int16_t raw;

    __disable_irq();
    raw = sReg[Yaw];
    __enable_irq();

    return raw * WIT_IMU_ANGLE_DEG_PER_LSB;
}

uint32_t WitImu_GetLastUpdateMs(void)
{
    uint32_t t;

    __disable_irq();
    t = s_wit_last_update_ms;
    __enable_irq();

    return t;
}

uint8_t WitImu_IsOnline(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    uint32_t last = WitImu_GetLastUpdateMs();

    if (last == 0)
    {
        return 0;
    }

    return ((uint32_t)(now - last) <= timeout_ms) ? 1 : 0;
}

int32_t WitImu_StartGyroAccCali(void)
{
    return WitStartAccCali();
}

int32_t WitImu_StopGyroAccCali(void)
{
    return WitStopAccCali();
}

int32_t WitImu_StartMagCali(void)
{
    return WitStartMagCali();
}

int32_t WitImu_StopMagCali(void)
{
    return WitStopMagCali();
}

int32_t WitImu_CaliRefAngle(void)
{
    return WitCaliRefAngle();
}

int32_t WitImu_SaveParameter(void)
{
    return WitSaveParameter();
}

int32_t WitImu_ConfigModuleOnce(uint8_t save_to_flash)
{
    int32_t ret;

    /*
     * 这个函数只用于“没有上位机”或者“想由单片机写配置”的情况。
     * 正常比赛小车里，不建议每次开机都保存参数。
     */

    ret = WitWriteReg(KEY, KEY_UNLOCK);
    HAL_Delay(20);
    if (ret != WIT_HAL_OK) return ret;

    /*
     * 九轴算法。
     */
    ret = WitWriteReg(AXIS6, ALGRITHM9);
    HAL_Delay(20);
    if (ret != WIT_HAL_OK) return ret;

    /*
     * 水平安装。
     */
    ret = WitWriteReg(ORIENT, ORIENT_HERIZONE);
    HAL_Delay(20);
    if (ret != WIT_HAL_OK) return ret;

    /*
     * 带宽 21Hz，对应上位机大约 20Hz。
     */
    ret = WitSetBandwidth(BANDWIDTH_21HZ);
    HAL_Delay(20);
    if (ret != WIT_HAL_OK) return ret;

    /*
     * 回传速率先 100Hz。
     * 稳定后可改成 RRATE_200HZ。
     */
    ret = WitSetOutputRate(RRATE_100HZ);
    HAL_Delay(20);
    if (ret != WIT_HAL_OK) return ret;

    /*
     * 输出内容：加速度 + 角速度 + 欧拉角 + 磁场。
     */
    ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG);
    HAL_Delay(20);
    if (ret != WIT_HAL_OK) return ret;

    if (save_to_flash)
    {
        ret = WitSaveParameter();
        HAL_Delay(100);
        if (ret != WIT_HAL_OK) return ret;
    }

    return WIT_HAL_OK;
}
