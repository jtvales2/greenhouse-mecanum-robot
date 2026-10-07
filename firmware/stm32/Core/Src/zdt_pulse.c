#include "zdt_pulse.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    volatile uint32_t target_pulse;
    volatile uint32_t now_pulse;
    volatile uint8_t busy;
    volatile uint8_t speed_mode;

    volatile int32_t cmd_pos_pulse;
    volatile int8_t dir_sign;

    float last_cmd_rpm;
} ZDT_PulseMotor_t;

static ZDT_PulseMotor_t s_m1;
static ZDT_PulseMotor_t s_m2;

static ZDT_PulseMotor_t *ZDT_GetMotor(uint8_t motor_id)
{
    if (motor_id == 1u)
    {
        return &s_m1;
    }

    if (motor_id == 2u)
    {
        return &s_m2;
    }

    return 0;
}

static uint32_t ZDT_GetTimerClockHz(TIM_HandleTypeDef *htim)
{
    uint32_t pclk;
    uint32_t timclk;

    /*
     * TIM10 / TIM11 在 APB2。
     * STM32F4 中，如果 APB2 分频不是 1，定时器时钟 = PCLK2 * 2。
     */
    pclk = HAL_RCC_GetPCLK2Freq();
    timclk = pclk;

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_HCLK_DIV1)
    {
        timclk = pclk * 2u;
    }

    (void)htim;
    return timclk;
}

static uint32_t ZDT_RpmToPulseHz(float rpm)
{
    float hz_f;
    uint32_t hz;

    if (rpm < ZDT_MIN_RPM)
    {
        rpm = ZDT_MIN_RPM;
    }

    if (rpm > ZDT_MAX_RPM)
    {
        rpm = ZDT_MAX_RPM;
    }

    /*
     * pulse_hz = rpm * pulse_per_rev / 60
     */
    hz_f = rpm * (float)ZDT_PULSE_PER_REV / 60.0f;

    if (hz_f < 1.0f)
    {
        hz_f = 1.0f;
    }

    hz = (uint32_t)(hz_f + 0.5f);

    return hz;
}

static uint8_t ZDT_SetTimerPulseHz(ZDT_PulseMotor_t *m, uint32_t pulse_hz)
{
    uint32_t timclk;
    uint32_t psc;
    uint32_t cntclk;
    uint32_t arr;

    if ((m == 0) || (m->htim == 0) || (pulse_hz == 0u))
    {
        return 0u;
    }

    timclk = ZDT_GetTimerClockHz(m->htim);

    /*
     * 你的 HAL 库没有 __HAL_TIM_GET_PRESCALER()，
     * 所以这里直接读 PSC 寄存器。
     *
     * 实际分频系数 = PSC + 1
     */
    psc = (uint32_t)(m->htim->Instance->PSC + 1u);

    if (psc == 0u)
    {
        return 0u;
    }

    cntclk = timclk / psc;

    /*
     * PWM频率 = cntclk / (ARR + 1)
     * 所以 ARR = cntclk / pulse_hz - 1
     */
    arr = cntclk / pulse_hz;

    if (arr < 2u)
    {
        arr = 2u;
    }

    arr = arr - 1u;

    if (arr > 0xFFFFu)
    {
        arr = 0xFFFFu;
    }

    __HAL_TIM_DISABLE(m->htim);

    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, (arr + 1u) / 2u);
    __HAL_TIM_SET_COUNTER(m->htim, 0u);

    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_CC1);

    return 1u;
}

static void ZDT_StopMotor(ZDT_PulseMotor_t *m)
{
    if (m == 0)
    {
        return;
    }

    /*
     * 速度模式和位置模式统一使用 Stop_IT。
     * 因为下面速度模式也改成 Start_IT，避免 HAL 状态不一致。
     */
    HAL_TIM_PWM_Stop_IT(m->htim, m->channel);

    m->busy = 0u;
    m->speed_mode = 0u;
    m->target_pulse = 0u;
    m->now_pulse = 0u;
    m->dir_sign = 0;

    __HAL_TIM_SET_COUNTER(m->htim, 0u);
}

void ZDT_Pulse_Init(void)
{
    s_m1.htim = &htim10;
    s_m1.channel = TIM_CHANNEL_1;
    s_m1.dir_port = LIFT1_DIR_GPIO_Port;
    s_m1.dir_pin = LIFT1_DIR_Pin;
    s_m1.target_pulse = 0u;
    s_m1.now_pulse = 0u;
    s_m1.busy = 0u;
    s_m1.speed_mode = 0u;
    s_m1.cmd_pos_pulse = 0;
    s_m1.dir_sign = 0;
    s_m1.last_cmd_rpm = 0.0f;

    s_m2.htim = &htim11;
    s_m2.channel = TIM_CHANNEL_1;
    s_m2.dir_port = LIFT2_DIR_GPIO_Port;
    s_m2.dir_pin = LIFT2_DIR_Pin;
    s_m2.target_pulse = 0u;
    s_m2.now_pulse = 0u;
    s_m2.busy = 0u;
    s_m2.speed_mode = 0u;
    s_m2.cmd_pos_pulse = 0;
    s_m2.dir_sign = 0;
    s_m2.last_cmd_rpm = 0.0f;

    ZDT_StopMotor(&s_m1);
    ZDT_StopMotor(&s_m2);

    ZDT_SetTimerPulseHz(&s_m1, ZDT_RpmToPulseHz(ZDT_DEFAULT_RPM));
    ZDT_SetTimerPulseHz(&s_m2, ZDT_RpmToPulseHz(ZDT_DEFAULT_RPM));
}

uint8_t ZDT_Pulse_IsBusy(uint8_t motor_id)
{
    ZDT_PulseMotor_t *m;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return 0u;
    }

    return m->busy;
}

void ZDT_Pulse_Stop(uint8_t motor_id)
{
    ZDT_PulseMotor_t *m;

    m = ZDT_GetMotor(motor_id);
    ZDT_StopMotor(m);
}

void ZDT_Pulse_StopAll(void)
{
    ZDT_Pulse_Stop(1u);
    ZDT_Pulse_Stop(2u);
}

uint8_t ZDT_Pulse_SetRpm(uint8_t motor_id, float rpm)
{
    ZDT_PulseMotor_t *m;
    uint32_t pulse_hz;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return 0u;
    }

    if (m->busy)
    {
        return 0u;
    }

    pulse_hz = ZDT_RpmToPulseHz(rpm);

    if (!ZDT_SetTimerPulseHz(m, pulse_hz))
    {
        return 0u;
    }

    m->last_cmd_rpm = rpm;

    return 1u;
}

uint8_t ZDT_Pulse_MovePulseAtRpm(uint8_t motor_id,
                                 uint8_t dir,
                                 uint32_t pulse,
                                 float rpm)
{
    ZDT_PulseMotor_t *m;
    HAL_StatusTypeDef ret;
    uint32_t pulse_hz;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return 0u;
    }

    if (pulse == 0u)
    {
        return 0u;
    }

    if (m->busy)
    {
        return 0u;
    }

    ZDT_StopMotor(m);

    if (dir == ZDT_DIR_CW)
    {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
        m->dir_sign = 1;
    }
    else
    {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
        m->dir_sign = -1;
    }

    pulse_hz = ZDT_RpmToPulseHz(rpm);

    if (!ZDT_SetTimerPulseHz(m, pulse_hz))
    {
        m->dir_sign = 0;
        return 0u;
    }

    __HAL_TIM_SET_COUNTER(m->htim, 0u);
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_CC1);

    m->target_pulse = pulse;
    m->now_pulse = 0u;
    m->busy = 1u;
    m->speed_mode = 0u;
    m->last_cmd_rpm = rpm;

    /*
     * 位置模式需要中断计数，输出够 target_pulse 后自动停。
     */
    ret = HAL_TIM_PWM_Start_IT(m->htim, m->channel);

    if (ret != HAL_OK)
    {
        m->busy = 0u;
        m->speed_mode = 0u;
        m->target_pulse = 0u;
        m->now_pulse = 0u;
        m->dir_sign = 0;
        return 0u;
    }

    return 1u;
}

uint8_t ZDT_Pulse_MovePulse(uint8_t motor_id, uint8_t dir, uint32_t pulse)
{
    return ZDT_Pulse_MovePulseAtRpm(motor_id,
                                    dir,
                                    pulse,
                                    ZDT_DEFAULT_RPM);
}

uint8_t ZDT_Pulse_MoveHalfRev(uint8_t motor_id, uint8_t dir)
{
    return ZDT_Pulse_MovePulseAtRpm(motor_id,
                                    dir,
                                    ZDT_PULSE_HALF_REV,
                                    ZDT_DEFAULT_RPM);
}

uint8_t ZDT_Pulse_MoveOneRev(uint8_t motor_id, uint8_t dir)
{
    return ZDT_Pulse_MovePulseAtRpm(motor_id,
                                    dir,
                                    ZDT_PULSE_ONE_REV,
                                    ZDT_DEFAULT_RPM);
}

uint8_t ZDT_Pulse_MoveAngle(uint8_t motor_id,
                            uint8_t dir,
                            float angle_deg,
                            float rpm)
{
    float pulse_f;
    uint32_t pulse;

    if (angle_deg < 0.0f)
    {
        angle_deg = -angle_deg;
    }

    if (angle_deg <= 0.0f)
    {
        return 0u;
    }

    /*
     * pulse = angle / 360 * pulse_per_rev
     */
    pulse_f = angle_deg * (float)ZDT_PULSE_PER_REV / 360.0f;
    pulse = (uint32_t)(pulse_f + 0.5f);

    if (pulse == 0u)
    {
        pulse = 1u;
    }

    return ZDT_Pulse_MovePulseAtRpm(motor_id, dir, pulse, rpm);
}

uint8_t ZDT_Pulse_MoveRev(uint8_t motor_id,
                          uint8_t dir,
                          float rev,
                          float rpm)
{
    float pulse_f;
    uint32_t pulse;

    if (rev < 0.0f)
    {
        rev = -rev;
    }

    if (rev <= 0.0f)
    {
        return 0u;
    }

    pulse_f = rev * (float)ZDT_PULSE_PER_REV;
    pulse = (uint32_t)(pulse_f + 0.5f);

    if (pulse == 0u)
    {
        pulse = 1u;
    }

    return ZDT_Pulse_MovePulseAtRpm(motor_id, dir, pulse, rpm);
}

uint8_t ZDT_Pulse_StartSpeed(uint8_t motor_id,
                             uint8_t dir,
                             float rpm)
{
    ZDT_PulseMotor_t *m;
    HAL_StatusTypeDef ret;
    uint32_t pulse_hz;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return 0u;
    }

    if (m->busy)
    {
        return 0u;
    }

    ZDT_StopMotor(m);

    if (dir == ZDT_DIR_CW)
    {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
        m->dir_sign = 1;
    }
    else
    {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
        m->dir_sign = -1;
    }

    pulse_hz = ZDT_RpmToPulseHz(rpm);

    if (!ZDT_SetTimerPulseHz(m, pulse_hz))
    {
        m->dir_sign = 0;
        return 0u;
    }

    __HAL_TIM_SET_COUNTER(m->htim, 0u);
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_CC1);

    /*
     * 速度模式：
     * target_pulse = 0 表示不按脉冲数自动停止。
     * 这里用 Start_IT，走和位置模式相同的启动路径。
     */
    m->target_pulse = 0u;
    m->now_pulse = 0u;
    m->busy = 1u;
    m->speed_mode = 1u;
    m->last_cmd_rpm = rpm;

    ret = HAL_TIM_PWM_Start_IT(m->htim, m->channel);

    if (ret != HAL_OK)
    {
        m->busy = 0u;
        m->speed_mode = 0u;
        m->target_pulse = 0u;
        m->now_pulse = 0u;
        m->dir_sign = 0;
        return 0u;
    }

    return 1u;
}

void ZDT_Pulse_ZeroCmdPos(uint8_t motor_id)
{
    ZDT_PulseMotor_t *m;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return;
    }

    m->cmd_pos_pulse = 0;
}

int32_t ZDT_Pulse_GetCmdPosPulse(uint8_t motor_id)
{
    ZDT_PulseMotor_t *m;
    int32_t pos;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return 0;
    }

    __disable_irq();
    pos = m->cmd_pos_pulse;
    __enable_irq();

    return pos;
}

float ZDT_Pulse_GetCmdPosDeg(uint8_t motor_id)
{
    int32_t pos_pulse;

    pos_pulse = ZDT_Pulse_GetCmdPosPulse(motor_id);

    return ((float)pos_pulse * 360.0f) / (float)ZDT_PULSE_PER_REV;
}

float ZDT_Pulse_GetLastCmdRpm(uint8_t motor_id)
{
    ZDT_PulseMotor_t *m;

    m = ZDT_GetMotor(motor_id);

    if (m == 0)
    {
        return 0.0f;
    }

    return m->last_cmd_rpm;
}

void ZDT_Pulse_TIM_Callback(TIM_HandleTypeDef *htim)
{
    ZDT_PulseMotor_t *m;

    m = 0;

    if (htim->Instance == TIM10)
    {
        m = &s_m1;
    }
    else if (htim->Instance == TIM11)
    {
        m = &s_m2;
    }
    else
    {
        return;
    }

    if (m->busy == 0u)
    {
        return;
    }

    /*
     * 速度模式不用中断计数。
     * 正常情况下速度模式用 HAL_TIM_PWM_Start()，不会进这个回调。
     * 这里留保护。
     */
    if (m->speed_mode)
    {
        return;
    }

    m->now_pulse++;

    if (m->dir_sign > 0)
    {
        m->cmd_pos_pulse++;
    }
    else if (m->dir_sign < 0)
    {
        m->cmd_pos_pulse--;
    }

    if ((m->target_pulse != 0u) && (m->now_pulse >= m->target_pulse))
    {
        HAL_TIM_PWM_Stop_IT(m->htim, m->channel);

        m->busy = 0u;
        m->speed_mode = 0u;
        m->target_pulse = 0u;
        m->now_pulse = 0u;
        m->dir_sign = 0;

        __HAL_TIM_SET_COUNTER(m->htim, 0u);
    }
}
