#include "MotorHAL.h"
#include <math.h>

#ifndef CLAMP01
#define CLAMP01(x) do{ if((x) < 0.0f) (x) = 0.0f; else if((x) > 1.0f) (x) = 1.0f; }while(0)
#endif

#ifndef MOTOR_ZERO_EPS
#define MOTOR_ZERO_EPS (0.001f)
#endif

#ifndef MOTOR_MAX_COUNT
#define MOTOR_MAX_COUNT (8u)
#endif

static const MotorCfg_t *g_cfg = NULL;
static uint8_t g_cnt = 0;
static int8_t  g_last_sign[MOTOR_MAX_COUNT] = {0};

static inline void _set_ccr(TIM_HandleTypeDef *htim, uint32_t ch, float duty_abs)
{
    uint32_t arr;
    uint32_t ccr;

    if (!htim) return;

    if (duty_abs < 0.0f) duty_abs = -duty_abs;
    CLAMP01(duty_abs);

    arr = __HAL_TIM_GET_AUTORELOAD(htim);
    ccr = (uint32_t)lroundf(duty_abs * (float)arr);
    if (ccr > arr) ccr = arr;

    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

static inline void _pwm_zero(const MotorCfg_t *c)
{
    if (!c) return;

    if (c->htim_pwm_a)
        __HAL_TIM_SET_COMPARE(c->htim_pwm_a, c->ch_pwm_a, 0);

    if (c->htim_pwm_b)
        __HAL_TIM_SET_COMPARE(c->htim_pwm_b, c->ch_pwm_b, 0);
}

static inline void _pwm_full_both(const MotorCfg_t *c)
{
    if (!c) return;

    if (c->htim_pwm_a)
        _set_ccr(c->htim_pwm_a, c->ch_pwm_a, 1.0f);

    if (c->htim_pwm_b)
        _set_ccr(c->htim_pwm_b, c->ch_pwm_b, 1.0f);
}

static inline void _dir_forward(const MotorCfg_t *c)
{
    if (!c || !c->in1_port || !c->in2_port) return;

    HAL_GPIO_WritePin(c->in1_port, c->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(c->in2_port, c->in2_pin, GPIO_PIN_RESET);
}

static inline void _dir_reverse(const MotorCfg_t *c)
{
    if (!c || !c->in1_port || !c->in2_port) return;

    HAL_GPIO_WritePin(c->in1_port, c->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(c->in2_port, c->in2_pin, GPIO_PIN_SET);
}

static inline void _dir_coast(const MotorCfg_t *c)
{
    if (!c || !c->in1_port || !c->in2_port) return;

    HAL_GPIO_WritePin(c->in1_port, c->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(c->in2_port, c->in2_pin, GPIO_PIN_RESET);
}

static inline void _dir_brake(const MotorCfg_t *c)
{
    if (!c || !c->in1_port || !c->in2_port) return;

    if (c->brake_is_short) {
        HAL_GPIO_WritePin(c->in1_port, c->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(c->in2_port, c->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(c->in1_port, c->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(c->in2_port, c->in2_pin, GPIO_PIN_RESET);
    }
}

static inline void _driver_enable(const MotorCfg_t *c, uint8_t en)
{
    GPIO_PinState s;

    if (!c || !c->en_port) return;

    if (c->en_active_high) {
        s = en ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        s = en ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(c->en_port, c->en_pin, s);
}

static inline int8_t _sign_from_duty(float duty)
{
    if (duty > MOTOR_ZERO_EPS) return +1;
    if (duty < -MOTOR_ZERO_EPS) return -1;
    return 0;
}

static inline int8_t _apply_dir_invert(const MotorCfg_t *c, int8_t sign)
{
    if (!c) return sign;
    return c->dir_inverted ? (int8_t)(-sign) : sign;
}

void Motor_InitAll(const MotorCfg_t *cfgs, uint8_t count)
{
    uint8_t i;

    g_cfg = cfgs;
    g_cnt = (count > MOTOR_MAX_COUNT) ? MOTOR_MAX_COUNT : count;

    for (i = 0u; i < g_cnt; i++) {
        const MotorCfg_t *c = &cfgs[i];

        g_last_sign[i] = 0;

        _driver_enable(c, 0u);
        _pwm_zero(c);
        _dir_coast(c);
    }

    for (i = 0u; i < g_cnt; i++) {
        const MotorCfg_t *c = &cfgs[i];

        if (c->htim_pwm_a) {
            HAL_TIM_PWM_Start(c->htim_pwm_a, c->ch_pwm_a);
            __HAL_TIM_SET_COMPARE(c->htim_pwm_a, c->ch_pwm_a, 0);
        }

        if ((c->mode == MOTOR_MODE_2PWM ||
             c->mode == MOTOR_MODE_2PWM_BRAKE) &&
             c->htim_pwm_b) {
            HAL_TIM_PWM_Start(c->htim_pwm_b, c->ch_pwm_b);
            __HAL_TIM_SET_COMPARE(c->htim_pwm_b, c->ch_pwm_b, 0);
        }
    }
}

void Motor_Enable(uint8_t idx, uint8_t en)
{
    if (idx >= g_cnt) return;
    _driver_enable(&g_cfg[idx], en);
}

void Motor_Set(uint8_t idx, float duty)
{
    const MotorCfg_t *c;
    float duty_abs;
    float inv_duty;
    int8_t raw_sign;
    int8_t phy_sign;

    if (idx >= g_cnt) return;

    c = &g_cfg[idx];

    duty_abs = fabsf(duty);
    CLAMP01(duty_abs);

    raw_sign = _sign_from_duty(duty);
    phy_sign = _apply_dir_invert(c, raw_sign);

    if (raw_sign == 0) {
        _pwm_zero(c);
        _dir_coast(c);
        g_last_sign[idx] = 0;
        return;
    }

    if (c->mode == MOTOR_MODE_1PWM_DIR) {
        /*
         * 1PWM + DIR 模式：
         * 用于 TB6612 / L298N 这类 PWM + 方向脚驱动。
         */
        if (phy_sign != g_last_sign[idx]) {
            _pwm_zero(c);

            if (phy_sign > 0)
                _dir_forward(c);
            else
                _dir_reverse(c);

            g_last_sign[idx] = phy_sign;
        }

        _set_ccr(c->htim_pwm_a, c->ch_pwm_a, duty_abs);

        if (c->htim_pwm_b) {
            _set_ccr(c->htim_pwm_b, c->ch_pwm_b, 0.0f);
        }
    }
    else if (c->mode == MOTOR_MODE_2PWM) {
        /*
         * 普通 2PWM 模式：
         *
         * 正转：A = duty, B = 0
         * 反转：A = 0,    B = duty
         *
         * 对 DRV8871 来说，这是 drive/coast。
         */
        g_last_sign[idx] = phy_sign;

        if (phy_sign > 0) {
            _set_ccr(c->htim_pwm_a, c->ch_pwm_a, duty_abs);
            _set_ccr(c->htim_pwm_b, c->ch_pwm_b, 0.0f);
        } else {
            _set_ccr(c->htim_pwm_a, c->ch_pwm_a, 0.0f);
            _set_ccr(c->htim_pwm_b, c->ch_pwm_b, duty_abs);
        }
    }
    else {
        /*
         * DRV8871 慢衰减 / drive-brake 模式：
         *
         * 正转：
         *   A = 100%
         *   B = 1 - duty
         *
         * 反转：
         *   A = 1 - duty
         *   B = 100%
         *
         * duty=0.60 时：
         *   60% 时间驱动
         *   40% 时间 IN1=IN2=1 短刹/慢衰减
         */
        g_last_sign[idx] = phy_sign;

        inv_duty = 1.0f - duty_abs;
        CLAMP01(inv_duty);

        if (phy_sign > 0) {
            _set_ccr(c->htim_pwm_a, c->ch_pwm_a, 1.0f);
            _set_ccr(c->htim_pwm_b, c->ch_pwm_b, inv_duty);
        } else {
            _set_ccr(c->htim_pwm_a, c->ch_pwm_a, inv_duty);
            _set_ccr(c->htim_pwm_b, c->ch_pwm_b, 1.0f);
        }
    }
}

void Motor_Brake(uint8_t idx)
{
    const MotorCfg_t *c;

    if (idx >= g_cnt) return;

    c = &g_cfg[idx];

    if (c->mode == MOTOR_MODE_2PWM_BRAKE && c->brake_is_short) {
        /*
         * DRV8871：IN1=IN2=1 为短刹。
         */
        _pwm_full_both(c);
    } else {
        _pwm_zero(c);
        _dir_brake(c);
    }

    g_last_sign[idx] = 0;
}

void Motor_Coast(uint8_t idx)
{
    const MotorCfg_t *c;

    if (idx >= g_cnt) return;

    c = &g_cfg[idx];

    _pwm_zero(c);
    _dir_coast(c);
    g_last_sign[idx] = 0;
}

void Motor_AllStop(void)
{
    uint8_t i;

    for (i = 0u; i < g_cnt; i++) {
        Motor_Coast(i);
    }
}

void Motor_MecanumMix(float fwd, float strafe, float yaw, float u[4])
{
    float fl;
    float fr;
    float rl;
    float rr;
    float maxv;

    fl = fwd - strafe - yaw;
    fr = fwd + strafe + yaw;
    rl = fwd + strafe - yaw;
    rr = fwd - strafe + yaw;

    maxv = fabsf(fl);
    if (fabsf(fr) > maxv) maxv = fabsf(fr);
    if (fabsf(rl) > maxv) maxv = fabsf(rl);
    if (fabsf(rr) > maxv) maxv = fabsf(rr);

    if (maxv < 1.0f) maxv = 1.0f;

    u[0] = fl / maxv;
    u[1] = fr / maxv;
    u[2] = rl / maxv;
    u[3] = rr / maxv;
}

void Motor_MecanumDrive(float fwd, float strafe, float yaw)
{
    float u[4];
    static const uint8_t motor_map[4] = {0, 3, 1, 2};
    uint8_t logical;

    if (g_cnt < 4u) return;

    Motor_MecanumMix(fwd, strafe, yaw, u);

    for (logical = 0u; logical < 4u; logical++) {
        Motor_Set(motor_map[logical], u[logical]);
    }
}
