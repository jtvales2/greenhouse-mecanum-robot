#pragma once
#include "tim.h"
#include "gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_MODE_1PWM_DIR       = 0,
    MOTOR_MODE_2PWM           = 1,

    /*
     * DRV8871 推荐测试模式：
     *
     * 正转：
     *   IN1 = 100%
     *   IN2 = 1 - duty
     *
     * 反转：
     *   IN1 = 1 - duty
     *   IN2 = 100%
     *
     * 这相当于 drive/brake，也就是慢衰减。
     * 比普通 2PWM 的 drive/coast 更有低速扭矩。
     */
    MOTOR_MODE_2PWM_BRAKE     = 2
} MotorMode_t;

typedef struct {
    TIM_HandleTypeDef *htim_pwm_a;
    uint32_t           ch_pwm_a;

    TIM_HandleTypeDef *htim_pwm_b;
    uint32_t           ch_pwm_b;

    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;

    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    uint8_t            en_active_high;

    MotorMode_t        mode;

    /*
     * 1 = Motor_Brake() 时短刹。
     * 对 DRV8871 的 2PWM_BRAKE 模式，短刹会把 IN1/IN2 都拉高。
     */
    uint8_t            brake_is_short;

    /*
     * 方向反相：
     * 1 = 把正负方向互换
     */
    uint8_t            dir_inverted;
} MotorCfg_t;

void Motor_InitAll(const MotorCfg_t *cfgs, uint8_t count);
void Motor_Enable(uint8_t idx, uint8_t en);
void Motor_Set(uint8_t idx, float duty);
void Motor_Brake(uint8_t idx);
void Motor_Coast(uint8_t idx);
void Motor_AllStop(void);

void Motor_MecanumMix(float fwd, float strafe, float yaw, float u[4]);
void Motor_MecanumDrive(float fwd, float strafe, float yaw);

#ifdef __cplusplus
}
#endif
