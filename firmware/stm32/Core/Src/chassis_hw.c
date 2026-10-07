#include "chassis_hw.h"

#include "gpio.h"
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *pwm_a_tim;
    uint32_t pwm_a_ch;
    TIM_HandleTypeDef *pwm_b_tim;
    uint32_t pwm_b_ch;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    int8_t inverted;
} ChassisHw_Motor_t;

typedef struct
{
    TIM_HandleTypeDef *tim;
    uint32_t prev;
    int32_t position;
    int32_t delta;
    int8_t sign;
} ChassisHw_Encoder_t;

static const ChassisHw_Motor_t s_motors[ROBOT_WHEEL_NUM] =
{
    /* 0 = FL */
    {&htim8, TIM_CHANNEL_1, &htim8, TIM_CHANNEL_2,
     EN2_GPIO_Port, EN2_Pin, ROBOT_MOTOR_INV_FL},

    /* 1 = RL */
    {&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_4,
     EN4_GPIO_Port, EN4_Pin, ROBOT_MOTOR_INV_RL},

    /* 2 = RR */
    {&htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2,
     EN1_GPIO_Port, EN1_Pin, ROBOT_MOTOR_INV_RR},

    /* 3 = FR */
    {&htim8, TIM_CHANNEL_3, &htim8, TIM_CHANNEL_4,
     EN3_GPIO_Port, EN3_Pin, ROBOT_MOTOR_INV_FR}
};

static ChassisHw_Encoder_t s_encoders[ROBOT_WHEEL_NUM] =
{
    {&htim3, 0u, 0, 0, ROBOT_ENC_SIGN_FL},
    {&htim2, 0u, 0, 0, ROBOT_ENC_SIGN_RL},
    {&htim4, 0u, 0, 0, ROBOT_ENC_SIGN_RR},
    {&htim5, 0u, 0, 0, ROBOT_ENC_SIGN_FR}
};

static const int8_t s_wheel_forward_sign[ROBOT_WHEEL_NUM] =
{
    ROBOT_WHEEL_FWD_SIGN_FL,
    ROBOT_WHEEL_FWD_SIGN_RL,
    ROBOT_WHEEL_FWD_SIGN_RR,
    ROBOT_WHEEL_FWD_SIGN_FR
};

static float ChassisHw_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float ChassisHw_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void ChassisHw_PwmSet(TIM_HandleTypeDef *tim,
                             uint32_t channel,
                             float duty)
{
    uint32_t arr;
    uint32_t compare;

    duty = ChassisHw_Clamp(duty, 0.0f, 1.0f);

    arr = __HAL_TIM_GET_AUTORELOAD(tim);
    compare = (uint32_t)(duty * (float)arr + 0.5f);

    if (compare > arr)
    {
        compare = arr;
    }

    __HAL_TIM_SET_COMPARE(tim, channel, compare);
}

static void ChassisHw_MotorSet(uint8_t index, float duty)
{
    const ChassisHw_Motor_t *motor;
    float magnitude;

    if (index >= ROBOT_WHEEL_NUM)
    {
        return;
    }

    motor = &s_motors[index];

    duty = ChassisHw_Clamp(duty, -ROBOT_DUTY_MAX, ROBOT_DUTY_MAX);

    if (motor->inverted != 0)
    {
        duty = -duty;
    }

    magnitude = ChassisHw_Abs(duty);

    if (magnitude < ROBOT_DUTY_DEADBAND)
    {
        ChassisHw_PwmSet(motor->pwm_a_tim, motor->pwm_a_ch, 0.0f);
        ChassisHw_PwmSet(motor->pwm_b_tim, motor->pwm_b_ch, 0.0f);
    }
    else if (duty > 0.0f)
    {
        ChassisHw_PwmSet(motor->pwm_a_tim, motor->pwm_a_ch, magnitude);
        ChassisHw_PwmSet(motor->pwm_b_tim, motor->pwm_b_ch, 0.0f);
    }
    else
    {
        ChassisHw_PwmSet(motor->pwm_a_tim, motor->pwm_a_ch, 0.0f);
        ChassisHw_PwmSet(motor->pwm_b_tim, motor->pwm_b_ch, magnitude);
    }
}

static void ChassisHw_EncoderInit(ChassisHw_Encoder_t *encoder)
{
    __HAL_TIM_SET_COUNTER(encoder->tim, 0u);

    encoder->prev = 0u;
    encoder->position = 0;
    encoder->delta = 0;

    (void)HAL_TIM_Encoder_Start(encoder->tim, TIM_CHANNEL_ALL);
}

static void ChassisHw_EncoderUpdate(ChassisHw_Encoder_t *encoder)
{
    uint32_t now;
    uint32_t arr;
    int32_t delta;

    now = __HAL_TIM_GET_COUNTER(encoder->tim);
    arr = __HAL_TIM_GET_AUTORELOAD(encoder->tim);

    if (arr <= 0xFFFFu)
    {
        delta = (int32_t)(int16_t)
                ((uint16_t)now - (uint16_t)encoder->prev);
        encoder->prev = (uint16_t)now;
    }
    else
    {
        delta = (int32_t)(now - encoder->prev);
        encoder->prev = now;
    }

    delta *= (encoder->sign >= 0) ? 1 : -1;

    if ((delta > ROBOT_ENC_DELTA_ABS_MAX) ||
        (delta < -ROBOT_ENC_DELTA_ABS_MAX))
    {
        delta = 0;
    }

    encoder->delta = delta;
    encoder->position += delta;
}

void ChassisHw_Init(void)
{
    uint8_t i;

    ChassisHw_Enable(0u);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);

    ChassisHw_StopAll();

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        ChassisHw_EncoderInit(&s_encoders[i]);
    }
}

void ChassisHw_Enable(uint8_t enable)
{
    uint8_t i;
    GPIO_PinState state;

    state = (enable != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        HAL_GPIO_WritePin(s_motors[i].en_port,
                          s_motors[i].en_pin,
                          state);
    }
}

void ChassisHw_SetWheelDuty(uint8_t index, float duty)
{
    ChassisHw_MotorSet(index, duty);
}

void ChassisHw_StopAll(void)
{
    uint8_t i;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        ChassisHw_MotorSet(i, 0.0f);
    }
}

void ChassisHw_ReadWheelDelta(int32_t delta[ROBOT_WHEEL_NUM])
{
    uint8_t i;

    if (delta == 0)
    {
        return;
    }

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        ChassisHw_EncoderUpdate(&s_encoders[i]);

        delta[i] = s_encoders[i].delta *
                   s_wheel_forward_sign[i];
    }
}

void ChassisHw_ResetEncoderPosition(void)
{
    uint8_t i;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        s_encoders[i].position = 0;
    }
}
