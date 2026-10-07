#include "chassis_ctrl.h"

#include "chassis_hw.h"
#include "main.h"

#include <string.h>

#define DEG2RAD 0.01745329252f
#define RAD2DEG 57.2957795f

typedef struct
{
    float x_cm;
    float y_cm;
    float yaw_deg;

    float vx_cm_s;
    float vy_cm_s;
    float wz_deg_s;

    float wheel_raw[ROBOT_WHEEL_NUM];
    float wheel_fb[ROBOT_WHEEL_NUM];
    float wheel_sp[ROBOT_WHEEL_NUM];
    float duty[ROBOT_WHEEL_NUM];
    float integral[ROBOT_WHEEL_NUM];
} ChassisCtrl_InternalState_t;

static ChassisCtrl_InternalState_t s_chassis;
static volatile ChassisCtrl_Command_t s_command;

static float ChassisCtrl_Clamp(float value,
                               float min_value,
                               float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float ChassisCtrl_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float ChassisCtrl_Sign(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static void ChassisCtrl_StopWheels(void)
{
    uint8_t i;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        s_chassis.wheel_sp[i] = 0.0f;
        s_chassis.duty[i] = 0.0f;
        s_chassis.integral[i] = 0.0f;
    }

    ChassisHw_StopAll();
}

static void ChassisCtrl_UpdateFeedback(float dt_s)
{
    float fl;
    float rl;
    float rr;
    float fr;
    float yaw_linear_speed;
    int32_t wheel_delta[ROBOT_WHEEL_NUM];
    uint8_t i;

    ChassisHw_ReadWheelDelta(wheel_delta);

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        s_chassis.wheel_raw[i] =
            (float)wheel_delta[i] * ROBOT_CM_PER_COUNT / dt_s;

        s_chassis.wheel_fb[i] +=
            ROBOT_FB_ALPHA *
            (s_chassis.wheel_raw[i] - s_chassis.wheel_fb[i]);
    }

    fl = s_chassis.wheel_fb[0];
    rl = s_chassis.wheel_fb[1];
    rr = s_chassis.wheel_fb[2];
    fr = s_chassis.wheel_fb[3];

    s_chassis.vx_cm_s =
      ROBOT_ODOM_VX_SCALE *
      0.25f * (fl + rl + rr + fr);
		
		s_chassis.vy_cm_s =
      ROBOT_ODOM_VY_SCALE *
      0.25f * (-fl + rl - rr + fr);
		yaw_linear_speed = 0.25f * (-fl - rl + rr + fr);

    s_chassis.wz_deg_s =
        yaw_linear_speed / ROBOT_YAW_RADIUS_CM * RAD2DEG;

    /*
     * Preserve the proven current behaviour.
     * This is still body-frame velocity integrated directly into x/y.
     * World-frame odometry rotation will be handled in a later odom stage.
     */
    s_chassis.x_cm += s_chassis.vx_cm_s * dt_s;
    s_chassis.y_cm += s_chassis.vy_cm_s * dt_s;
    s_chassis.yaw_deg += s_chassis.wz_deg_s * dt_s;
}

static void ChassisCtrl_BodyToWheel(
    float vx,
    float vy,
    float wz_deg_s,
    float wheel[ROBOT_WHEEL_NUM])
{
    float yaw_speed;
    float maximum;
    float scale;
    uint8_t i;

    yaw_speed = wz_deg_s * DEG2RAD * ROBOT_YAW_RADIUS_CM;

    wheel[0] = vx - vy - yaw_speed; /* FL */
    wheel[1] = vx + vy - yaw_speed; /* RL */
    wheel[2] = vx - vy + yaw_speed; /* RR */
    wheel[3] = vx + vy + yaw_speed; /* FR */

    maximum = 0.0f;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        if (ChassisCtrl_Abs(wheel[i]) > maximum)
        {
            maximum = ChassisCtrl_Abs(wheel[i]);
        }
    }

    if (maximum > ROBOT_WHEEL_MAX_CM_S)
    {
        scale = ROBOT_WHEEL_MAX_CM_S / maximum;

        for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
        {
            wheel[i] *= scale;
        }
    }
}

static void ChassisCtrl_ApplyCommand(
    const ChassisCtrl_Command_t *command,
    float dt_s)
{
    float vx;
    float vy;
    float wz;
    uint8_t i;

    vx = ChassisCtrl_Clamp(command->vx_cm_s,
                           -ROBOT_VX_MAX_CM_S,
                           ROBOT_VX_MAX_CM_S);

    vy = ChassisCtrl_Clamp(command->vy_cm_s,
                           -ROBOT_VY_MAX_CM_S,
                           ROBOT_VY_MAX_CM_S);

    wz = ChassisCtrl_Clamp(command->wz_deg_s,
                           -ROBOT_WZ_MAX_DEG_S,
                           ROBOT_WZ_MAX_DEG_S);

    if ((ChassisCtrl_Abs(vx) < ROBOT_BODY_CMD_DEADBAND) &&
        (ChassisCtrl_Abs(vy) < ROBOT_BODY_CMD_DEADBAND) &&
        (ChassisCtrl_Abs(wz) < ROBOT_BODY_CMD_DEADBAND))
    {
        ChassisCtrl_StopWheels();
        return;
    }

    ChassisCtrl_BodyToWheel(vx, vy, wz, s_chassis.wheel_sp);

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        float setpoint;
        float feedback;
        float error;
        float duty;

        setpoint = s_chassis.wheel_sp[i];
        feedback = s_chassis.wheel_fb[i];

        if (ChassisCtrl_Abs(setpoint) <
            ROBOT_WHEEL_SP_DEADBAND_CM_S)
        {
            s_chassis.integral[i] = 0.0f;
            s_chassis.duty[i] = 0.0f;
            ChassisHw_SetWheelDuty(i, 0.0f);
            continue;
        }

        error = setpoint - feedback;

        s_chassis.integral[i] +=
            ROBOT_WHEEL_KI * error * dt_s;

        s_chassis.integral[i] =
            ChassisCtrl_Clamp(s_chassis.integral[i],
                              -ROBOT_WHEEL_I_MAX,
                              ROBOT_WHEEL_I_MAX);

        duty =
            ROBOT_WHEEL_KS * ChassisCtrl_Sign(setpoint) +
            ROBOT_WHEEL_KV * setpoint +
            ROBOT_WHEEL_KP * error +
            s_chassis.integral[i];

        if ((ChassisCtrl_Abs(setpoint) > ROBOT_START_SP_CM_S) &&
            (ChassisCtrl_Abs(feedback) < ROBOT_START_FB_CM_S))
        {
            float start_duty;

            start_duty =
                ROBOT_START_DUTY * ChassisCtrl_Sign(setpoint);

            if (ChassisCtrl_Abs(duty) <
                ChassisCtrl_Abs(start_duty))
            {
                duty = start_duty;
            }
        }

        /* Keep the proven no-active-reverse protection. */
        if ((setpoint > 0.0f) && (duty < 0.0f))
        {
            duty = 0.0f;
        }

        if ((setpoint < 0.0f) && (duty > 0.0f))
        {
            duty = 0.0f;
        }

        duty = ChassisCtrl_Clamp(duty,
                                 -ROBOT_DUTY_MAX,
                                 ROBOT_DUTY_MAX);

        s_chassis.duty[i] = duty;
        ChassisHw_SetWheelDuty(i, duty);
    }
}

void ChassisCtrl_Init(void)
{
    memset(&s_chassis, 0, sizeof(s_chassis));

    s_command.vx_cm_s = 0.0f;
    s_command.vy_cm_s = 0.0f;
    s_command.wz_deg_s = 0.0f;

    ChassisHw_Init();
    ChassisCtrl_StopWheels();
}

void ChassisCtrl_Enable(uint8_t enable)
{
    ChassisHw_Enable(enable);
}

void ChassisCtrl_SetBodyVelocity(float vx_cm_s,
                                 float vy_cm_s,
                                 float wz_deg_s)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    s_command.vx_cm_s =
        ChassisCtrl_Clamp(vx_cm_s,
                          -ROBOT_VX_MAX_CM_S,
                          ROBOT_VX_MAX_CM_S);

    s_command.vy_cm_s =
        ChassisCtrl_Clamp(vy_cm_s,
                          -ROBOT_VY_MAX_CM_S,
                          ROBOT_VY_MAX_CM_S);

    s_command.wz_deg_s =
        ChassisCtrl_Clamp(wz_deg_s,
                          -ROBOT_WZ_MAX_DEG_S,
                          ROBOT_WZ_MAX_DEG_S);

    __set_PRIMASK(primask);
}

void ChassisCtrl_StopCommand(void)
{
    ChassisCtrl_SetBodyVelocity(0.0f, 0.0f, 0.0f);
}

void ChassisCtrl_Update(float dt_s, uint8_t active)
{
    ChassisCtrl_Command_t command;

    ChassisCtrl_UpdateFeedback(dt_s);
    ChassisCtrl_GetCommand(&command);

    if (active != 0u)
    {
        ChassisCtrl_ApplyCommand(&command, dt_s);
    }
    else
    {
        ChassisCtrl_StopWheels();
    }
}

void ChassisCtrl_ForceStop(void)
{
    ChassisCtrl_StopCommand();
    ChassisCtrl_StopWheels();
}

void ChassisCtrl_ResetOdometry(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    s_chassis.x_cm = 0.0f;
    s_chassis.y_cm = 0.0f;
    s_chassis.yaw_deg = 0.0f;

    ChassisHw_ResetEncoderPosition();

    __set_PRIMASK(primask);
}

void ChassisCtrl_GetCommand(ChassisCtrl_Command_t *command)
{
    uint32_t primask;

    if (command == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    command->vx_cm_s = s_command.vx_cm_s;
    command->vy_cm_s = s_command.vy_cm_s;
    command->wz_deg_s = s_command.wz_deg_s;

    __set_PRIMASK(primask);
}

void ChassisCtrl_GetState(ChassisCtrl_State_t *state)
{
    uint8_t i;
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    state->x_cm = s_chassis.x_cm;
    state->y_cm = s_chassis.y_cm;
    state->yaw_deg = s_chassis.yaw_deg;

    state->vx_cm_s = s_chassis.vx_cm_s;
    state->vy_cm_s = s_chassis.vy_cm_s;
    state->wz_deg_s = s_chassis.wz_deg_s;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        state->wheel_speed_cm_s[i] = s_chassis.wheel_fb[i];
        state->wheel_setpoint_cm_s[i] = s_chassis.wheel_sp[i];
        state->wheel_duty[i] = s_chassis.duty[i];
    }

    __set_PRIMASK(primask);
}
