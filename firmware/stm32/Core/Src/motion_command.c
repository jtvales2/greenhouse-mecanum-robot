#include "motion_command.h"

#include "main.h"
#include "robot_config.h"
#include "yaw_ctrl.h"

#include <string.h>

static volatile MotionCommand_BodyVelocity_t s_request;
static MotionCommand_State_t s_state;

static float MotionCommand_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float MotionCommand_Clamp(float value,
                                 float min_value,
                                 float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint8_t MotionCommand_IsTranslation(
    const MotionCommand_BodyVelocity_t *command)
{
    if (MotionCommand_Abs(command->vx_cm_s) >=
        ROBOT_BODY_CMD_DEADBAND)
    {
        return 1u;
    }

    if (MotionCommand_Abs(command->vy_cm_s) >=
        ROBOT_BODY_CMD_DEADBAND)
    {
        return 1u;
    }

    return 0u;
}

static uint8_t MotionCommand_IsStop(
    const MotionCommand_BodyVelocity_t *command)
{
    if (MotionCommand_Abs(command->vx_cm_s) >=
        ROBOT_BODY_CMD_DEADBAND)
    {
        return 0u;
    }

    if (MotionCommand_Abs(command->vy_cm_s) >=
        ROBOT_BODY_CMD_DEADBAND)
    {
        return 0u;
    }

    if (MotionCommand_Abs(command->wz_deg_s) >=
        ROBOT_BODY_CMD_DEADBAND)
    {
        return 0u;
    }

    return 1u;
}

static uint8_t MotionCommand_ModeWasYawHold(void)
{
    if (s_state.mode ==
        MOTION_COMMAND_MODE_YAW_HOLD_NORMAL)
    {
        return 1u;
    }

    if (s_state.mode ==
        MOTION_COMMAND_MODE_YAW_HOLD_STRAFE)
    {
        return 1u;
    }

    return 0u;
}

static void MotionCommand_CopyRequest(
    MotionCommand_BodyVelocity_t *command)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    command->vx_cm_s = s_request.vx_cm_s;
    command->vy_cm_s = s_request.vy_cm_s;
    command->wz_deg_s = s_request.wz_deg_s;

    __set_PRIMASK(primask);
}

static void MotionCommand_StoreState(
    const MotionCommand_State_t *state)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    s_state = *state;

    __set_PRIMASK(primask);
}

static void MotionCommand_LoadYawState(
    MotionCommand_State_t *state)
{
    YawCtrl_State_t yaw;

    YawCtrl_GetState(&yaw);

    state->yaw_hold_active = yaw.active;
    state->yaw_target_deg = yaw.target_yaw_deg;
    state->yaw_current_deg = yaw.current_yaw_deg;
    state->yaw_error_deg = yaw.error_deg;
    state->gz_deg_s = yaw.gz_deg_s;
    state->yaw_output_wz_deg_s =
        yaw.output_wz_deg_s;
}

void MotionCommand_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));

    s_request.vx_cm_s = 0.0f;
    s_request.vy_cm_s = 0.0f;
    s_request.wz_deg_s = 0.0f;

    s_state.mode = MOTION_COMMAND_MODE_DISABLED;

    YawCtrl_Init();
}

void MotionCommand_SetBodyVelocity(float vx_cm_s,
                                   float vy_cm_s,
                                   float wz_deg_s)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    s_request.vx_cm_s =
        MotionCommand_Clamp(vx_cm_s,
                            -ROBOT_VX_MAX_CM_S,
                            ROBOT_VX_MAX_CM_S);

    s_request.vy_cm_s =
        MotionCommand_Clamp(vy_cm_s,
                            -ROBOT_VY_MAX_CM_S,
                            ROBOT_VY_MAX_CM_S);

    s_request.wz_deg_s =
        MotionCommand_Clamp(wz_deg_s,
                            -ROBOT_WZ_MAX_DEG_S,
                            ROBOT_WZ_MAX_DEG_S);

    __set_PRIMASK(primask);
}

void MotionCommand_Stop(void)
{
    MotionCommand_SetBodyVelocity(0.0f, 0.0f, 0.0f);
}

void MotionCommand_Update(uint8_t active,
                          uint8_t imu_ready,
                          float yaw_deg,
                          float gz_deg_s)
{
    MotionCommand_State_t next;
    MotionCommand_BodyVelocity_t request;
    YawCtrl_Profile_t profile;
    uint8_t was_yaw_hold;
    float yaw_output;

    memset(&next, 0, sizeof(next));

    MotionCommand_CopyRequest(&request);

    next.request = request;
    next.imu_ready = imu_ready;

    was_yaw_hold = MotionCommand_ModeWasYawHold();

    if (active == 0u)
    {
        YawCtrl_Reset();

        next.mode = MOTION_COMMAND_MODE_DISABLED;

        MotionCommand_LoadYawState(&next);
        MotionCommand_StoreState(&next);
        return;
    }

    if (MotionCommand_IsStop(&request) != 0u)
    {
        YawCtrl_Reset();

        next.mode = MOTION_COMMAND_MODE_STOP;

        MotionCommand_LoadYawState(&next);
        MotionCommand_StoreState(&next);
        return;
    }

#if ROBOT_USE_YAW_HOLD
    if (imu_ready == 0u)
    {
        YawCtrl_Reset();

        next.mode = MOTION_COMMAND_MODE_IMU_BYPASS;
        next.effective = request;

        MotionCommand_LoadYawState(&next);
        MotionCommand_StoreState(&next);
        return;
    }
#else
    (void)yaw_deg;
    (void)gz_deg_s;

    YawCtrl_Reset();

    next.mode = MOTION_COMMAND_MODE_IMU_BYPASS;
    next.effective = request;

    MotionCommand_LoadYawState(&next);
    MotionCommand_StoreState(&next);
    return;
#endif

    /*
     * Explicit ROS angular command wins.
     *
     * A pure rotation request also always stays direct, even when its
     * magnitude is below the translating-command turn deadband.
     */
    if ((MotionCommand_Abs(request.wz_deg_s) >
         ROBOT_YAW_DIRECT_DEADBAND_DEG_S) ||
        (MotionCommand_IsTranslation(&request) == 0u))
    {
        YawCtrl_Reset();

        next.mode = MOTION_COMMAND_MODE_DIRECT_TURN;
        next.effective = request;

        MotionCommand_LoadYawState(&next);
        MotionCommand_StoreState(&next);
        return;
    }

    /*
     * Translation + no active turn:
     * preserve the proven gyro-integrated yaw-hold.
     *
     * Capture only when entering yaw-hold. Switching between normal
     * and strafe gains does not recapture the target heading.
     */
    if (was_yaw_hold == 0u)
    {
        YawCtrl_Capture(yaw_deg);
    }

    if (MotionCommand_Abs(request.vy_cm_s) >
        MotionCommand_Abs(request.vx_cm_s))
    {
        profile = YAW_CTRL_PROFILE_STRAFE;
        next.mode =
            MOTION_COMMAND_MODE_YAW_HOLD_STRAFE;
    }
    else
    {
        profile = YAW_CTRL_PROFILE_NORMAL;
        next.mode =
            MOTION_COMMAND_MODE_YAW_HOLD_NORMAL;
    }

    yaw_output =
        YawCtrl_Update(profile, yaw_deg, gz_deg_s);

#if ROBOT_FWD_YAW_RESERVE_ENABLE
    /*
     * Preserve the old high-speed forward yaw-reserve intent.
     * It is not applied to backward or strafe-dominant motion.
     */
    if ((profile == YAW_CTRL_PROFILE_NORMAL) &&
        (request.vx_cm_s >= ROBOT_FWD_MIN_SPEED_CM_S))
    {
        yaw_output *= ROBOT_FWD_YAW_RESERVE_GAIN;
        next.reserve_applied = 1u;
    }
#endif

    next.effective.vx_cm_s = request.vx_cm_s;
    next.effective.vy_cm_s = request.vy_cm_s;
    next.effective.wz_deg_s =
        MotionCommand_Clamp(yaw_output,
                            -ROBOT_WZ_MAX_DEG_S,
                            ROBOT_WZ_MAX_DEG_S);

    MotionCommand_LoadYawState(&next);

    /*
     * Keep the exact yaw-controller output in diagnostics.
     * effective.wz_deg_s is the final chassis-limited command.
     */
    next.yaw_output_wz_deg_s = yaw_output;

    MotionCommand_StoreState(&next);
}

void MotionCommand_GetRequest(
    MotionCommand_BodyVelocity_t *command)
{
    if (command == 0)
    {
        return;
    }

    MotionCommand_CopyRequest(command);
}

void MotionCommand_GetEffective(
    MotionCommand_BodyVelocity_t *command)
{
    uint32_t primask;

    if (command == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *command = s_state.effective;

    __set_PRIMASK(primask);
}

void MotionCommand_GetState(MotionCommand_State_t *state)
{
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *state = s_state;

    __set_PRIMASK(primask);
}
