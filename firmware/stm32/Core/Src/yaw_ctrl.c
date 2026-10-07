#include "yaw_ctrl.h"

#include "main.h"
#include "robot_config.h"

#include <string.h>

static YawCtrl_State_t s_yaw;

static float YawCtrl_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float YawCtrl_Clamp(float value,
                           float min_value,
                           float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float YawCtrl_WrapDeg(float angle_deg)
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

static void YawCtrl_SelectProfile(YawCtrl_Profile_t profile)
{
    s_yaw.profile = profile;

    if (profile == YAW_CTRL_PROFILE_STRAFE)
    {
        s_yaw.kp = ROBOT_STRAFE_YAW_HOLD_KP;
        s_yaw.kd = ROBOT_STRAFE_YAW_HOLD_KD;
        s_yaw.max_wz_deg_s = ROBOT_STRAFE_YAW_HOLD_MAX_DPS;
        s_yaw.deadband_deg =
            ROBOT_STRAFE_YAW_DEADBAND_DEG;
    }
    else
    {
        s_yaw.kp = ROBOT_YAW_HOLD_KP;
        s_yaw.kd = ROBOT_YAW_HOLD_KD;
        s_yaw.max_wz_deg_s = ROBOT_YAW_HOLD_MAX_DEG_S;
        s_yaw.deadband_deg =
            ROBOT_YAW_HOLD_DEADBAND_DEG;
    }
}

void YawCtrl_Init(void)
{
    memset(&s_yaw, 0, sizeof(s_yaw));
    YawCtrl_SelectProfile(YAW_CTRL_PROFILE_NORMAL);
}

void YawCtrl_Reset(void)
{
    float target_yaw_deg;

    target_yaw_deg = s_yaw.target_yaw_deg;

    memset(&s_yaw, 0, sizeof(s_yaw));

    s_yaw.target_yaw_deg = target_yaw_deg;
    YawCtrl_SelectProfile(YAW_CTRL_PROFILE_NORMAL);
}

void YawCtrl_Capture(float current_yaw_deg)
{
    s_yaw.target_yaw_deg =
        YawCtrl_WrapDeg(current_yaw_deg);

    s_yaw.current_yaw_deg =
        YawCtrl_WrapDeg(current_yaw_deg);

    s_yaw.error_deg = 0.0f;
    s_yaw.gz_deg_s = 0.0f;
    s_yaw.output_wz_deg_s = 0.0f;
    s_yaw.active = 1u;
}

float YawCtrl_Update(YawCtrl_Profile_t profile,
                     float current_yaw_deg,
                     float gz_deg_s)
{
    float error_deg;
    float output_wz_deg_s;

    if (s_yaw.active == 0u)
    {
        return 0.0f;
    }

    YawCtrl_SelectProfile(profile);

    s_yaw.current_yaw_deg =
        YawCtrl_WrapDeg(current_yaw_deg);

    s_yaw.gz_deg_s = gz_deg_s;

    error_deg = YawCtrl_WrapDeg(
        s_yaw.target_yaw_deg -
        s_yaw.current_yaw_deg);

    if (YawCtrl_Abs(error_deg) <
        s_yaw.deadband_deg)
    {
        error_deg = 0.0f;
    }

    /*
     * Preserve the proven competition yaw-hold core:
     *
     *   wz = Kp * yaw_error - Kd * gyro_z
     *
     * Coordinate contract already re-verified:
     *   left / CCW = +yaw = +gyro_z = +wz
     */
    output_wz_deg_s =
        s_yaw.kp * error_deg -
        s_yaw.kd * gz_deg_s;

    output_wz_deg_s =
        YawCtrl_Clamp(output_wz_deg_s,
                      -s_yaw.max_wz_deg_s,
                      s_yaw.max_wz_deg_s);

    s_yaw.error_deg = error_deg;
    s_yaw.output_wz_deg_s = output_wz_deg_s;

    return output_wz_deg_s;
}

void YawCtrl_GetState(YawCtrl_State_t *state)
{
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *state = s_yaw;

    __set_PRIMASK(primask);
}
