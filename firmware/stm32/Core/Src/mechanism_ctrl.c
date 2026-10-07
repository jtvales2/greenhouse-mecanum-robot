#include "mechanism_ctrl.h"

#include "robot_config.h"
#include "zdt_pulse.h"

#include <stdio.h>

static MechanismCtrl_State_t s_state;
static uint8_t s_m1_was_busy;

static uint8_t s_m2_dir;
static float s_m2_rpm;
static uint32_t s_m2_next_retry_ms;

static uint8_t MechanismCtrl_StartLink(
    MechanismCtrl_State_t target_state,
    uint8_t m1_dir,
    float m1_angle_deg,
    float m1_rpm,
    uint8_t m2_dir,
    float m2_rpm)
{
    uint8_t m1_started;
    uint8_t m2_started;

    if (ZDT_Pulse_IsBusy(1u) != 0u)
    {
        printf("[MECH] reject: M1 busy\r\n");
        return 0u;
    }

    /*
     * M2 is speed-mode. Stop it before changing Link direction.
     */
    ZDT_Pulse_Stop(2u);

    m1_started = ZDT_Pulse_MoveAngle(
        1u,
        m1_dir,
        m1_angle_deg,
        m1_rpm);

    if (m1_started == 0u)
    {
        s_state = MECHANISM_CTRL_ERROR;
        printf("[MECH] ERROR: M1 start failed\r\n");
        return 0u;
    }

    m2_started = ZDT_Pulse_StartSpeed(
        2u,
        m2_dir,
        m2_rpm);

    if (m2_started == 0u)
    {
        ZDT_Pulse_Stop(1u);

        s_state = MECHANISM_CTRL_ERROR;
        s_m1_was_busy = 0u;

        printf("[MECH] ERROR: M2 start failed\r\n");
        return 0u;
    }

    s_state = target_state;
    s_m1_was_busy = 1u;

    s_m2_dir = m2_dir;
    s_m2_rpm = m2_rpm;
    s_m2_next_retry_ms =
        HAL_GetTick() + ROBOT_MECH_M2_RETRY_MS;

    printf("[MECH] Link %c start "
           "M1(dir=%u angle=%.1f rpm=%.1f) "
           "M2(dir=%u rpm=%.1f)\r\n",
           (target_state == MECHANISM_CTRL_LINK_A) ? 'A' : 'B',
           (unsigned)m1_dir,
           m1_angle_deg,
           m1_rpm,
           (unsigned)m2_dir,
           m2_rpm);

    return 1u;
}

void MechanismCtrl_Init(void)
{
    ZDT_Pulse_Init();

    s_state = MECHANISM_CTRL_IDLE;
    s_m1_was_busy = 0u;

    s_m2_dir = 0u;
    s_m2_rpm = 0.0f;
    s_m2_next_retry_ms = 0u;
}

uint8_t MechanismCtrl_StartLinkA(void)
{
    return MechanismCtrl_StartLink(
        MECHANISM_CTRL_LINK_A,
        ROBOT_MECH_LINK_A_M1_DIR,
        ROBOT_MECH_LINK_A_M1_ANGLE_DEG,
        ROBOT_MECH_LINK_A_M1_RPM,
        ROBOT_MECH_LINK_A_M2_DIR,
        ROBOT_MECH_LINK_A_M2_RPM);
}

uint8_t MechanismCtrl_StartLinkB(void)
{
    return MechanismCtrl_StartLink(
        MECHANISM_CTRL_LINK_B,
        ROBOT_MECH_LINK_B_M1_DIR,
        ROBOT_MECH_LINK_B_M1_ANGLE_DEG,
        ROBOT_MECH_LINK_B_M1_RPM,
        ROBOT_MECH_LINK_B_M2_DIR,
        ROBOT_MECH_LINK_B_M2_RPM);
}

void MechanismCtrl_Stop(void)
{
    ZDT_Pulse_StopAll();

    s_state = MECHANISM_CTRL_IDLE;
    s_m1_was_busy = 0u;
    s_m2_next_retry_ms = 0u;

    printf("[MECH] STOP\r\n");
}

void MechanismCtrl_Update(uint32_t now_ms)
{
    uint8_t m1_busy;

    if ((s_state != MECHANISM_CTRL_LINK_A) &&
        (s_state != MECHANISM_CTRL_LINK_B))
    {
        return;
    }

    m1_busy = ZDT_Pulse_IsBusy(1u);

    if (m1_busy != 0u)
    {
        s_m1_was_busy = 1u;
    }
    else if (s_m1_was_busy != 0u)
    {
        s_m1_was_busy = 0u;

        printf("[MECH] M1 move complete; M2 keeps running\r\n");
    }

    /*
     * Preserve the proven old mechanism behaviour:
     * if M2 speed mode stops unexpectedly, retry the current Link speed.
     */
    if ((ZDT_Pulse_IsBusy(2u) == 0u) &&
        ((int32_t)(now_ms - s_m2_next_retry_ms) >= 0))
    {
        if (ZDT_Pulse_StartSpeed(
                2u,
                s_m2_dir,
                s_m2_rpm) != 0u)
        {
            printf("[MECH] M2 speed restarted\r\n");
        }

        s_m2_next_retry_ms =
            now_ms + ROBOT_MECH_M2_RETRY_MS;
    }
}

MechanismCtrl_State_t MechanismCtrl_GetState(void)
{
    return s_state;
}

uint8_t MechanismCtrl_IsM1Busy(void)
{
    return ZDT_Pulse_IsBusy(1u);
}

uint8_t MechanismCtrl_IsM2Busy(void)
{
    return ZDT_Pulse_IsBusy(2u);
}

void MechanismCtrl_TimPwmPulseFinishedCallback(
    TIM_HandleTypeDef *htim)
{
    ZDT_Pulse_TIM_Callback(htim);
}
