#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float vx_cm_s;
    float vy_cm_s;
    float wz_deg_s;
} MotionCommand_BodyVelocity_t;

typedef enum
{
    MOTION_COMMAND_MODE_DISABLED = 0,
    MOTION_COMMAND_MODE_STOP,
    MOTION_COMMAND_MODE_DIRECT_TURN,
    MOTION_COMMAND_MODE_YAW_HOLD_NORMAL,
    MOTION_COMMAND_MODE_YAW_HOLD_STRAFE,
    MOTION_COMMAND_MODE_IMU_BYPASS
} MotionCommand_Mode_t;

typedef struct
{
    MotionCommand_Mode_t mode;

    uint8_t imu_ready;
    uint8_t yaw_hold_active;
    uint8_t reserve_applied;

    MotionCommand_BodyVelocity_t request;
    MotionCommand_BodyVelocity_t effective;

    float yaw_target_deg;
    float yaw_current_deg;
    float yaw_error_deg;
    float gz_deg_s;
    float yaw_output_wz_deg_s;
} MotionCommand_State_t;

void MotionCommand_Init(void);

void MotionCommand_SetBodyVelocity(float vx_cm_s,
                                   float vy_cm_s,
                                   float wz_deg_s);

void MotionCommand_Stop(void);

/*
 * active:
 *   0 = safety gate closed; effective command is forced to zero.
 *   1 = command manager may execute the stored request.
 *
 * imu_ready:
 *   0 = yaw-hold is bypassed, raw request still remains executable.
 *   1 = translation with no active turn may use gyro yaw-hold.
 */
void MotionCommand_Update(uint8_t active,
                          uint8_t imu_ready,
                          float yaw_deg,
                          float gz_deg_s);

void MotionCommand_GetRequest(
    MotionCommand_BodyVelocity_t *command);

void MotionCommand_GetEffective(
    MotionCommand_BodyVelocity_t *command);

void MotionCommand_GetState(MotionCommand_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_COMMAND_H */
