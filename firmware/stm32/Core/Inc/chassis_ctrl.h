#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include "robot_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float vx_cm_s;
    float vy_cm_s;
    float wz_deg_s;
} ChassisCtrl_Command_t;

typedef struct
{
    float x_cm;
    float y_cm;
    float yaw_deg;

    float vx_cm_s;
    float vy_cm_s;
    float wz_deg_s;

    float wheel_speed_cm_s[ROBOT_WHEEL_NUM];
    float wheel_setpoint_cm_s[ROBOT_WHEEL_NUM];
    float wheel_duty[ROBOT_WHEEL_NUM];
} ChassisCtrl_State_t;

void ChassisCtrl_Init(void);
void ChassisCtrl_Enable(uint8_t enable);

/*
 * Coordinate convention:
 *   +vx = forward
 *   +vy = left
 *   +wz = counter-clockwise / left turn
 */
void ChassisCtrl_SetBodyVelocity(float vx_cm_s,
                                 float vy_cm_s,
                                 float wz_deg_s);

void ChassisCtrl_StopCommand(void);

/*
 * Update feedback first, then either execute the current body command
 * or force wheel outputs to zero when active == 0.
 */
void ChassisCtrl_Update(float dt_s, uint8_t active);

/* Immediate controller-state and wheel-output stop. */
void ChassisCtrl_ForceStop(void);

void ChassisCtrl_ResetOdometry(void);

void ChassisCtrl_GetCommand(ChassisCtrl_Command_t *command);
void ChassisCtrl_GetState(ChassisCtrl_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CTRL_H */
