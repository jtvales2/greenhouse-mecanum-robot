#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t launched;
    float x_cm;
    float y_cm;
    float yaw_deg;
    float vx_cm_s;
    float vy_cm_s;
    float wz_deg_s;
    float wheel_speed_cm_s[4];
} RobotApp_State_t;

void RobotApp_Init(void);
void RobotApp_Loop(void);

/*
 * Chassis coordinate system:
 *   vx > 0: forward,  vy > 0: left,  wz > 0: turn left (CCW).
 * vx and wz may be non-zero together for a curved turn.
 */
void RobotApp_SetBodyVelocity(float vx_cm_s,
                              float vy_cm_s,
                              float wz_deg_s);

void RobotApp_Stop(void);
void RobotApp_StopAll(void);
uint8_t RobotApp_IsLaunched(void);

void RobotApp_Forward(float speed_cm_s);
void RobotApp_Backward(float speed_cm_s);
void RobotApp_StrafeLeft(float speed_cm_s);
void RobotApp_StrafeRight(float speed_cm_s);
void RobotApp_TurnLeft(float speed_deg_s);
void RobotApp_TurnRight(float speed_deg_s);

/*
 * Mechanism commands are accepted only after the KEY safety gate
 * has latched launched=1.
 */
uint8_t RobotApp_MechanismLinkA(void);
uint8_t RobotApp_MechanismLinkB(void);
void RobotApp_MechanismStop(void);

void RobotApp_ResetOdometry(void);
void RobotApp_GetState(RobotApp_State_t *state);

void RobotApp_UartTxCpltCallback(UART_HandleTypeDef *huart);
void RobotApp_UartRxCpltCallback(UART_HandleTypeDef *huart);
void RobotApp_UartErrorCallback(UART_HandleTypeDef *huart);
void RobotApp_TimPwmPulseFinishedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_APP_H */
