#ifndef MECHANISM_CTRL_H
#define MECHANISM_CTRL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MECHANISM_CTRL_IDLE = 0,
    MECHANISM_CTRL_LINK_A = 1,
    MECHANISM_CTRL_LINK_B = 2,
    MECHANISM_CTRL_ERROR = 3
} MechanismCtrl_State_t;

void MechanismCtrl_Init(void);

/*
 * Link A / B command semantics:
 *   M1 runs one configured angle move and then stops.
 *   M2 runs in speed mode and keeps running until another Link command
 *   changes its direction or MechanismCtrl_Stop() is called.
 *
 * A new Link command is rejected while M1 is still moving.
 */
uint8_t MechanismCtrl_StartLinkA(void);
uint8_t MechanismCtrl_StartLinkB(void);

void MechanismCtrl_Stop(void);
void MechanismCtrl_Update(uint32_t now_ms);

MechanismCtrl_State_t MechanismCtrl_GetState(void);
uint8_t MechanismCtrl_IsM1Busy(void);
uint8_t MechanismCtrl_IsM2Busy(void);

void MechanismCtrl_TimPwmPulseFinishedCallback(
    TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* MECHANISM_CTRL_H */
