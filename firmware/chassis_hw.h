#ifndef CHASSIS_HW_H
#define CHASSIS_HW_H

#include "robot_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Wheel order:
 *   0 = FL
 *   1 = RL
 *   2 = RR
 *   3 = FR
 *
 * Hardware abstraction contract:
 *   duty > 0  : wheel rolls toward robot forward
 *   delta > 0 : wheel rolls toward robot forward
 */

void ChassisHw_Init(void);
void ChassisHw_Enable(uint8_t enable);

void ChassisHw_SetWheelDuty(uint8_t index, float duty);
void ChassisHw_StopAll(void);

void ChassisHw_ReadWheelDelta(int32_t delta[ROBOT_WHEEL_NUM]);

/*
 * Preserve the old RobotApp_ResetOdometry() behaviour:
 * clear only the software encoder accumulation.
 * Timer counters and previous samples remain unchanged.
 */
void ChassisHw_ResetEncoderPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_HW_H */
