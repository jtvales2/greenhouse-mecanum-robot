#ifndef ZDT_PULSE_H
#define ZDT_PULSE_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZDT_DIR_CW      0u
#define ZDT_DIR_CCW     1u

/*
 * 电机菜单 MStep = 16 时：
 * 1.8°步进电机：200整步/圈
 * 16细分：200 * 16 = 3200脉冲/圈
 *
 * 如果你以后把电机菜单 MStep 改成 32，这里要改成 6400。
 */
#define ZDT_PULSE_PER_REV_16STEP   3200u
#define ZDT_PULSE_PER_REV          ZDT_PULSE_PER_REV_16STEP

#define ZDT_PULSE_HALF_REV         (ZDT_PULSE_PER_REV / 2u)
#define ZDT_PULSE_ONE_REV          (ZDT_PULSE_PER_REV)

#define ZDT_DEFAULT_RPM            20.0f
#define ZDT_MIN_RPM                1.0f
#define ZDT_MAX_RPM                1000.0f

void ZDT_Pulse_Init(void);

uint8_t ZDT_Pulse_IsBusy(uint8_t motor_id);

void ZDT_Pulse_Stop(uint8_t motor_id);
void ZDT_Pulse_StopAll(void);

/* 位置模式：按脉冲数运动，使用默认转速 */
uint8_t ZDT_Pulse_MovePulse(uint8_t motor_id,
                            uint8_t dir,
                            uint32_t pulse);

uint8_t ZDT_Pulse_MoveHalfRev(uint8_t motor_id,
                              uint8_t dir);

uint8_t ZDT_Pulse_MoveOneRev(uint8_t motor_id,
                             uint8_t dir);

/* 位置模式：按脉冲数运动，并指定 RPM */
uint8_t ZDT_Pulse_MovePulseAtRpm(uint8_t motor_id,
                                 uint8_t dir,
                                 uint32_t pulse,
                                 float rpm);

/* 位置模式：按角度运动，并指定 RPM */
uint8_t ZDT_Pulse_MoveAngle(uint8_t motor_id,
                            uint8_t dir,
                            float angle_deg,
                            float rpm);

/* 位置模式：按圈数运动，并指定 RPM */
uint8_t ZDT_Pulse_MoveRev(uint8_t motor_id,
                          uint8_t dir,
                          float rev,
                          float rpm);

/*
 * 速度模式：一直按指定 RPM 转，直到 ZDT_Pulse_Stop()。
 * 注意：脉冲模式下这是“指令速度”，不是编码器反馈速度。
 */
uint8_t ZDT_Pulse_StartSpeed(uint8_t motor_id,
                             uint8_t dir,
                             float rpm);

/* 空闲时只修改某一路 PWM 频率，不启动运动 */
uint8_t ZDT_Pulse_SetRpm(uint8_t motor_id,
                         float rpm);

/*
 * STM32 自己估算的指令位置。
 * 不是电机编码器真实位置。
 */
void ZDT_Pulse_ZeroCmdPos(uint8_t motor_id);
int32_t ZDT_Pulse_GetCmdPosPulse(uint8_t motor_id);
float ZDT_Pulse_GetCmdPosDeg(uint8_t motor_id);
float ZDT_Pulse_GetLastCmdRpm(uint8_t motor_id);

/* 放到 HAL_TIM_PWM_PulseFinishedCallback() 里调用 */
void ZDT_Pulse_TIM_Callback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif
