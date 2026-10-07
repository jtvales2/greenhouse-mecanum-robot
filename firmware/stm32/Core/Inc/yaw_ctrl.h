#ifndef YAW_CTRL_H
#define YAW_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    YAW_CTRL_PROFILE_NORMAL = 0,
    YAW_CTRL_PROFILE_STRAFE = 1
} YawCtrl_Profile_t;

typedef struct
{
    uint8_t active;
    YawCtrl_Profile_t profile;

    float target_yaw_deg;
    float current_yaw_deg;
    float error_deg;
    float gz_deg_s;
    float output_wz_deg_s;

    float kp;
    float kd;
    float max_wz_deg_s;
    float deadband_deg;
} YawCtrl_State_t;

void YawCtrl_Init(void);
void YawCtrl_Reset(void);

void YawCtrl_Capture(float current_yaw_deg);

float YawCtrl_Update(YawCtrl_Profile_t profile,
                     float current_yaw_deg,
                     float gz_deg_s);

void YawCtrl_GetState(YawCtrl_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* YAW_CTRL_H */
