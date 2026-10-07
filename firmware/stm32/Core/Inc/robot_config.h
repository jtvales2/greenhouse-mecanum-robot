#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DFH ROS2 base controller - single configuration source
 *
 * Parameter source:
 *   proven values migrated from the legacy robot_config.h.
 *
 * Bottom-layer and mechanism parameters are consumed directly through
 * ROBOT_* names. This is the only configuration source for the active base.
 */

/* ============================ Base scheduler ============================ */

#define ROBOT_WHEEL_NUM                         4u
#define ROBOT_CONTROL_PERIOD_MS                 10u
#define ROBOT_LOG_PERIOD_MS                     200u
#define ROBOT_START_KEY_DEBOUNCE_COUNT          5u

/*
 * 1 = 上电后自动进入 launched=1
 * 0 = 仍然需要按 KEY 解锁
 */
#define ROBOT_AUTO_LAUNCH_ENABLE                 1u
/* ============================ Wheel calibration ========================= */

#define ROBOT_WHEEL_DIAM_CM                     15.20f
#define ROBOT_PI                                3.14159265359f
#define ROBOT_COUNTS_PER_WHEEL_REV              3190.72f
#define ROBOT_CM_PER_COUNT \
    ((ROBOT_WHEEL_DIAM_CM * ROBOT_PI) / ROBOT_COUNTS_PER_WHEEL_REV)

#define ROBOT_YAW_RADIUS_CM               40.72f
#define ROBOT_ODOM_CAL_TURN_TARGET_DEG    360.0f
#define ROBOT_ODOM_CAL_TURN_DEG_S          30.0f
/*
 * 里程计校准系数
 * 先保持 1.0，稍后用卷尺实测后再改
 */
#define ROBOT_ODOM_VX_SCALE                     1.0250f
#define ROBOT_ODOM_VY_SCALE                     0.9400f
/*
 * Temporary odometry calibration test.
 *
 * 1 = KEY starts a fixed-distance forward calibration run.
 * 0 = disabled for normal ROS2/Nav2 operation.
 *
 * Test procedure:
 *   power on
 *   wait for IMU ready
 *   press KEY once
 *   odometry is reset
 *   chassis drives forward automatically
 *   stops when encoder odom reaches target
 */
#define ROBOT_ODOM_CAL_TEST_ENABLE               0u

#define ROBOT_ODOM_CAL_TARGET_CM               100.0f
#define ROBOT_ODOM_CAL_SPEED_CM_S               20.0f
/* ============================ Motion limits ============================= */

#define ROBOT_VX_MAX_CM_S                       140.0f
#define ROBOT_VY_MAX_CM_S                       100.0f
#define ROBOT_WZ_MAX_DEG_S                      180.0f
#define ROBOT_WHEEL_MAX_CM_S                    180.0f

/* ============================ Wheel speed loop ========================== */

#define ROBOT_WHEEL_KS                          0.19f
#define ROBOT_WHEEL_KV                          0.0070f
#define ROBOT_WHEEL_KP                          0.0045f
#define ROBOT_WHEEL_KI                          0.0000f
#define ROBOT_WHEEL_I_MAX                       0.08f

#define ROBOT_DUTY_MAX                          1.0f
#define ROBOT_DUTY_DEADBAND                     0.001f
#define ROBOT_WHEEL_SP_DEADBAND_CM_S            1.0f
#define ROBOT_BODY_CMD_DEADBAND                 0.05f
#define ROBOT_FB_ALPHA                          0.25f

#define ROBOT_START_SP_CM_S                     3.0f
#define ROBOT_START_FB_CM_S                     3.0f
#define ROBOT_START_DUTY                        0.40f

/* ============================ Motor directions ========================== */

#define ROBOT_MOTOR_INV_FL                      1
#define ROBOT_MOTOR_INV_RL                      1
#define ROBOT_MOTOR_INV_RR                      0
#define ROBOT_MOTOR_INV_FR                      0

/* ============================ Encoder directions ======================== */

#define ROBOT_ENC_SIGN_FL                       (+1)
#define ROBOT_ENC_SIGN_RL                       (+1)
#define ROBOT_ENC_SIGN_RR                       (-1)
#define ROBOT_ENC_SIGN_FR                       (-1)

#define ROBOT_WHEEL_FWD_SIGN_FL                 (+1)
#define ROBOT_WHEEL_FWD_SIGN_RL                 (+1)
#define ROBOT_WHEEL_FWD_SIGN_RR                 (+1)
#define ROBOT_WHEEL_FWD_SIGN_FR                 (+1)

#define ROBOT_ENC_DELTA_ABS_MAX                 500

/* ============================ WIT IMU / yaw hold ======================== */

/*
 * WIT module is used as a gyro source only for control.
 * Magnetometer and WIT fused yaw are intentionally not used by the
 * active yaw-control chain.
 */
#define ROBOT_USE_WIT_IMU                       1u

#define ROBOT_IMU_YAW_SIGN                      (+1.0f)
#define ROBOT_IMU_GZ_SIGN                       (+1.0f)
#define ROBOT_IMU_ONLINE_TIMEOUT_MS             300u
#define ROBOT_IMU_STARTUP_WAIT_MS               3000u

#define ROBOT_GYRO_BIAS_CAL_MS                  5000u
#define ROBOT_GYRO_BIAS_MIN_SAMPLES             250u
#define ROBOT_GYRO_BIAS_MAX_ABS_DPS             8.0f
#define ROBOT_GYRO_ZERO_DEADBAND_DPS            0.50f

#define ROBOT_USE_YAW_HOLD                      1u

#define ROBOT_YAW_HOLD_KP                       12.0f
#define ROBOT_YAW_HOLD_KD                       0.65f
#define ROBOT_YAW_HOLD_MAX_DEG_S                95.0f
#define ROBOT_YAW_HOLD_DEADBAND_DEG             0.05f

#define ROBOT_FWD_YAW_RESERVE_ENABLE            1u
#define ROBOT_FWD_YAW_RESERVE_GAIN              1.4f
#define ROBOT_FWD_MIN_SPEED_CM_S                90.0f

#define ROBOT_STRAFE_YAW_HOLD_KP                16.0f
#define ROBOT_STRAFE_YAW_HOLD_KD                0.55f
#define ROBOT_STRAFE_YAW_HOLD_MAX_DPS           145.0f
#define ROBOT_STRAFE_YAW_DEADBAND_DEG           0.05f


/* ============================ Motion command / yaw manager ============== */

/*
 * ROS cmd_vel angular residuals smaller than this are treated as
 * "no active turn" while translating, so gyro yaw-hold may take over.
 */
#define ROBOT_YAW_DIRECT_DEADBAND_DEG_S          0.50f

/*
 * Temporary no-Raspberry-Pi regression.
 *
 * First KEY press:
 *   safety enable only
 *
 * Following KEY sequence:
 *   FORWARD_HOLD -> STOP
 *   STRAFE_LEFT_HOLD -> STOP
 *   DIRECT_TURN_LEFT -> STOP
 *
 * Set to 0 after yaw-hold regression passes.
 */
#define ROBOT_YAW_SELF_TEST_ENABLE               0u
#define ROBOT_YAW_SELF_TEST_LINEAR_CM_S          30.0f
#define ROBOT_YAW_SELF_TEST_TURN_DEG_S           30.0f

/* ============================ Local mechanism self-test ================= */

/*
 * 1 = no Raspberry Pi required.
 *
 * KEY sequence:
 *   first KEY  -> latch controller safety enable
 *   next KEY   -> Link A
 *   next KEY   -> mechanism STOP
 *   next KEY   -> Link B
 *   next KEY   -> mechanism STOP
 *
 * Wait for "[MECH] M1 move complete" before advancing from Link A/B.
 * Set back to 0 after the mechanism regression test.
 */
#define ROBOT_MECH_SELF_TEST_ENABLE             0u

/* ============================ Mechanism Link A / B ====================== */

#define ROBOT_MECH_LINK_A_M1_DIR                0u
#define ROBOT_MECH_LINK_A_M1_ANGLE_DEG          420.0f
#define ROBOT_MECH_LINK_A_M1_RPM                150.0f

#define ROBOT_MECH_LINK_A_M2_DIR                1u
#define ROBOT_MECH_LINK_A_M2_RPM                100.0f

#define ROBOT_MECH_LINK_B_M1_DIR                1u
#define ROBOT_MECH_LINK_B_M1_ANGLE_DEG          420.0f
#define ROBOT_MECH_LINK_B_M1_RPM                150.0f

#define ROBOT_MECH_LINK_B_M2_DIR                0u
#define ROBOT_MECH_LINK_B_M2_RPM                100.0f

#define ROBOT_MECH_M2_RETRY_MS                  500u

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_H */
