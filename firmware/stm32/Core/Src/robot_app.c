#include "robot_app.h"

#include "chassis_ctrl.h"
#include "log_uart_dma.h"
#include "imu_yaw.h"
#include "mechanism_ctrl.h"
#include "motion_command.h"
#include "robot_config.h"
#include "ros_serial_proto.h"
#include "usart.h"

#include <stdio.h>

static uint32_t s_next_control_ms;
static uint32_t s_last_log_ms;
static uint8_t s_scheduler_started;

static uint8_t s_launched;
static uint8_t s_key_count;
static uint8_t s_key_stable;
static uint8_t s_key_stable_last;

#if ROBOT_MECH_SELF_TEST_ENABLE
static uint8_t s_mech_test_step;
#endif

#if ROBOT_YAW_SELF_TEST_ENABLE
static uint8_t s_yaw_test_step;
#endif

#if ROBOT_ODOM_CAL_TEST_ENABLE
static uint8_t s_odom_cal_running;
static float s_odom_cal_turn_accum_deg;
#endif

#if ((ROBOT_MECH_SELF_TEST_ENABLE + \
      ROBOT_YAW_SELF_TEST_ENABLE + \
      ROBOT_ODOM_CAL_TEST_ENABLE) > 1)
#error "Enable only one local KEY self-test at a time"
#endif

static float absf_local(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t start_key_pressed_event(void)
{
    uint8_t pressed;
    uint8_t event;

    pressed =
        (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ?
        1u : 0u;

    if (pressed != 0u)
    {
        if (s_key_count < ROBOT_START_KEY_DEBOUNCE_COUNT)
        {
            s_key_count++;
        }
    }
    else if (s_key_count > 0u)
    {
        s_key_count--;
    }

    s_key_stable =
        (s_key_count >= ROBOT_START_KEY_DEBOUNCE_COUNT) ? 1u : 0u;

    event =
        ((s_key_stable != 0u) &&
         (s_key_stable_last == 0u)) ? 1u : 0u;

    s_key_stable_last = s_key_stable;

    return event;
}

#if ROBOT_MECH_SELF_TEST_ENABLE
static void mechanism_self_test_next(void)
{
    switch (s_mech_test_step)
    {
        case 0u:
            if (RobotApp_MechanismLinkA() != 0u)
            {
                s_mech_test_step = 1u;
                printf("[MECH_TEST] LINK_A\r\n");
            }
            break;

        case 1u:
            RobotApp_MechanismStop();
            s_mech_test_step = 2u;
            printf("[MECH_TEST] STOP\r\n");
            break;

        case 2u:
            if (RobotApp_MechanismLinkB() != 0u)
            {
                s_mech_test_step = 3u;
                printf("[MECH_TEST] LINK_B\r\n");
            }
            break;

        case 3u:
        default:
            RobotApp_MechanismStop();
            s_mech_test_step = 0u;
            printf("[MECH_TEST] STOP\r\n");
            break;
    }
}
#endif

#if ROBOT_YAW_SELF_TEST_ENABLE
static void yaw_self_test_next(void)
{
    switch (s_yaw_test_step)
    {
        case 0u:
            RobotApp_SetBodyVelocity(
                ROBOT_YAW_SELF_TEST_LINEAR_CM_S,
                0.0f,
                0.0f);

            s_yaw_test_step = 1u;

            printf("[YAW_TEST] FORWARD_HOLD vx=%.1f\r\n",
                   ROBOT_YAW_SELF_TEST_LINEAR_CM_S);
            break;

        case 1u:
            RobotApp_Stop();
            s_yaw_test_step = 2u;
            printf("[YAW_TEST] STOP\r\n");
            break;

        case 2u:
            RobotApp_SetBodyVelocity(
                0.0f,
                ROBOT_YAW_SELF_TEST_LINEAR_CM_S,
                0.0f);

            s_yaw_test_step = 3u;

            printf("[YAW_TEST] STRAFE_LEFT_HOLD vy=%.1f\r\n",
                   ROBOT_YAW_SELF_TEST_LINEAR_CM_S);
            break;

        case 3u:
            RobotApp_Stop();
            s_yaw_test_step = 4u;
            printf("[YAW_TEST] STOP\r\n");
            break;

        case 4u:
            RobotApp_SetBodyVelocity(
                0.0f,
                0.0f,
                ROBOT_YAW_SELF_TEST_TURN_DEG_S);

            s_yaw_test_step = 5u;

            printf("[YAW_TEST] DIRECT_TURN_LEFT wz=%.1f\r\n",
                   ROBOT_YAW_SELF_TEST_TURN_DEG_S);
            break;

        case 5u:
        default:
            RobotApp_Stop();
            s_yaw_test_step = 0u;
            printf("[YAW_TEST] STOP\r\n");
            break;
    }
}
#endif

#if ROBOT_ODOM_CAL_TEST_ENABLE

static void odom_cal_test_key_event(void)
{
    ImuYaw_State_t imu;

    /*
     * 再按一次 KEY 可以人工中止标定。
     */
    if (s_odom_cal_running != 0u)
    {
        RobotApp_Stop();
        s_odom_cal_running = 0u;

        printf("[ODOM_CAL] ABORT by KEY\r\n");
        return;
    }

    ImuYaw_GetState(&imu);

    /*
     * 直线标定使用 yaw-hold。
     * IMU 未完成零偏校准时不允许起跑。
     */
    if (imu.ready == 0u)
    {
        printf("[ODOM_CAL] reject: IMU not ready\r\n");
        return;
    }

    /*
     * 每次测试重新建立统一原点：
     * encoder x/y = 0
     * WIT yaw      = 0
     */
    RobotApp_Stop();
RobotApp_ResetOdometry();

s_odom_cal_turn_accum_deg = 0.0f;
s_odom_cal_running = 1u;

printf("[ODOM_CAL] TURN START target=%.1f deg speed=%.1f deg/s\r\n",
       ROBOT_ODOM_CAL_TURN_TARGET_DEG,
       ROBOT_ODOM_CAL_TURN_DEG_S);
}


static void odom_cal_test_update(void)
{
    ChassisCtrl_State_t chassis;
    ImuYaw_State_t imu;
    float dt_s;

    if (s_odom_cal_running == 0u)
    {
        return;
    }

    ChassisCtrl_GetState(&chassis);
    ImuYaw_GetState(&imu);

    if (imu.ready == 0u)
    {
        RobotApp_Stop();

        printf("[ODOM_CAL] ABORT: IMU not ready\r\n");
        return;
    }

    dt_s = 0.001f * (float)ROBOT_CONTROL_PERIOD_MS;

    /*
     * IMU yaw itself wraps at +/-180 deg,
     * so accumulate gyro rate directly.
     *
     * abs() is intentional here because this calibration
     * measures total rotation magnitude.
     */
    s_odom_cal_turn_accum_deg +=
        absf_local(imu.gz_deg_s) * dt_s;

    /*
     * IMU says the chassis has completed one full revolution.
     */
    if (s_odom_cal_turn_accum_deg >=
        ROBOT_ODOM_CAL_TURN_TARGET_DEG)
    {
        RobotApp_Stop();

        printf("[ODOM_CAL] TURN DONE "
               "imu=%.2f deg "
               "encoder=%.2f deg "
               "Rold=%.3f cm\r\n",
               s_odom_cal_turn_accum_deg,
               chassis.yaw_deg,
               ROBOT_YAW_RADIUS_CM);

        return;
    }

    /*
     * Pure CCW / left rotation.
     */
    RobotApp_SetBodyVelocity(
        0.0f,
        0.0f,
        ROBOT_ODOM_CAL_TURN_DEG_S);
}

#endif

static void control_10ms(uint32_t now)
{
    ImuYaw_State_t imu;
    MotionCommand_BodyVelocity_t effective;
    uint8_t key_event;

    key_event = start_key_pressed_event();
	
	if ((s_launched == 0u) &&
    (key_event != 0u))
{
    s_launched = 1u;

    printf("[START] KEY pressed, controller enabled\r\n");
}
#if ROBOT_ODOM_CAL_TEST_ENABLE
else if ((s_launched != 0u) &&
         (key_event != 0u))
{
    odom_cal_test_key_event();
}
#elif ROBOT_MECH_SELF_TEST_ENABLE
else if ((s_launched != 0u) &&
         (key_event != 0u))
{
    mechanism_self_test_next();
}
#elif ROBOT_YAW_SELF_TEST_ENABLE
else if ((s_launched != 0u) &&
         (key_event != 0u))
{
    yaw_self_test_next();
}
#endif

    ImuYaw_Update10ms(
        0.001f * (float)ROBOT_CONTROL_PERIOD_MS,
        now);

    ImuYaw_GetState(&imu);
				
		#if ROBOT_ODOM_CAL_TEST_ENABLE
      odom_cal_test_update();
		#endif
				
    MotionCommand_Update(
        s_launched,
        imu.ready,
        imu.yaw_deg,
        imu.gz_deg_s);

    MotionCommand_GetEffective(&effective);

    ChassisCtrl_SetBodyVelocity(
        effective.vx_cm_s,
        effective.vy_cm_s,
        effective.wz_deg_s);

    MechanismCtrl_Update(now);

    ChassisCtrl_Update(
        0.001f * (float)ROBOT_CONTROL_PERIOD_MS,
        s_launched);
}

static void print_status(uint32_t now)
{
    ChassisCtrl_Command_t command;
    ChassisCtrl_State_t chassis;
    ImuYaw_State_t imu;
    MotionCommand_State_t motion;

    if ((uint32_t)(now - s_last_log_ms) < ROBOT_LOG_PERIOD_MS)
    {
        return;
    }

    s_last_log_ms = now;

    ChassisCtrl_GetCommand(&command);
    ChassisCtrl_GetState(&chassis);
    ImuYaw_GetState(&imu);
    MotionCommand_GetState(&motion);

    printf("[CHASSIS] launch=%u mech=(%u,%u,%u) "
           "cmd=(%.1f,%.1f,%.1f) "
           "pose=(%.1f,%.1f,%.1f) "
           "speed=(%.1f,%.1f,%.1f) "
           "sp=(%.1f,%.1f,%.1f,%.1f) "
           "fb=(%.1f,%.1f,%.1f,%.1f) "
           "u=(%.2f,%.2f,%.2f,%.2f)\r\n",
           (unsigned)s_launched,
           (unsigned)MechanismCtrl_GetState(),
           (unsigned)MechanismCtrl_IsM1Busy(),
           (unsigned)MechanismCtrl_IsM2Busy(),
           command.vx_cm_s,
           command.vy_cm_s,
           command.wz_deg_s,
           chassis.x_cm,
           chassis.y_cm,
           chassis.yaw_deg,
           chassis.vx_cm_s,
           chassis.vy_cm_s,
           chassis.wz_deg_s,
           chassis.wheel_setpoint_cm_s[0],
           chassis.wheel_setpoint_cm_s[1],
           chassis.wheel_setpoint_cm_s[2],
           chassis.wheel_setpoint_cm_s[3],
           chassis.wheel_speed_cm_s[0],
           chassis.wheel_speed_cm_s[1],
           chassis.wheel_speed_cm_s[2],
           chassis.wheel_speed_cm_s[3],
           chassis.wheel_duty[0],
           chassis.wheel_duty[1],
           chassis.wheel_duty[2],
           chassis.wheel_duty[3]);


    printf("[MOTION] mode=%u imu=%u hold=%u reserve=%u "
           "req=(%.1f,%.1f,%.1f) "
           "eff=(%.1f,%.1f,%.1f) "
           "yaw=(tar=%.2f cur=%.2f err=%.2f) "
           "gz=%.2f ywz=%.2f\r\n",
           (unsigned)motion.mode,
           (unsigned)motion.imu_ready,
           (unsigned)motion.yaw_hold_active,
           (unsigned)motion.reserve_applied,
           motion.request.vx_cm_s,
           motion.request.vy_cm_s,
           motion.request.wz_deg_s,
           motion.effective.vx_cm_s,
           motion.effective.vy_cm_s,
           motion.effective.wz_deg_s,
           motion.yaw_target_deg,
           motion.yaw_current_deg,
           motion.yaw_error_deg,
           motion.gz_deg_s,
           motion.yaw_output_wz_deg_s);

    printf("[IMU] online=%u fresh=%u ready=%u bias_ok=%u "
           "n=%u raw=%.3f bias=%.3f gz=%.3f yaw=%.2f\r\n",
           (unsigned)imu.online,
           (unsigned)imu.fresh,
           (unsigned)imu.ready,
           (unsigned)imu.bias_ok,
           (unsigned)imu.bias_sample_count,
           imu.gz_raw_deg_s,
           imu.bias_deg_s,
           imu.gz_deg_s,
           imu.yaw_deg);
}

void RobotApp_SetBodyVelocity(float vx_cm_s,
                              float vy_cm_s,
                              float wz_deg_s)
{
    MotionCommand_SetBodyVelocity(
        vx_cm_s,
        vy_cm_s,
        wz_deg_s);
}

void RobotApp_Stop(void)
{
#if ROBOT_ODOM_CAL_TEST_ENABLE
    /*
     * Any STOP source must also abort the local odometry
     * calibration run. This includes ROS STOP/watchdog.
     */
    s_odom_cal_running = 0u;
#endif

    MotionCommand_Stop();
    ChassisCtrl_StopCommand();
}

void RobotApp_StopAll(void)
{
#if ROBOT_ODOM_CAL_TEST_ENABLE
    s_odom_cal_running = 0u;
#endif

    MotionCommand_Stop();
    ChassisCtrl_ForceStop();
    MechanismCtrl_Stop();

    s_launched = 0u;
}

uint8_t RobotApp_IsLaunched(void)
{
    return s_launched;
}

void RobotApp_Forward(float speed_cm_s)
{
    RobotApp_SetBodyVelocity(
        absf_local(speed_cm_s),
        0.0f,
        0.0f);
}

void RobotApp_Backward(float speed_cm_s)
{
    RobotApp_SetBodyVelocity(
        -absf_local(speed_cm_s),
        0.0f,
        0.0f);
}

void RobotApp_StrafeLeft(float speed_cm_s)
{
    RobotApp_SetBodyVelocity(
        0.0f,
        absf_local(speed_cm_s),
        0.0f);
}

void RobotApp_StrafeRight(float speed_cm_s)
{
    RobotApp_SetBodyVelocity(
        0.0f,
        -absf_local(speed_cm_s),
        0.0f);
}

void RobotApp_TurnLeft(float speed_deg_s)
{
    RobotApp_SetBodyVelocity(
        0.0f,
        0.0f,
        absf_local(speed_deg_s));
}

void RobotApp_TurnRight(float speed_deg_s)
{
    RobotApp_SetBodyVelocity(
        0.0f,
        0.0f,
        -absf_local(speed_deg_s));
}

uint8_t RobotApp_MechanismLinkA(void)
{
    if (s_launched == 0u)
    {
        printf("[MECH] reject Link A: KEY not enabled\r\n");
        return 0u;
    }

    return MechanismCtrl_StartLinkA();
}

uint8_t RobotApp_MechanismLinkB(void)
{
    if (s_launched == 0u)
    {
        printf("[MECH] reject Link B: KEY not enabled\r\n");
        return 0u;
    }

    return MechanismCtrl_StartLinkB();
}

void RobotApp_MechanismStop(void)
{
    MechanismCtrl_Stop();
}

void RobotApp_ResetOdometry(void)
{
    ChassisCtrl_ResetOdometry();
    ImuYaw_ResetYaw();
}

void RobotApp_GetState(RobotApp_State_t *state)
{
    ChassisCtrl_State_t chassis;
	  ImuYaw_State_t imu;
    uint8_t i;
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    ChassisCtrl_GetState(&chassis);
		ImuYaw_GetState(&imu);

    primask = __get_PRIMASK();
    __disable_irq();
    state->launched = s_launched;
    __set_PRIMASK(primask);

    state->x_cm = chassis.x_cm;
		state->y_cm = chassis.y_cm;

/* 导航版：
 * x/y/vx/vy 使用编码器
 * yaw/wz 使用 WIT 陀螺仪
 */
		state->yaw_deg = imu.yaw_deg;
		state->vx_cm_s = chassis.vx_cm_s;
		state->vy_cm_s = chassis.vy_cm_s;
		state->wz_deg_s = imu.gz_deg_s;

    for (i = 0u; i < ROBOT_WHEEL_NUM; i++)
    {
        state->wheel_speed_cm_s[i] =
            chassis.wheel_speed_cm_s[i];
    }
}

int fputc(int ch, FILE *file)
{
    uint8_t data;

    data = (uint8_t)ch;
    (void)file;

    LogUart_Write(&data, 1u);

    return ch;
}

void RobotApp_Init(void)
{
    LogUart_Init(&huart1);

    ChassisCtrl_Init();
    ChassisCtrl_StopCommand();
    MotionCommand_Init();
    MechanismCtrl_Init();
    ImuYaw_Init(&huart2);

  #if ROBOT_AUTO_LAUNCH_ENABLE
    s_launched = 1u;
	#else
    s_launched = 0u;
	#endif
    s_key_count = 0u;
    s_key_stable = 0u;
    s_key_stable_last = 0u;
#if ROBOT_MECH_SELF_TEST_ENABLE
    s_mech_test_step = 0u;
#endif
#if ROBOT_YAW_SELF_TEST_ENABLE
    s_yaw_test_step = 0u;
#endif

    #if ROBOT_ODOM_CAL_TEST_ENABLE
    s_odom_cal_running = 0u;
    s_odom_cal_turn_accum_deg = 0.0f;
#endif

    RosSerial_Init(&huart3);

    ChassisCtrl_Enable(1u);

    s_next_control_ms = HAL_GetTick();
    s_last_log_ms = s_next_control_ms;
    s_scheduler_started = 1u;

    printf("\r\n[ROBOT] base controller ready\r\n");
    #if ROBOT_AUTO_LAUNCH_ENABLE
    printf("[ROBOT] auto launch enabled; zero command at startup\r\n");
#else
    printf("[ROBOT] waiting for KEY safety enable\r\n");
#endif
    printf("[ROBOT] +vx forward, +vy left, +wz left turn\r\n");
    printf("[ROBOT] WIT gyro yaw-hold enabled\r\n");
    printf("[ROBOT] mechanism commands: MECH A / MECH B / MECH STOP\r\n");
#if ROBOT_MECH_SELF_TEST_ENABLE
    printf("[MECH_TEST] KEY after enable: A -> STOP -> B -> STOP\r\n");
#elif ROBOT_YAW_SELF_TEST_ENABLE
    printf("[YAW_TEST] KEY after enable: FWD_HOLD -> STOP -> "
           "STRAFE_HOLD -> STOP -> DIRECT_TURN -> STOP\r\n");
#endif
#if ROBOT_ODOM_CAL_TEST_ENABLE
    printf("[ODOM_CAL] KEY: reset odom -> forward %.1f cm -> auto stop\r\n",
           ROBOT_ODOM_CAL_TARGET_CM);
#endif
}

void RobotApp_Loop(void)
{
    uint32_t now;

    now = HAL_GetTick();

    LogUart_Service();
    RosSerial_Service(now);

    if (s_scheduler_started == 0u)
    {
        s_next_control_ms = now;
        s_scheduler_started = 1u;
    }

    while ((int32_t)(now - s_next_control_ms) >= 0)
    {
        s_next_control_ms += ROBOT_CONTROL_PERIOD_MS;

        control_10ms(now);

        if ((int32_t)(now - s_next_control_ms) >= 0)
        {
            s_next_control_ms =
                now + ROBOT_CONTROL_PERIOD_MS;

            break;
        }
    }

    print_status(now);
}

void RobotApp_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
    LogUart_TxCpltCallback(huart);
}

void RobotApp_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    ImuYaw_UartRxCpltCallback(huart);
    RosSerial_RxCpltCallback(huart);
}

void RobotApp_UartErrorCallback(UART_HandleTypeDef *huart)
{
    ImuYaw_UartErrorCallback(huart);
    RosSerial_ErrorCallback(huart);
}

void RobotApp_TimPwmPulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    MechanismCtrl_TimPwmPulseFinishedCallback(htim);
}
