#include "ros_serial_proto.h"
#include "robot_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_LINE_MAX     96u
#define TX_LINE_MAX     192u

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    char rx_line[RX_LINE_MAX];
    volatile uint16_t rx_len;
    volatile uint8_t line_ready;
    volatile uint8_t overflow;
    uint32_t last_cmd_ms;
    uint32_t last_status_ms;
    uint32_t rx_line_count;
    uint8_t online;
} RosSerial_t;

static RosSerial_t s_ros;

static void RosSerial_StartRx(void)
{
    if (s_ros.huart != 0)
    {
        (void)HAL_UART_Receive_IT(s_ros.huart, &s_ros.rx_byte, 1u);
    }
}

static float clamp_local(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void RosSerial_ProcessLine(char *line, uint32_t now_ms)
{
    float vx;
    float vy;
    float wz;
    char cmd[16];
    char mech_arg[16];

    while ((*line == ' ') || (*line == '\t')) line++;
    if (*line == '\0') return;

    if (sscanf(line, "%15s", cmd) != 1) return;

    if ((strcmp(cmd, "V") == 0) || (strcmp(cmd, "v") == 0))
    {
        if (sscanf(line, "%*s %f %f %f", &vx, &vy, &wz) == 3)
        {
            /* Units: vx/vy cm/s, wz deg/s. Limit again on STM32 side. */
            vx = clamp_local(vx, -300.0f, 300.0f);
            vy = clamp_local(vy, -300.0f, 300.0f);
            wz = clamp_local(wz, -360.0f, 360.0f);
            RobotApp_SetBodyVelocity(vx, vy, wz);
            s_ros.last_cmd_ms = now_ms;
            s_ros.online = 1u;
        }
        return;
    }

    if ((strcmp(cmd, "STOP") == 0) || (strcmp(cmd, "stop") == 0))
    {
        RobotApp_Stop();
        s_ros.last_cmd_ms = now_ms;
        s_ros.online = 1u;
        return;
    }


    if ((strcmp(cmd, "MECH") == 0) || (strcmp(cmd, "mech") == 0))
    {
        if (sscanf(line, "%*s %15s", mech_arg) == 1)
        {
            if ((strcmp(mech_arg, "A") == 0) ||
                (strcmp(mech_arg, "a") == 0))
            {
                (void)RobotApp_MechanismLinkA();
            }
            else if ((strcmp(mech_arg, "B") == 0) ||
                     (strcmp(mech_arg, "b") == 0))
            {
                (void)RobotApp_MechanismLinkB();
            }
            else if ((strcmp(mech_arg, "STOP") == 0) ||
                     (strcmp(mech_arg, "stop") == 0))
            {
                RobotApp_MechanismStop();
            }
            else
            {
                return;
            }

            s_ros.last_cmd_ms = now_ms;
            s_ros.online = 1u;
        }

        return;
    }

    if ((strcmp(cmd, "RESET_ODOM") == 0) || (strcmp(cmd, "reset_odom") == 0))
    {
        RobotApp_ResetOdometry();
        s_ros.last_cmd_ms = now_ms;
        s_ros.online = 1u;
        return;
    }

    if ((strcmp(cmd, "PING") == 0) || (strcmp(cmd, "ping") == 0))
    {
        const char *pong = "PONG\n";
        (void)HAL_UART_Transmit(s_ros.huart, (uint8_t *)pong, (uint16_t)strlen(pong), 20u);
        s_ros.last_cmd_ms = now_ms;
        s_ros.online = 1u;
        return;
    }
}

static void RosSerial_SendStatus(uint32_t now_ms)
{
    RobotApp_State_t st;
    char tx[TX_LINE_MAX];
    int n;

    if (s_ros.huart == 0) return;
    if ((uint32_t)(now_ms - s_ros.last_status_ms) < ROS_SERIAL_STATUS_PERIOD_MS) return;
    s_ros.last_status_ms = now_ms;

    RobotApp_GetState(&st);

    n = snprintf(tx, sizeof(tx),
                 "S %u %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                 (unsigned)st.launched,
                 st.x_cm,
                 st.y_cm,
                 st.yaw_deg,
                 st.vx_cm_s,
                 st.vy_cm_s,
                 st.wz_deg_s,
                 st.wheel_speed_cm_s[0],
                 st.wheel_speed_cm_s[1],
                 st.wheel_speed_cm_s[2],
                 st.wheel_speed_cm_s[3]);

    if ((n > 0) && (n < (int)sizeof(tx)))
    {
        (void)HAL_UART_Transmit(s_ros.huart, (uint8_t *)tx, (uint16_t)n, 20u);
    }
}

void RosSerial_Init(UART_HandleTypeDef *huart)
{
    memset(&s_ros, 0, sizeof(s_ros));
    s_ros.huart = huart;
    s_ros.last_cmd_ms = HAL_GetTick();
    s_ros.last_status_ms = s_ros.last_cmd_ms;
    RosSerial_StartRx();
}

void RosSerial_Service(uint32_t now_ms)
{
    char line[RX_LINE_MAX];

    if (s_ros.line_ready != 0u)
    {
        __disable_irq();
        strncpy(line, s_ros.rx_line, sizeof(line));
        line[sizeof(line) - 1u] = '\0';
        s_ros.line_ready = 0u;
        __enable_irq();

        RosSerial_ProcessLine(line, now_ms);
    }

    if ((s_ros.online != 0u) &&
        ((uint32_t)(now_ms - s_ros.last_cmd_ms) > ROS_SERIAL_CMD_TIMEOUT_MS))
    {
        RobotApp_Stop();
        s_ros.online = 0u;
    }

    RosSerial_SendStatus(now_ms);
}

void RosSerial_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t b;

    if ((s_ros.huart == 0) || (huart->Instance != s_ros.huart->Instance)) return;

    b = s_ros.rx_byte;

    if ((b == '\n') || (b == '\r'))
    {
        if (s_ros.rx_len > 0u)
        {
            if (s_ros.rx_len >= RX_LINE_MAX) s_ros.rx_len = RX_LINE_MAX - 1u;
            s_ros.rx_line[s_ros.rx_len] = '\0';
            s_ros.rx_len = 0u;
            s_ros.line_ready = 1u;
            s_ros.rx_line_count++;
        }
    }
    else
    {
        if (s_ros.rx_len < (RX_LINE_MAX - 1u))
        {
            s_ros.rx_line[s_ros.rx_len++] = (char)b;
        }
        else
        {
            s_ros.rx_len = 0u;
            s_ros.overflow = 1u;
        }
    }

    RosSerial_StartRx();
}

void RosSerial_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_ros.huart == 0) || (huart->Instance != s_ros.huart->Instance)) return;

    (void)HAL_UART_AbortReceive_IT(s_ros.huart);

#ifdef __HAL_UART_CLEAR_OREFLAG
    __HAL_UART_CLEAR_OREFLAG(s_ros.huart);
#endif
#ifdef __HAL_UART_CLEAR_FEFLAG
    __HAL_UART_CLEAR_FEFLAG(s_ros.huart);
#endif
#ifdef __HAL_UART_CLEAR_NEFLAG
    __HAL_UART_CLEAR_NEFLAG(s_ros.huart);
#endif

    s_ros.rx_len = 0u;
    RosSerial_StartRx();
}

uint8_t RosSerial_IsOnline(void)
{
    return s_ros.online;
}

uint32_t RosSerial_GetRxLineCount(void)
{
    return s_ros.rx_line_count;
}
