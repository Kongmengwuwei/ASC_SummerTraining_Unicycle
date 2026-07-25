#include "PID_config.h"

/*
 * Gains intentionally remain zero until the motor, encoder direction and
 * feedback signs have been verified on the vehicle.
 */
#pragma section all "cpu0_dsram"
pid_controller_t angular_rate_pid;
pid_controller_t angle_pid;
pid_controller_t speed_pid;

pid_config_t angular_rate_pid_config =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 3000.0f,
    .output_limit = 10000.0f,
    .derivative_alpha = 0.8f,
    .sample_time_s = 0.001f
};

pid_config_t angle_pid_config =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 500.0f,
    .derivative_alpha = 0.8f,
    .sample_time_s = 0.005f
};

pid_config_t speed_pid_config =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 10.0f,
    .output_limit = 30.0f,
    .derivative_alpha = 0.9f,
    .sample_time_s = 0.010f
};
#pragma section all restore

void PID_ConfigInit(void)
{
    PID_Init(&angular_rate_pid, &angular_rate_pid_config);
    PID_Init(&angle_pid, &angle_pid_config);
    PID_Init(&speed_pid, &speed_pid_config);
}

void PID_ConfigClearAll(void)
{
    PID_Clear(&angular_rate_pid);
    PID_Clear(&angle_pid);
    PID_Clear(&speed_pid);
}
