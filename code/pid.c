#include "pid.h"

#include <stddef.h>

#define PID_MIN_SAMPLE_TIME_S (1.0e-6f)

static float PID_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PID_Clamp(float value, float limit)
{
    float positive_limit = PID_Absolute(limit);

    if (value > positive_limit)
    {
        return positive_limit;
    }
    if (value < -positive_limit)
    {
        return -positive_limit;
    }
    return value;
}

static float PID_ClampUnit(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

static void PID_PushError(pid_controller_t *pid,
                          float measurement,
                          float setpoint)
{
    pid->measurement = measurement;
    pid->setpoint = setpoint;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = setpoint - measurement;
}

void PID_Init(pid_controller_t *pid, const pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return;
    }

    pid->measurement = 0.0f;
    pid->setpoint = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->previous_output = 0.0f;
    PID_UpdateConfig(pid, config);
}

void PID_UpdateConfig(pid_controller_t *pid, const pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return;
    }

    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->integral_limit = PID_Absolute(config->integral_limit);
    pid->output_limit = PID_Absolute(config->output_limit);
    pid->derivative_alpha = PID_ClampUnit(config->derivative_alpha);
    pid->sample_time_s =
        (config->sample_time_s >= PID_MIN_SAMPLE_TIME_S)
            ? config->sample_time_s
            : PID_MIN_SAMPLE_TIME_S;

    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
    pid->output = PID_Clamp(pid->output, pid->output_limit);
    pid->previous_output =
        PID_Clamp(pid->previous_output, pid->output_limit);
}

void PID_Clear(pid_controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->measurement = 0.0f;
    pid->setpoint = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->previous_output = 0.0f;
}

float PID_PositionCalculate(pid_controller_t *pid,
                            float measurement,
                            float setpoint)
{
    float raw_derivative;
    float unsaturated_output;

    if (pid == NULL)
    {
        return 0.0f;
    }

    PID_PushError(pid, measurement, setpoint);

    pid->proportional = pid->kp * pid->error[0];
    pid->integral += pid->ki * pid->error[0] * pid->sample_time_s;
    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);

    raw_derivative =
        pid->kd * (pid->error[0] - pid->error[1]) /
        pid->sample_time_s;
    pid->derivative =
        pid->derivative_alpha * raw_derivative +
        (1.0f - pid->derivative_alpha) * pid->derivative;

    unsaturated_output =
        pid->proportional + pid->integral + pid->derivative;
    pid->previous_output = pid->output;
    pid->output = PID_Clamp(unsaturated_output, pid->output_limit);
    return pid->output;
}

float PID_IncrementalCalculate(pid_controller_t *pid,
                               float measurement,
                               float setpoint)
{
    float proportional_delta;
    float integral_delta;
    float derivative_delta;
    float raw_derivative_delta;
    float output_delta;

    if (pid == NULL)
    {
        return 0.0f;
    }

    PID_PushError(pid, measurement, setpoint);

    proportional_delta = pid->kp * (pid->error[0] - pid->error[1]);
    integral_delta = pid->ki * pid->error[0] * pid->sample_time_s;
    integral_delta = PID_Clamp(integral_delta, pid->integral_limit);

    raw_derivative_delta =
        pid->kd *
        (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) /
        pid->sample_time_s;
    derivative_delta =
        pid->derivative_alpha * raw_derivative_delta +
        (1.0f - pid->derivative_alpha) * pid->derivative;

    pid->proportional = proportional_delta;
    pid->integral = integral_delta;
    pid->derivative = derivative_delta;
    output_delta =
        proportional_delta + integral_delta + derivative_delta;

    pid->previous_output = pid->output;
    pid->output =
        PID_Clamp(pid->previous_output + output_delta,
                  pid->output_limit);
    return pid->output;
}
