#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float derivative_alpha;
    float sample_time_s;
} pid_config_t;

typedef struct
{
    float measurement;
    float setpoint;
    float error[3];

    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float derivative_alpha;
    float sample_time_s;

    float proportional;
    float integral;
    float derivative;
    float output;
    float previous_output;
} pid_controller_t;

void PID_Init(pid_controller_t *pid, const pid_config_t *config);
void PID_UpdateConfig(pid_controller_t *pid, const pid_config_t *config);
void PID_Clear(pid_controller_t *pid);

float PID_PositionCalculate(pid_controller_t *pid,
                            float measurement,
                            float setpoint);
float PID_IncrementalCalculate(pid_controller_t *pid,
                               float measurement,
                               float setpoint);

#endif
