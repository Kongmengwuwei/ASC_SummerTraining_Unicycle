#ifndef CODE_PID_CONFIG_H_
#define CODE_PID_CONFIG_H_

#include "pid.h"

/*
 * Reserved balance-control cascade:
 * speed PID       -> desired body angle
 * angle PID       -> desired body angular rate
 * angular-rate PID-> actuator command
 *
 * PID_ConfigInit() is intentionally not called by the current application.
 */
extern pid_controller_t angular_rate_pid;
extern pid_controller_t angle_pid;
extern pid_controller_t speed_pid;

extern pid_config_t angular_rate_pid_config;
extern pid_config_t angle_pid_config;
extern pid_config_t speed_pid_config;

void PID_ConfigInit(void);
void PID_ConfigClearAll(void);

#endif
