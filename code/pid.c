#include "pid.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数限幅
// 参数说明     amt             输入值, low/high 为下限和上限
// 返回参数     float           限幅后的值
// 使用示例     offset = constrain_float(offset, -LEAN_LIMIT, LEAN_LIMIT);
//-------------------------------------------------------------------------------------------------------------------
float constrain_float(float amt, float low, float high)
{
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     int16 数值限幅
// 参数说明     amt             输入值, low/high 为下限和上限
// 返回参数     int16           限幅后的值
// 使用示例     a = (int16)constrain_short(a, -FLYWHEEL_OUT_LIMIT, FLYWHEEL_OUT_LIMIT);
//-------------------------------------------------------------------------------------------------------------------
int16 constrain_short(int16 amt, int16 low, int16 high)
{
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 PID 参数并清除历史状态
// 参数说明     p               PID 结构体, kp/ki/kd 为系数, imax 为积分限幅
// 返回参数     void
// 使用示例     pid_set(&p_vel_pid, P_VEL_KP, P_VEL_KI, P_VEL_KD, P_VEL_IMAX);
//-------------------------------------------------------------------------------------------------------------------
void pid_set(pid_t *p, float kp, float ki, float kd, float imax)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->imax = imax;
    pid_reset(p);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除 PID 积分与误差历史
// 参数说明     p               PID 结构体指针
// 返回参数     void
// 使用示例     pid_reset(&p_vel_pid);
//-------------------------------------------------------------------------------------------------------------------
void pid_reset(pid_t *p)
{
    p->out_p = p->out_i = p->out_d = p->out = 0;
    p->integrator = 0;
    p->last_error = 0;
    p->last_derivative = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算位置式 PID 输出
// 参数说明     p               PID 结构体, error 为当前误差
// 返回参数     float           PID 输出
// 使用示例     pid_loc_calc(&p_angle_pid, p_vel_pid.out - att.pitch + zero);
//-------------------------------------------------------------------------------------------------------------------
float pid_loc_calc(pid_t *p, float error)
{
    p->integrator += error;
    p->integrator = constrain_float(p->integrator, -p->imax, p->imax);  // 积分限幅

    p->out_p = p->kp * error;
    p->out_i = p->ki * p->integrator;
    p->out_d = p->kd * (error - p->last_error);
    p->last_error = error;

    p->out = p->out_p + p->out_i + p->out_d;
    return p->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算增量式 PID 累积输出
// 参数说明     p               PID 结构体, error 为当前误差
// 返回参数     float           PID 累积输出
// 使用示例     pid_inc_calc(&pid, error);
//-------------------------------------------------------------------------------------------------------------------
float pid_inc_calc(pid_t *p, float error)
{
    p->out_p = p->kp * (error - p->last_error);
    p->out_i = p->ki * error;
    p->out_d = p->kd * ((error - p->last_error) - p->last_derivative);

    p->last_derivative = error - p->last_error;
    p->last_error = error;

    p->out += p->out_p + p->out_i + p->out_d;
    return p->out;
}
