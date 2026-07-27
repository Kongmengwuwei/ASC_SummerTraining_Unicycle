#include "pid.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点限幅
// 参数说明     amt/low/high    输入值与上下限
// 返回参数     float           限幅后的值
// 使用示例     offset = constrain_float(offset, -LEAN_LIMIT, LEAN_LIMIT);
//-------------------------------------------------------------------------------------------------------------------
float constrain_float(float amt, float low, float high)
{
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入增益并清掉历史状态
// 参数说明     p/kp/ki/kd/imax PID 实例、三个增益和积分限幅
// 返回参数     void
// 使用示例     pid_set(&p_vel_pid, P_VEL_KP, P_VEL_KI, P_VEL_KD, P_VEL_IMAX);
//-------------------------------------------------------------------------------------------------------------------
void pid_set(pid_t *p, float kp, float ki, float kd, float imax)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->imax = imax;
    pid_reset(p);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清积分、误差历史和增量式的累积输出
// 参数说明     p               PID 实例
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
// 函数简介     算一拍位置式 PID，只对积分限幅，输出不限幅
// 参数说明     p/error         PID 实例与当前误差
// 返回参数     float           本次绝对输出
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
// 函数简介     算一拍增量式 PID 并累加到 p->out，函数内不限幅，调用者必须自己钳并写回
// 参数说明     p/error         PID 实例与当前误差
// 返回参数     float           累加后的总输出，不是本次增量
// 使用示例     pid_inc_calc(&r_rate_pid, att.roll_rate + r_angle_pid.out);
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
