#include "balance.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include "board_config.h"
#include "pid.h"
#include "attitude.h"
#include "imu.h"
#include "control.h"
#include <math.h>

// 全局状态
start_state_t start_flag = START_STOP;                 // 启动状态机标志, 默认全停

float g_roll_zero, g_pitch_zero;                       // 横滚/俯仰机械零点(静态)
float g_lean_offset = 0;                               // 压弯动态零点偏移(叠加到横滚零点)
float g_pwm_roll, g_pwm_pitch, g_pwm_yaw;              // 各轴串级最终输出
int16 g_motor_a, g_motor_b, g_motor_c;                 // 最终三电机输出量
int   g_target_distance = 0;                           // 期望行进速度目标(pitch 速度环)
float g_yaw_target = 0;                                // 航向目标(°)
balance_dbg_t g_bal_dbg;                               // 串级各环中间量(供 vofa 波形遥测快照)

static pid_t r_rcy_pid, r_angle_pid, r_rate_pid;       // roll : 飞轮回收/角度/角速度
static pid_t p_vel_pid, p_angle_pid, p_rate_pid;       // pitch: 速度/角度/角速度
static pid_t y_angle_pid, y_rate_pid;                  // yaw  : 转向(外)/角速度(内)

static volatile uint8 s_test_running;                  // Test/Wave 运行标志
static tune_axis_t s_test_axis;
static tune_ring_t s_test_ring;
static uint16 s_test_tick;
static int16 s_test_speed_c;

// 内部函数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     分别向 W_Motor 与 Y_Motor 下发三电机控制量
// 参数说明     motor_a/b/c     动量轮 A、动量轮 B、行进轮 C 控制量
// 返回参数     void
// 使用示例     balance_motor_output(g_motor_a, g_motor_b, g_motor_c);
//-------------------------------------------------------------------------------------------------------------------
static void balance_motor_output(int16 motor_a, int16 motor_b, int16 motor_c)
{
    W_Motor_SetDuty(motor_a, motor_b);
    Y_Motor_SetDuty(motor_c);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     锁定动量轮并停止三路电机输出
// 参数说明     void
// 返回参数     void
// 使用示例     balance_motor_stop();
//-------------------------------------------------------------------------------------------------------------------
static void balance_motor_stop(void)
{
    W_Motor_Stop();
    Y_Motor_Stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把菜单可调的各环增益刷进 pid 结构(不清运行状态), 每控制周期调一次
// 参数说明     void
// 返回参数     void
// 使用示例     cascade_gain_refresh();
//-------------------------------------------------------------------------------------------------------------------
static void cascade_gain_refresh(void)
{
    r_rcy_pid.kp   = R_RCY_KP;    r_rcy_pid.ki   = R_RCY_KI;   r_rcy_pid.kd   = R_RCY_KD;
    r_angle_pid.kp = R_ANGLE_KP;  r_angle_pid.ki = R_ANGLE_KI; r_angle_pid.kd = R_ANGLE_KD;
    r_rate_pid.kp  = R_RATE_KP;   r_rate_pid.ki  = R_RATE_KI;  r_rate_pid.kd  = R_RATE_KD;

    p_vel_pid.kp   = P_VEL_KP;    p_vel_pid.ki   = P_VEL_KI;   p_vel_pid.kd   = P_VEL_KD;
    p_angle_pid.kp = P_ANGLE_KP;  p_angle_pid.ki = P_ANGLE_KI; p_angle_pid.kd = P_ANGLE_KD;
    p_rate_pid.kp  = P_RATE_KP;   p_rate_pid.ki  = P_RATE_KI;  p_rate_pid.kd  = P_RATE_KD;

    y_angle_pid.kp = Y_ANGLE_KP;  y_angle_pid.ki = Y_ANGLE_KI; y_angle_pid.kd = Y_ANGLE_KD;
    y_rate_pid.kp  = Y_RATE_KP;   y_rate_pid.ki  = Y_RATE_KI;  y_rate_pid.kd  = Y_RATE_KD;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位全部串级 PID 状态(积分/误差历史/增量式累积输出)与压弯偏移
// 参数说明     void
// 返回参数     void
// 使用示例     cascade_reset();
//-------------------------------------------------------------------------------------------------------------------
static void cascade_reset(void)
{
    pid_reset(&r_rcy_pid);  pid_reset(&r_angle_pid); pid_reset(&r_rate_pid);
    pid_reset(&p_vel_pid);  pid_reset(&p_angle_pid); pid_reset(&p_rate_pid);
    pid_reset(&y_angle_pid); pid_reset(&y_rate_pid);
    g_lean_offset = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断浮点是否为有限值(非 NaN / 非 ±Inf)
// 参数说明     v               待判值
// 返回参数     uint8           1=有限值 0=NaN/Inf
// 使用示例     if (!bal_is_finite(att.roll)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 bal_is_finite(float v)
{
    if (v != v) return 0;                               // NaN
    if (v > 3.0e38f || v < -3.0e38f) return 0;          // ±Inf / 量级异常
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按测试限幅驱动行进轮 C，动量轮保持软件刹车
// 参数说明     command         控制输出，方向与死区补偿在 Y_Motor 内处理
// 返回参数     void
// 使用示例     test_drive_c_output(g_pwm_pitch);
//-------------------------------------------------------------------------------------------------------------------
static void test_drive_c_output(float command)
{
    int32 out = func_limit((int32)command, BAL_TEST_DRIVE_LIMIT);

    W_Motor_Stop();
    Y_Motor_SetDuty(out);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查测试所需的姿态数据与保护角
// 参数说明     void
// 返回参数     uint8           1=数据有效 0=立即停止测试
// 使用示例     if (!test_input_valid()) balance_test_stop();
//-------------------------------------------------------------------------------------------------------------------
static uint8 test_input_valid(void)
{
    if (!g_imu_ok || imu_link_lost() || attitude_diverged() || !attitude_converged()) return 0;
    if (s_test_axis != TUNE_AXIS_PITCH && W_Motor_LinkLost()) return 0;  // 飞轮测试要求驱动通讯在线
    if (!bal_is_finite(att.roll) || !bal_is_finite(att.pitch) || !bal_is_finite(att.roll_rate) ||
        !bal_is_finite(att.pitch_rate) || !bal_is_finite(imu.gyro_z)) return 0;
    if (fabsf(att.roll - g_roll_zero) > ROLL_PROTECT_ANGLE) return 0;
    if (fabsf(att.pitch - g_pitch_zero) > PITCH_PROTECT_ANGLE) return 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行 Roll 测试级联并严格旁路未选外环
// 参数说明     run5/run20      角度环与回收环分频标志
// 返回参数     float           Roll 测试输出
// 使用示例     output = roll_test_ctrl(run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float roll_test_ctrl(uint8 run5, uint8 run20)
{
    if (s_test_ring >= TUNE_RING_VEL)
    {
        if (run20) pid_loc_calc(&r_rcy_pid, -(float)(W_Motor_GetSpeed2() - W_Motor_GetSpeed1()));
    }
    else
    {
        pid_reset(&r_rcy_pid);
    }

    if (s_test_ring >= TUNE_RING_ANGLE)
    {
        if (run5) pid_loc_calc(&r_angle_pid, r_rcy_pid.out + att.roll - g_roll_zero);
    }
    else
    {
        pid_reset(&r_angle_pid);
    }

    pid_inc_calc(&r_rate_pid, att.roll_rate + r_angle_pid.out);
    r_rate_pid.out = constrain_float(r_rate_pid.out, -BAL_TEST_FLY_LIMIT, BAL_TEST_FLY_LIMIT);

    g_bal_dbg.r_rcy_set = 0.0f;
    g_bal_dbg.r_rcy_fb = (float)(W_Motor_GetSpeed2() - W_Motor_GetSpeed1());
    g_bal_dbg.r_rcy_out = r_rcy_pid.out;
    g_bal_dbg.r_ang_fb = att.roll;
    g_bal_dbg.r_ang_out = r_angle_pid.out;
    g_bal_dbg.r_rate_fb = att.roll_rate;
    g_bal_dbg.r_pwm = r_rate_pid.out;
    return r_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行 Pitch 测试级联并严格旁路未选外环
// 参数说明     run5/run20      角度环与速度环分频标志
// 返回参数     float           Pitch 测试输出
// 使用示例     output = pitch_test_ctrl(run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float pitch_test_ctrl(uint8 run5, uint8 run20)
{
    if (s_test_ring >= TUNE_RING_VEL)
    {
        if (run20)
        {
            s_test_speed_c = Y_Motor_GetSpeed20ms();
            pid_loc_calc(&p_vel_pid, (float)s_test_speed_c - (float)g_target_distance);
        }
    }
    else
    {
        pid_reset(&p_vel_pid);
    }

    if (s_test_ring >= TUNE_RING_ANGLE)
    {
        if (run5) pid_loc_calc(&p_angle_pid, p_vel_pid.out - att.pitch + g_pitch_zero);
    }
    else
    {
        pid_reset(&p_angle_pid);
    }

    pid_inc_calc(&p_rate_pid, -att.pitch_rate + p_angle_pid.out);
    p_rate_pid.out = constrain_float(p_rate_pid.out, -BAL_TEST_DRIVE_LIMIT, BAL_TEST_DRIVE_LIMIT);

    g_bal_dbg.p_vel_set = (float)g_target_distance;
    g_bal_dbg.p_vel_fb = (float)s_test_speed_c;
    g_bal_dbg.p_vel_out = p_vel_pid.out;
    g_bal_dbg.p_ang_fb = att.pitch;
    g_bal_dbg.p_ang_out = p_angle_pid.out;
    g_bal_dbg.p_rate_fb = att.pitch_rate;
    g_bal_dbg.p_pwm = p_rate_pid.out;
    return p_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行 Yaw 测试级联并严格旁路未选外环
// 参数说明     run5            转向角度环分频标志
// 返回参数     float           Yaw 测试输出
// 使用示例     output = yaw_test_ctrl(run5);
//-------------------------------------------------------------------------------------------------------------------
static float yaw_test_ctrl(uint8 run5)
{
    if (s_test_ring >= TUNE_RING_ANGLE)
    {
        if (run5) pid_loc_calc(&y_angle_pid, g_yaw_target - imu_get_angle_yaw());
    }
    else
    {
        pid_reset(&y_angle_pid);
    }

    pid_loc_calc(&y_rate_pid, y_angle_pid.out - imu.gyro_z);
    y_rate_pid.out = constrain_float(y_rate_pid.out, -BAL_TEST_FLY_LIMIT, BAL_TEST_FLY_LIMIT);

    g_bal_dbg.y_set = g_yaw_target;
    g_bal_dbg.y_fb = imu_get_angle_yaw();
    g_bal_dbg.y_out = y_angle_pid.out;
    g_bal_dbg.y_rate_fb = imu.gyro_z;
    g_bal_dbg.y_pwm = y_rate_pid.out;
    return y_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     压弯动态零点更新: 偏移 += 转角值*LEAN_K1, 限幅(固定/变), 转角≈0 时缓慢衰减回 0
// 参数说明     turn            转角值(取转向外环输出=内环目标角速度, 物理上代表期望转弯快慢)
// 返回参数     float           本周期动态零点偏移(度, 叠加到 g_roll_zero)
// 使用示例     g_lean_offset = lean_offset_update(y_angle_pid.out);
//-------------------------------------------------------------------------------------------------------------------
static float lean_offset_update(float turn)
{
    float offset = g_lean_offset;
    float limit;

    if (fabsf(turn) > LEAN_TURN_DEAD)                   // 有转向需求: 按转角累加偏移
        offset += turn * LEAN_K1;
    else                                                // 直行/停车: 缓慢衰减回 0(零点回归)
        offset *= LEAN_DECAY;

    if (LEAN_LIMIT_MODE == 0)                           // 固定限幅
        limit = LEAN_LIMIT;
    else                                                // 1=变限幅: 速度越快/转得越急允许倾得越多
        limit = fabsf((float)Y_Motor_GetSpeed20ms() * turn * LEAN_K2);

    limit = constrain_float(limit, 0, LEAN_LIMIT_MAX);  // 压弯硬限幅
    return constrain_float(offset, -limit, limit);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     roll 串级(动量轮A/B差动): 飞轮回收环->角度环->角速度环
// 参数说明     zero/run5/run20 横滚零点/5ms角度环标志/20ms回收环标志
// 返回参数     float           横滚角速度环(增量式)输出量 PWM_roll
// 使用示例     g_pwm_roll = roll_cascade_ctrl(g_roll_zero + g_lean_offset, run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float roll_cascade_ctrl(float zero, uint8 run5, uint8 run20)
{
    if (run20) pid_loc_calc(&r_rcy_pid, -(float)(W_Motor_GetSpeed2() - W_Motor_GetSpeed1())); // 回收环(20ms): 飞轮差速期望0防饱和
    if (run5)  pid_loc_calc(&r_angle_pid, r_rcy_pid.out + att.roll - zero);                   // 角度环(5ms, 位置式)
    pid_inc_calc(&r_rate_pid, att.roll_rate + r_angle_pid.out);                               // 角速度环(1ms, 增量式)
    r_rate_pid.out = constrain_float(r_rate_pid.out, -FLYWHEEL_OUT_LIMIT, FLYWHEEL_OUT_LIMIT); // 增量累积限幅防饱和

    // 保存Roll波形量。
    g_bal_dbg.r_rcy_set  = 0.0f;
    g_bal_dbg.r_rcy_fb   = (float)(W_Motor_GetSpeed2() - W_Motor_GetSpeed1());
    g_bal_dbg.r_rcy_out  = r_rcy_pid.out;
    g_bal_dbg.r_ang_fb   = att.roll;
    g_bal_dbg.r_ang_out  = r_angle_pid.out;
    g_bal_dbg.r_rate_fb  = att.roll_rate;
    g_bal_dbg.r_pwm      = r_rate_pid.out;
    return r_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     pitch 串级(行进轮C): 速度环(正反馈)->角度环->角速度环
// 参数说明     zero/run5/run20 俯仰零点/5ms角度环标志/20ms速度环标志
// 返回参数     float           俯仰角速度环(增量式)输出量 PWM_pitch
// 使用示例     g_pwm_pitch = pitch_cascade_ctrl(g_pitch_zero, run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float pitch_cascade_ctrl(float zero, uint8 run5, uint8 run20)
{
    if (run20) pid_loc_calc(&p_vel_pid, (float)Y_Motor_GetSpeed20ms() - (float)g_target_distance);     // 速度环(20ms, 位置式,正反馈)
    if (run5)  pid_loc_calc(&p_angle_pid, p_vel_pid.out - att.pitch + zero);                          // 角度环(5ms, 位置式)
    pid_inc_calc(&p_rate_pid, -att.pitch_rate + p_angle_pid.out);                                     // 角速度环(1ms, 增量式)
    p_rate_pid.out = constrain_float(p_rate_pid.out, -DRIVE_OUT_LIMIT, DRIVE_OUT_LIMIT);              // 增量累积限幅

    // 保存Pitch波形量。
    g_bal_dbg.p_vel_set  = (float)g_target_distance;
    g_bal_dbg.p_vel_fb   = (float)Y_Motor_GetSpeed20ms();
    g_bal_dbg.p_vel_out  = p_vel_pid.out;
    g_bal_dbg.p_ang_fb   = att.pitch;
    g_bal_dbg.p_ang_out  = p_angle_pid.out;
    g_bal_dbg.p_rate_fb  = att.pitch_rate;
    g_bal_dbg.p_pwm      = p_rate_pid.out;
    return p_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     yaw 串级(动量轮A/B同向): 转向环(外,位置式)->角速度环(内,位置式)
// 参数说明     run5            本拍是否为 5ms 分频拍(跑转向外环)
// 返回参数     float           航向角速度环输出量 PWM_yaw
// 使用示例     g_pwm_yaw = yaw_cascade_ctrl(run5);
//-------------------------------------------------------------------------------------------------------------------
static float yaw_cascade_ctrl(uint8 run5)
{
    if (run5) pid_loc_calc(&y_angle_pid, g_yaw_target - imu_get_angle_yaw());               // 转向环(位置式), 5ms
    pid_loc_calc(&y_rate_pid, y_angle_pid.out - imu.gyro_z);                            // 角速度环(位置式), 1ms

    // 保存Yaw波形量。
    g_bal_dbg.y_set     = g_yaw_target;
    g_bal_dbg.y_fb      = imu_get_angle_yaw();
    g_bal_dbg.y_out     = y_angle_pid.out;
    g_bal_dbg.y_rate_fb = imu.gyro_z;
    g_bal_dbg.y_pwm     = y_rate_pid.out;
    return y_rate_pid.out;
}

// 外部接口

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入零点初值 + 串级各环 PID 参数, 复位状态机为 STOP, 开机调一次
// 参数说明     void
// 返回参数     void
// 使用示例     balance_init();
//-------------------------------------------------------------------------------------------------------------------
void balance_init(void)
{
    g_roll_zero  = ROLL_ZERO_INIT;
    g_pitch_zero = PITCH_ZERO_INIT;
    g_yaw_target = 0;
    start_flag   = START_STOP;

    pid_set(&r_rcy_pid,   R_RCY_KP,   R_RCY_KI,   R_RCY_KD,   R_RCY_IMAX);
    pid_set(&r_angle_pid, R_ANGLE_KP, R_ANGLE_KI, R_ANGLE_KD, R_ANGLE_IMAX);
    pid_set(&r_rate_pid,  R_RATE_KP,  R_RATE_KI,  R_RATE_KD,  R_RATE_IMAX);

    pid_set(&p_vel_pid,   P_VEL_KP,   P_VEL_KI,   P_VEL_KD,   P_VEL_IMAX);
    pid_set(&p_angle_pid, P_ANGLE_KP, P_ANGLE_KI, P_ANGLE_KD, P_ANGLE_IMAX);
    pid_set(&p_rate_pid,  P_RATE_KP,  P_RATE_KI,  P_RATE_KD,  P_RATE_IMAX);

    pid_set(&y_angle_pid, Y_ANGLE_KP, Y_ANGLE_KI, Y_ANGLE_KD, Y_ANGLE_IMAX);
    pid_set(&y_rate_pid,  Y_RATE_KP,  Y_RATE_KI,  Y_RATE_KD,  Y_RATE_IMAX);

    s_test_running = 0;
    s_test_axis = TUNE_AXIS_ROLL;
    s_test_ring = TUNE_RING_RATE;
    s_test_tick = 0;
    s_test_speed_c = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴和指定级联层级的电机测试
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=启动成功 0=参数无效或飞轮引脚冲突
// 使用示例     balance_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 balance_test_start(tune_axis_t axis, tune_ring_t ring)
{
    if (axis >= TUNE_AXIS_MAX || ring >= TUNE_RING_MAX) return 0;
    if (axis == TUNE_AXIS_YAW && ring == TUNE_RING_VEL) return 0;

    cascade_reset();
    g_pwm_roll = 0.0f;
    g_pwm_pitch = 0.0f;
    g_pwm_yaw = 0.0f;
    g_motor_a = 0;
    g_motor_b = 0;
    g_motor_c = 0;
    g_target_distance = 0;
    g_yaw_target = imu_get_angle_yaw();

    s_test_axis = axis;
    s_test_ring = ring;
    s_test_tick = 0;
    s_test_speed_c = 0;
    if (axis == TUNE_AXIS_PITCH) Y_Motor_EncoderClear();

    s_test_running = 1;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止电机测试并清除全部 PID 状态
// 参数说明     void
// 返回参数     void
// 使用示例     balance_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void balance_test_stop(void)
{
    uint8 was_running = s_test_running;

    s_test_running = 0;
    cascade_reset();
    g_pwm_roll = 0.0f;
    g_pwm_pitch = 0.0f;
    g_pwm_yaw = 0.0f;
    g_motor_a = 0;
    g_motor_b = 0;
    g_motor_c = 0;
    g_target_distance = 0;
    start_flag = START_STOP;

    if (!was_running) return;
    balance_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询电机测试是否正在运行
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (balance_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 balance_test_running(void)
{
    return s_test_running;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一次独立轴测试控制
// 参数说明     void
// 返回参数     void
// 使用示例     balance_test_run();
//-------------------------------------------------------------------------------------------------------------------
void balance_test_run(void)
{
    uint8 run5;
    uint8 run20;
    float output;

    if (!s_test_running) return;
    if (!test_input_valid())
    {
        balance_test_stop();
        return;
    }

    s_test_tick++;
    if (s_test_tick >= CTRL_DIV_SPEED) s_test_tick = 0;
    run5 = (uint8)((s_test_tick % CTRL_DIV_ATT) == 0);
    run20 = (uint8)(s_test_tick == 0);

    cascade_gain_refresh();
    g_pwm_roll = 0.0f;
    g_pwm_pitch = 0.0f;
    g_pwm_yaw = 0.0f;
    g_motor_a = 0;
    g_motor_b = 0;
    g_motor_c = 0;

    if (s_test_axis == TUNE_AXIS_ROLL)
    {
        output = roll_test_ctrl(run5, run20);
        if (!bal_is_finite(output))
        {
            balance_test_stop();
            return;
        }
        g_pwm_roll = output;
        g_motor_a = (int16)(-output);
        g_motor_b = (int16)(output);
        W_Motor_Release();
        balance_motor_output(g_motor_a, g_motor_b, 0);
    }
    else if (s_test_axis == TUNE_AXIS_PITCH)
    {
        output = pitch_test_ctrl(run5, run20);
        if (!bal_is_finite(output))
        {
            balance_test_stop();
            return;
        }
        g_pwm_pitch = output;
        g_motor_c = (int16)output;
        test_drive_c_output(output);
    }
    else
    {
        output = yaw_test_ctrl(run5);
        if (!bal_is_finite(output))
        {
            balance_test_stop();
            return;
        }
        g_pwm_yaw = output;
        g_motor_a = (int16)output;
        g_motor_b = (int16)output;
        W_Motor_Release();
        balance_motor_output(g_motor_a, g_motor_b, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     平衡主调度: 刷增益->压弯动态零点->三轴串级->合成A/B/C->角度超限保护->按状态机下发
// 参数说明     void
// 返回参数     void
// 使用示例     balance_run();
//-------------------------------------------------------------------------------------------------------------------
void balance_run(void)
{
    static uint16 tick;                                  // 1ms 基准分频计数(20ms 一轮回绕)
    uint8 run5, run20;

    tick++;
    if (tick >= CTRL_DIV_SPEED) tick = 0;                // 20ms 一轮
    run5  = (uint8)((tick % CTRL_DIV_ATT) == 0);         // 5ms 分频拍: 角度环/转向外环/压弯零点
    run20 = (uint8)(tick == 0);                          // 20ms 分频拍: 速度环/飞轮回收环

    cascade_gain_refresh();                              // 菜单/无线调参下周期即生效
    // IMU 或无刷驱动通讯不可用时立即停机。
    // 驱动断链时闭环输出发不出去, 继续跑串级只会让增量式积分堆积。
    if (!g_imu_ok || imu_link_lost() || W_Motor_LinkLost()) {
        cascade_reset();
        g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
        g_motor_a = g_motor_b = g_motor_c = 0;
        start_flag = START_STOP;
        balance_motor_stop();
        return;
    }

    // 姿态数据异常时立即停机。
    if (attitude_diverged() ||
        !bal_is_finite(att.roll)  || !bal_is_finite(att.pitch) ||
        !bal_is_finite(att.roll_rate) || !bal_is_finite(att.pitch_rate) || !bal_is_finite(imu.gyro_z)) {
        cascade_reset();
        g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
        g_motor_a = g_motor_b = g_motor_c = 0;
        start_flag = START_STOP;
        balance_motor_stop();
        return;
    }

    if (start_flag == START_STOP) {
        cascade_reset();                                 // 停止: 清积分/增量累积/压弯偏移
        g_yaw_target = imu_get_angle_yaw();                  // 目标跟随当前航向, 防起步跳变
    }
    if (start_flag == START_DRIVE_ONLY) {
        pid_reset(&r_rcy_pid); pid_reset(&r_angle_pid); pid_reset(&r_rate_pid);  // 飞轮未上电,
        pid_reset(&y_angle_pid); pid_reset(&y_rate_pid);                         // 防增量/积分堆积
        g_lean_offset = 0;
        g_yaw_target  = imu_get_angle_yaw();
    }

    // 计算三轴串级控制。
    g_pwm_yaw = yaw_cascade_ctrl(run5);
    if (run5)                                                      // 压弯动态零点与转向外环同拍(5ms, 衰减/累加系数按 5ms 节拍整定)
        g_lean_offset = lean_offset_update(y_angle_pid.out);
    g_pwm_roll  = roll_cascade_ctrl(g_roll_zero + g_lean_offset, run5, run20);
    g_pwm_pitch = pitch_cascade_ctrl(g_pitch_zero, run5, run20);

    // Roll优先，Yaw使用飞轮剩余输出范围。
    float roll_cmd = constrain_float(g_pwm_roll, -(float)FLYWHEEL_OUT_LIMIT, (float)FLYWHEEL_OUT_LIMIT);
    float yaw_room = (float)FLYWHEEL_OUT_LIMIT - fabsf(roll_cmd);        // 留给转向的余量
    float yaw_cmd  = constrain_float(g_pwm_yaw, -yaw_room, yaw_room);

    int32 mix_a = (int32)(-roll_cmd + yaw_cmd);                  // 飞轮A = -横滚 + 转向
    int32 mix_b = (int32)(+roll_cmd + yaw_cmd);                  // 飞轮B = +横滚 + 转向 (差动=平衡,同向=转向)
    int32 mix_c = (int32)(g_pwm_pitch);                          // 行进轮C = 俯仰
    // 浮点转整数前再次限幅。
    g_motor_a = (int16)func_limit(mix_a, FLYWHEEL_OUT_LIMIT);
    g_motor_b = (int16)func_limit(mix_b, FLYWHEEL_OUT_LIMIT);
    g_motor_c = (int16)func_limit(mix_c, DRIVE_OUT_LIMIT);

    // 姿态角超限时立即停机。
    float roll_err  = att.roll  - g_roll_zero;
    float pitch_err = att.pitch - g_pitch_zero;
    if (fabsf(roll_err) > ROLL_PROTECT_ANGLE || fabsf(pitch_err) > PITCH_PROTECT_ANGLE) {
        g_motor_a = g_motor_b = g_motor_c = 0;
        start_flag = START_STOP;
        W_Motor_Stop();
    }

    // 根据启动状态下发电机输出。
    switch (start_flag) {
    case START_STOP:
        balance_motor_stop();                        // 全停
        break;
    case START_DRIVE_ONLY:
        W_Motor_Stop();
        Y_Motor_SetDuty(g_motor_c);
        break;
    case START_BALANCE:
        W_Motor_Release();
        balance_motor_output(g_motor_a, g_motor_b, g_motor_c); // 全启动
        break;
    default:
        break;
    }
}
