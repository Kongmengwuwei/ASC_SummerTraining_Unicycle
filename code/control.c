#include "control.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include "board_config.h"
#include "imu.h"
#include "attitude.h"
#include "vofa.h"
#include "param.h"
#include "pid.h"
#include "vision_core.h"
#include <math.h>

// ====================== 对外全局量 ======================

start_state_t start_flag = START_STOP;          // 发车状态机，上电全停

float g_roll_zero, g_pitch_zero;                // 机械零点(°)
float g_lean_offset = 0;                        // 压弯动态零点偏移(°)
float g_pwm_roll, g_pwm_pitch, g_pwm_yaw;       // 三轴串级输出，混控前
int16 g_motor_a, g_motor_b, g_motor_c;          // 混控后的三电机控制量
int   g_target_distance = 0;                    // Pitch 速度环目标(counts/20ms)
float g_yaw_target = 0;                         // 转向外环目标航向(°)
balance_dbg_t g_bal_dbg;                        // 串级各环中间量，只给 vofa 波形用

float g_dbg_error   = 0.0f;                     // 中线偏差，右偏为正
uint8 g_imu_ok      = 0;                        // IMU660RB 初始化结果
uint8 g_cam_ok      = 0;                        // CPU1 摄像头就绪标志
uint8 g_track_valid = 0;                        // 最新一帧循迹是否有效
uint16 g_track_lost_frames = 0;                 // 连续无效帧计数
volatile uint16 g_vision_age_ms = 0;            // 距上一帧视觉结果的时间(ms)
volatile uint32 g_control_uptime_ms = 0;        // 1ms 中断累计运行时间(ms)
volatile uint32 g_vision_frame_seq = 0;         // 最新视觉帧序号
volatile uint32 g_vision_heartbeat = 0;         // CPU1 主循环心跳
uint16 g_vision_threshold = 0;                  // 最新大津阈值
uint16 g_vision_search_stop = 0;                // 最新有效前瞻行数
uint16 g_vision_left_lost = 0;                  // 最新左边线丢线行数
uint16 g_vision_right_lost = 0;                 // 最新右边线丢线行数
uint16 g_vision_both_lost = 0;                  // 最新双边丢线行数
uint8  g_vision_active_elem = 0;                // 最新元素编号
uint8  g_vision_island_state = 0;               // 最新环岛状态号，0=空闲
float  g_vision_speed_scale = 1.0f;             // 元素建议速度倍率，Run 尚未使用
uint8  g_vision_stop_request = 0;               // 元素停车请求，Run 尚未使用
float  g_vision_fps = 0.0f;                     // CPU1 出帧率(帧/s)，1ms 中断每 VISION_FPS_WIN_MS 结算一次
float  g_vision_vsync_fps = 0.0f;               // 摄像头 VSYNC 频率(帧/s)
float  g_vision_dma_fps = 0.0f;                 // DMA 完整帧频率(帧/s)
float  g_vision_drop_fps = 0.0f;                // CPU1 忙导致的丢帧频率(帧/s)
uint32 g_vision_grab_us = 0;                    // 最近一帧 ROI 复制耗时
uint32 g_vision_binarize_us = 0;                // 最近一帧大津与二值化耗时
uint32 g_vision_edge_us = 0;                    // 最近一帧八邻域提边耗时
uint32 g_vision_element_us = 0;                 // 最近一帧元素处理耗时
uint32 g_vision_process_us = 0;                 // 最近一帧 CPU1 总处理耗时
uint32 g_vision_process_max_us = 0;             // 本次摄像头启动后的最大处理耗时

// ====================== 内部状态 ======================

static pid_t r_rcy_pid, r_angle_pid, r_rate_pid;    // Roll ：飞轮回收 / 角度 / 角速度
static pid_t p_vel_pid, p_angle_pid, p_rate_pid;    // Pitch：速度 / 角度 / 角速度
static pid_t y_angle_pid, y_rate_pid;               // Yaw  ：转向外环 / 角速度内环

static float s_speed_ramp;                      // 斜坡后的速度环实际目标(counts/20ms)
static float s_rcy_fb;                          // 飞轮差速 A-B(RPM)，只在 20ms 拍更新
static float s_lean_raw;                        // 压弯零点累加值，g_lean_offset 限速率跟随它

// Test 状态，前台启停、1ms 中断读
static volatile uint8 s_test_running;           // 单轴测试运行标志
static tune_axis_t s_test_axis;                 // 当前测试轴
static tune_ring_t s_test_ring;                 // 当前测试的最高启用环
static uint16 s_test_tick;                      // 测试用 1ms 分频计数
static int16  s_test_speed_c;                   // 测试用行进轮速度快照
static volatile uint8 s_control_test_active;    // 菜单看到的 Test 运行标志
static volatile control_test_status_t s_test_status;    // 最近一次启动结果

// 架空点动状态，没有时限，靠再按一次动作行、返回键或驱动掉线停
static volatile motor_jog_t s_jog_target;       // 点动目标电机
static volatile int16       s_jog_duty;         // 点动占空比，带符号

// 姿态与驱动通信的阻断原因，只在本文件内用来生成菜单提示
typedef enum
{
    CTRL_BLOCK_NONE = 0,        // 无阻断
    CTRL_BLOCK_IMU_FAIL,        // IMU 初始化失败
    CTRL_BLOCK_IMU_CALIB,       // IMU 静止标定无效
    CTRL_BLOCK_ATT_CONVERGING,  // 姿态解算尚未收敛
    CTRL_BLOCK_ATT_DIVERGED,    // 四元数发散
    CTRL_BLOCK_IMU_LOST,        // IMU 链路中断
    CTRL_BLOCK_BLDC_LOST,       // CYT2BL3 通信中断
} control_block_t;

static void test_stop(void);                    // test_run 里检测到异常要回调它

// ====================== 电机输出 ======================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止行进轮并锁定动量轮软件刹车
// 参数说明     void
// 返回参数     void
// 使用示例     control_motor_stop();
//-------------------------------------------------------------------------------------------------------------------
static void control_motor_stop(void)
{
    W_Motor_Stop();
    Y_Motor_Stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向 W_Motor 与 Y_Motor 下发三电机控制量
// 参数说明     motor_a/b/c     动量轮 A、动量轮 B、行进轮 C 控制量
// 返回参数     void
// 使用示例     control_motor_output(g_motor_a, g_motor_b, g_motor_c);
//-------------------------------------------------------------------------------------------------------------------
static void control_motor_output(int16 motor_a, int16 motor_b, int16 motor_c)
{
    W_Motor_SetDuty(motor_a, motor_b);
    Y_Motor_SetDuty(motor_c);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断浮点是否为有限值，即非 NaN 且非 ±Inf
// 参数说明     v               待判值
// 返回参数     uint8           1=有限值 0=NaN/Inf
// 使用示例     if (!ctrl_is_finite(att.roll)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 ctrl_is_finite(float v)
{
    if (v != v) return 0;                               // NaN
    if (v > 3.0e38f || v < -3.0e38f) return 0;          // ±Inf 或量级异常
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断动量轮是否已经跑飞。回收环旁路时输出会停在非零值上，飞轮一路加速到极速，
//              触发 CYT2BL3 堵转保护后必须重启驱动，所以抢在它之前停
// 参数说明     void
// 返回参数     uint8           1=已超 FLY_SPEED_LIMIT 0=正常或保护关闭
// 使用示例     if (fly_overspeed()) control_jog_stop();
//-------------------------------------------------------------------------------------------------------------------
static uint8 fly_overspeed(void)
{
    int32 limit = FLY_SPEED_LIMIT;
    int32 speed_a;
    int32 speed_b;

    if (limit <= 0) return 0;               // 0 表示关掉这道保护
    speed_a = (int32)W_Motor_GetSpeed1();
    speed_b = (int32)W_Motor_GetSpeed2();
    if (speed_a < 0) speed_a = -speed_a;
    if (speed_b < 0) speed_b = -speed_b;
    return (uint8)(speed_a > limit || speed_b > limit);
}

// ====================== 串级公共部分 ======================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把菜单改过的增益刷进 PID 结构，不清运行状态，每控制周期调一次
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
// 函数简介     清全部串级 PID 的积分、误差历史和增量累积输出，同时清压弯偏移与速度斜坡
// 参数说明     void
// 返回参数     void
// 使用示例     cascade_reset();
//-------------------------------------------------------------------------------------------------------------------
static void cascade_reset(void)
{
    pid_reset(&r_rcy_pid);   pid_reset(&r_angle_pid); pid_reset(&r_rate_pid);
    pid_reset(&p_vel_pid);   pid_reset(&p_angle_pid); pid_reset(&p_rate_pid);
    pid_reset(&y_angle_pid); pid_reset(&y_rate_pid);
    g_lean_offset = 0;
    s_lean_raw = 0;
    s_speed_ramp = 0.0f;
    s_rcy_fb = 0.0f;                                 // 回收环反馈
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把速度环目标按斜坡挪向 g_target_distance，20ms 一拍，避免目标阶跃踹速度环
// 参数说明     void
// 返回参数     void
// 使用示例     if (run20) speed_ramp_update();
//-------------------------------------------------------------------------------------------------------------------
static void speed_ramp_update(void)
{
    float target = (float)g_target_distance;
    float delta  = target - s_speed_ramp;
    float step;

    if (delta == 0.0f) return;

    // 朝远离 0 的方向算加速，朝 0 的方向算减速，两个方向速率不同
    step = (fabsf(target) > fabsf(s_speed_ramp)) ? SPEED_UP_RATE : SPEED_DOWN_RATE;
    if (step <= 0.0f)                       // 速率给 0 表示不限速率，直接跟上
    {
        s_speed_ramp = target;
        return;
    }

    if (delta > step)       s_speed_ramp += step;
    else if (delta < -step) s_speed_ramp -= step;
    else                    s_speed_ramp = target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新压弯动态零点的目标值：有转向需求时按转角累加，直行时按 LEAN_DECAY 衰减回 0
// 参数说明     turn            转向外环输出，即内环目标角速度，代表期望转弯快慢
// 返回参数     float           压弯零点目标值(°)，再经 lean_slew_update() 限速率后才生效
// 使用示例     s_lean_raw = lean_offset_update(y_angle_pid.out);
//-------------------------------------------------------------------------------------------------------------------
static float lean_offset_update(float turn)
{
    float offset = s_lean_raw;
    float limit;

    // 压弯是"过弯时车往内侧倾"，只在真的在走的时候才有意义。
    // s_speed_ramp==0 就是原地(Balance 的 g_target_distance 恒 0)，此时没有弯可压，
    // 再让 Yaw 外环的输出去挪横滚目标，就是拿一个没整定的环去命令车体歪几度。
    // 实测这条路径能把 roll 目标顶到 ±LEAN_LIMIT(10°)，车追到 -23°、A/B 一半时间在极速
    if (s_speed_ramp != 0.0f && fabsf(turn) > LEAN_TURN_DEAD)   // 在走且有转向需求，按转角累加偏移
        offset += turn * LEAN_K1;
    else                                                // 原地、直行或停车，缓慢衰减回 0
        offset *= LEAN_DECAY;

    if (LEAN_LIMIT_MODE == 0)                           // 固定限幅
        limit = LEAN_LIMIT;
    else                                                // 动态限幅：速度越快转得越急，允许倾得越多
        limit = fabsf((float)Y_Motor_GetSpeed20ms() * turn * LEAN_K2);

    limit = constrain_float(limit, 0, LEAN_LIMIT_MAX);  // 压弯角硬上限
    s_lean_raw = constrain_float(offset, -limit, limit);
    return s_lean_raw;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把压弯零点按速率挪向目标值，避免零点阶跃直接变成横滚角度环的目标阶跃
// 参数说明     target          压弯零点目标值(°)
// 返回参数     float           本拍生效的压弯零点(°)
// 使用示例     g_lean_offset = lean_slew_update(s_lean_raw);
//-------------------------------------------------------------------------------------------------------------------
static float lean_slew_update(float target)
{
    float step  = LEAN_SLEW;
    float delta = target - g_lean_offset;

    if (step <= 0.0f) return target;        // 速率给 0 表示不限速率
    if (delta > step)  return g_lean_offset + step;
    if (delta < -step) return g_lean_offset - step;
    return target;
}

// ====================== 三轴串级 ======================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍飞轮回收环(20ms)，返回它给角度环的零点偏移(°)，非 20ms 拍返回上一次的值
// 参数说明     run20           20ms 分频标志
// 返回参数     float           零点偏移(°)，正值表示命令车体往一侧歪
// 使用示例     float rcy = roll_recovery_offset(run20);
//-------------------------------------------------------------------------------------------------------------------
static float roll_recovery_offset(uint8 run20)
{
    if (!run20) return r_rcy_pid.out;

    // 回收环直接使用 CYT2BL3 回传的两轮转速差，不再增加额外低通。
    s_rcy_fb = (float)(W_Motor_GetSpeed1() - W_Motor_GetSpeed2());

    // 输出直接加进角度环误差，不在回收环内附加输出限幅。
    pid_loc_calc(&r_rcy_pid, s_rcy_fb);
    return r_rcy_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍 Roll 角速度内环(1ms)，位置式 + 输出限幅
// 参数说明     limit           输出限幅，Test 用 BAL_TEST_FLY_LIMIT，三轴串级用 FLYWHEEL_OUT_LIMIT
// 返回参数     float           限幅后的占空比输出
// 使用示例     return roll_rate_ctrl((float)BAL_TEST_FLY_LIMIT);
//-------------------------------------------------------------------------------------------------------------------
static float roll_rate_ctrl(float limit)
{
    // 位置式，不是增量式。增量式的 out 是个永久累加器：误差回到 0 时输出停在原地不动，
    // 占空比恒定 -> 飞轮到极速 -> dω/dt=0 -> 反作用力矩为零 -> 车必倒，
    // 实测就是"能站 2~3 秒然后直接倒"，倒的瞬间飞轮都在 96%~99% 极速上。
    // 位置式的输出跟当前误差绑定，误差回零输出就回零，飞轮自己会减速，跑不出这个失效模式。
    // 两个参考独轮工程的飞轮串级也都是位置式，且积分项全为 0。
    //
    // R_RATE_KI 保持 0：R_RATE_IMAX 是 100，而这一环误差量级是几十到几百 °/s，
    // 积分器一两拍就顶死，out_i 会变成 ±ki*imax 这么个只跟符号有关的常数偏置。
    // R_RATE_KD 现在可用：位置式的 kd 是一阶差分(角加速度)，不是增量式那个二阶差分。
    // 用带限幅的版本：限幅在 PID 内部，积分才能做条件抗饱和。
    // 外面再 constrain 一次的话，PID 自己不知道输出被夹住了，积分会一路顶到 imax
    return pid_loc_calc_limited(&r_rate_pid, att.roll_rate + r_angle_pid.out, -limit, limit);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Roll 串级，输出由 A/B 动量轮差动执行：回收环 -> 角度环 -> 角速度环
// 参数说明     zero/run5/run20 横滚零点、5ms 分频标志、20ms 分频标志
// 返回参数     float           横滚角速度环的累计输出 PWM_roll
// 使用示例     g_pwm_roll = roll_cascade_ctrl(g_roll_zero + g_lean_offset, run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float roll_cascade_ctrl(float zero, uint8 run5, uint8 run20)
{
    float rcy = roll_recovery_offset(run20);         // 回收环给出的零点偏移(°)

    if (run5)
        pid_loc_calc(&r_angle_pid, rcy + att.roll - zero);   // 角度环，位置式，输出是角速度命令(°/s)
    roll_rate_ctrl((float)FLYWHEEL_OUT_LIMIT);               // 角速度环，位置式

    // 角度环输出不单独限幅：内环 pid_loc_calc_limited() 已经把占空比夹在 ±FLYWHEEL_OUT_LIMIT，
    // r_rate_ki 又是 0，没有会卷起来的积分。多一层限幅只会悄悄卡死静态输出上限
    // (曾经 Ang Lim=150 把上限锁在 |kp|×150，角度环 kp 加多大都不动)。积分靠 R_ANGLE_IMAX 管
    g_bal_dbg.r_rcy_set  = 0.0f;
    g_bal_dbg.r_rcy_fb   = s_rcy_fb;                 // 回收环使用的 A-B 转速差
    g_bal_dbg.r_rcy_out  = r_rcy_pid.out;
    g_bal_dbg.r_ang_fb   = att.roll;
    g_bal_dbg.r_ang_out  = r_angle_pid.out;
    g_bal_dbg.r_rate_fb  = att.roll_rate;
    g_bal_dbg.r_pwm      = r_rate_pid.out;
    return r_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Pitch 串级，输出由 C 行进轮执行：速度环 -> 角度环 -> 角速度环
// 参数说明     zero/run5/run20 俯仰零点、5ms 分频标志、20ms 分频标志
// 返回参数     float           俯仰角速度环的累计输出 PWM_pitch
// 使用示例     g_pwm_pitch = pitch_cascade_ctrl(g_pitch_zero, run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float pitch_cascade_ctrl(float zero, uint8 run5, uint8 run20)
{
    // 速度环输出的是倾角目标：想加速就往前倒一点，所以误差取 反馈 - 目标
    // 目标用斜坡后的 s_speed_ramp，不用 g_target_distance，避免目标阶跃踹速度环
    if (run20) pid_loc_calc(&p_vel_pid, (float)Y_Motor_GetSpeed20ms() - s_speed_ramp);
    if (run5)  pid_loc_calc(&p_angle_pid, p_vel_pid.out - att.pitch + zero);             // 角度环，位置式
    pid_inc_calc_limited(&p_rate_pid, -att.pitch_rate + p_angle_pid.out,
                         -DRIVE_OUT_LIMIT, DRIVE_OUT_LIMIT);                             // 角速度环，增量式

    g_bal_dbg.p_vel_set  = s_speed_ramp;
    g_bal_dbg.p_vel_fb   = (float)Y_Motor_GetSpeed20ms();
    g_bal_dbg.p_vel_out  = p_vel_pid.out;
    g_bal_dbg.p_ang_fb   = att.pitch;
    g_bal_dbg.p_ang_out  = p_angle_pid.out;
    g_bal_dbg.p_rate_fb  = att.pitch_rate;
    g_bal_dbg.p_pwm      = p_rate_pid.out;
    return p_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Yaw 串级，输出由 A/B 动量轮同向执行：转向外环 -> 角速度内环，两级都是位置式
// 参数说明     run5            5ms 分频标志，为 1 时跑转向外环
// 返回参数     float           航向角速度内环输出 PWM_yaw
// 使用示例     g_pwm_yaw = yaw_cascade_ctrl(run5);
//-------------------------------------------------------------------------------------------------------------------
static float yaw_cascade_ctrl(uint8 run5)
{
    // 航向用连续角，环岛要累计 340°，这里不能换成 ±180° 包装角
    if (run5) pid_loc_calc(&y_angle_pid, g_yaw_target - imu_get_angle_yaw());   // 转向外环，5ms
    pid_loc_calc(&y_rate_pid, y_angle_pid.out - imu.gyro_z);                    // 角速度内环，1ms

    g_bal_dbg.y_set     = g_yaw_target;
    g_bal_dbg.y_fb      = imu_get_angle_yaw();
    g_bal_dbg.y_out     = y_angle_pid.out;
    g_bal_dbg.y_rate_fb = imu.gyro_z;
    g_bal_dbg.y_pwm     = y_rate_pid.out;
    return y_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安全闸把三轴串级停回 STOP，只在原来确实在跑时记下停机原因供菜单显示
// 参数说明     reason          停机原因，菜单用 control_test_last_status() 读走
// 返回参数     void
// 使用示例     cascade_stop(CTRL_TEST_STATUS_BLDC_LOST);
//-------------------------------------------------------------------------------------------------------------------
static void cascade_stop(control_test_status_t reason)
{
    // start_flag 已经是 STOP 说明车本来就没跑，别用它去盖掉上一次 Test 的状态码
    if (start_flag != START_STOP) s_test_status = reason;

    cascade_reset();
    g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
    g_motor_a = g_motor_b = g_motor_c = 0;
    start_flag = START_STOP;
    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍三轴串级：刷增益 -> 安全闸 -> 三轴串级 -> 混控 -> 保护角 -> 按状态机下发
// 参数说明     void
// 返回参数     void
// 使用示例     cascade_run();
//-------------------------------------------------------------------------------------------------------------------
static void cascade_run(void)
{
    static uint16 tick;                                  // 1ms 分频计数，20ms 一轮回绕
    uint8 run5, run20;
    float roll_cmd, yaw_room, yaw_cmd;
    float roll_err, pitch_err;
    int32 mix_a, mix_b, mix_c;

    tick++;
    if (tick >= CTRL_DIV_SPEED) tick = 0;
    run5  = (uint8)((tick % CTRL_DIV_ATT) == 0);         // 5ms 拍：角度环、转向外环、压弯零点
    run20 = (uint8)(tick == 0);                          // 20ms 拍：速度环、飞轮回收环

    cascade_gain_refresh();                              // 菜单或串口改的增益下一拍就生效

    // IMU、标定或驱动通信不可信就停机。驱动断链时输出发不出去，继续跑串级只会让增量式积分堆积
    if (!g_imu_ok)             { cascade_stop(CTRL_TEST_STATUS_IMU_FAIL);  return; }
    if (imu_link_lost())       { cascade_stop(CTRL_TEST_STATUS_IMU_LOST);  return; }
    if (imu_calib_state() != IMU_CALIB_OK)
                               { cascade_stop(CTRL_TEST_STATUS_IMU_CALIB); return; }
    if (W_Motor_LinkLost())    { cascade_stop(CTRL_TEST_STATUS_BLDC_LOST); return; }

    // 姿态出现 NaN/Inf 立即停机。这一判必须排在保护角之前，fabsf(NaN) > 阈值 恒假，拦不住 NaN
    if (attitude_diverged() ||
        !ctrl_is_finite(att.roll)      || !ctrl_is_finite(att.pitch) ||
        !ctrl_is_finite(att.roll_rate) || !ctrl_is_finite(att.pitch_rate) ||
        !ctrl_is_finite(imu.gyro_z))
    {
        cascade_stop(CTRL_TEST_STATUS_ATT_DIVERGED);
        return;
    }

    // 三轴一起跑时回收环在环内，但它整定不好飞轮照样会单向堆积。
    // 抢在 CYT2BL3 的堵转保护闩死之前停，闩死了要重启驱动才恢复
    if (start_flag == START_BALANCE && fly_overspeed())
    {
        cascade_stop(CTRL_TEST_STATUS_FLY_OVERSPEED);
        return;
    }

    if (start_flag == START_STOP)
    {
        cascade_reset();                                 // 清积分、增量累积和压弯偏移
        g_yaw_target = imu_get_angle_yaw();              // 目标跟随当前航向，防起步跳变
    }
    if (start_flag == START_DRIVE_ONLY)
    {
        // 飞轮还锁着刹车，Roll 与 Yaw 的积分和增量累积必须清掉，否则松刹车瞬间会甩出去
        pid_reset(&r_rcy_pid); pid_reset(&r_angle_pid); pid_reset(&r_rate_pid);
        pid_reset(&y_angle_pid); pid_reset(&y_rate_pid);
        g_lean_offset = 0;
        g_yaw_target  = imu_get_angle_yaw();
    }

    g_pwm_yaw = yaw_cascade_ctrl(run5);
    if (run5)                                            // 压弯零点与转向外环同拍，系数按 5ms 节拍整定
        g_lean_offset = lean_slew_update(lean_offset_update(y_angle_pid.out));
    if (run20) speed_ramp_update();                      // 速度目标斜坡与速度环同拍
    g_pwm_roll  = roll_cascade_ctrl(g_roll_zero + g_lean_offset, run5, run20);
    g_pwm_pitch = pitch_cascade_ctrl(g_pitch_zero, run5, run20);

    // 混控：平衡优先，Yaw 只能用 Roll 剩下的余量。不许改成先相加再统一钳幅
    roll_cmd = constrain_float(g_pwm_roll, -(float)FLYWHEEL_OUT_LIMIT, (float)FLYWHEEL_OUT_LIMIT);
    yaw_room = (float)FLYWHEEL_OUT_LIMIT - fabsf(roll_cmd);
    yaw_cmd  = constrain_float(g_pwm_yaw, -yaw_room, yaw_room);

    mix_a = (int32)(-roll_cmd + yaw_cmd);                // A：横滚分量取负
    mix_b = (int32)(+roll_cmd + yaw_cmd);                // B：差动出平衡，同向出转向
    mix_c = (int32)(g_pwm_pitch);                        // 行进轮 C
    // 浮点转整数前再限一次幅
    g_motor_a = (int16)func_limit(mix_a, FLYWHEEL_OUT_LIMIT);
    g_motor_b = (int16)func_limit(mix_b, FLYWHEEL_OUT_LIMIT);
    g_motor_c = (int16)func_limit(mix_c, DRIVE_OUT_LIMIT);

    // 横滚或俯仰误差超过保护角，立即停机并锁死刹车
    roll_err  = att.roll  - g_roll_zero;
    pitch_err = att.pitch - g_pitch_zero;
    if (fabsf(roll_err) > ROLL_PROTECT_ANGLE || fabsf(pitch_err) > PITCH_PROTECT_ANGLE)
    {
        if (start_flag != START_STOP)
            s_test_status = (fabsf(roll_err) > ROLL_PROTECT_ANGLE) ? CTRL_TEST_STATUS_ROLL_PROT
                                                                   : CTRL_TEST_STATUS_PITCH_PROT;
        g_motor_a = g_motor_b = g_motor_c = 0;
        start_flag = START_STOP;
        W_Motor_Stop();
    }

    // 每个分支都必须给 W_Motor 下发一帧，喂住驱动固件的失控保护看门狗
    switch (start_flag)
    {
    case START_STOP:
        control_motor_stop();
        break;
    case START_DRIVE_ONLY:
        W_Motor_Stop();
        Y_Motor_SetDuty(g_motor_c);
        break;
    case START_BALANCE:
        W_Motor_Release();
        control_motor_output(g_motor_a, g_motor_b, g_motor_c);
        break;
    default:
        break;
    }
}

// ====================== 单轴 Test ======================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按测试限幅驱动行进轮 C，动量轮保持软件刹车锁死
// 参数说明     command         控制量，方向与死区补偿由 Y_Motor 内部处理
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
// 函数简介     检查 Test 的输入是否可信：IMU、标定、驱动通信、NaN 和保护角
// 参数说明     void
// 返回参数     control_test_status_t OK 表示可以继续，其余为具体的停机原因
// 使用示例     if (test_input_reason() != CTRL_TEST_STATUS_OK) test_stop();
//-------------------------------------------------------------------------------------------------------------------
static control_test_status_t test_input_reason(void)
{
    // 逐条分开判，菜单要显示到底是哪一条把测试停掉的
    if (!g_imu_ok)                          return CTRL_TEST_STATUS_IMU_FAIL;
    if (imu_link_lost())                    return CTRL_TEST_STATUS_IMU_LOST;
    if (attitude_diverged())                return CTRL_TEST_STATUS_ATT_DIVERGED;
    if (!attitude_converged())              return CTRL_TEST_STATUS_ATT_CONVERGING;
    if (imu_calib_state() != IMU_CALIB_OK)  return CTRL_TEST_STATUS_IMU_CALIB;

    // 动飞轮就要求驱动在线，Pitch 只驱动 C 轮不受此限
    if (s_test_axis != TUNE_AXIS_PITCH && W_Motor_LinkLost())
        return CTRL_TEST_STATUS_BLDC_LOST;

    // NaN/Inf 必须排在保护角之前，fabsf(NaN) > 阈值 恒假
    if (!ctrl_is_finite(att.roll) || !ctrl_is_finite(att.pitch) || !ctrl_is_finite(att.roll_rate) ||
        !ctrl_is_finite(att.pitch_rate) || !ctrl_is_finite(imu.gyro_z))
        return CTRL_TEST_STATUS_ATT_DIVERGED;

    if (fabsf(att.roll  - g_roll_zero)  > ROLL_PROTECT_ANGLE)  return CTRL_TEST_STATUS_ROLL_PROT;
    if (fabsf(att.pitch - g_pitch_zero) > PITCH_PROTECT_ANGLE) return CTRL_TEST_STATUS_PITCH_PROT;

    if (s_test_axis != TUNE_AXIS_PITCH && fly_overspeed())
        return CTRL_TEST_STATUS_FLY_OVERSPEED;
    return CTRL_TEST_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍 Roll 测试串级，比选中环更外的环整个旁路并保持清零
// 参数说明     run5/run20      5ms 与 20ms 分频标志
// 返回参数     float           Roll 测试输出
// 使用示例     output = roll_test_ctrl(run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float roll_test_ctrl(uint8 run5, uint8 run20)
{
    if (s_test_ring >= TUNE_RING_VEL)
    {
        (void)roll_recovery_offset(run20);           // 与三轴串级使用同一条回收路径
    }
    else
    {
        pid_reset(&r_rcy_pid);
    }

    if (s_test_ring >= TUNE_RING_ANGLE)
    {
        if (run5)
            pid_loc_calc(&r_angle_pid, r_rcy_pid.out + att.roll - g_roll_zero);
    }
    else
    {
        pid_reset(&r_angle_pid);
    }

    roll_rate_ctrl((float)BAL_TEST_FLY_LIMIT);

    g_bal_dbg.r_rcy_set = 0.0f;
    g_bal_dbg.r_rcy_fb  = s_rcy_fb;                  // 回收环使用的 A-B 转速差
    g_bal_dbg.r_rcy_out = r_rcy_pid.out;
    g_bal_dbg.r_ang_fb  = att.roll;
    g_bal_dbg.r_ang_out = r_angle_pid.out;
    g_bal_dbg.r_rate_fb = att.roll_rate;
    g_bal_dbg.r_pwm     = r_rate_pid.out;
    return r_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍 Pitch 测试串级，比选中环更外的环整个旁路并保持清零
// 参数说明     run5/run20      5ms 与 20ms 分频标志
// 返回参数     float           Pitch 测试输出
// 使用示例     output = pitch_test_ctrl(run5, run20);
//-------------------------------------------------------------------------------------------------------------------
static float pitch_test_ctrl(uint8 run5, uint8 run20)
{
    if (s_test_ring >= TUNE_RING_VEL)
    {
        if (run20)
        {
            speed_ramp_update();
            s_test_speed_c = Y_Motor_GetSpeed20ms();
            pid_loc_calc(&p_vel_pid, (float)s_test_speed_c - s_speed_ramp);
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

    pid_inc_calc_limited(&p_rate_pid, -att.pitch_rate + p_angle_pid.out,
                         -BAL_TEST_DRIVE_LIMIT, BAL_TEST_DRIVE_LIMIT);

    g_bal_dbg.p_vel_set  = s_speed_ramp;
    g_bal_dbg.p_vel_fb   = (float)s_test_speed_c;
    g_bal_dbg.p_vel_out  = p_vel_pid.out;
    g_bal_dbg.p_ang_fb   = att.pitch;
    g_bal_dbg.p_ang_out  = p_angle_pid.out;
    g_bal_dbg.p_rate_fb  = att.pitch_rate;
    g_bal_dbg.p_pwm      = p_rate_pid.out;
    return p_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍 Yaw 测试串级，未选转向外环时整个旁路并保持清零
// 参数说明     run5            5ms 分频标志
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

    pid_loc_calc_limited(&y_rate_pid, y_angle_pid.out - imu.gyro_z,
                         -(float)BAL_TEST_FLY_LIMIT, (float)BAL_TEST_FLY_LIMIT);

    g_bal_dbg.y_set     = g_yaw_target;
    g_bal_dbg.y_fb      = imu_get_angle_yaw();
    g_bal_dbg.y_out     = y_angle_pid.out;
    g_bal_dbg.y_rate_fb = imu.gyro_z;
    g_bal_dbg.y_pwm     = y_rate_pid.out;
    return y_rate_pid.out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动单轴 Test，只闭对应轴，比选中环更外的环全部旁路
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=启动成功 0=轴环组合非法
// 使用示例     test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
static uint8 test_start(tune_axis_t axis, tune_ring_t ring)
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
// 函数简介     停止单轴 Test，清全部 PID 状态并停三电机
// 参数说明     void
// 返回参数     void
// 使用示例     test_stop();
//-------------------------------------------------------------------------------------------------------------------
static void test_stop(void)
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
    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑一拍单轴 Test，输入不合法或输出发散时自动停车
// 参数说明     void
// 返回参数     void
// 使用示例     test_run();
//-------------------------------------------------------------------------------------------------------------------
static void test_run(void)
{
    uint8 run5;
    uint8 run20;
    float output;

    control_test_status_t reason;

    if (!s_test_running) return;
    reason = test_input_reason();
    if (reason != CTRL_TEST_STATUS_OK)
    {
        test_stop();
        s_test_status = reason;         // 排在 test_stop 之后，别被它的通用状态码盖掉
        return;
    }

    s_test_tick++;
    if (s_test_tick >= CTRL_DIV_SPEED) s_test_tick = 0;
    run5  = (uint8)((s_test_tick % CTRL_DIV_ATT) == 0);
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
        if (!ctrl_is_finite(output))
        {
            test_stop();
            s_test_status = CTRL_TEST_STATUS_OUTPUT_INVALID;
            return;
        }
        g_pwm_roll = output;
        // 浮点转整数前限幅。内环已经夹在 ±BAL_TEST_FLY_LIMIT，这里是兜底
        output = constrain_float(output, -(float)BAL_TEST_FLY_LIMIT, (float)BAL_TEST_FLY_LIMIT);
        g_motor_a = (int16)(-output);
        g_motor_b = (int16)(+output);
        W_Motor_Release();
        control_motor_output(g_motor_a, g_motor_b, 0);
    }
    else if (s_test_axis == TUNE_AXIS_PITCH)
    {
        output = pitch_test_ctrl(run5, run20);
        if (!ctrl_is_finite(output))
        {
            test_stop();
            s_test_status = CTRL_TEST_STATUS_OUTPUT_INVALID;
            return;
        }
        g_pwm_pitch = output;
        g_motor_c = (int16)output;
        // Pitch 只驱动 C 轮，但每拍仍要给 A/B 下发零输出帧，
        // 否则整段测试期间 CYT2BL3 收不到帧，会闩进失控保护
        W_Motor_Stop();
        test_drive_c_output(output);
    }
    else
    {
        output = yaw_test_ctrl(run5);
        if (!ctrl_is_finite(output))
        {
            test_stop();
            s_test_status = CTRL_TEST_STATUS_OUTPUT_INVALID;
            return;
        }
        g_pwm_yaw = output;
        // 浮点转整数前限幅。内环已经夹在 ±BAL_TEST_FLY_LIMIT，这里是兜底
        output = constrain_float(output, -(float)BAL_TEST_FLY_LIMIT, (float)BAL_TEST_FLY_LIMIT);
        g_motor_a = (int16)output;                       // Yaw 同向，A/B 同号
        g_motor_b = (int16)output;
        W_Motor_Release();
        control_motor_output(g_motor_a, g_motor_b, 0);
    }
}

// ====================== 安全闸 ======================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取姿态与 IMU 的阻断原因
// 参数说明     void
// 返回参数     control_block_t 阻断原因，CTRL_BLOCK_NONE 表示姿态可用
// 使用示例     block = control_attitude_block_reason();
//-------------------------------------------------------------------------------------------------------------------
static control_block_t control_attitude_block_reason(void)
{
    if (!g_imu_ok)              return CTRL_BLOCK_IMU_FAIL;
    if (imu_link_lost())        return CTRL_BLOCK_IMU_LOST;
    if (imu_calib_state() != IMU_CALIB_OK) return CTRL_BLOCK_IMU_CALIB;
    if (attitude_diverged())    return CTRL_BLOCK_ATT_DIVERGED;
    if (!attitude_converged())  return CTRL_BLOCK_ATT_CONVERGING;
    return CTRL_BLOCK_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把阻断原因翻译成 Test 状态码
// 参数说明     block           阻断原因
// 返回参数     control_test_status_t 对应状态码
// 使用示例     s_test_status = control_block_to_test_status(block);
//-------------------------------------------------------------------------------------------------------------------
static control_test_status_t control_block_to_test_status(control_block_t block)
{
    switch (block)
    {
        case CTRL_BLOCK_IMU_FAIL:     return CTRL_TEST_STATUS_IMU_FAIL;
        case CTRL_BLOCK_IMU_CALIB:    return CTRL_TEST_STATUS_IMU_CALIB;
        case CTRL_BLOCK_IMU_LOST:     return CTRL_TEST_STATUS_IMU_LOST;
        case CTRL_BLOCK_ATT_DIVERGED: return CTRL_TEST_STATUS_ATT_DIVERGED;
        case CTRL_BLOCK_BLDC_LOST:    return CTRL_TEST_STATUS_BLDC_LOST;
        default:                      return CTRL_TEST_STATUS_ATT_CONVERGING;
    }
}

// ====================== 对外接口 ======================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化参数、按键、两路电机、三轴串级、波形和 IMU660RB，最后开 1ms 中断
// 参数说明     void
// 返回参数     void
// 使用示例     control_init();
//-------------------------------------------------------------------------------------------------------------------
void control_init(void)
{
    g_cam_ok = 0;
    g_track_valid = 0;
    g_track_lost_frames = 0;
    g_vision_age_ms = 0;
    g_control_uptime_ms = 0;
    g_vision_frame_seq = 0;
    g_vision_heartbeat = 0;
    g_vision_threshold = 0;
    g_vision_search_stop = 0;
    g_vision_left_lost = 0;
    g_vision_right_lost = 0;
    g_vision_both_lost = 0;
    g_vision_active_elem = 0;
    g_vision_island_state = 0;
    g_vision_speed_scale = 1.0f;
    g_vision_stop_request = 0;
    g_vision_fps = 0.0f;
    g_vision_vsync_fps = 0.0f;
    g_vision_dma_fps = 0.0f;
    g_vision_drop_fps = 0.0f;
    g_vision_grab_us = 0;
    g_vision_binarize_us = 0;
    g_vision_edge_us = 0;
    g_vision_element_us = 0;
    g_vision_process_us = 0;
    g_vision_process_max_us = 0;
    s_control_test_active = 0;
    s_test_status = CTRL_TEST_STATUS_OK;
    s_jog_target = MOTOR_JOG_NONE;
    s_jog_duty = 0;

    param_init();
    key_init(CTRL_DIV_KEY);             // 扫描周期必须等于下面 control_loop 里调 key_scanner 的分频
    Y_Motor_Init();                     // C 轮 PWM/DIR 与脉冲方向编码器

    // 三轴串级初值
    g_roll_zero   = ROLL_ZERO_INIT;
    g_pitch_zero  = PITCH_ZERO_INIT;
    g_yaw_target  = 0;
    g_lean_offset = 0;
    s_lean_raw    = 0;
    s_speed_ramp  = 0.0f;
    start_flag    = START_STOP;

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

    vofa_init();                        // 无线转串口模块，波形与调参的唯一通道

    g_imu_ok = (imu_init() == 0) ? 1 : 0;
    if (g_imu_ok)
    {
        imu_calibrate();                // 600~1800ms 阻塞，期间没有任何东西在跑
        attitude_init();
    }

    // W_Motor_Init() 必须排在这里：CYT2BL3 的失控保护是 500ms 收不到占空比指令就闩死，
    // 而上面的静止标定要阻塞 600~1800ms。先初始化就等于开机必然把驱动闩进保护。
    // 放在这里，第一帧占空比和下面 1ms 中断开始连续下发之间几乎没有间隔。
    W_Motor_Init();                     // 上电锁定 A/B 软件刹车并请求转速回传
    pit_ms_init(CTRL_PIT_CH, CTRL_PERIOD_MS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止 Test 与点动，三电机清零并锁死动量轮软件刹车
// 参数说明     void
// 返回参数     void
// 使用示例     control_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_stop(void)
{
    control_test_stop();
    control_jog_stop();
    cascade_reset();                    // 三轴一起跑时的积分与增量累积也要清掉
    g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
    g_motor_a = g_motor_b = g_motor_c = 0;
    g_target_distance = 0;
    start_flag = START_STOP;
    if (g_vofa_mode == VOFA_BAL) g_vofa_mode = VOFA_OFF;
    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动三轴同时闭环，原地平衡，速度目标恒 0，视觉与转向命令都不参与
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止，原因见 control_test_last_status()
// 使用示例     if (!control_balance_start()) menu_status(...);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_balance_start(void)
{
    control_block_t block;

    // 不和单轴 Test、架空点动叠加，两边都在写同一批 PID 实例和同一批电机
    if (s_test_running || s_jog_target != MOTOR_JOG_NONE)
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }

    block = control_attitude_block_reason();
    if (block != CTRL_BLOCK_NONE)
    {
        s_test_status = control_block_to_test_status(block);
        return 0;
    }

    // A/B 要松刹车，驱动必须在线
    if (W_Motor_LinkLost())
    {
        s_test_status = CTRL_TEST_STATUS_BLDC_LOST;
        return 0;
    }

    // 起步时车必须已经扶到保护角以内。倒着按下去等于松刹车瞬间就给一个满幅冲击，
    // 而且下一拍保护角照样会把它停掉，白折腾一次起停
    if (fabsf(att.roll - g_roll_zero) > ROLL_PROTECT_ANGLE)
    {
        s_test_status = CTRL_TEST_STATUS_ROLL_PROT;
        return 0;
    }
    if (fabsf(att.pitch - g_pitch_zero) > PITCH_PROTECT_ANGLE)
    {
        s_test_status = CTRL_TEST_STATUS_PITCH_PROT;
        return 0;
    }

    cascade_reset();
    g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
    g_motor_a = g_motor_b = g_motor_c = 0;
    g_target_distance = 0;              // 原地平衡，速度环目标恒 0，Run 接进来之前不给非零速度
    g_yaw_target = imu_get_angle_yaw(); // 目标跟随当前航向，防松刹车瞬间的航向阶跃
    Y_Motor_EncoderClear();

    s_test_status = CTRL_TEST_STATUS_OK;
    start_flag = START_BALANCE;
    g_vofa_mode = VOFA_BAL;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动一次限时架空点动，用于确认电机转向与转速回读符号
// 参数说明     target/forward  点动电机与方向，forward 为 1 表示正转
// 返回参数     uint8           1=已启动 0=姿态、标定、运行状态或驱动条件不满足
// 使用示例     control_jog_start(MOTOR_JOG_A, 1);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_jog_start(motor_jog_t target, uint8 forward)
{
    control_block_t block;
    int   duty_i;
    int16 duty;

    if (target == MOTOR_JOG_NONE || start_flag != START_STOP || s_test_running)
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }

    block = control_attitude_block_reason();
    if (block != CTRL_BLOCK_NONE)
    {
        s_test_status = control_block_to_test_status(block);
        return 0;
    }
    if (target != MOTOR_JOG_C && W_Motor_LinkLost())
    {
        s_test_status = CTRL_TEST_STATUS_BLDC_LOST;
        return 0;
    }

    // 占空比是运行参数，Params -> Motor 的 Duty Fly / Duty Drv 两行可调
    duty_i = (target == MOTOR_JOG_C) ? JOG_DUTY_DRIVE : JOG_DUTY_FLY;
    if (duty_i < 0)     duty_i = 0;
    if (duty_i > 10000) duty_i = 10000;          // 转 int16 前先钳到满量程
    duty = (int16)duty_i;
    if (!forward) duty = (int16)(-duty);

    if (target != MOTOR_JOG_C) W_Motor_Release();
    s_jog_duty = duty;
    g_vofa_mode = VOFA_MOTOR;
    s_jog_target = target;              // 最后赋值：1ms 中断读到 target 时其余字段已就绪
    s_test_status = CTRL_TEST_STATUS_OK;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即结束点动并重新锁死动量轮软件刹车
// 参数说明     void
// 返回参数     void
// 使用示例     control_jog_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_jog_stop(void)
{
    uint8 was_running = (uint8)(s_jog_target != MOTOR_JOG_NONE);

    s_jog_target = MOTOR_JOG_NONE;
    s_jog_duty = 0;
    g_motor_a = 0;
    g_motor_b = 0;
    g_motor_c = 0;
    if (was_running && g_vofa_mode == VOFA_MOTOR) g_vofa_mode = VOFA_OFF;
    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询当前正在点动的电机
// 参数说明     void
// 返回参数     motor_jog_t     点动目标，MOTOR_JOG_NONE 表示未运行
// 使用示例     if (control_jog_running() == MOTOR_JOG_A) { ... }
//-------------------------------------------------------------------------------------------------------------------
motor_jog_t control_jog_running(void)
{
    return s_jog_target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一拍点动输出并倒计时，由 1ms 控制中断调用
// 参数说明     void
// 返回参数     void
// 使用示例     control_jog_run();
//-------------------------------------------------------------------------------------------------------------------
static void control_jog_run(void)
{
    motor_jog_t target = s_jog_target;

    if (target == MOTOR_JOG_NONE) return;
    // 点动没有时限，一直转到再按一次动作行、按返回键、驱动掉线或飞轮超速为止
    if (target != MOTOR_JOG_C && (W_Motor_LinkLost() || fly_overspeed()))
    {
        control_jog_stop();
        return;
    }

    g_motor_a = (int16)((target == MOTOR_JOG_A) ? s_jog_duty : 0);
    g_motor_b = (int16)((target == MOTOR_JOG_B) ? s_jog_duty : 0);
    g_motor_c = (int16)((target == MOTOR_JOG_C) ? s_jog_duty : 0);
    if (target == MOTOR_JOG_C)
    {
        W_Motor_Stop();
        Y_Motor_SetDuty(g_motor_c);
    }
    else
    {
        W_Motor_Release();
        control_motor_output(g_motor_a, g_motor_b, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴与最高启用环的 Test
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     control_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_start(tune_axis_t axis, tune_ring_t ring)
{
    control_block_t block;

    control_test_stop();
    control_jog_stop();
    if (axis >= TUNE_AXIS_MAX || ring >= TUNE_RING_MAX ||
        (axis == TUNE_AXIS_YAW && ring == TUNE_RING_VEL))
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }

    // Pitch 只驱动 DRV8701E 的 C 轮，不要求无刷驱动在线；Roll/Yaw 动飞轮则必须在线
    block = control_attitude_block_reason();
    if (block == CTRL_BLOCK_NONE && axis != TUNE_AXIS_PITCH && W_Motor_LinkLost())
        block = CTRL_BLOCK_BLDC_LOST;
    if (block != CTRL_BLOCK_NONE)
    {
        s_test_status = control_block_to_test_status(block);
        return 0;
    }

    if (!test_start(axis, ring))
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }

    g_tune_axis = axis;
    g_tune_ring = ring;
    if (axis == TUNE_AXIS_ROLL) g_vofa_mode = VOFA_ROLL;
    else if (axis == TUNE_AXIS_PITCH) g_vofa_mode = VOFA_PITCH;
    else g_vofa_mode = VOFA_YAW;

    s_control_test_active = 1;
    s_test_status = CTRL_TEST_STATUS_OK;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止 Test 并关闭波形输出
// 参数说明     void
// 返回参数     void
// 使用示例     control_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_test_stop(void)
{
    if (s_test_running) test_stop();
    s_control_test_active = 0;
    g_vofa_mode = VOFA_OFF;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 Test 是否正在运行，顺带识别被安全闸停掉的情况
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (control_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_running(void)
{
    if (s_control_test_active && !s_test_running)
    {
        s_control_test_active = 0;
        if (s_test_status == CTRL_TEST_STATUS_OK)
            s_test_status = CTRL_TEST_STATUS_SAFETY;
    }
    return s_control_test_active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次 Test 的启动结果或停止原因
// 参数说明     void
// 返回参数     control_test_status_t 状态码
// 使用示例     status = control_test_last_status();
//-------------------------------------------------------------------------------------------------------------------
control_test_status_t control_test_last_status(void)
{
    (void)control_test_running();
    return s_test_status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向 CPU1 发布视觉参数并接收上一帧循迹结果
// 参数说明     publish_feedback 1=本拍发布参数 0=只接收结果
// 返回参数     void
// 使用示例     control_vision_exchange(run5);
//-------------------------------------------------------------------------------------------------------------------
static void control_vision_exchange(uint8 publish_feedback)
{
    static uint32 input_seq;            // 本核发布序号，用于给结果配对
    static uint32 last_frame_seq;       // 上次已处理的视觉帧序号
    static uint32 fps_win_start_ms;     // 帧率统计窗口起点(ms)
    static uint16 fps_frames;           // 本窗口内收到的新帧数
    static uint32 camera_vsync_count;    // 最近一次收到的摄像头 VSYNC 累计值
    static uint32 camera_dma_count;      // 最近一次收到的 DMA 完整帧累计值
    static uint32 camera_drop_count;     // 最近一次收到的忙丢帧累计值
    static uint32 last_vsync_count;      // 上个统计窗口的 VSYNC 累计值
    static uint32 last_dma_count;        // 上个统计窗口的 DMA 累计值
    static uint32 last_drop_count;       // 上个统计窗口的忙丢帧累计值
    uint32 fps_win_ms;                  // 本窗口实际长度(ms)
    vision_feedback_t feedback;
    vision_result_t result;

    if (publish_feedback)
    {
        feedback.input_seq = ++input_seq;
        feedback.uptime_ms = g_control_uptime_ms;
        feedback.param_revision = g_param_revision;
        feedback.drive_count_total = Y_Motor_GetTotalCount();
        feedback.element_yaw = imu_get_angle_element();
        feedback.pitch = att.pitch;
        feedback.pitch_rate = att.pitch_rate;
        feedback.err_offset = g_param.err_offset;
        feedback.speed_ramp_gain = g_param.speed_ramp_gain;
        feedback.speed_ring_gain = g_param.speed_ring_gain;
        feedback.elem_en_zebra = (uint8)g_param.elem_en_zebra;
        feedback.elem_en_cross = (uint8)g_param.elem_en_cross;
        feedback.elem_en_ring = (uint8)g_param.elem_en_ring;
        feedback.elem_en_ramp = (uint8)g_param.elem_en_ramp;
        feedback.zebra_jump_cnt = g_param.zebra_jump_cnt;
        feedback.cross_lost_cnt = g_param.cross_lost_cnt;
        feedback.ring_angle = g_param.ring_angle;
        feedback.ring_s2_cnt_l = g_param.ring_s2_cnt_l;
        feedback.ring_s2_cnt_r = g_param.ring_s2_cnt_r;
        feedback.ring_side_offset = g_param.ring_side_offset;
        feedback.ring_timeout_cnt = g_param.ring_timeout_cnt;
        feedback.elem_guard_cnt = g_param.elem_guard_cnt;
        feedback.road_wide_near = g_param.road_wide_near;
        feedback.road_wide_far = g_param.road_wide_far;
        feedback.cam_exposure = (uint16)g_param.cam_exposure;
        feedback.reserved = 0;
        vision_feedback_publish(&feedback);
    }

    g_vision_heartbeat = vision_core_heartbeat();
    g_cam_ok = (uint8)(vision_core_state() == VISION_CORE_READY);
    if (vision_result_read(&result) &&
        result.frame_seq != 0u &&
        result.frame_seq != last_frame_seq)
    {
        last_frame_seq = result.frame_seq;
        g_vision_frame_seq = result.frame_seq;
        g_vision_age_ms = 0;
        if (fps_frames < 0xFFFFu) fps_frames++;
        camera_vsync_count = result.camera_vsync_count;
        camera_dma_count = result.camera_dma_count;
        camera_drop_count = result.camera_drop_count;
        g_vision_threshold = result.threshold;
        g_vision_search_stop = result.search_stop_line;
        g_vision_left_lost = result.left_lost;
        g_vision_right_lost = result.right_lost;
        g_vision_both_lost = result.both_lost;
        g_vision_active_elem = (uint8)result.active_elem;
        g_vision_island_state = result.island_state;
        g_vision_speed_scale = result.speed_scale;
        g_vision_stop_request = result.stop_request;
        g_track_valid = result.track_valid;
        g_vision_grab_us = result.grab_us;
        g_vision_binarize_us = result.binarize_us;
        g_vision_edge_us = result.edge_us;
        g_vision_element_us = result.element_us;
        g_vision_process_us = result.process_us;
        g_vision_process_max_us = result.process_max_us;

        // 丢线不等于居中：偏差回 0 的同时丢线计数往上走，控制层据此判断视觉是否可信
        if (result.track_valid)
        {
            g_track_lost_frames = 0;
            g_dbg_error = result.track_error;
        }
        else
        {
            if (g_track_lost_frames < 60000u) g_track_lost_frames++;
            g_dbg_error = 0.0f;
        }
    }

    // 帧率结算。本函数每 1ms 调一次，所以窗口长度就是 VISION_FPS_WIN_MS。
    // 摄像头停帧时窗口内计数为 0，读数自然掉到 0，不需要另设超时。
    fps_win_ms = g_control_uptime_ms - fps_win_start_ms;
    if (fps_win_ms >= VISION_FPS_WIN_MS)
    {
        uint32 vsync_frames = (camera_vsync_count >= last_vsync_count)
                            ? (camera_vsync_count - last_vsync_count) : camera_vsync_count;
        uint32 dma_frames = (camera_dma_count >= last_dma_count)
                          ? (camera_dma_count - last_dma_count) : camera_dma_count;
        uint32 drop_frames = (camera_drop_count >= last_drop_count)
                           ? (camera_drop_count - last_drop_count) : camera_drop_count;

        g_vision_fps = (float)fps_frames * 1000.0f / (float)fps_win_ms;
        g_vision_vsync_fps = (float)vsync_frames * 1000.0f / (float)fps_win_ms;
        g_vision_dma_fps = (float)dma_frames * 1000.0f / (float)fps_win_ms;
        g_vision_drop_fps = (float)drop_frames * 1000.0f / (float)fps_win_ms;
        last_vsync_count = camera_vsync_count;
        last_dma_count = camera_dma_count;
        last_drop_count = camera_drop_count;
        fps_frames = 0;
        fps_win_start_ms = g_control_uptime_ms;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一拍 1ms 控制周期，由 CCU60_CH0 中断调用
// 参数说明     void
// 返回参数     void
// 使用示例     control_loop();
//-------------------------------------------------------------------------------------------------------------------
void control_loop(void)
{
    static uint8 key_tick;              // 按键扫描分频计数
    static uint8 tick;                  // 5ms 分频计数

    g_control_uptime_ms++;
    if (g_vision_age_ms < 0xFFFFu) g_vision_age_ms++;
    W_Motor_Tick1ms();                  // 刷新驱动通信超时并按需补发转速请求

    key_tick++;
    if (key_tick >= CTRL_DIV_KEY)
    {
        key_tick = 0;
        key_scanner();
    }

    tick++;
    if (tick >= CTRL_DIV_ATT) tick = 0;
    if (tick == 0)
        Y_Motor_EncoderUpdate5ms();     // C 轮编码器唯一硬件读取点

    if (g_imu_ok)
    {
        imu_update_gyro();
        attitude_update_rate();

        if (tick == 0)
            attitude_update();

        g_bal_dbg.r_ang_fb  = att.roll;
        g_bal_dbg.r_rate_fb = att.roll_rate;
        g_bal_dbg.p_ang_fb  = att.pitch;
        g_bal_dbg.p_rate_fb = att.pitch_rate;
        g_bal_dbg.y_fb      = att.yaw;
        g_bal_dbg.y_rate_fb = att.yaw_rate;
    }

    control_vision_exchange((uint8)(tick == 0));

    // 三个输出分支互斥，每拍必有一个给 W_Motor 下发帧，喂住驱动侧失控保护看门狗
    if (s_test_running)
    {
        test_run();
        if (!s_test_running)
        {
            s_control_test_active = 0;
            if (s_test_status == CTRL_TEST_STATUS_OK)
                s_test_status = CTRL_TEST_STATUS_SAFETY;
        }
    }
    else if (s_jog_target != MOTOR_JOG_NONE)
    {
        control_jog_run();
    }
    else
    {
        cascade_run();
    }
    vofa_snapshot();
    vofa_tick1ms();                     // 搬运无线串口收发字节，与主循环刷屏解耦
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 摄像头状态，失败时请求重新初始化
// 参数说明     void
// 返回参数     uint8           1=摄像头就绪 0=未就绪
// 使用示例     control_camera_debug_start();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_camera_debug_start(void)
{
    vision_core_state_t state = vision_core_state();

    if (state == VISION_CORE_FAILED)
        (void)vision_command_request(VISION_CMD_RESTART_CAMERA, 0);
    g_cam_ok = (uint8)(state == VISION_CORE_READY);
    return g_cam_ok;
}
