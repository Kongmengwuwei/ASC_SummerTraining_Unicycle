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
float g_flywheel_common_rpm = 0.0f;             // (A+B)/2共模RPM
float g_yaw_momentum_scale = 1.0f;              // 共模动量剩余权限

float g_dbg_error   = 0.0f;                     // 归一化横向偏差，正值要求正向 Yaw
uint8 g_imu_ok      = 0;                        // IMU660RB 初始化结果
uint8 g_cam_ok      = 0;                        // CPU1 摄像头就绪标志
uint8 g_vision_ipm_ok = 0;                      // CPU1 侧逆透视是否可用，Camera 页显示 IPM0/1
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
float  g_vision_lateral_error = 0.0f;           // 车道半宽归一化横向误差
float  g_vision_heading_error = 0.0f;           // 赛道航向误差(°)
float  g_vision_curvature = 0.0f;               // 有符号归一化曲率
float  g_vision_direction_camera = 0.0f;        // 报告同款加权方向偏差
uint8  g_vision_track_mode = TRACK_MODE_MIDDLE; // 当前元素选择的跟踪对象
float  g_vision_quality = 0.0f;                 // 循迹质量(0..1)
float  g_vision_speed_limit_mps = RUN_SPEED_MAX_MPS; // 视觉/元素绝对限速(m/s)
uint8  g_vision_stop_request = 0;               // 斑马线终点请求
float  g_run_speed_target_mps = 0.0f;           // 正式 Run 当前规划速度(m/s)
float  g_vision_fps = 0.0f;                     // CPU1 出帧率(帧/s)，1ms 中断每 VISION_FPS_WIN_MS 结算一次

// ====================== 内部状态 ======================

static pid_t r_rcy_pid, r_angle_pid, r_rate_pid;    // Roll ：飞轮回收 / 角度 / 角速度
static pid_t p_vel_pid, p_angle_pid, p_rate_pid;    // Pitch：速度 / 角度 / 角速度
static pid_t y_angle_pid, y_rate_pid;               // Yaw  ：转向外环 / 角速度内环

static float s_speed_ramp;                      // 斜坡后的速度环实际目标(counts/20ms)
static float s_rcy_fb;                          // 飞轮差速 A-B(RPM)，只在 20ms 拍更新

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

// 无线 Run Test 状态。主循环写命令，1ms 中断读取并更新速度与航向目标。
static volatile uint8  s_remote_active;          // 1=Run Test 已启动
static volatile uint8  s_remote_cmd_seen;        // 1=至少收到过一条合法 speed 命令
static volatile uint8  s_remote_timeout_latched; // 1=本次命令已执行过超时急停
static volatile uint16 s_remote_cmd_age_ms;      // 距最新合法命令的时间
static volatile float  s_remote_steer_angle;     // 最近一次相对航向指令(°)
static volatile float  s_remote_speed_mps;       // 行进速度目标(m/s)
static volatile float  s_remote_yaw_target;      // 当前绝对航向目标(°)

// 正式跑车(Run)状态。目标更新在 1ms 中断，菜单只负责启停和显示
static volatile uint8  s_run_active;             // 1=Run 已启动
static volatile run_stop_t s_run_stop;           // 停车原因，菜单读走显示
static volatile uint16 s_run_lost_ms;            // 连续丢线时间(ms)
static float           s_direction_offset;       // 滤波后的平均像素偏差
static float           s_direction_last_error;   // 方向PD上一拍滤波误差
static float           s_direction_yaw_rate_target; // 视觉生成的目标横摆角速度
static float           s_direction_yaw_rate_cmd; // 斜率限制后的目标横摆角速度
static float           s_direction_hold_yaw;     // 十字或丢线进入时锁存的航向
static uint8           s_direction_last_mode;    // 上一拍跟踪模式
static float           s_roll_angle_target;      // Roll 回收与压弯合成后的实际角度目标
static float           s_yaw_output_applied;     // Roll 优先混控后实际分配给 Yaw 的输出
static uint8           s_zebra_stop_latched;      // 斑马线终点请求锁存
static int32           s_zebra_stop_start_count;  // 锁存请求时的 C 轮累计里程
static uint8           s_run_vision_armed;         // 已收到应用本次 Run 快照的视觉帧
static volatile uint8  s_ipm_calib_seq_last;     // 上次已取走的 CPU1 标定序号
static volatile uint8  s_ipm_new;                // 1=有新矩阵等着写 Flash

// 1m 里程验证状态。目标更新在 1ms 中断，菜单只负责启停和显示。
static volatile odom_test_state_t s_odom_test_state;
static int32 s_odom_test_start_count;
static float s_odom_test_yaw_zero;

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
// 函数简介     判断动量轮是否超过软件转速保护阈值
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
    s_speed_ramp = 0.0f;
    s_rcy_fb = 0.0f;                                 // 回收环反馈
    g_flywheel_common_rpm = 0.0f;
    g_yaw_momentum_scale = 1.0f;
    s_direction_offset = 0.0f;
    s_direction_last_error = 0.0f;
    s_direction_yaw_rate_target = 0.0f;
    s_direction_yaw_rate_cmd = 0.0f;
    s_direction_hold_yaw = 0.0f;
    s_direction_last_mode = TRACK_MODE_MIDDLE;
    s_roll_angle_target = g_roll_zero;
    s_yaw_output_applied = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除正式 Run、无线 Run Test 与 1m 里程验证的运行状态
// 参数说明     void
// 返回参数     void
// 使用示例     remote_state_reset();
//-------------------------------------------------------------------------------------------------------------------
static void remote_state_reset(void)
{
    s_remote_active = 0;
    s_remote_cmd_seen = 0;
    s_remote_timeout_latched = 0;
    s_remote_cmd_age_ms = 0;
    s_remote_steer_angle = 0.0f;
    s_remote_speed_mps = 0.0f;
    s_remote_yaw_target = 0.0f;
    s_odom_test_state = ODOM_TEST_IDLE;
    s_odom_test_start_count = 0;
    s_odom_test_yaw_zero = 0.0f;
    s_run_active = 0;
    s_run_vision_armed = 0;
    s_run_lost_ms = 0;
    g_run_speed_target_mps = 0.0f;
    s_zebra_stop_latched = 0;
    s_zebra_stop_start_count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 m/s 速度目标换算并限制为 Pitch 速度环使用的 counts/20ms
// 参数说明     speed_mps       目标线速度(m/s)
// 返回参数     int             受保护的速度环目标(counts/20ms)
// 使用示例     g_target_distance = speed_target_from_mps(0.20f);
//-------------------------------------------------------------------------------------------------------------------
static int speed_target_from_mps(float speed_mps)
{
    float target;

    speed_mps = constrain_float(speed_mps, -RUN_SPEED_MAX_MPS, RUN_SPEED_MAX_MPS);
    target = Y_Motor_MpsToCount20ms(speed_mps);
    return (target >= 0.0f) ? (int)(target + 0.5f) : (int)(target - 0.5f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将正式跑车分段速度限制在直道速度以内
// 参数说明     speed_mps       分段目标速度(m/s)
// 返回参数     float           受限速度(m/s)
// 使用示例     speed = run_speed_limit(RUN_SPEED_CURVE);
//-------------------------------------------------------------------------------------------------------------------
static float run_speed_limit(float speed_mps)
{
    float straight = constrain_float(RUN_SPEED_STRAIGHT, 0.0f, RUN_SPEED_MAX_MPS);
    return constrain_float(speed_mps, 0.0f, straight);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 是否已经停帧
// 参数说明     void
// 返回参数     uint8           1=超过 VISION_LINK_TIMEOUT_MS 没收到新帧
// 使用示例     if (vision_link_lost()) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_link_lost(void)
{
    return (uint8)(g_vision_age_ms >= VISION_LINK_TIMEOUT_MS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     结束正式跑车并保持原地平衡
// 参数说明     reason          停车原因
// 返回参数     void
// 使用示例     run_hold_balance(RUN_STOP_VISION);
//-------------------------------------------------------------------------------------------------------------------
static void run_hold_balance(run_stop_t reason)
{
    s_run_stop = reason;
    s_run_active = 0;
    s_run_vision_armed = 0;
    s_run_lost_ms = 0;
    g_run_speed_target_mps = 0.0f;
    g_target_distance = 0;
    s_speed_ramp = 0.0f;
    s_direction_yaw_rate_target = 0.0f;
    s_direction_yaw_rate_cmd = 0.0f;
    s_direction_offset = 0.0f;
    s_direction_last_error = 0.0f;
    s_direction_last_mode = TRACK_MODE_HOLD;
    pid_reset(&p_vel_pid);
    g_lean_offset = 0.0f;
    g_yaw_target = imu_get_angle_yaw();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据最新视觉结果生成正式 Run 的分段速度目标
// 参数说明     void
// 返回参数     void
// 使用示例     run_vision_target_update();
//-------------------------------------------------------------------------------------------------------------------
static void run_vision_target_update(void)
{
    float curve_ratio;
    float quality_ratio;
    float quality_limit;
    float speed;
    float speed_limit;

    if (!s_run_active || !s_run_vision_armed)
    {
        g_run_speed_target_mps = 0.0f;
        return;
    }

    curve_ratio = fabsf(g_vision_curvature) /
                  constrain_float(TRACK_CURVE_FULL_SCALE, 0.01f, 10.0f);
    curve_ratio = constrain_float(curve_ratio, 0.0f, 1.0f);
    speed = run_speed_limit(RUN_SPEED_STRAIGHT) +
            (run_speed_limit(RUN_SPEED_CURVE) - run_speed_limit(RUN_SPEED_STRAIGHT)) * curve_ratio;

    if (g_vision_active_elem == (uint8)ELEM_CROSS)
        speed = run_speed_limit(RUN_SPEED_CROSS);
    else if (g_vision_active_elem == (uint8)ELEM_RING_LEFT ||
             g_vision_active_elem == (uint8)ELEM_RING_RIGHT)
        speed = run_speed_limit(RUN_SPEED_RING);
    else if (g_vision_active_elem == (uint8)ELEM_RAMP)
        speed = run_speed_limit(RUN_SPEED_RAMP);
    else if (g_vision_active_elem == (uint8)ELEM_ZEBRA)
        speed = run_speed_limit(RUN_SPEED_CROSS);

    if (!g_track_valid)
    {
        speed = run_speed_limit(RUN_SPEED_LOST);
    }
    else if (g_vision_active_elem == (uint8)ELEM_NONE)
    {
        float quality_min = constrain_float(TRACK_QUALITY_MIN, 0.0f, 1.0f);
        float valid_speed_floor = run_speed_limit(RUN_VALID_SPEED_FLOOR_MPS);

        if (quality_min >= 0.999f)
            quality_ratio = (g_vision_quality >= quality_min) ? 1.0f : 0.0f;
        else
            quality_ratio = constrain_float((g_vision_quality - quality_min) /
                                             (1.0f - quality_min), 0.0f, 1.0f);
        quality_limit = run_speed_limit(RUN_SPEED_LOST) +
                        (run_speed_limit(RUN_SPEED_STRAIGHT) - run_speed_limit(RUN_SPEED_LOST)) * quality_ratio;
        if (quality_limit < valid_speed_floor) quality_limit = valid_speed_floor;
        if (speed > quality_limit) speed = quality_limit;
    }

    speed_limit = run_speed_limit(g_vision_speed_limit_mps);
    if (speed > speed_limit) speed = speed_limit;

    g_run_speed_target_mps = run_speed_limit(speed * g_yaw_momentum_scale);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跑车模式下每 1ms 更新一次转向与速度目标
// 参数说明     void
// 返回参数     void
// 使用示例     run_target_update();
//-------------------------------------------------------------------------------------------------------------------
static void run_target_update(void)
{
    float speed_mps;

    if (vision_link_lost())
    {
        run_hold_balance(RUN_STOP_VISION);
        return;
    }

    if (g_vision_stop_request && !s_zebra_stop_latched)
    {
        s_zebra_stop_latched = 1;
        s_zebra_stop_start_count = Y_Motor_GetTotalCount();
        s_run_stop = RUN_STOP_ZEBRA;
    }

    if (s_zebra_stop_latched)
    {
        float distance = fabsf(Y_Motor_CountToMeter(Y_Motor_GetTotalCount() -
                                                    s_zebra_stop_start_count));

        if (distance < ZEBRA_STOP_OFFSET_M)
        {
            speed_mps = run_speed_limit(RUN_SPEED_CROSS);
            if (g_run_speed_target_mps > 0.0f && g_run_speed_target_mps < speed_mps)
                speed_mps = g_run_speed_target_mps;
            g_target_distance = speed_target_from_mps(speed_mps);
            return;
        }

        g_target_distance = 0;
        if (fabsf(s_speed_ramp) < 1.0f &&
            func_abs(Y_Motor_GetSpeed20ms()) <= RUN_STOP_SPEED_CNT)
        {
            run_hold_balance(RUN_STOP_ZEBRA);
        }
        return;
    }

    if (!g_track_valid)
    {
        if (s_run_lost_ms < 0xFFFFu) s_run_lost_ms++;
        if (s_run_lost_ms >= RUN_LOST_STOP_MS)
        {
            run_hold_balance(RUN_STOP_LOST);
            return;
        }
        speed_mps = run_speed_limit(RUN_SPEED_LOST);
    }
    else
    {
        s_run_lost_ms = 0;
        speed_mps = g_run_speed_target_mps;
    }

    g_target_distance = speed_target_from_mps(speed_mps);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新无线遥控或 1m 验证的速度与航向目标
// 参数说明     void
// 返回参数     void
// 使用示例     motion_target_update();
//-------------------------------------------------------------------------------------------------------------------
static void motion_target_update(void)
{
    if (s_odom_test_state == ODOM_TEST_DONE && start_flag == START_BALANCE)
    {
        g_target_distance = 0;
        g_yaw_target = s_odom_test_yaw_zero;
        return;
    }

    if (s_odom_test_state == ODOM_TEST_RUNNING && start_flag == START_BALANCE)
    {
        int32 count = Y_Motor_GetTotalCount() - s_odom_test_start_count;
        float distance = fabsf(Y_Motor_CountToMeter(count));
        float remain = ODOM_TEST_DISTANCE_M - distance;
        float speed_mps = ODOM_TEST_SPEED;

        g_yaw_target = s_odom_test_yaw_zero;
        if (distance >= ODOM_TEST_DISTANCE_M)
        {
            s_odom_test_state = ODOM_TEST_DONE;
            g_target_distance = 0;
            s_speed_ramp = 0.0f;
            pid_reset(&p_vel_pid);
            return;
        }

        if (remain < ODOM_TEST_SLOW_DISTANCE_M)
        {
            speed_mps *= remain / ODOM_TEST_SLOW_DISTANCE_M;
            if (speed_mps < ODOM_TEST_MIN_SPEED_MPS)
                speed_mps = ODOM_TEST_MIN_SPEED_MPS;
        }
        g_target_distance = speed_target_from_mps(speed_mps);
        return;
    }

    // 正式跑车。速度规划、航向积分与斑马线停车均由 Run 状态更新。
    if (s_run_active && start_flag == START_BALANCE)
    {
        run_target_update();
        return;
    }

    if (!s_remote_active || start_flag != START_BALANCE) return;

    if (s_remote_cmd_age_ms < 0xFFFFu) s_remote_cmd_age_ms++;
    if (!s_remote_cmd_seen)
    {
        g_target_distance = 0;
        return;
    }

    if (s_remote_cmd_age_ms > REMOTE_CMD_TIMEOUT_MS)
    {
        // 失联停车不能继续走普通速度斜坡，否则最大速度下还要数秒才归零。
        // 只在跨过超时门限时清一次 PID，随后仍保留速度闭环以维持原地平衡。
        if (!s_remote_timeout_latched)
        {
            s_remote_timeout_latched = 1;
            /* Also release steering: do not finish an old turn after link loss. */
            s_remote_steer_angle = 0.0f;
            s_remote_yaw_target = imu_get_angle_yaw();
            g_yaw_target = s_remote_yaw_target;
            s_speed_ramp = 0.0f;
            pid_reset(&p_vel_pid);
        }
        s_remote_speed_mps = 0.0f;
        g_target_distance = 0;
        return;
    }

    // 航向与速度目标在同一拍发布，速度斜坡只限制加速度，不等待转向完成。
    g_yaw_target = s_remote_yaw_target;
    g_target_distance = speed_target_from_mps(s_remote_speed_mps);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按 20ms 周期平滑更新速度环目标
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

    // 正式 Run 的斜坡参数使用 SI 单位；遥控与 1m 验证保持原 counts/20ms 步长。
    if (s_run_active)
    {
        float accel = (fabsf(target) > fabsf(s_speed_ramp)) ? RUN_ACCEL_MPS2 : RUN_DECEL_MPS2;
        float delta_mps = accel * (float)CTRL_DIV_SPEED * 0.001f;

        step = fabsf(Y_Motor_MpsToCount20ms(delta_mps));
    }
    else
    {
        step = (fabsf(target) > fabsf(s_speed_ramp)) ? MOTION_SPEED_UP_STEP_COUNT
                                                     : MOTION_SPEED_DOWN_STEP_COUNT;
    }
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
// 函数简介     将滤波后的方向偏差、赛道航向和曲率合成为目标横摆角速度
// 参数说明     direction_now   当前方向偏差
// 参数说明     direction_zero  方向偏差目标
// 返回参数     float           目标横摆角速度(°/s)
// 使用示例     yaw_rate = direction_balance_Control(offset, 0.0f);
//-------------------------------------------------------------------------------------------------------------------
static float direction_balance_Control(float direction_now, float direction_zero)
{
    float error = direction_now - direction_zero;
    float error_rate = (error - s_direction_last_error) / 0.020f;
    float heading = -(float)STEER_DIR * g_vision_heading_error;
    float curvature = -(float)STEER_DIR * g_vision_curvature;
    float yaw_rate;

    error_rate = constrain_float(error_rate,
                                 -DIRECTION_D_RATE_LIMIT,
                                  DIRECTION_D_RATE_LIMIT);
    s_direction_last_error = error;
    yaw_rate = DIRECTION_PIXEL_KP * error +
               DIRECTION_HEADING_KP * heading +
               DIRECTION_CURVE_KFF * fabsf(Y_Motor_GetSpeedMps()) * curvature +
               DIRECTION_BALANCE_KD * error_rate;
    if (!ctrl_is_finite(yaw_rate)) return 0.0f;
    return constrain_float(yaw_rate,
                           -DIRECTION_YAW_RATE_LIMIT * g_yaw_momentum_scale,
                            DIRECTION_YAW_RATE_LIMIT * g_yaw_momentum_scale);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按跟踪模式更新方向偏差、目标航向和压弯共用控制量
// 参数说明     void
// 返回参数     void
// 使用示例     if (run20) direction_control_update();
//-------------------------------------------------------------------------------------------------------------------
static void direction_control_update(void)
{
    float raw_offset;
    uint8 mode;

    if (!s_run_active)
    {
        s_direction_offset = 0.0f;
        s_direction_last_error = 0.0f;
        s_direction_yaw_rate_target = 0.0f;
        s_direction_yaw_rate_cmd = 0.0f;
        s_direction_last_mode = TRACK_MODE_MIDDLE;
        return;
    }

    mode = g_vision_track_mode;
    if (mode > TRACK_MODE_HOLD) mode = TRACK_MODE_MIDDLE;

    if (mode == TRACK_MODE_HOLD || !g_track_valid)
    {
        if (s_direction_last_mode != TRACK_MODE_HOLD)
            s_direction_hold_yaw = imu_get_angle_yaw();
        s_direction_offset = 0.0f;
        s_direction_last_error = 0.0f;
        s_direction_yaw_rate_target = 0.0f;
        g_yaw_target = s_direction_hold_yaw;
        mode = TRACK_MODE_HOLD;
    }
    else
    {
        raw_offset = (float)STEER_DIR *
                     constrain_float(g_vision_direction_camera,
                                     -DIRECTION_CAMERA_LIMIT,
                                      DIRECTION_CAMERA_LIMIT) /
                     DIRECTION_CAMERA_WEIGHT_SUM;
        if (mode != s_direction_last_mode)
        {
            s_direction_offset = raw_offset;
            s_direction_last_error = raw_offset;
        }
        else
        {
            s_direction_offset += DIRECTION_ERROR_ALPHA *
                                  (raw_offset - s_direction_offset);
        }
        s_direction_yaw_rate_target = direction_balance_Control(s_direction_offset, 0.0f);
    }
    s_direction_last_mode = mode;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     平滑目标横摆角速度并积分生成连续航向目标
// 参数说明     void
// 返回参数     void
// 使用示例     direction_yaw_target_update();
//-------------------------------------------------------------------------------------------------------------------
static void direction_yaw_target_update(void)
{
    float yaw_now;
    float delta;
    float step;

    if (!s_run_active) return;
    if (s_direction_last_mode == TRACK_MODE_HOLD)
    {
        s_direction_yaw_rate_cmd = 0.0f;
        g_yaw_target = s_direction_hold_yaw;
        return;
    }

    delta = s_direction_yaw_rate_target - s_direction_yaw_rate_cmd;
    step = DIRECTION_YAW_SLEW * 0.001f;
    if (delta > step)       s_direction_yaw_rate_cmd += step;
    else if (delta < -step) s_direction_yaw_rate_cmd -= step;
    else                    s_direction_yaw_rate_cmd = s_direction_yaw_rate_target;

    yaw_now = imu_get_angle_yaw();
    g_yaw_target += s_direction_yaw_rate_cmd * 0.001f;
    g_yaw_target = yaw_now + constrain_float(g_yaw_target - yaw_now,
                                              -DIRECTION_YAW_LEAD_LIMIT,
                                               DIRECTION_YAW_LEAD_LIMIT);
    if (!ctrl_is_finite(g_yaw_target))
    {
        g_yaw_target = yaw_now;
        s_direction_yaw_rate_target = 0.0f;
        s_direction_yaw_rate_cmd = 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按山大方向输出与 C 轮速度生成压弯角，并增加可调动态零点上限
// 参数说明     direction_output 由航向目标差按山大 200 倍关系还原的方向输出
// 返回参数     float           本拍生效的压弯角(°)
// 使用示例     g_lean_offset = lean_offset_update((g_yaw_target - yaw) / 200.0f);
//-------------------------------------------------------------------------------------------------------------------
static float lean_offset_update(float direction_output)
{
    float limit = constrain_float(LEAN_MAX_ANGLE, 0.0f, DIRECTION_LEAN_LIMIT);
    float target = 0.0f;
    float delta;

    if (s_run_active && fabsf(g_vision_direction_camera) > DIRECTION_LEAN_ERROR_DEAD)
    {
        target = direction_output * (float)Y_Motor_GetSpeed20ms() *
                 DIRECTION_ROLL_KP / DIRECTION_LEAN_FORMULA_DIV;
        target = constrain_float(target, -limit, limit);
    }
    delta = target - g_lean_offset;
    if (delta > DIRECTION_LEAN_SLEW)       return g_lean_offset + DIRECTION_LEAN_SLEW;
    if (delta < -DIRECTION_LEAN_SLEW)      return g_lean_offset - DIRECTION_LEAN_SLEW;
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
    float warn_rpm;
    float hard_rpm;

    if (!run20) return r_rcy_pid.out;

    // 回收环直接使用 CYT2BL3 回传的两轮转速差，不再增加额外低通。
    s_rcy_fb = (float)(W_Motor_GetSpeed1() - W_Motor_GetSpeed2());

    // 单项实测：共模动量在2500~6000RPM区间降低转向与Run速度，不改Roll差速回收路径。
    g_flywheel_common_rpm = 0.5f * ((float)W_Motor_GetSpeed1() +
                                    (float)W_Motor_GetSpeed2());
    g_yaw_momentum_scale = 1.0f;
    warn_rpm = YAW_MOMENTUM_WARN_RPM;
    hard_rpm = YAW_MOMENTUM_HARD_RPM;
    if (warn_rpm >= 0.0f && hard_rpm > warn_rpm)
    {
        if (fabsf(g_flywheel_common_rpm) >= hard_rpm)
            g_yaw_momentum_scale = YAW_MOMENTUM_MIN_SCALE;
        else if (fabsf(g_flywheel_common_rpm) > warn_rpm && hard_rpm > warn_rpm)
            g_yaw_momentum_scale = 1.0f -
                (1.0f - YAW_MOMENTUM_MIN_SCALE) *
                (fabsf(g_flywheel_common_rpm) - warn_rpm) / (hard_rpm - warn_rpm);
    }

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
    // 位置式内环在误差回零时同步撤销输出，并在 PID 内完成抗饱和。
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
    float angle_target = zero - rcy;

    angle_target = g_roll_zero + constrain_float(angle_target - g_roll_zero,
                                                  -ROLL_TARGET_LIMIT,
                                                   ROLL_TARGET_LIMIT);
    s_roll_angle_target = angle_target;

    if (run5)
        pid_loc_calc(&r_angle_pid, att.roll - angle_target); // 回收与压弯合成目标限制在 ±10°
    roll_rate_ctrl((float)FLYWHEEL_OUT_LIMIT);               // 角速度环，位置式

    // 角度环不重复限幅，最终输出由角速度环限制。
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
    // 速度环使用斜坡后的目标。输出必须走 _limited：它是角度命令，不限幅会命令出几十度
    if (run20) pid_loc_calc_limited(&p_vel_pid, (float)Y_Motor_GetSpeed20ms() - s_speed_ramp,
                                    -P_VEL_LIMIT, P_VEL_LIMIT);
    if (run5)  pid_loc_calc(&p_angle_pid, p_vel_pid.out - att.pitch + zero);             // 角度环，位置式
    pid_inc_calc_limited(&p_rate_pid, -att.pitch_rate + p_angle_pid.out,
                         -DRIVE_OUT_LIMIT, DRIVE_OUT_LIMIT);                             // 角速度环，增量式

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
    if (s_run_active && s_run_stop == RUN_STOP_NONE) s_run_stop = RUN_STOP_SAFETY;

    cascade_reset();
    remote_state_reset();
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
    float roll_cmd, yaw_cmd;
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

    if (run20) direction_control_update();
    direction_yaw_target_update();
    if (run5)
    {
        float direction_output = constrain_float(g_yaw_target - imu_get_angle_yaw(),
                                                  -DIRECTION_YAW_LEAD_LIMIT,
                                                   DIRECTION_YAW_LEAD_LIMIT) /
                                 DIRECTION_LEAN_OUTPUT_SCALE;
        g_lean_offset = lean_offset_update(direction_output);
    }
    g_pwm_yaw = yaw_cascade_ctrl(run5);
    if (run20) speed_ramp_update();                      // 速度目标斜坡与速度环同拍
    g_pwm_roll  = roll_cascade_ctrl(g_roll_zero + LEAN_DIR * g_lean_offset, run5, run20);
    g_pwm_pitch = pitch_cascade_ctrl(g_pitch_zero, run5, run20);

    // Roll优先：先完整保留平衡控制量，Yaw只使用A/B两侧共同剩余的对称余量。
    roll_cmd = constrain_float(g_pwm_roll, -(float)FLYWHEEL_OUT_LIMIT, (float)FLYWHEEL_OUT_LIMIT);
    yaw_cmd  = constrain_float(g_pwm_yaw,
                               -((float)FLYWHEEL_OUT_LIMIT - fabsf(roll_cmd)),
                                ((float)FLYWHEEL_OUT_LIMIT - fabsf(roll_cmd)));
    s_yaw_output_applied = yaw_cmd;

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
        cascade_stop((fabsf(roll_err) > ROLL_PROTECT_ANGLE) ? CTRL_TEST_STATUS_ROLL_PROT
                                                            : CTRL_TEST_STATUS_PITCH_PROT);
        return;
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
            pid_loc_calc_limited(&p_vel_pid, (float)s_test_speed_c - s_speed_ramp,
                                 -P_VEL_LIMIT, P_VEL_LIMIT);
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
    g_vision_lateral_error = 0.0f;
    g_vision_heading_error = 0.0f;
    g_vision_curvature = 0.0f;
    g_vision_direction_camera = 0.0f;
    g_vision_track_mode = TRACK_MODE_MIDDLE;
    g_vision_quality = 0.0f;
    g_vision_speed_limit_mps = RUN_SPEED_MAX_MPS;
    g_vision_stop_request = 0;
    g_run_speed_target_mps = 0.0f;
    g_vision_fps = 0.0f;
    s_control_test_active = 0;
    s_test_status = CTRL_TEST_STATUS_OK;
    s_jog_target = MOTOR_JOG_NONE;
    s_jog_duty = 0;
    remote_state_reset();

    param_init();
    key_init(CTRL_DIV_KEY);             // 扫描周期必须等于下面 control_loop 里调 key_scanner 的分频
    Y_Motor_Init();                     // C 轮 PWM/DIR 与脉冲方向编码器

    // 三轴串级初值
    g_roll_zero   = ROLL_ZERO_INIT;
    g_pitch_zero  = PITCH_ZERO_INIT;
    g_yaw_target  = 0;
    g_lean_offset = 0;
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

    vofa_init();                        // 无线串口：Run Test 命令与姿态波形

    g_imu_ok = (imu_init() == 0) ? 1 : 0;
    if (g_imu_ok)
    {
        imu_calibrate();                // 600~1800ms 阻塞，期间没有任何东西在跑
        attitude_init();
    }

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
    remote_state_reset();
    g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
    g_motor_a = g_motor_b = g_motor_c = 0;
    g_target_distance = 0;
    start_flag = START_STOP;
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
    remote_state_reset();
    g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
    g_motor_a = g_motor_b = g_motor_c = 0;
    g_target_distance = 0;              // 普通 Balance 原地平衡；Run Test 启动后再由无线命令更新
    g_yaw_target = imu_get_angle_yaw(); // 目标跟随当前航向，防松刹车瞬间的航向阶跃
    Y_Motor_EncoderClear();

    s_test_status = CTRL_TEST_STATUS_OK;
    start_flag = START_BALANCE;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 用当前这一帧直道图标定逆透视
// 参数说明     void
// 返回参数     uint8           1=命令已发出
// 使用示例     control_ipm_calib_request();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_ipm_calib_request(void)
{
    if (vision_core_state() != VISION_CORE_READY) return 0;
    return vision_command_request(VISION_CMD_CALIB_IPM, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询是否还有标定好的逆透视矩阵在等着写 Flash
// 参数说明     void
// 返回参数     uint8           1=还没写进 Flash 0=已经落盘或本来就没有新矩阵
// 使用示例     if (!control_ipm_pending()) menu_status("IPM SAVED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_ipm_pending(void)
{
    return (uint8)s_ipm_new;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在主循环将新标定的逆透视矩阵写入 Flash
// 参数说明     void
// 返回参数     uint8           1=有待存矩阵且 Flash 写入成功，0=无待存矩阵或保存失败
// 使用示例     if (control_ipm_flush()) menu_status("IPM SAVED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_ipm_flush(void)
{
    static const char *const names[9] =
    {
        "ipm_h0", "ipm_h1", "ipm_h2",
        "ipm_h3", "ipm_h4", "ipm_h5",
        "ipm_h6", "ipm_h7", "ipm_h8",
    };

    if (!s_ipm_new || start_flag != START_STOP ||
        control_test_running() || control_jog_running() != MOTOR_JOG_NONE)
        return 0;
    if (!param_save_names(names, 9u)) return 0;
    s_ipm_new = 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动正式跑车，先按三轴平衡的全部安全条件完成发车
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     if (!control_run_start()) menu_status("RUN BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_run_start(void)
{
    if (start_flag != START_STOP)
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }
    // 视觉不可用就不许发车：跑车全靠它给转向
    if (vision_core_state() != VISION_CORE_READY || vision_link_lost() ||
        !g_vision_ipm_ok || !g_track_valid)
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }
    if (!control_balance_start()) return 0;

    s_run_active = 1;
    s_run_stop = RUN_STOP_NONE;
    s_run_lost_ms = 0;
    g_run_speed_target_mps = 0.0f;
    s_direction_offset = 0.0f;
    s_direction_last_error = 0.0f;
    s_direction_yaw_rate_target = 0.0f;
    s_direction_yaw_rate_cmd = 0.0f;
    s_direction_hold_yaw = imu_get_angle_yaw();
    s_direction_last_mode = TRACK_MODE_MIDDLE;
    s_zebra_stop_latched = 0;
    s_zebra_stop_start_count = 0;
    s_run_vision_armed = 0;
    g_vision_stop_request = 0;
    g_target_distance = 0;
    g_yaw_target = imu_get_angle_yaw();
    g_vofa_mode = VOFA_RUN;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止正式跑车，目标速度归零并保持平衡制动
// 参数说明     void
// 返回参数     void
// 使用示例     control_run_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_run_stop(void)
{
    if (s_run_active)
    {
        run_hold_balance(RUN_STOP_MANUAL);
        return;
    }
    control_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询正式跑车是否在跑
// 参数说明     void
// 返回参数     uint8           1=跑车中
// 使用示例     if (control_run_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_run_running(void)
{
    // 安全闸可能已经把 start_flag 打回 STOP，这里发现并补上停车原因
    if (s_run_active && start_flag != START_BALANCE)
    {
        s_run_active = 0;
        if (s_run_stop == RUN_STOP_NONE) s_run_stop = RUN_STOP_SAFETY;
    }
    return (uint8)s_run_active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次跑车的停车原因
// 参数说明     void
// 返回参数     run_stop_t      停车原因
// 使用示例     menu_status(run_stop_text(control_run_stop_reason()));
//-------------------------------------------------------------------------------------------------------------------
run_stop_t control_run_stop_reason(void)
{
    return s_run_stop;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复制正式 Run 的全链路诊断状态，保持 1ms 中断路径定长且不做字符串格式化
// 参数说明     out             输出快照地址，允许为 0
// 返回参数     void
// 使用示例     control_run_diag_snapshot(&snapshot);
//-------------------------------------------------------------------------------------------------------------------
void control_run_diag_snapshot(volatile control_run_diag_t *out)
{
    uint16 flags = 0u;

    if (out == 0) return;
    if (s_run_active)                    flags |= 0x0001u;
    if (start_flag == START_BALANCE)     flags |= 0x0002u;
    if (g_track_valid)                   flags |= 0x0004u;
    if (s_run_vision_armed)              flags |= 0x0008u;
    if (g_vision_ipm_ok)                 flags |= 0x0010u;
    if (s_direction_last_mode == TRACK_MODE_HOLD) flags |= 0x0020u;
    if (s_zebra_stop_latched)            flags |= 0x0040u;
    flags |= (uint16)(((uint16)g_vision_active_elem & 0x000Fu) << 8);
    flags |= (uint16)(((uint16)s_run_stop & 0x0007u) << 12);

    out->uptime_ms            = g_control_uptime_ms;
    out->roll                 = att.roll;
    out->roll_target          = s_roll_angle_target;
    out->roll_rate            = att.roll_rate;
    out->recovery_feedback    = s_rcy_fb;
    out->recovery_output      = r_rcy_pid.out;
    out->roll_output          = g_pwm_roll;
    out->lean_offset          = g_lean_offset;
    out->yaw_rate_target      = s_direction_yaw_rate_target;
    out->yaw_rate_command     = s_direction_yaw_rate_cmd;
    out->yaw_rate_actual      = imu.gyro_z;
    out->yaw_output_raw       = g_pwm_yaw;
    out->yaw_output_applied   = s_yaw_output_applied;
    out->flywheel_common_rpm  = g_flywheel_common_rpm;
    out->direction_offset     = s_direction_offset;
    out->lateral_error        = g_vision_lateral_error;
    out->heading_error        = g_vision_heading_error;
    out->curvature            = g_vision_curvature;
    out->speed_plan_mps       = g_run_speed_target_mps;
    out->speed_ramp_mps       = Y_Motor_Count20msToMps(s_speed_ramp);
    out->speed_actual_mps     = Y_Motor_GetSpeedMps();
    out->momentum_scale       = g_yaw_momentum_scale;
    out->vision_quality       = g_vision_quality;
    out->vision_age_ms        = g_vision_age_ms;
    out->state_flags          = flags;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动无线 Run Test，先按三轴平衡的全部安全条件完成发车
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     if (!control_remote_start()) menu_status("RUN BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_remote_start(void)
{
    if (start_flag != START_STOP)
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }
    if (!control_balance_start()) return 0;

    s_remote_active = 1;
    s_remote_cmd_seen = 0;
    s_remote_timeout_latched = 0;
    s_remote_cmd_age_ms = 0;
    s_remote_steer_angle = 0.0f;
    s_remote_speed_mps = 0.0f;
    s_remote_yaw_target = imu_get_angle_yaw();
    g_target_distance = 0;
    g_yaw_target = s_remote_yaw_target;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止无线 Run Test 并锁停全部电机
// 参数说明     void
// 返回参数     void
// 使用示例     control_remote_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_remote_stop(void)
{
    control_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收一条无线遥控命令并刷新命令看门狗
// 参数说明     steer_angle/speed_mps 相对当前航向角度(°)与线速度(m/s)
// 返回参数     uint8           1=已接受 0=未运行或数值越界
// 使用示例     control_remote_command(30.0f, 0.20f);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_remote_command(float steer_angle, float speed_mps)
{
    uint32 interrupt_state;
    float yaw_now;

    if (!ctrl_is_finite(steer_angle) || !ctrl_is_finite(speed_mps) ||
        steer_angle < -REMOTE_STEER_INPUT_LIMIT ||
        steer_angle > REMOTE_STEER_INPUT_LIMIT ||
        speed_mps < -REMOTE_SPEED_LIMIT_MPS ||
        speed_mps > REMOTE_SPEED_LIMIT_MPS)
        return 0;

    yaw_now = imu_get_angle_yaw();
    interrupt_state = interrupt_global_disable();
    if (!s_remote_active || start_flag != START_BALANCE)
    {
        interrupt_global_enable(interrupt_state);
        return 0;
    }
    // 每条命令均以接收时航向为基准；航向和速度在下一次 1ms 控制拍同时生效。
    // 转向意图是相对行进方向说的，车掉头之后要跟着翻，见 STEER_DIR
    s_remote_yaw_target = yaw_now + (float)STEER_DIR * steer_angle;
    s_remote_steer_angle = steer_angle;
    s_remote_speed_mps = speed_mps;
    s_remote_cmd_age_ms = 0;
    s_remote_cmd_seen = 1;
    s_remote_timeout_latched = 0;
    interrupt_global_enable(interrupt_state);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询无线 Run Test 是否仍在运行
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止或被安全保护切断
// 使用示例     if (control_remote_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_remote_running(void)
{
    return (uint8)(s_remote_active && start_flag == START_BALANCE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取无线 Run Test 的最新命令状态
// 参数说明     steer_angle/speed_mps/age_ms/seen 对应输出地址，可传 0 忽略
// 返回参数     void
// 使用示例     control_remote_status(&turn, &speed, &age, &seen);
//-------------------------------------------------------------------------------------------------------------------
void control_remote_status(float *steer_angle, float *speed_mps, uint16 *age_ms, uint8 *seen)
{
    uint32 interrupt_state = interrupt_global_disable();

    if (steer_angle != 0) *steer_angle = s_remote_steer_angle;
    if (speed_mps != 0) *speed_mps = s_remote_speed_mps;
    if (age_ms != 0) *age_ms = s_remote_cmd_age_ms;
    if (seen != 0) *seen = s_remote_cmd_seen;
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在停车状态清零 C 轮累计计数，用于手推 1m 读取脉冲数
// 参数说明     void
// 返回参数     uint8           1=已清零 0=电机正在运行
// 使用示例     control_odometry_counter_reset();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_odometry_counter_reset(void)
{
    if (start_flag != START_STOP || control_test_running() ||
        control_jog_running() != MOTOR_JOG_NONE)
        return 0;

    Y_Motor_EncoderClear();
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动带三轴平衡的直行 1m 里程验证
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     if (!control_odometry_test_start()) menu_status("1M BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_odometry_test_start(void)
{
    if (start_flag != START_STOP)
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }
    if (!control_balance_start()) return 0;

    s_odom_test_start_count = Y_Motor_GetTotalCount();
    s_odom_test_yaw_zero = imu_get_angle_yaw();
    s_odom_test_state = ODOM_TEST_RUNNING;
    g_target_distance = 0;
    g_yaw_target = s_odom_test_yaw_zero;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止 1m 里程验证并锁停全部电机
// 参数说明     void
// 返回参数     void
// 使用示例     control_odometry_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_odometry_test_stop(void)
{
    control_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 1m 里程验证状态
// 参数说明     void
// 返回参数     odom_test_state_t 当前状态
// 使用示例     state = control_odometry_test_state();
//-------------------------------------------------------------------------------------------------------------------
odom_test_state_t control_odometry_test_state(void)
{
    return s_odom_test_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 1m 标定页需要的原始计数、距离和速度
// 参数说明     count/distance/speed 输出地址，可传 0 忽略
// 返回参数     void
// 使用示例     control_odometry_status(&count, &distance, &speed);
//-------------------------------------------------------------------------------------------------------------------
void control_odometry_status(int32 *count, float *distance, float *speed)
{
    int32 current = Y_Motor_GetTotalCount();

    if (s_odom_test_state != ODOM_TEST_IDLE)
        current -= s_odom_test_start_count;
    if (count != 0) *count = current;
    if (distance != 0) *distance = Y_Motor_CountToMeter(current);
    if (speed != 0) *speed = Y_Motor_GetSpeedMps();
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
    s_jog_target = MOTOR_JOG_NONE;
    s_jog_duty = 0;
    g_motor_a = 0;
    g_motor_b = 0;
    g_motor_c = 0;
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

    s_control_test_active = 1;
    s_test_status = CTRL_TEST_STATUS_OK;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止单轴 Test
// 参数说明     void
// 返回参数     void
// 使用示例     control_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_test_stop(void)
{
    if (s_test_running) test_stop();
    s_control_test_active = 0;
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
        feedback.pitch = att.pitch - g_pitch_zero;
        feedback.pitch_rate = att.pitch_rate;
        feedback.drive_speed_mps = Y_Motor_GetSpeedMps();
        feedback.drive_output = g_pwm_pitch;
        feedback.speed_cross_mps = g_param.run_speed_cross;
        feedback.speed_ring_mps = g_param.run_speed_ring;
        feedback.speed_ramp_mps = g_param.run_speed_ramp;
        feedback.elem_en_zebra = (uint8)g_param.elem_en_zebra;
        feedback.elem_en_cross = (uint8)g_param.elem_en_cross;
        feedback.elem_en_ring = (uint8)g_param.elem_en_ring;
        feedback.elem_en_ramp = (uint8)g_param.elem_en_ramp;
        feedback.run_active = (uint8)s_run_active;
        feedback.err_front_row = g_param.err_front_row;
        memcpy(feedback.ipm_h, g_param.ipm_h, sizeof(feedback.ipm_h));
        feedback.ring_angle = g_param.ring_angle;
        feedback.ring_s2_cnt_l = g_param.ring_s2_cnt_l;
        feedback.ring_s2_cnt_r = g_param.ring_s2_cnt_r;
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
        // CPU1 完成一次逆透视标定，把矩阵取回来存进运行参数
        if (result.ipm_calib_seq != s_ipm_calib_seq_last)
        {
            s_ipm_calib_seq_last = result.ipm_calib_seq;
            // 只在 CPU1 那边确实可用时才收。ipm_store() 在 ipm_ready()==0 时发回的是全 0，
            // 无条件收下就等于用一次失败的标定把 Flash 里已经存好的那份擦掉
            if (result.ipm_ok)
            {
                memcpy(g_param.ipm_h, result.ipm_h, sizeof(g_param.ipm_h));
                g_param_revision++;
                s_ipm_new = 1;              // 主循环看到这个标志去写 Flash，中断里不碰 Flash
            }
        }
        g_vision_ipm_ok = result.ipm_ok;
        g_vision_threshold = result.threshold;
        g_vision_search_stop = result.search_stop_line;
        g_vision_left_lost = result.left_lost;
        g_vision_right_lost = result.right_lost;
        g_vision_both_lost = result.both_lost;
        g_vision_active_elem = (uint8)result.active_elem;
        g_vision_island_state = result.island_state;
        g_vision_lateral_error = ctrl_is_finite(result.lateral_error)
                               ? constrain_float(result.lateral_error, -2.0f, 2.0f) : 0.0f;
        g_vision_heading_error = ctrl_is_finite(result.heading_error)
                               ? constrain_float(result.heading_error, -90.0f, 90.0f) : 0.0f;
        g_vision_curvature = ctrl_is_finite(result.curvature)
                           ? constrain_float(result.curvature, -1.0f, 1.0f) : 0.0f;
        g_vision_direction_camera = ctrl_is_finite(result.direction_camera)
                                  ? constrain_float(result.direction_camera,
                                                    -DIRECTION_CAMERA_LIMIT,
                                                     DIRECTION_CAMERA_LIMIT) : 0.0f;
        g_vision_track_mode = (result.track_mode <= TRACK_MODE_HOLD)
                            ? (uint8)result.track_mode : (uint8)TRACK_MODE_MIDDLE;
        g_vision_quality = ctrl_is_finite(result.quality)
                         ? constrain_float(result.quality, 0.0f, 1.0f) : 0.0f;
        g_vision_speed_limit_mps = ctrl_is_finite(result.speed_limit_mps)
                                 ? constrain_float(result.speed_limit_mps, 0.0f, RUN_SPEED_MAX_MPS)
                                 : 0.0f;
        if (s_run_active && result.run_active)
        {
            s_run_vision_armed = 1;
            g_vision_stop_request = result.stop_request;
        }
        else
        {
            g_vision_stop_request = 0;
        }
        g_track_valid = result.track_valid;

        // 丢线不等于居中：转向误差清零，丢线计数继续累加。
        if (result.track_valid)
        {
            g_track_lost_frames = 0;
            g_dbg_error = g_vision_lateral_error;
        }
        else
        {
            if (g_track_lost_frames < 60000u) g_track_lost_frames++;
            g_dbg_error = 0.0f;
        }
        run_vision_target_update();
    }

    // 帧率结算。本函数每 1ms 调一次，所以窗口长度就是 VISION_FPS_WIN_MS。
    // 摄像头停帧时窗口内计数为 0，读数自然掉到 0，不需要另设超时。
    fps_win_ms = g_control_uptime_ms - fps_win_start_ms;
    if (fps_win_ms >= VISION_FPS_WIN_MS)
    {
        g_vision_fps = (float)fps_frames * 1000.0f / (float)fps_win_ms;
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

    }

    motion_target_update();
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
