#include "control.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include "board_config.h"
#include "imu.h"
#include "key.h"
#include "attitude.h"
#include "balance.h"
#include "vofa.h"
#include "param.h"
#include "pid.h"
#include "vision_core.h"
#include <stddef.h>
#include <string.h>

// 直立串级最终执行器限幅，在各级 out_limit 之上再限一次，防转弯分量叠加后越界
#define CONTROL_DRIVE_OUTPUT_LIMIT      (8000.0f)   // 行进轮输出限幅
#define CONTROL_FLYWHEEL_OUTPUT_LIMIT   (8000.0f)   // 单只动量轮输出限幅
#define CONTROL_TURN_OUTPUT_LIMIT       (2000.0f)   // 转弯前馈量限幅

#define CONTROL_FALL_ANGLE_LIMIT_DEG    (30.0f)     // 防倒角硬上限，与菜单保护角取较小者
#define CONTROL_SPEED_DT_S              (0.010f)    // 速度外环差分周期(s)

float g_dbg_error   = 0.0f;                      // 中线偏差
uint8 g_imu_ok      = 0;                         // IMU 初始化状态
uint8 g_cam_ok      = 0;                         // 摄像头初始化状态
uint8 g_track_valid = 0;                         // 当前循迹帧状态
uint16 g_track_lost_frames = 0;                  // 连续无效帧计数
volatile uint16 g_vision_age_ms = 0;             // 图像帧间隔(ms)
volatile uint32 g_control_uptime_ms = 0;          // 控制运行时间(ms)
volatile uint32 g_vision_frame_seq = 0;           // 最新视觉帧序号
volatile uint32 g_vision_heartbeat = 0;           // CPU1 主循环心跳
uint16 g_vision_threshold = 0;                    // 最新大津阈值
uint16 g_vision_search_stop = 0;                  // 最新有效前瞻行数
uint16 g_vision_left_lost = 0;                    // 最新左边丢线数
uint16 g_vision_right_lost = 0;                   // 最新右边丢线数
uint16 g_vision_both_lost = 0;                    // 最新双边丢线数
// uint8  g_vision_active_elem = 0;               // 元素识别启用后恢复

static volatile uint8 s_control_test_active;
static volatile control_test_status_t s_test_status;
// static uint8 s_vision_timeout_active;          // 完整跑车流程启用后恢复
// static uint8 s_vision_recover_count;
// static float s_vision_hold_yaw;

// Motor 页架空点动，倒计时在 1ms 中断里跑，菜单卡死也会自动停
static volatile motor_jog_t s_jog_target;
static volatile int16       s_jog_duty;
static volatile uint16      s_jog_left_ms;

// 三串级初始增益，全 0 表示未整定，按角速度内环->角度中环->速度外环逐级调
// 限幅只是软件保护初值，不代表实车可以安全用到该输出
control_cascade_config_t control_fore_aft_config =            // 前后通道，行进轮
{
    { 0.0f, 0.0f, 0.0f,   10.0f,   15.0f },                         // 速度外环
    { 0.0f, 0.0f, 0.0f,  100.0f,  300.0f },                         // 角度中环
    { 0.0f, 0.0f, 0.0f, 1000.0f, CONTROL_DRIVE_OUTPUT_LIMIT }       // 角速度内环
};

control_cascade_config_t control_left_right_config =          // 左右通道，两只动量轮
{
    { 0.0f, 0.0f, 0.0f,   10.0f,   15.0f },                         // 速度外环
    { 0.0f, 0.0f, 0.0f,  100.0f,  300.0f },                         // 角度中环
    { 0.0f, 0.0f, 0.0f, 1000.0f, CONTROL_FLYWHEEL_OUTPUT_LIMIT }    // 角速度内环
};

static pid_t s_fore_aft_speed_pid;      // 前后速度外环
static pid_t s_fore_aft_angle_pid;      // 前后角度中环
static pid_t s_fore_aft_rate_pid;       // 前后角速度内环
static pid_t s_left_right_speed_pid;    // 左右速度外环
static pid_t s_left_right_angle_pid;    // 左右角度中环
static pid_t s_left_right_rate_pid;     // 左右角速度内环

static volatile uint8 s_cascade_ready;              // Control_Init 已执行
static volatile uint8 s_cascade_enabled;            // 直立闭环使能
static volatile control_fault_t s_cascade_fault;    // 当前故障原因

// 以下四项由前台写、1ms 中断读
static volatile float s_forward_speed_target;       // 前进速度目标(counts/s)
static volatile float s_lateral_speed_target;       // 横向速度目标
static volatile float s_lateral_speed_feedback;     // 横向速度反馈，无传感器时为 0
static volatile float s_turn_output_target;         // 转弯前馈量(占空比)
static volatile float s_pitch_zero;                 // 俯仰机械零点(°)
static volatile float s_roll_zero;                  // 横滚机械零点(°)

static control_actuator_sign_t s_actuator_signs;    // 执行器方向符号，默认全 0 = 未确认
static uint8  s_angle_divider;                      // 角度中环分频计数
static uint8  s_speed_divider;                      // 速度外环分频计数
static int32  s_previous_encoder_total;             // 速度外环差分基准
static volatile control_state_t s_cascade_state;    // 供前台读取的状态快照

static void Control_Init(void);                     // 定义在下方，control_init() 先用到

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
// 函数简介     分别向 W_Motor 与 Y_Motor 下发三电机控制量
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
// 函数简介     初始化参数、按键、IMU660RB、姿态解算与调试串口
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
//  g_vision_active_elem = 0;
    s_control_test_active = 0;
    s_test_status = CTRL_TEST_STATUS_OK;
//  s_vision_timeout_active = 0;
//  s_vision_recover_count = 0;
//  s_vision_hold_yaw = 0.0f;
    s_jog_target = MOTOR_JOG_NONE;
    s_jog_duty = 0;
    s_jog_left_ms = 0;

    param_init();
    key_io_init();
    W_Motor_Init();                     // 上电锁定 A/B 软件刹车并请求转速回传
    Y_Motor_Init();                     // C 轮 PWM/DIR 与 5ms 编码器
    balance_init();
    Control_Init();                     // 直立串级控制器，初始化后不使能任何输出
    vofa_init();

    g_imu_ok = (imu_init() == 0) ? 1 : 0;
    if (g_imu_ok)
    {
        imu_calibrate();
        attitude_init();
    }
    pit_ms_init(CTRL_PIT_CH, CTRL_PERIOD_MS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取姿态闭环被拒的安全原因
// 参数说明     void
// 返回参数     control_block_t 姿态或 IMU 阻断原因
// 使用示例     block = control_attitude_block_reason();
//-------------------------------------------------------------------------------------------------------------------
static control_block_t control_attitude_block_reason(void)
{
    if (!g_imu_ok)              return CTRL_BLOCK_IMU_FAIL;
    if (imu_link_lost())        return CTRL_BLOCK_IMU_LOST;
    if (attitude_diverged())    return CTRL_BLOCK_ATT_DIVERGED;
    if (!attitude_converged())  return CTRL_BLOCK_ATT_CONVERGING;
    return CTRL_BLOCK_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把阻断原因翻译成 Test/Wave 状态码
// 参数说明     block           阻断原因
// 返回参数     control_test_status_t 对应的测试状态码
// 使用示例     s_test_status = control_block_to_test_status(block);
//-------------------------------------------------------------------------------------------------------------------
static control_test_status_t control_block_to_test_status(control_block_t block)
{
    switch (block)
    {
        case CTRL_BLOCK_IMU_FAIL:     return CTRL_TEST_STATUS_IMU_FAIL;
        case CTRL_BLOCK_IMU_LOST:     return CTRL_TEST_STATUS_IMU_LOST;
        case CTRL_BLOCK_ATT_DIVERGED: return CTRL_TEST_STATUS_ATT_DIVERGED;
        case CTRL_BLOCK_BLDC_LOST:    return CTRL_TEST_STATUS_BLDC_LOST;
        default:                      return CTRL_TEST_STATUS_ATT_CONVERGING;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查是否允许离开 STOP 状态
// 参数说明     void
// 返回参数     uint8           1=允许 0=禁止
// 使用示例     if (control_allow_start()) control_start_balance();
//-------------------------------------------------------------------------------------------------------------------
// uint8 control_allow_start(void)
// {
//     return (uint8)(control_start_block_reason() == CTRL_BLOCK_NONE);
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取正常发车被拒的安全原因
// 参数说明     void
// 返回参数     control_block_t 姿态、IMU 或驱动通讯阻断原因
// 使用示例     switch (control_start_block_reason()) { ... }
//-------------------------------------------------------------------------------------------------------------------
// control_block_t control_start_block_reason(void)
// {
//     control_block_t block = control_attitude_block_reason();
//
//     if (block != CTRL_BLOCK_NONE) return block;
//     if (W_Motor_LinkLost())       return CTRL_BLOCK_BLDC_LOST;   // 发车必须先有飞轮
//     return CTRL_BLOCK_NONE;
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安全进入三轴平衡运行状态
// 参数说明     void
// 返回参数     uint8           1=已启动 0=安全条件不允许
// 使用示例     if (!control_start_balance()) display_start_blocked_screen(control_start_block_reason());
//-------------------------------------------------------------------------------------------------------------------
// uint8 control_start_balance(void)
// {
//     if (!control_allow_start()) return 0;
//     control_test_stop();
//     control_jog_stop();
//     g_target_distance = 0;
//     g_yaw_target = imu_get_angle_yaw();
//     s_vision_timeout_active = 0;
//     s_vision_recover_count = 0;
//     start_flag = START_BALANCE;
//     return 1;
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     进入完整跑赛道流程：平衡闭环 + 视觉循迹驱动转向与速度
// 参数说明     void
// 返回参数     uint8           1=已发车 0=流程未启用或安全条件不允许
// 使用示例     if (!control_start_run()) menu_status("RUN BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
// uint8 control_start_run(void)
// {
// #if RUN_FLOW_ENABLE
//     // 循迹目标由 control_vision_exchange() 在 START_BALANCE 下写入 g_yaw_target 与 g_target_distance，
//     // 因此发车流程本身只需要确认视觉在线后进入平衡态。
//     if (vision_core_state() != VISION_CORE_READY) return 0;
//     if (!control_start_balance()) return 0;
//     return 1;
// #else
//     return 0;                       // 分轴 PID 与电机方向未实车验证前不允许发车
// #endif
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止正常运行与 Test/Wave 并锁定可用电机
// 参数说明     void
// 返回参数     void
// 使用示例     control_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_stop(void)
{
    control_test_stop();
    control_jog_stop();
    if (s_cascade_enabled) Control_EmergencyStop(CONTROL_FAULT_MANUAL_STOP);
    g_target_distance = 0;
    start_flag = START_STOP;
//  s_vision_timeout_active = 0;
//  s_vision_recover_count = 0;
    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动一次限时架空点动，用于确认电机转向与转速反馈符号
// 参数说明     target/forward  点动电机与方向, forward 为 1 表示正转
// 返回参数     uint8           1=已启动 0=正在跑其它闭环或安全条件不允许
// 使用示例     control_jog_start(MOTOR_JOG_A, 1);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_jog_start(motor_jog_t target, uint8 forward)
{
    int16 duty;

    if (target == MOTOR_JOG_NONE) return 0;
    if (start_flag != START_STOP || balance_test_running() || s_cascade_enabled) return 0;
    if (target != MOTOR_JOG_C && W_Motor_LinkLost())
    {
        s_test_status = CTRL_TEST_STATUS_BLDC_LOST;
        return 0;
    }

    duty = (int16)((target == MOTOR_JOG_C) ? MOTOR_JOG_DRIVE_DUTY : MOTOR_JOG_FLY_DUTY);
    if (!forward) duty = (int16)(-duty);

    if (target != MOTOR_JOG_C) W_Motor_Release();
    s_jog_duty = duty;
    s_jog_left_ms = MOTOR_JOG_MS;
    s_jog_target = target;              // 最后赋值: 1ms 中断读到 target 时其余字段已就绪
    s_test_status = CTRL_TEST_STATUS_OK;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即结束点动并重新锁死动量轮
// 参数说明     void
// 返回参数     void
// 使用示例     control_jog_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_jog_stop(void)
{
    s_jog_target = MOTOR_JOG_NONE;
    s_jog_left_ms = 0;
    s_jog_duty = 0;
    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询当前正在点动的电机
// 参数说明     void
// 返回参数     motor_jog_t     点动目标, MOTOR_JOG_NONE 表示未运行
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
    if (s_jog_left_ms == 0u || (target != MOTOR_JOG_C && W_Motor_LinkLost()))
    {
        control_jog_stop();
        return;
    }
    s_jog_left_ms--;

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
// 函数简介     判断浮点是否为有限值(非 NaN / 非 ±Inf)
// 参数说明     v               待判值
// 返回参数     uint8           1=有限值 0=NaN/Inf
// 使用示例     if (!control_is_finite(att.roll)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 control_is_finite(float v)
{
    if (v != v) return 0;                                   // NaN
    if (v > 3.0e38f || v < -3.0e38f) return 0;              // ±Inf
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取浮点绝对值，不引入数学库
// 参数说明     v               输入值
// 返回参数     float           绝对值
// 使用示例     if (control_fabs(err) > limit) { ... }
//-------------------------------------------------------------------------------------------------------------------
static float control_fabs(float v)
{
    return (v < 0.0f) ? -v : v;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算一级位置式环并按本级限幅截断
// 参数说明     p/cfg           PID 实例与增益，measurement/target 为反馈与目标
// 返回参数     float           限幅后的本级输出
// 使用示例     out = control_pos_stage(&s_fore_aft_angle_pid, &cfg->angle, pitch, target);
//-------------------------------------------------------------------------------------------------------------------
static float control_pos_stage(pid_t *p, const control_pid_config_t *cfg,
                               float measurement, float target)
{
    p->out = constrain_float(pid_loc_calc(p, target - measurement),
                             -cfg->out_limit, cfg->out_limit);
    return p->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算角速度增量式环，累积输出限幅防饱和
// 参数说明     p/cfg           PID 实例与增益，measurement/target 为反馈与目标
// 返回参数     float           限幅后的累积输出，不是单次增量
// 使用示例     out = control_rate_stage(&s_fore_aft_rate_pid, &cfg->angular_rate, rate, target);
//-------------------------------------------------------------------------------------------------------------------
static float control_rate_stage(pid_t *p, const control_pid_config_t *cfg,
                                float measurement, float target)
{
    (void)pid_inc_calc(p, target - measurement);
    p->out = constrain_float(p->out, -cfg->out_limit, cfg->out_limit);
    return p->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点控制量限幅并截断为整数占空比
// 参数说明     value/limit     控制量与限幅值
// 返回参数     int32           限幅后的整数占空比
// 使用示例     drive = control_to_output(out, CONTROL_DRIVE_OUTPUT_LIMIT);
//-------------------------------------------------------------------------------------------------------------------
static int32 control_to_output(float value, float limit)
{
    return (int32)constrain_float(value, -limit, limit);    // 向零截断，不足 1 个单位自然归零
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     校验单个执行器方向符号
// 参数说明     sign            待校验符号
// 返回参数     uint8           1=合法(±1) 0=非法
// 使用示例     if (!control_sign_valid(signs->drive_motor_sign)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 control_sign_valid(int8 sign)
{
    return (uint8)((sign == 1) || (sign == -1));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     校验五个执行器方向符号是否都已实测确认
// 参数说明     void
// 返回参数     uint8           1=全部合法 0=存在未确认项
// 使用示例     if (!control_signs_ready()) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 control_signs_ready(void)
{
    return (uint8)(control_sign_valid(s_actuator_signs.drive_motor_sign) &&
                   control_sign_valid(s_actuator_signs.flywheel_balance_sign_1) &&
                   control_sign_valid(s_actuator_signs.flywheel_balance_sign_2) &&
                   control_sign_valid(s_actuator_signs.flywheel_turn_sign_1) &&
                   control_sign_valid(s_actuator_signs.flywheel_turn_sign_2));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查俯仰与横滚是否都在防倒角以内
// 参数说明     pitch/roll      当前俯仰角与横滚角(°)
// 返回参数     uint8           1=安全 0=超过防倒角
// 使用示例     if (!control_angle_safe(att.pitch, att.roll)) Control_EmergencyStop(...);
//-------------------------------------------------------------------------------------------------------------------
static uint8 control_angle_safe(float pitch, float roll)
{
    float pitch_limit = (PITCH_PROTECT_ANGLE < CONTROL_FALL_ANGLE_LIMIT_DEG)
                      ? PITCH_PROTECT_ANGLE : CONTROL_FALL_ANGLE_LIMIT_DEG;
    float roll_limit  = (ROLL_PROTECT_ANGLE  < CONTROL_FALL_ANGLE_LIMIT_DEG)
                      ? ROLL_PROTECT_ANGLE  : CONTROL_FALL_ANGLE_LIMIT_DEG;

    return (uint8)((control_fabs(pitch - s_pitch_zero) <= pitch_limit) &&
                   (control_fabs(roll  - s_roll_zero)  <= roll_limit));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空六个 PID 的历史状态与分频计数
// 参数说明     void
// 返回参数     void
// 使用示例     control_cascade_clear();
//-------------------------------------------------------------------------------------------------------------------
static void control_cascade_clear(void)
{
    pid_reset(&s_fore_aft_speed_pid);
    pid_reset(&s_fore_aft_angle_pid);
    pid_reset(&s_fore_aft_rate_pid);
    pid_reset(&s_left_right_speed_pid);
    pid_reset(&s_left_right_angle_pid);
    pid_reset(&s_left_right_rate_pid);
    s_angle_divider = 0;
    s_speed_divider = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把公开增益结构装载进六个 PID 实例
// 参数说明     void
// 返回参数     void
// 使用示例     control_cascade_load();
//-------------------------------------------------------------------------------------------------------------------
static void control_cascade_load(void)
{
    pid_set(&s_fore_aft_speed_pid,   control_fore_aft_config.speed.kp,
            control_fore_aft_config.speed.ki, control_fore_aft_config.speed.kd,
            control_fore_aft_config.speed.imax);
    pid_set(&s_fore_aft_angle_pid,   control_fore_aft_config.angle.kp,
            control_fore_aft_config.angle.ki, control_fore_aft_config.angle.kd,
            control_fore_aft_config.angle.imax);
    pid_set(&s_fore_aft_rate_pid,    control_fore_aft_config.angular_rate.kp,
            control_fore_aft_config.angular_rate.ki, control_fore_aft_config.angular_rate.kd,
            control_fore_aft_config.angular_rate.imax);

    pid_set(&s_left_right_speed_pid, control_left_right_config.speed.kp,
            control_left_right_config.speed.ki, control_left_right_config.speed.kd,
            control_left_right_config.speed.imax);
    pid_set(&s_left_right_angle_pid, control_left_right_config.angle.kp,
            control_left_right_config.angle.ki, control_left_right_config.angle.kd,
            control_left_right_config.angle.imax);
    pid_set(&s_left_right_rate_pid,  control_left_right_config.angular_rate.kp,
            control_left_right_config.angular_rate.ki, control_left_right_config.angular_rate.kd,
            control_left_right_config.angular_rate.imax);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化直立串级控制器，不使能任何电机输出
// 参数说明     void
// 返回参数     void
// 使用示例     Control_Init();   // control_init() 内调用
//-------------------------------------------------------------------------------------------------------------------
static void Control_Init(void)
{
    y_motor_encoder_data_t encoder;

    memset((void *)&s_cascade_state, 0, sizeof(control_state_t));
    memset(&s_actuator_signs, 0, sizeof(control_actuator_sign_t));

    s_cascade_ready = 1;
    s_cascade_enabled = 0;
    s_cascade_fault = CONTROL_FAULT_MANUAL_STOP;

    s_forward_speed_target = 0.0f;
    s_lateral_speed_target = 0.0f;
    s_lateral_speed_feedback = 0.0f;
    s_turn_output_target = 0.0f;
    s_pitch_zero = PITCH_ZERO_INIT;                 // 与菜单 Zero 页同源
    s_roll_zero = ROLL_ZERO_INIT;

    control_cascade_load();
    control_cascade_clear();

    Y_Motor_GetEncoder(&encoder);
    s_previous_encoder_total = encoder.total_count;

    s_cascade_state.initialized = 1;
    s_cascade_state.fault = s_cascade_fault;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在控制器失能时把公开增益装载进 PID 实例
// 参数说明     void
// 返回参数     uint8           1=已装载 0=控制器正在运行，拒绝更新
// 使用示例     if (!Control_ApplyPidConfig()) menu_status("DISABLE FIRST");
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_ApplyPidConfig(void)
{
    uint32 interrupt_state;

    if (!s_cascade_ready || s_cascade_enabled) return 0;

    // 增益被菜单逐字段改，装载时关中断防止读到一半新一半旧
    interrupt_state = interrupt_global_disable();
    control_cascade_load();
    interrupt_global_enable(interrupt_state);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置前进速度、横向速度与转弯三个运动目标
// 参数说明     forward_speed_counts_per_s/lateral_speed/turn_output 三个运动目标
// 返回参数     void
// 使用示例     Control_SetMotionTarget(0.0f, 0.0f, 0.0f);
//-------------------------------------------------------------------------------------------------------------------
void Control_SetMotionTarget(float forward_speed_counts_per_s,
                             float lateral_speed,
                             float turn_output)
{
    uint32 interrupt_state = interrupt_global_disable();

    s_forward_speed_target = forward_speed_counts_per_s;
    s_lateral_speed_target = lateral_speed;
    s_turn_output_target = constrain_float(turn_output,
                                           -CONTROL_TURN_OUTPUT_LIMIT,
                                           CONTROL_TURN_OUTPUT_LIMIT);
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入左右通道速度反馈，未写入时保持 0
// 参数说明     lateral_speed   横向速度反馈
// 返回参数     void
// 使用示例     Control_SetLateralSpeedFeedback(0.0f);
//-------------------------------------------------------------------------------------------------------------------
void Control_SetLateralSpeedFeedback(float lateral_speed)
{
    s_lateral_speed_feedback = lateral_speed;       // 单个 32 位标量，写入本身原子
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置直立机械零点，只允许在失能状态下修改
// 参数说明     pitch_zero_deg/roll_zero_deg 俯仰与横滚机械零点(°)
// 返回参数     void
// 使用示例     Control_SetBalanceZero(PITCH_ZERO_INIT, ROLL_ZERO_INIT);
//-------------------------------------------------------------------------------------------------------------------
void Control_SetBalanceZero(float pitch_zero_deg, float roll_zero_deg)
{
    uint32 interrupt_state;

    if (s_cascade_enabled) return;                  // 运行中改零点会产生目标阶跃

    interrupt_state = interrupt_global_disable();
    s_pitch_zero = pitch_zero_deg;
    s_roll_zero = roll_zero_deg;
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置五个执行器方向符号，必须全部为 ±1
// 参数说明     signs           符号结构指针
// 返回参数     uint8           1=已生效 0=有非法符号或控制器正在运行
// 使用示例     Control_SetActuatorSigns(&signs);
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_SetActuatorSigns(const control_actuator_sign_t *signs)
{
    uint32 interrupt_state;

    if (signs == NULL || s_cascade_enabled) return 0;
    if (!control_sign_valid(signs->drive_motor_sign) ||
        !control_sign_valid(signs->flywheel_balance_sign_1) ||
        !control_sign_valid(signs->flywheel_balance_sign_2) ||
        !control_sign_valid(signs->flywheel_turn_sign_1) ||
        !control_sign_valid(signs->flywheel_turn_sign_2)) return 0;

    interrupt_state = interrupt_global_disable();
    s_actuator_signs = *signs;
    interrupt_global_enable(interrupt_state);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停止：清零三电机输出并记录故障原因
// 参数说明     fault           故障原因
// 返回参数     void
// 使用示例     Control_EmergencyStop(CONTROL_FAULT_FALL_ANGLE);
//-------------------------------------------------------------------------------------------------------------------
void Control_EmergencyStop(control_fault_t fault)
{
    uint32 interrupt_state = interrupt_global_disable();

    s_cascade_enabled = 0;
    s_cascade_fault = fault;
    control_cascade_clear();

    s_cascade_state.enabled = 0;
    s_cascade_state.fault = fault;
    s_cascade_state.drive_balance_output = 0.0f;
    s_cascade_state.flywheel_balance_output = 0.0f;
    s_cascade_state.drive_output = 0;
    s_cascade_state.flywheel_output_1 = 0;
    s_cascade_state.flywheel_output_2 = 0;
    interrupt_global_enable(interrupt_state);

    control_motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     正常失能：清零三电机输出并清空 PID 历史
// 参数说明     void
// 返回参数     void
// 使用示例     Control_Disable();
//-------------------------------------------------------------------------------------------------------------------
void Control_Disable(void)
{
    Control_EmergencyStop(CONTROL_FAULT_MANUAL_STOP);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空 PID 历史与编码器基准后使能直立控制
// 参数说明     void
// 返回参数     uint8           1=已使能 0=安全条件不满足，原因见 Control_GetState()
// 使用示例     if (!Control_Enable()) menu_status("BALANCE BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_Enable(void)
{
    y_motor_encoder_data_t encoder;
    uint32 interrupt_state;

    if (!s_cascade_ready) return 0;
    if (balance_test_running() || s_jog_target != MOTOR_JOG_NONE) return 0;

    if (!control_signs_ready())
    {
        Control_EmergencyStop(CONTROL_FAULT_NOT_CONFIGURED);
        return 0;
    }
    if (control_attitude_block_reason() != CTRL_BLOCK_NONE ||
        !control_is_finite(att.pitch) || !control_is_finite(att.roll) ||
        !control_is_finite(att.pitch_rate) || !control_is_finite(att.roll_rate))
    {
        Control_EmergencyStop(CONTROL_FAULT_ATTITUDE_INVALID);
        return 0;
    }
    if (W_Motor_LinkLost())
    {
        Control_EmergencyStop(CONTROL_FAULT_BLDC_LINK);
        return 0;
    }
    if (!control_angle_safe(att.pitch, att.roll))
    {
        Control_EmergencyStop(CONTROL_FAULT_FALL_ANGLE);
        return 0;
    }

    Y_Motor_GetEncoder(&encoder);

    interrupt_state = interrupt_global_disable();
    control_cascade_clear();
    s_previous_encoder_total = encoder.total_count;

    // 速度外环首次执行要等 10ms，先把目标放在机械零点，避免中环用到未初始化值
    s_cascade_state.forward_speed_measurement = 0.0f;
    s_cascade_state.lateral_speed_measurement = s_lateral_speed_feedback;
    s_cascade_state.pitch_target = s_pitch_zero;
    s_cascade_state.roll_target = s_roll_zero;
    s_cascade_state.pitch_rate_target = 0.0f;
    s_cascade_state.roll_rate_target = 0.0f;

    s_cascade_fault = CONTROL_FAULT_NONE;
    s_cascade_enabled = 1;
    s_cascade_state.enabled = 1;
    s_cascade_state.fault = CONTROL_FAULT_NONE;
    interrupt_global_enable(interrupt_state);

    W_Motor_Release();                              // 解除软件刹车闩
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询直立控制器是否处于使能状态
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已失能
// 使用示例     if (Control_IsEnabled()) menu_status("BALANCING");
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_IsEnabled(void)
{
    return s_cascade_enabled;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一拍直立串级控制并下发三电机，由 1ms 控制中断调用
// 参数说明     void
// 返回参数     void
// 使用示例     control_cascade_run();
//-------------------------------------------------------------------------------------------------------------------
static void control_cascade_run(void)
{
    float pitch = att.pitch;
    float roll = att.roll;
    float pitch_rate = att.pitch_rate;
    float roll_rate = att.roll_rate;
    float drive_balance;
    float flywheel_balance;
    float turn;
    int32 drive_out;
    int32 fly_out_1;
    int32 fly_out_2;

    // NaN 判据必须排在防倒角之前，fabsf(NaN) > 阈值 恒假，角度判据拦不住 NaN
    if (!control_is_finite(pitch) || !control_is_finite(roll) ||
        !control_is_finite(pitch_rate) || !control_is_finite(roll_rate) ||
        control_attitude_block_reason() != CTRL_BLOCK_NONE)
    {
        Control_EmergencyStop(CONTROL_FAULT_ATTITUDE_INVALID);
        return;
    }
    if (W_Motor_LinkLost())
    {
        Control_EmergencyStop(CONTROL_FAULT_BLDC_LINK);
        return;
    }
    if (!control_angle_safe(pitch, roll))
    {
        Control_EmergencyStop(CONTROL_FAULT_FALL_ANGLE);
        return;
    }

    // 速度外环，输出叠加到机械零点得到角度目标
    // 前进速度取累计编码器的 10ms 差分，不用单次 5ms 采样
    s_speed_divider++;
    if (s_speed_divider >= (CONTROL_SPEED_PERIOD_MS / CONTROL_RATE_PERIOD_MS))
    {
        y_motor_encoder_data_t encoder;
        int32 delta;

        s_speed_divider = 0;
        Y_Motor_GetEncoder(&encoder);
        delta = encoder.total_count - s_previous_encoder_total;
        s_previous_encoder_total = encoder.total_count;

        s_cascade_state.forward_speed_measurement = (float)delta / CONTROL_SPEED_DT_S;
        s_cascade_state.lateral_speed_measurement = s_lateral_speed_feedback;

        s_cascade_state.pitch_target = s_pitch_zero +
            control_pos_stage(&s_fore_aft_speed_pid, &control_fore_aft_config.speed,
                              s_cascade_state.forward_speed_measurement,
                              s_forward_speed_target);
        s_cascade_state.roll_target = s_roll_zero +
            control_pos_stage(&s_left_right_speed_pid, &control_left_right_config.speed,
                              s_cascade_state.lateral_speed_measurement,
                              s_lateral_speed_target);
        s_cascade_state.speed_update_count++;
    }

    // 角度中环，输出角速度目标
    s_angle_divider++;
    if (s_angle_divider >= (CONTROL_ANGLE_PERIOD_MS / CONTROL_RATE_PERIOD_MS))
    {
        s_angle_divider = 0;
        s_cascade_state.pitch_rate_target =
            control_pos_stage(&s_fore_aft_angle_pid, &control_fore_aft_config.angle,
                              pitch, s_cascade_state.pitch_target);
        s_cascade_state.roll_rate_target =
            control_pos_stage(&s_left_right_angle_pid, &control_left_right_config.angle,
                              roll, s_cascade_state.roll_target);
        s_cascade_state.angle_update_count++;
    }

    // 角速度内环，增量式返回值已是累计总输出，外部不许再累加
    drive_balance = control_rate_stage(&s_fore_aft_rate_pid,
                                       &control_fore_aft_config.angular_rate,
                                       pitch_rate, s_cascade_state.pitch_rate_target);
    flywheel_balance = control_rate_stage(&s_left_right_rate_pid,
                                          &control_left_right_config.angular_rate,
                                          roll_rate, s_cascade_state.roll_rate_target);
    turn = constrain_float(s_turn_output_target,
                           -CONTROL_TURN_OUTPUT_LIMIT, CONTROL_TURN_OUTPUT_LIMIT);

    // 混控，行进轮只承担前后平衡，两只动量轮叠加左右平衡与转弯分量
    // 不写死同向/反向，由五个实测符号分配
    drive_out = control_to_output((float)s_actuator_signs.drive_motor_sign * drive_balance,
                                  CONTROL_DRIVE_OUTPUT_LIMIT);
    fly_out_1 = control_to_output((float)s_actuator_signs.flywheel_balance_sign_1 * flywheel_balance +
                                  (float)s_actuator_signs.flywheel_turn_sign_1 * turn,
                                  CONTROL_FLYWHEEL_OUTPUT_LIMIT);
    fly_out_2 = control_to_output((float)s_actuator_signs.flywheel_balance_sign_2 * flywheel_balance +
                                  (float)s_actuator_signs.flywheel_turn_sign_2 * turn,
                                  CONTROL_FLYWHEEL_OUTPUT_LIMIT);

    // 三环算完再统一下发
    Y_Motor_SetDuty(drive_out);
    W_Motor_SetDuty(fly_out_1, fly_out_2);

    g_motor_a = (int16)fly_out_1;       // 复用现有波形与菜单显示通道
    g_motor_b = (int16)fly_out_2;
    g_motor_c = (int16)drive_out;

    s_cascade_state.forward_speed_target = s_forward_speed_target;
    s_cascade_state.lateral_speed_target = s_lateral_speed_target;
    s_cascade_state.turn_output_target = turn;
    s_cascade_state.pitch_measurement = pitch;
    s_cascade_state.roll_measurement = roll;
    s_cascade_state.pitch_rate_measurement = pitch_rate;
    s_cascade_state.roll_rate_measurement = roll_rate;
    s_cascade_state.drive_balance_output = drive_balance;
    s_cascade_state.flywheel_balance_output = flywheel_balance;
    s_cascade_state.drive_output = drive_out;
    s_cascade_state.flywheel_output_1 = fly_out_1;
    s_cascade_state.flywheel_output_2 = fly_out_2;
    s_cascade_state.rate_update_count++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原子读取完整控制状态快照
// 参数说明     state           状态输出地址
// 返回参数     void
// 使用示例     Control_GetState(&state);
//-------------------------------------------------------------------------------------------------------------------
void Control_GetState(control_state_t *state)
{
    uint32 interrupt_state;

    if (state == NULL) return;

    // 快照是多字结构，复制期间关中断，避免前台读到两个周期拼出来的状态
    interrupt_state = interrupt_global_disable();
    memcpy(state, (const void *)&s_cascade_state, sizeof(control_state_t));
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴和级联层级的 Test/Wave
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=启动成功 0=启动被安全条件阻止
// 使用示例     control_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_start(tune_axis_t axis, tune_ring_t ring)
{
    control_block_t block;

    control_test_stop();
    control_jog_stop();
    if (s_cascade_enabled) Control_EmergencyStop(CONTROL_FAULT_MANUAL_STOP);
    if (axis >= TUNE_AXIS_MAX || ring >= TUNE_RING_MAX ||
        (axis == TUNE_AXIS_YAW && ring == TUNE_RING_VEL))
    {
        s_test_status = CTRL_TEST_STATUS_INVALID;
        return 0;
    }

    // Pitch 只驱动 DRV8701E 的 C 轮, 不要求无刷驱动在线; Roll/Yaw 动飞轮则必须在线。
    block = control_attitude_block_reason();
    if (block == CTRL_BLOCK_NONE && axis != TUNE_AXIS_PITCH && W_Motor_LinkLost())
        block = CTRL_BLOCK_BLDC_LOST;
    if (block != CTRL_BLOCK_NONE)
    {
        s_test_status = control_block_to_test_status(block);
        return 0;
    }

    if (!balance_test_start(axis, ring))
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
// 函数简介     立即停止 Test/Wave 并清除控制状态
// 参数说明     void
// 返回参数     void
// 使用示例     control_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_test_stop(void)
{
    if (balance_test_running()) balance_test_stop();
    s_control_test_active = 0;
    g_vofa_mode = VOFA_OFF;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 Test/Wave 是否正在运行
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (control_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_running(void)
{
    if (s_control_test_active && !balance_test_running())
    {
        s_control_test_active = 0;
        s_test_status = CTRL_TEST_STATUS_SAFETY;
    }
    return s_control_test_active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次 Test/Wave 启动结果
// 参数说明     void
// 返回参数     control_test_status_t 启动结果或阻断原因
// 使用示例     status = control_test_last_status();
//-------------------------------------------------------------------------------------------------------------------
control_test_status_t control_test_last_status(void)
{
    (void)control_test_running();
    return s_test_status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布普通图像参数并接收 CPU1 图像调试结果
// 参数说明     publish_feedback 1=本拍发布反馈 0=仅接收结果
// 返回参数     void
// 使用示例     control_vision_exchange(run5);
//-------------------------------------------------------------------------------------------------------------------
static void control_vision_exchange(uint8 publish_feedback)
{
    static uint32 input_seq;
    static uint32 last_frame_seq;
    vision_feedback_t feedback;
    vision_result_t result;

    if (publish_feedback)
    {
        feedback.input_seq = ++input_seq;
        feedback.uptime_ms = g_control_uptime_ms;
        feedback.param_revision = g_param_revision;
//      feedback.drive_count_total = Y_Motor_GetTotalCount();
//      feedback.element_yaw = imu_get_angle_element();
//      feedback.pitch = att.pitch;
//      feedback.pitch_rate = att.pitch_rate;
        feedback.err_offset = g_param.err_offset;
//      feedback.speed_ramp_gain = g_param.speed_ramp_gain;
//      feedback.speed_ring_gain = g_param.speed_ring_gain;
//      feedback.obs_narrow_ratio = g_param.obs_narrow_ratio;
//      feedback.zebra_jump_cnt = g_param.zebra_jump_cnt;
//      feedback.cross_lost_cnt = g_param.cross_lost_cnt;
//      feedback.ring_angle = g_param.ring_angle;
//      feedback.ring_s2_cnt_l = g_param.ring_s2_cnt_l;
//      feedback.ring_s2_cnt_r = g_param.ring_s2_cnt_r;
//      feedback.ring_side_offset = g_param.ring_side_offset;
//      feedback.obs_line_offset = g_param.obs_line_offset;
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
        g_vision_threshold = result.threshold;
        g_vision_search_stop = result.search_stop_line;
        g_vision_left_lost = result.left_lost;
        g_vision_right_lost = result.right_lost;
        g_vision_both_lost = result.both_lost;
//      g_vision_active_elem = (uint8)result.active_elem;
        g_track_valid = result.track_valid;

        if (result.track_valid)
        {
            g_track_lost_frames = 0;
            g_dbg_error = result.track_error;
//          视觉驱动 Yaw、速度及失联恢复逻辑在完整跑车阶段恢复。
        }
        else
        {
//          s_vision_recover_count = 0;
            if (g_track_lost_frames < 60000u) g_track_lost_frames++;
            g_dbg_error = 0.0f;
//          丢线停车与航向保持在完整跑车阶段恢复。
        }
    }

//  if (start_flag != START_BALANCE) { ... }
//  else if (g_vision_age_ms >= VISION_TIMEOUT_MS) { ... }
//  视觉看门狗只属于完整跑车流程，当前不参与电机控制。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行 1ms 姿态更新、Test/Wave 控制与波形快照
// 参数说明     void
// 返回参数     void
// 使用示例     control_loop();   // 在 isr.c 的 cc60_pit_ch0_isr 内调用(1ms)
//-------------------------------------------------------------------------------------------------------------------
void control_loop(void)
{
    static uint8 key_tick;
    static uint8 tick;                      // 5ms 分频计数

    g_control_uptime_ms++;
    if (g_vision_age_ms < 0xFFFFu) g_vision_age_ms++;
    W_Motor_Tick1ms();                  // 刷新无刷驱动通讯超时并按需补发转速请求
    key_tick++;
    if (key_tick >= CTRL_DIV_KEY)
    {
        key_tick = 0;
        key_scanner();
    }
    tick++;
    if (tick >= CTRL_DIV_ATT) tick = 0;
    if (tick == 0)
        Y_Motor_EncoderUpdate5ms();      // C 轮编码器唯一硬件读取点

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

    if (balance_test_running())
    {
        balance_test_run();
        if (!balance_test_running())
        {
            s_control_test_active = 0;
            s_test_status = CTRL_TEST_STATUS_SAFETY;
        }
    }
    else if (s_jog_target != MOTOR_JOG_NONE)
    {
        control_jog_run();
    }
    else if (s_cascade_enabled)
    {
        control_cascade_run();          // 直立串级闭环, 与下面的 balance_run 互斥
    }
    else
    {
        balance_run();
    }
    vofa_snapshot();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 摄像头状态并在失败时请求重新初始化
// 参数说明     void
// 返回参数     uint8           1=初始化成功 0=初始化失败
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取行进轮 C 的 20ms 增量速度
// 参数说明     void
// 返回参数     int             行进轮 C 速度(counts/20ms)
// 使用示例     g_island.state2_count += control_get_enc_speed();
//-------------------------------------------------------------------------------------------------------------------
// int control_get_enc_speed(void)
// {
//     return (int)Y_Motor_GetSpeed20ms();
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 是否发布了新的视觉调试帧
// 参数说明     void
// 返回参数     uint8           1=处理了新帧 0=无新帧
// 使用示例     if (control_vision_debug()) display_track_view();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_vision_debug(void)
{
    static uint32 last_frame_seq;

    if (g_vision_frame_seq == 0u || g_vision_frame_seq == last_frame_seq) return 0;
    last_frame_seq = g_vision_frame_seq;
    return 1;
}
