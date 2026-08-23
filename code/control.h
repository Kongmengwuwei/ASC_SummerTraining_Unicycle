#ifndef CONTROL_H_
#define CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "vofa.h"

// 发车状态，上电默认 START_STOP
typedef enum {
    START_STOP = 0,     // 三电机零输出，A/B 刹车锁死
    START_DRIVE_ONLY,   // 只跑 C 行进轮
    START_BALANCE,      // 三轴闭环，A/B 松刹车
} start_state_t;

extern start_state_t start_flag;

extern float g_roll_zero, g_pitch_zero;              // 机械零点(°)，由 Zero 页手动调整
extern float g_lean_offset;                          // 压弯动态零点偏移(°)，叠加到横滚零点
extern float g_pwm_roll, g_pwm_pitch, g_pwm_yaw;     // 三轴串级输出，混控前
extern int16 g_motor_a, g_motor_b, g_motor_c;        // 混控后的三电机控制量
extern int   g_target_distance;                      // Pitch 速度环目标(counts/20ms)
extern float g_yaw_target;                           // 转向外环目标航向(°)
extern float g_flywheel_common_rpm;                  // (A+B)/2共模RPM，用于Yaw动量预算
extern float g_yaw_momentum_scale;                   // 共模动量剩余权限(0..1)

extern float  g_dbg_error;                  // 中线偏差，右偏为正
extern uint8  g_imu_ok;                     // IMU660RB 初始化结果
extern uint8  g_cam_ok;                     // CPU1 摄像头就绪标志
extern uint8  g_vision_ipm_ok;               // CPU1 侧逆透视是否可用
extern uint8  g_track_valid;                // 最新一帧循迹是否有效
extern uint16 g_track_lost_frames;          // 连续无效帧计数
extern volatile uint16 g_vision_age_ms;     // 距上一帧视觉结果的时间(ms)
extern volatile uint32 g_control_uptime_ms; // 1ms 中断累计运行时间(ms)
extern volatile uint32 g_vision_frame_seq;  // 最新视觉帧序号
extern volatile uint32 g_vision_heartbeat;  // CPU1 主循环心跳
extern uint16 g_vision_threshold;           // 最新大津阈值
extern uint16 g_vision_search_stop;         // 最新有效前瞻行数
extern uint16 g_vision_left_lost;           // 最新左边线丢线行数
extern uint16 g_vision_right_lost;          // 最新右边线丢线行数
extern uint16 g_vision_both_lost;           // 最新双边丢线行数
extern uint8  g_vision_active_elem;          // 最新元素编号
extern uint8  g_vision_island_state;         // 最新环岛状态号，0=空闲
extern float  g_vision_lateral_error;        // 车道半宽归一化横向误差
extern float  g_vision_heading_error;        // 赛道航向误差(°)
extern float  g_vision_curvature;            // 有符号归一化曲率
extern float  g_vision_direction_camera;     // 报告同款加权方向偏差
extern uint8  g_vision_track_mode;           // 中线、单边或航向保持模式
extern float  g_vision_quality;              // 循迹质量(0..1)
extern float  g_vision_speed_limit_mps;      // 视觉/元素绝对限速(m/s)
extern uint8  g_vision_stop_request;         // 斑马线终点请求
extern float  g_run_speed_target_mps;        // 正式 Run 当前规划速度(m/s)
extern float  g_vision_fps;                  // CPU1 出帧率(帧/s)，1ms 中断写，菜单与图像页读

typedef enum
{
    RUN_STOP_NONE = 0,          // 还没停过
    RUN_STOP_ZEBRA,             // 过斑马线正常终点停车
    RUN_STOP_LOST,              // 连续丢线超时
    RUN_STOP_VISION,            // CPU1 停帧
    RUN_STOP_MANUAL,            // 人为停车
    RUN_STOP_SAFETY,            // 姿态、IMU、驱动或超速保护
} run_stop_t;

// 正式 Run 诊断快照。由 CPU0 1ms 控制中断填写，VOFA 前台只读取完整快照并格式化。
// state_flags 位定义见“调参命令.md”的 run: 通道表。
typedef struct
{
    uint32 uptime_ms;
    float  roll;
    float  roll_target;
    float  roll_rate;
    float  recovery_feedback;
    float  recovery_output;
    float  roll_output;
    float  lean_offset;
    float  yaw_rate_target;
    float  yaw_rate_command;
    float  yaw_rate_actual;
    float  yaw_output_raw;
    float  yaw_output_applied;
    float  flywheel_common_rpm;
    float  direction_offset;
    float  lateral_error;
    float  heading_error;
    float  curvature;
    float  speed_plan_mps;
    float  speed_ramp_mps;
    float  speed_actual_mps;
    float  momentum_scale;
    float  vision_quality;
    uint16 vision_age_ms;
    uint16 state_flags;
} control_run_diag_t;

typedef enum
{
    CTRL_TEST_STATUS_OK = 0,        // 已启动
    CTRL_TEST_STATUS_INVALID,       // 轴或环组合非法
    CTRL_TEST_STATUS_IMU_FAIL,      // IMU 初始化失败
    CTRL_TEST_STATUS_IMU_CALIB,     // IMU 静止标定无效
    CTRL_TEST_STATUS_ATT_CONVERGING,// 姿态解算尚未收敛
    CTRL_TEST_STATUS_ATT_DIVERGED,  // 四元数发散
    CTRL_TEST_STATUS_IMU_LOST,      // IMU 链路中断
    CTRL_TEST_STATUS_BLDC_LOST,     // CYT2BL3 通信中断
    CTRL_TEST_STATUS_ROLL_PROT,     // 横滚超保护角
    CTRL_TEST_STATUS_PITCH_PROT,    // 俯仰超保护角
    CTRL_TEST_STATUS_FLY_OVERSPEED, // 动量轮转速超上限，抢在驱动堵转保护之前停
    CTRL_TEST_STATUS_OUTPUT_INVALID,// PID 输出出现 NaN 或无穷
    CTRL_TEST_STATUS_SAFETY,        // 运行中被安全闸停掉
} control_test_status_t;

// 架空点动目标，用于确认电机转向与转速回读符号
typedef enum
{
    MOTOR_JOG_NONE = 0,     // 未点动
    MOTOR_JOG_A,            // 动量轮 A
    MOTOR_JOG_B,            // 动量轮 B
    MOTOR_JOG_C,            // 行进轮 C
} motor_jog_t;

// 1m 里程验证状态
typedef enum
{
    ODOM_TEST_IDLE = 0,
    ODOM_TEST_RUNNING,
    ODOM_TEST_DONE,
} odom_test_state_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化参数、按键、电机、控制、无线串口和 IMU660RB，最后开 1ms 中断
// 参数说明     void
// 返回参数     void
// 使用示例     control_init();
//-------------------------------------------------------------------------------------------------------------------
void control_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一拍 1ms 控制周期，由 CCU60_CH0 中断调用
// 参数说明     void
// 返回参数     void
// 使用示例     control_loop();
//-------------------------------------------------------------------------------------------------------------------
void control_loop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止 Test、点动与三轴平衡，三电机清零并锁死动量轮软件刹车
// 参数说明     void
// 返回参数     void
// 使用示例     control_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动三轴同时闭环(START_BALANCE)，原地平衡，速度目标恒 0，视觉不参与
//              停车用 control_stop()；安全闸也会把 start_flag 打回 START_STOP
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止，原因见 control_test_last_status()
// 使用示例     if (!control_balance_start()) menu_status(...);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_balance_start(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 用当前这一帧直道图标定逆透视
// 参数说明     void
// 返回参数     uint8           1=命令已发出
// 使用示例     control_ipm_calib_request();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_ipm_calib_request(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环里把新标定出的逆透视矩阵写进 Flash
// 参数说明     void
// 返回参数     uint8           1=有待存矩阵且 Flash 写入成功，0=无待存矩阵或保存失败
// 使用示例     if (control_ipm_flush()) menu_status("IPM SAVED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_ipm_flush(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询是否还有标定好的逆透视矩阵在等着写 Flash
// 参数说明     void
// 返回参数     uint8           1=还没写进 Flash 0=已经落盘或本来就没有新矩阵
// 使用示例     if (!control_ipm_pending()) menu_status("IPM SAVED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_ipm_pending(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动正式跑车，先按三轴平衡的全部安全条件完成发车
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     if (!control_run_start()) menu_status("RUN BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_run_start(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止正式跑车，目标速度归零并保持平衡制动
// 参数说明     void
// 返回参数     void
// 使用示例     control_run_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_run_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询正式跑车是否在跑
// 参数说明     void
// 返回参数     uint8           1=跑车中
// 使用示例     if (control_run_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_run_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次跑车的停车原因
// 参数说明     void
// 返回参数     run_stop_t      停车原因
// 使用示例     menu_status(run_stop_text(control_run_stop_reason()));
//-------------------------------------------------------------------------------------------------------------------
run_stop_t control_run_stop_reason(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复制正式 Run 的定长诊断状态；只做字段赋值，可由 1ms 中断调用
// 参数说明     out             输出快照地址，允许为 0
// 返回参数     void
// 使用示例     control_run_diag_snapshot(&snapshot);
//-------------------------------------------------------------------------------------------------------------------
void control_run_diag_snapshot(volatile control_run_diag_t *out);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动无线 Run Test，进入三轴平衡并等待 speed:turn,speed 命令
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     if (!control_remote_start()) menu_status("RUN BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 control_remote_start(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止无线 Run Test 并锁停全部电机
// 参数说明     void
// 返回参数     void
// 使用示例     control_remote_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_remote_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入无线遥控的相对航向角与行进速度目标
// 参数说明     steer_angle/speed_mps 相对接收时航向角度(°)与速度(m/s)
// 返回参数     uint8           1=已接受 0=未运行或命令越界
// 使用示例     control_remote_command(30.0f, 0.20f);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_remote_command(float steer_angle, float speed_mps);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询无线 Run Test 是否仍在运行
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (control_remote_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_remote_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取无线 Run Test 的最新命令状态
// 参数说明     steer_angle/speed_mps/age_ms/seen 对应输出地址，可传 0 忽略
// 返回参数     void
// 使用示例     control_remote_status(&turn, &speed, &age, &seen);
//-------------------------------------------------------------------------------------------------------------------
void control_remote_status(float *steer_angle, float *speed_mps, uint16 *age_ms, uint8 *seen);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在停车状态清零 C 轮累计计数，用于手推 1m 读取脉冲数
// 参数说明     void
// 返回参数     uint8           1=已清零 0=电机正在运行
// 使用示例     control_odometry_counter_reset();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_odometry_counter_reset(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动带三轴平衡的直行 1m 里程验证
// 参数说明     void
// 返回参数     uint8           1=已启动 0=被安全条件阻止
// 使用示例     control_odometry_test_start();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_odometry_test_start(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止 1m 里程验证并锁停全部电机
// 参数说明     void
// 返回参数     void
// 使用示例     control_odometry_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_odometry_test_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 1m 里程验证状态
// 参数说明     void
// 返回参数     odom_test_state_t 当前状态
// 使用示例     state = control_odometry_test_state();
//-------------------------------------------------------------------------------------------------------------------
odom_test_state_t control_odometry_test_state(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 1m 标定页需要的原始计数、距离和速度
// 参数说明     count/distance/speed 输出地址，可传 0 忽略
// 返回参数     void
// 使用示例     control_odometry_status(&count, &distance, &speed);
//-------------------------------------------------------------------------------------------------------------------
void control_odometry_status(int32 *count, float *distance, float *speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴与最高启用环的 Test
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=已启动 0=被安全条件阻止，原因见 control_test_last_status()
// 使用示例     control_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_start(tune_axis_t axis, tune_ring_t ring);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止单轴 Test
// 参数说明     void
// 返回参数     void
// 使用示例     control_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_test_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 Test 是否正在运行，顺带识别被安全闸停掉的情况
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (control_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次 Test 的启动结果或停止原因
// 参数说明     void
// 返回参数     control_test_status_t 状态码
// 使用示例     status = control_test_last_status();
//-------------------------------------------------------------------------------------------------------------------
control_test_status_t control_test_last_status(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动架空点动，持续转到停为止，占空比取 jog_duty_fly / jog_duty_drive
// 参数说明     target/forward  点动电机与方向，forward 为 1 表示正转
// 返回参数     uint8           1=已启动 0=姿态、标定、运行状态或驱动条件不满足
// 使用示例     control_jog_start(MOTOR_JOG_A, 1);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_jog_start(motor_jog_t target, uint8 forward);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即结束点动并重新锁死动量轮软件刹车
// 参数说明     void
// 返回参数     void
// 使用示例     control_jog_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_jog_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询当前正在点动的电机
// 参数说明     void
// 返回参数     motor_jog_t     点动目标，MOTOR_JOG_NONE 表示未运行
// 使用示例     if (control_jog_running() == MOTOR_JOG_A) { ... }
//-------------------------------------------------------------------------------------------------------------------
motor_jog_t control_jog_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 摄像头状态，失败时请求重新初始化
// 参数说明     void
// 返回参数     uint8           1=摄像头就绪 0=未就绪
// 使用示例     control_camera_debug_start();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_camera_debug_start(void);

#endif
