#ifndef CONTROL_H_
#define CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "vofa.h"

// Roll  车体左右倾：飞轮回收环(20ms) -> 角度环(5ms) -> 角速度环(1ms)，A/B 动量轮差动
// Pitch 车体前后倾：速度环(20ms)     -> 角度环(5ms) -> 角速度环(1ms)，C 行进轮
// Yaw   车体航向  ：转向外环(5ms)                   -> 角速度内环(1ms)，A/B 动量轮同向

// 发车状态，上电默认 START_STOP
typedef enum {
    START_STOP = 0,     // 三电机零输出，A/B 刹车锁死
    START_DRIVE_ONLY,   // 只跑 C 行进轮
    START_BALANCE,      // 三轴闭环，A/B 松刹车
} start_state_t;

extern start_state_t start_flag;

// 串级各环的中间量快照，只给 vofa 波形用：set=目标，fb=反馈，out=本环输出，pwm=该轴最终控制量
typedef struct
{
    float r_rcy_set;    // Roll 回收环目标，恒为 0
    float r_rcy_fb;     // Roll 回收环反馈，两飞轮转速差(RPM)
    float r_rcy_out;    // Roll 回收环输出，叠加到角度环误差
    float r_ang_fb;     // 横滚角反馈(°)
    float r_ang_out;    // Roll 角度环输出，即角速度环目标
    float r_rate_fb;    // 横滚角速度反馈(°/s)
    float r_pwm;        // Roll 最终控制量

    float p_vel_set;    // Pitch 速度环目标，斜坡后的值(counts/20ms)
    float p_vel_fb;     // Pitch 速度环反馈(counts/20ms)
    float p_vel_out;    // Pitch 速度环输出，叠加到角度环误差
    float p_ang_fb;     // 俯仰角反馈(°)
    float p_ang_out;    // Pitch 角度环输出，即角速度环目标
    float p_rate_fb;    // 俯仰角速度反馈(°/s)
    float p_pwm;        // Pitch 最终控制量

    float y_set;        // 航向目标(°)
    float y_fb;         // 航向反馈(°)
    float y_out;        // 转向外环输出，即角速度内环目标
    float y_rate_fb;    // 航向角速度反馈(°/s)
    float y_pwm;        // Yaw 最终控制量
} balance_dbg_t;

extern balance_dbg_t g_bal_dbg;

extern float g_roll_zero, g_pitch_zero;              // 机械零点(°)，由 Zero 页标定
extern float g_lean_offset;                          // 压弯动态零点偏移(°)，叠加到横滚零点
extern float g_pwm_roll, g_pwm_pitch, g_pwm_yaw;     // 三轴串级输出，混控前
extern int16 g_motor_a, g_motor_b, g_motor_c;        // 混控后的三电机控制量
extern int   g_target_distance;                      // Pitch 速度环目标(counts/20ms)
extern float g_yaw_target;                           // 转向外环目标航向(°)

extern float  g_dbg_error;                  // 中线偏差，右偏为正
extern uint8  g_imu_ok;                     // IMU660RB 初始化结果
extern uint8  g_cam_ok;                     // CPU1 摄像头就绪标志
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
extern float  g_vision_speed_scale;          // 元素建议速度倍率，Run 尚未使用
extern uint8  g_vision_stop_request;         // 元素停车请求，Run 尚未使用

// Test/Wave 启动结果，菜单据此显示可操作的提示
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化参数、按键、两路电机、平衡控制、波形和 IMU660RB，最后开 1ms 中断
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
// 函数简介     立即停止 Test/Wave 与点动，三电机清零并锁死动量轮软件刹车
// 参数说明     void
// 返回参数     void
// 使用示例     control_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴与最高启用环的 Test/Wave
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=已启动 0=被安全条件阻止，原因见 control_test_last_status()
// 使用示例     control_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_start(tune_axis_t axis, tune_ring_t ring);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止 Test/Wave 并关闭波形输出
// 参数说明     void
// 返回参数     void
// 使用示例     control_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_test_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 Test/Wave 是否正在运行，顺带识别被安全闸停掉的情况
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (control_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次 Test/Wave 的启动结果或停止原因
// 参数说明     void
// 返回参数     control_test_status_t 状态码
// 使用示例     status = control_test_last_status();
//-------------------------------------------------------------------------------------------------------------------
control_test_status_t control_test_last_status(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动一次限时架空点动，倒计时在 1ms 中断里跑，菜单卡死也会自动停
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 是否发布了新的视觉帧
// 参数说明     void
// 返回参数     uint8           1=有新帧 0=无新帧
// 使用示例     if (control_vision_debug()) display_track_view();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_vision_debug(void);

#endif
