#ifndef CONTROL_H_
#define CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "vofa.h"

// 直立串级控制周期，采样周期被增益吸收，改周期必须重整定。
#define CONTROL_RATE_PERIOD_MS       (1U)    // 角速度内环周期(ms)
#define CONTROL_ANGLE_PERIOD_MS      (5U)    // 角度中环周期(ms)
#define CONTROL_SPEED_PERIOD_MS      (10U)   // 速度外环周期(ms)

extern float  g_dbg_error;          // 中线偏差
extern uint8  g_imu_ok;             // IMU 初始化状态
extern uint8  g_cam_ok;             // 摄像头初始化状态
extern uint8  g_track_valid;        // 当前循迹帧状态
extern uint16 g_track_lost_frames;  // 连续无效帧计数
extern volatile uint16 g_vision_age_ms;  // 图像帧间隔(ms)
extern volatile uint32 g_control_uptime_ms; // 控制运行时间(ms)
extern volatile uint32 g_vision_frame_seq;  // 最新视觉帧序号
extern volatile uint32 g_vision_heartbeat;  // CPU1 主循环心跳
extern uint16 g_vision_threshold;           // 最新大津阈值
extern uint16 g_vision_search_stop;         // 最新有效前瞻行数
extern uint16 g_vision_left_lost;           // 最新左边丢线数
extern uint16 g_vision_right_lost;          // 最新右边丢线数
extern uint16 g_vision_both_lost;           // 最新双边丢线数
// extern uint8  g_vision_active_elem;      // 元素识别启用后恢复

// 发车阻断状态
typedef enum
{
    CTRL_BLOCK_NONE = 0,
    CTRL_BLOCK_IMU_FAIL,
    CTRL_BLOCK_ATT_CONVERGING,
    CTRL_BLOCK_ATT_DIVERGED,
    CTRL_BLOCK_IMU_LOST,
    CTRL_BLOCK_BLDC_LOST,
} control_block_t;

typedef enum
{
    CTRL_TEST_STATUS_OK = 0,
    CTRL_TEST_STATUS_INVALID,
    CTRL_TEST_STATUS_IMU_FAIL,
    CTRL_TEST_STATUS_ATT_CONVERGING,
    CTRL_TEST_STATUS_ATT_DIVERGED,
    CTRL_TEST_STATUS_IMU_LOST,
    CTRL_TEST_STATUS_BLDC_LOST,
    CTRL_TEST_STATUS_SAFETY,
} control_test_status_t;

// 架空点动目标
typedef enum
{
    MOTOR_JOG_NONE = 0,     // 未点动
    MOTOR_JOG_A,            // 动量轮 A
    MOTOR_JOG_B,            // 动量轮 B
    MOTOR_JOG_C,            // 行进轮 C
} motor_jog_t;

// 单级 PID 增益
typedef struct
{
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float imax;             // 积分限幅，增量式不使用
    float out_limit;        // 本级输出限幅
} control_pid_config_t;

// 单通道三串级增益，速度环出角度目标，角度环出角速度目标，角速度环出占空比
typedef struct
{
    control_pid_config_t speed;         // 速度外环，位置式
    control_pid_config_t angle;         // 角度中环，位置式
    control_pid_config_t angular_rate;  // 角速度内环，增量式
} control_cascade_config_t;

// 执行器方向符号，架空实测确认后只能取 ±1
// 默认全 0 表示未确认，Control_Enable() 拒绝使能，不猜电机方向
// W_Motor/Y_Motor 内部还会各乘一次 MOTOR_DIR_A/B/C，用本控制器时那三个参数保持 +1
typedef struct
{
    int8 drive_motor_sign;          // 前后平衡量到行进轮
    int8 flywheel_balance_sign_1;   // 左右平衡量到动量轮 A
    int8 flywheel_balance_sign_2;   // 左右平衡量到动量轮 B
    int8 flywheel_turn_sign_1;      // 转弯量到动量轮 A
    int8 flywheel_turn_sign_2;      // 转弯量到动量轮 B
} control_actuator_sign_t;

// 直立控制器停止原因
typedef enum
{
    CONTROL_FAULT_NONE = 0,
    CONTROL_FAULT_NOT_CONFIGURED,   // 执行器符号未实测确认
    CONTROL_FAULT_ATTITUDE_INVALID, // IMU 失效、未收敛、发散或出现 NaN
    CONTROL_FAULT_BLDC_LINK,        // CYT2BL3 通信中断
    CONTROL_FAULT_FALL_ANGLE,       // 超过防倒角
    CONTROL_FAULT_MANUAL_STOP       // 手动失能
} control_fault_t;

// 直立控制状态快照，measurement 为反馈量，target 为下一级目标量
typedef struct
{
    uint8 initialized;                  // 已初始化
    uint8 enabled;                      // 闭环使能中
    control_fault_t fault;              // 当前故障原因

    float forward_speed_target;         // 前进速度目标(counts/s)
    float forward_speed_measurement;    // 前进速度反馈(counts/s)
    float lateral_speed_target;         // 横向速度目标
    float lateral_speed_measurement;    // 横向速度反馈
    float turn_output_target;           // 转弯前馈量(占空比)

    float pitch_measurement;            // 俯仰角反馈(°)
    float roll_measurement;             // 横滚角反馈(°)
    float pitch_rate_measurement;       // 俯仰角速度反馈(°/s)
    float roll_rate_measurement;        // 横滚角速度反馈(°/s)

    float pitch_target;                 // 俯仰角目标(°)
    float roll_target;                  // 横滚角目标(°)
    float pitch_rate_target;            // 俯仰角速度目标(°/s)
    float roll_rate_target;             // 横滚角速度目标(°/s)

    float drive_balance_output;         // 前后角速度环输出
    float flywheel_balance_output;      // 左右角速度环输出
    int32 drive_output;                 // 行进轮最终占空比
    int32 flywheel_output_1;            // 动量轮 A 最终占空比
    int32 flywheel_output_2;            // 动量轮 B 最终占空比

    uint32 rate_update_count;           // 角速度环执行次数
    uint32 angle_update_count;          // 角度环执行次数
    uint32 speed_update_count;          // 速度环执行次数
} control_state_t;

// 两组增益对外开放供菜单整定，初始增益全 0，只预置限幅
extern control_cascade_config_t control_fore_aft_config;
extern control_cascade_config_t control_left_right_config;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化参数、按键、IMU660RB、姿态解算与调试串口
// 参数说明     void
// 返回参数     void
// 使用示例     control_init();
//-------------------------------------------------------------------------------------------------------------------
void control_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安全进入三轴平衡运行状态
// 参数说明     void
// 返回参数     uint8           1=已启动 0=安全条件不允许
// 使用示例     if (!control_start_balance()) display_start_blocked_screen(control_start_block_reason());
//-------------------------------------------------------------------------------------------------------------------
// uint8 control_start_balance(void);       // 完整跑车流程启用后恢复

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     进入完整跑赛道流程：平衡闭环 + 视觉循迹驱动转向与速度
// 参数说明     void
// 返回参数     uint8           1=已发车 0=RUN_FLOW_ENABLE 为 0 或安全条件不允许
// 使用示例     if (!control_start_run()) menu_status("RUN BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
// uint8 control_start_run(void);           // 完整跑车流程启用后恢复

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动一次限时架空点动，用于确认电机转向与转速反馈符号
// 参数说明     target/forward  点动电机与方向, forward 为 1 表示正转
// 返回参数     uint8           1=已启动 0=正在跑其它闭环或安全条件不允许
// 使用示例     control_jog_start(MOTOR_JOG_A, 1);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_jog_start(motor_jog_t target, uint8 forward);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即结束点动并重新锁死动量轮
// 参数说明     void
// 返回参数     void
// 使用示例     control_jog_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_jog_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询当前正在点动的电机
// 参数说明     void
// 返回参数     motor_jog_t     点动目标, MOTOR_JOG_NONE 表示未运行
// 使用示例     if (control_jog_running() == MOTOR_JOG_A) { ... }
//-------------------------------------------------------------------------------------------------------------------
motor_jog_t control_jog_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止正常运行与 Test/Wave 并锁定可用电机
// 参数说明     void
// 返回参数     void
// 使用示例     control_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 摄像头状态并在失败时请求重新初始化
// 参数说明     void
// 返回参数     uint8           1=初始化成功 0=初始化失败
// 使用示例     control_camera_debug_start();
//-------------------------------------------------------------------------------------------------------------------
uint8 control_camera_debug_start(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查是否允许离开 STOP 状态
// 参数说明     void
// 返回参数     uint8           1=允许 0=禁止
// 使用示例     if (control_allow_start()) start_flag = START_DRIVE_ONLY;
//-------------------------------------------------------------------------------------------------------------------
// uint8 control_allow_start(void);         // 完整跑车流程启用后恢复

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发车被拒的具体原因(供菜单/屏幕显示可操作的提示)
// 参数说明     void
// 返回参数     control_block_t 见本文件枚举
// 使用示例     if (!control_allow_start()) display_block_screen(control_start_block_reason());
//-------------------------------------------------------------------------------------------------------------------
// control_block_t control_start_block_reason(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行 1ms 姿态更新、Test/Wave 控制与波形快照
// 参数说明     void
// 返回参数     void
// 使用示例     control_loop();   // 在 isr.c 的 cc60_pit_ch0_isr 内调用(1ms)
//-------------------------------------------------------------------------------------------------------------------
void control_loop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CPU1 是否发布了新的视觉调试帧
// 参数说明     void
// 返回参数     uint8           1=处理了新帧 0=无新帧
// 使用示例     if (control_vision_debug()) display_track_view();   // 菜单 Camera 页
//-------------------------------------------------------------------------------------------------------------------
uint8 control_vision_debug(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴和级联层级的 Test/Wave
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=启动成功 0=启动被安全条件阻止
// 使用示例     control_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_start(tune_axis_t axis, tune_ring_t ring);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     立即停止 Test/Wave 并清除控制状态
// 参数说明     void
// 返回参数     void
// 使用示例     control_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void control_test_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 Test/Wave 是否正在运行
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (control_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 control_test_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取最近一次 Test/Wave 启动结果
// 参数说明     void
// 返回参数     control_test_status_t 启动结果或阻断原因
// 使用示例     status = control_test_last_status();
//-------------------------------------------------------------------------------------------------------------------
control_test_status_t control_test_last_status(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     行进轮速度(供环岛元素按帧累计行程)
// 参数说明     void
// 返回参数     int             行进轮 C 当前速度(脉冲/方向编码器 counts/20ms)
// 使用示例     g_island.state2_count += control_get_enc_speed();
//-------------------------------------------------------------------------------------------------------------------
// int  control_get_enc_speed(void);        // 元素状态机启用后恢复

// 直立串级控制器，control_loop() 调度优先级：Test/Wave > 点动 > 本控制器 > balance_run()

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在控制器失能时把公开增益装载进 PID 实例
// 参数说明     void
// 返回参数     uint8           1=已装载 0=控制器正在运行，拒绝更新
// 使用示例     if (!Control_ApplyPidConfig()) menu_status("DISABLE FIRST");
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_ApplyPidConfig(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置前进速度、横向速度与转弯三个运动目标
// 参数说明     forward_speed_counts_per_s 行进轮编码器每秒计数目标
// 参数说明     lateral_speed              左右通道速度外环目标，无横向传感器时传 0
// 参数说明     turn_output                直接叠加到两动量轮的转弯分量(占空比)
// 返回参数     void
// 使用示例     Control_SetMotionTarget(0.0f, 0.0f, 0.0f);
//-------------------------------------------------------------------------------------------------------------------
void Control_SetMotionTarget(float forward_speed_counts_per_s,
                             float lateral_speed,
                             float turn_output);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入左右通道速度反馈，未写入时保持 0
// 参数说明     lateral_speed   横向速度反馈
// 返回参数     void
// 使用示例     Control_SetLateralSpeedFeedback((float)(W_Motor_GetSpeed2() - W_Motor_GetSpeed1()));
//-------------------------------------------------------------------------------------------------------------------
void Control_SetLateralSpeedFeedback(float lateral_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置直立机械零点，只允许在失能状态下修改
// 参数说明     pitch_zero_deg/roll_zero_deg 俯仰与横滚机械零点(°)
// 返回参数     void
// 使用示例     Control_SetBalanceZero(PITCH_ZERO_INIT, ROLL_ZERO_INIT);
//-------------------------------------------------------------------------------------------------------------------
void Control_SetBalanceZero(float pitch_zero_deg, float roll_zero_deg);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置五个执行器方向符号，必须全部为 ±1
// 参数说明     signs           符号结构指针
// 返回参数     uint8           1=已生效 0=有非法符号或控制器正在运行
// 使用示例     Control_SetActuatorSigns(&signs);
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_SetActuatorSigns(const control_actuator_sign_t *signs);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空 PID 历史与编码器基准后使能直立控制
// 参数说明     void
// 返回参数     uint8           1=已使能 0=安全条件不满足，原因见 Control_GetState()
// 使用示例     if (!Control_Enable()) menu_status("BALANCE BLOCKED");
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_Enable(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     正常失能：清零三电机输出并清空 PID 历史
// 参数说明     void
// 返回参数     void
// 使用示例     Control_Disable();
//-------------------------------------------------------------------------------------------------------------------
void Control_Disable(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停止：清零三电机输出并记录故障原因
// 参数说明     fault           故障原因
// 返回参数     void
// 使用示例     Control_EmergencyStop(CONTROL_FAULT_FALL_ANGLE);
//-------------------------------------------------------------------------------------------------------------------
void Control_EmergencyStop(control_fault_t fault);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询直立控制器是否处于使能状态
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已失能
// 使用示例     if (Control_IsEnabled()) menu_status("BALANCING");
//-------------------------------------------------------------------------------------------------------------------
uint8 Control_IsEnabled(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原子读取完整控制状态快照
// 参数说明     state           状态输出地址
// 返回参数     void
// 使用示例     Control_GetState(&state);
//-------------------------------------------------------------------------------------------------------------------
void Control_GetState(control_state_t *state);

#endif
