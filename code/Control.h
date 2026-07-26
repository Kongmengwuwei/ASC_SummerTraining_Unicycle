#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

/*
 * 独轮车控制周期。
 *
 * 角速度内环每 1 ms 执行一次；
 * 角度中环每 5 ms 执行一次；
 * 速度外环每 10 ms 执行一次。
 *
 * 这三个周期必须分别与各 PID 配置中的 sample_time_s 保持一致。
 */
#define CONTROL_RATE_PERIOD_MS       (1U)
#define CONTROL_ANGLE_PERIOD_MS      (5U)
#define CONTROL_SPEED_PERIOD_MS      (10U)

/*
 * 一组三串级 PID 的参数。
 *
 * 数据流向为：
 *     速度位置式 PID -> 期望车身角度
 *     角度位置式 PID -> 期望车身角速度
 *     角速度增量式 PID -> 电机输出
 */
typedef struct
{
    pid_config_t speed;
    pid_config_t angle;
    pid_config_t angular_rate;
} control_cascade_config_t;

/*
 * 执行器方向和混控符号。
 *
 * 每个成员在实车确认后只能设为 +1 或 -1：
 *     drive_motor_sign：
 *         前后平衡控制量到行进轮电机的方向。
 *
 *     flywheel_balance_sign_1/2：
 *         左右平衡控制量分别到前、后动量轮的方向。
 *
 *     flywheel_turn_sign_1/2：
 *         转弯控制量分别到前、后动量轮的方向。
 *
 * 默认全部为 0，Control_Enable() 会拒绝使能。这样在尚未验证安装方向时，
 * 即使误调用使能函数，也不会猜测电机正负方向。
 */
typedef struct
{
    int8_t drive_motor_sign;
    int8_t flywheel_balance_sign_1;
    int8_t flywheel_balance_sign_2;
    int8_t flywheel_turn_sign_1;
    int8_t flywheel_turn_sign_2;
} control_actuator_sign_t;

/* 控制器停止工作的原因，便于后续菜单或调试器显示。 */
typedef enum
{
    CONTROL_FAULT_NONE = 0,
    CONTROL_FAULT_NOT_CONFIGURED,
    CONTROL_FAULT_ATTITUDE_INVALID,
    CONTROL_FAULT_FALL_ANGLE,
    CONTROL_FAULT_MANUAL_STOP
} control_fault_t;

/*
 * 控制状态快照。
 *
 * measurement 为本周期反馈量，target 为本级 PID 的目标量；
 * drive_output、flywheel_output_1/2 是经过限幅和方向混控后的最终命令。
 */
typedef struct
{
    bool initialized;
    bool enabled;
    control_fault_t fault;

    float forward_speed_target;
    float forward_speed_measurement;
    float lateral_speed_target;
    float lateral_speed_measurement;
    float turn_output_target;

    float pitch_measurement;
    float roll_measurement;
    float pitch_rate_measurement;
    float roll_rate_measurement;

    float pitch_target;
    float roll_target;
    float pitch_rate_target;
    float roll_rate_target;

    float drive_balance_output;
    float flywheel_balance_output;
    int32_t drive_output;
    int32_t flywheel_output_1;
    int32_t flywheel_output_2;

    uint32_t rate_update_count;
    uint32_t angle_update_count;
    uint32_t speed_update_count;
} control_state_t;

/*
 * 两组 PID 参数均对外开放，方便后续通过菜单或调试器整定。
 *
 * fore_aft：行进轮负责的前后平衡通道；
 * left_right：两个动量轮负责的左右平衡通道。
 *
 * 初始增益全部为 0，只有输出限幅和采样周期预先设置。
 */
extern control_cascade_config_t control_fore_aft_config;
extern control_cascade_config_t control_left_right_config;

/*
 * 初始化两组三串级 PID 和控制状态，但不使能任何电机输出。
 * 应在 CPU0 完成电机和姿态模块初始化后、cpu_wait_event_ready() 前调用。
 */
void Control_Init(void);

/*
 * 在控制器失能时，将上面的公开参数装载到 PID 实例。
 * 修改 PID 参数后调用本函数；控制器正在运行时会拒绝更新并返回 false。
 */
bool Control_ApplyPidConfig(void);

/*
 * 设置三个运动目标：
 *
 * forward_speed_counts_per_s：
 *     行进轮编码器每秒计数目标。正负方向需在架空测试后确认。
 *
 * lateral_speed：
 *     左右通道速度外环目标。当前工程没有左右速度传感器，正常静止平衡时传 0。
 *
 * turn_output：
 *     直接加入两个动量轮混控器的转弯分量，单位为 CYT2BL3 占空比命令，
 *     范围会被限制到 Control.c 中的安全上限。
 */
void Control_SetMotionTarget(float forward_speed_counts_per_s,
                             float lateral_speed,
                             float turn_output);

/*
 * 写入左右通道速度反馈。
 *
 * 当前 CYT2BL3 驱动只实现命令发送，尚未解析飞轮转速，所以控制框架不能
 * 自行得到该反馈。后续可将驱动板返回的等效飞轮速度、或独立测得的横向速度
 * 在每次更新后写入这里。未写入时反馈保持为 0。
 */
void Control_SetLateralSpeedFeedback(float lateral_speed);

/*
 * 设置直立机械零点，单位为度。
 * 默认 pitch=0、roll=0 只是数学初值，必须根据车辆实际静止直立姿态标定。
 */
void Control_SetBalanceZero(float pitch_zero_deg, float roll_zero_deg);

/*
 * 设置行进电机方向和两个飞轮的平衡/转弯混控方向。
 * 五个参数必须全部为 +1 或 -1，否则保持原配置并返回 false。
 */
bool Control_SetActuatorSigns(const control_actuator_sign_t *signs);

/*
 * 清空 PID 历史量和编码器速度基准后使能控制。
 *
 * 只有在以下条件同时满足时才会成功：
 *     1. 姿态解算有效；
 *     2. 五个执行器符号均已设置为 +1 或 -1；
 *     3. 当前俯仰角和横滚角没有超过防倒阈值。
 *
 * 注意：本函数只提供软件条件检查，不能替代机械限位、急停开关和架空测试。
 */
bool Control_Enable(void);

/* 正常失能：立即清零三个电机输出并清空 PID 历史状态。 */
void Control_Disable(void);

/* 紧急停止：立即清零三个电机输出，并记录指定故障原因。 */
void Control_EmergencyStop(control_fault_t fault);

/*
 * 1 ms 控制任务，只能在 CPU0 的 CCU61_CH0 中断中调用。
 * 调用顺序必须位于 Attitude_Timer_1ms_ISR() 之后，以使用本周期最新角速度。
 */
void Control_Timer1ms_ISR(void);

/* 原子读取完整控制状态，供前台菜单或调试器使用。 */
void Control_GetState(control_state_t *state);

#endif
