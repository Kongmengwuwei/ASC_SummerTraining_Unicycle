#include "Control.h"

#include <stddef.h>
#include <string.h>

#include "Attitude.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include "zf_common_headfile.h"

/*
 * 最终执行器限幅。
 *
 * PID 自身还有一级 output_limit；这里再限一次，是为了保证转弯混控叠加后
 * 仍然不会越过电机驱动允许范围。初次整定时建议进一步减小这些上限。
 */
#define CONTROL_DRIVE_OUTPUT_LIMIT          (8000.0f)
#define CONTROL_FLYWHEEL_OUTPUT_LIMIT       (8000.0f)
#define CONTROL_TURN_OUTPUT_LIMIT           (2000.0f)

/*
 * 当任一平衡角相对机械零点超过该值时立即停机。
 * 30 度是软件框架的保守初值，仍需结合车辆结构和保护架验证。
 */
#define CONTROL_FALL_ANGLE_LIMIT_DEG        (30.0f)

/* 由 10 ms 内编码器累计计数换算为每秒计数。 */
#define CONTROL_SPEED_DT_S                  (0.010f)

/*
 * 两组三串级 PID 的初始参数。
 *
 * 所有 kp/ki/kd 均故意保持为 0：
 *     - 先验证传感器轴和符号；
 *     - 再验证电机命令方向；
 *     - 最后按照“角速度内环 -> 角度中环 -> 速度外环”的顺序逐级整定。
 *
 * 这些限幅只是软件保护初值，不代表车辆已经可以安全使用对应最大输出。
 */
#pragma section all "cpu0_dsram"
control_cascade_config_t control_fore_aft_config =
{
    .speed =
    {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 10.0f,
        .output_limit = 15.0f,
        .derivative_alpha = 0.9f,
        .sample_time_s = 0.010f
    },
    .angle =
    {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 100.0f,
        .output_limit = 300.0f,
        .derivative_alpha = 0.8f,
        .sample_time_s = 0.005f
    },
    .angular_rate =
    {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 1000.0f,
        .output_limit = CONTROL_DRIVE_OUTPUT_LIMIT,
        .derivative_alpha = 0.8f,
        .sample_time_s = 0.001f
    }
};

control_cascade_config_t control_left_right_config =
{
    .speed =
    {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 10.0f,
        .output_limit = 15.0f,
        .derivative_alpha = 0.9f,
        .sample_time_s = 0.010f
    },
    .angle =
    {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 100.0f,
        .output_limit = 300.0f,
        .derivative_alpha = 0.8f,
        .sample_time_s = 0.005f
    },
    .angular_rate =
    {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 1000.0f,
        .output_limit = CONTROL_FLYWHEEL_OUTPUT_LIMIT,
        .derivative_alpha = 0.8f,
        .sample_time_s = 0.001f
    }
};

/* 前后平衡三环。 */
static pid_controller_t fore_aft_speed_pid;
static pid_controller_t fore_aft_angle_pid;
static pid_controller_t fore_aft_rate_pid;

/* 左右平衡三环。 */
static pid_controller_t left_right_speed_pid;
static pid_controller_t left_right_angle_pid;
static pid_controller_t left_right_rate_pid;

static volatile bool control_initialized;
static volatile bool control_enabled;
static volatile control_fault_t control_fault;

/* 前台写入、1 ms 中断读取的目标量。 */
static volatile float forward_speed_target;
static volatile float lateral_speed_target;
static volatile float lateral_speed_feedback;
static volatile float turn_output_target;

/* 实际机械直立时的姿态零点。 */
static volatile float pitch_zero;
static volatile float roll_zero;

/*
 * 默认符号全部为 0，表示“尚未通过架空试验确认”。
 * Control_Enable() 会检查这些值，不允许带着未知方向启动。
 */
static control_actuator_sign_t actuator_signs;

/* 三层任务分频计数。 */
static uint8_t angle_divider_count;
static uint8_t speed_divider_count;

/* 速度外环通过累计编码器差分计算 10 ms 平均速度。 */
static int32_t previous_encoder_total;

/* 由 1 ms 中断更新、供前台读取的完整状态。 */
static volatile control_state_t control_state;
#pragma section all restore

static float Control_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Control_Clamp(float value, float limit)
{
    float positive_limit = Control_Absolute(limit);

    if (value > positive_limit)
    {
        return positive_limit;
    }
    if (value < -positive_limit)
    {
        return -positive_limit;
    }
    return value;
}

static int32_t Control_FloatToOutput(float value, float limit)
{
    float limited = Control_Clamp(value, limit);

    /*
     * 电机接口使用整数占空比。这里使用向零截断，不引入数学库和额外执行时间；
     * 小于 1 个占空比单位的控制量会自然变为 0。
     */
    return (int32_t)limited;
}

static bool Control_IsSignValid(int8_t sign)
{
    return (sign == 1) || (sign == -1);
}

static bool Control_AreActuatorSignsValid(void)
{
    return Control_IsSignValid(actuator_signs.drive_motor_sign) &&
           Control_IsSignValid(actuator_signs.flywheel_balance_sign_1) &&
           Control_IsSignValid(actuator_signs.flywheel_balance_sign_2) &&
           Control_IsSignValid(actuator_signs.flywheel_turn_sign_1) &&
           Control_IsSignValid(actuator_signs.flywheel_turn_sign_2);
}

static bool Control_AngleIsSafe(float pitch, float roll)
{
    return (Control_Absolute(pitch - pitch_zero) <=
            CONTROL_FALL_ANGLE_LIMIT_DEG) &&
           (Control_Absolute(roll - roll_zero) <=
            CONTROL_FALL_ANGLE_LIMIT_DEG);
}

static void Control_ClearPidState(void)
{
    PID_Clear(&fore_aft_speed_pid);
    PID_Clear(&fore_aft_angle_pid);
    PID_Clear(&fore_aft_rate_pid);

    PID_Clear(&left_right_speed_pid);
    PID_Clear(&left_right_angle_pid);
    PID_Clear(&left_right_rate_pid);

    angle_divider_count = 0U;
    speed_divider_count = 0U;
}

static void Control_StopActuators(void)
{
    /*
     * 行进轮 PWM 立即清零；CYT2BL3 收到一帧双路零输出。
     * 本函数不包含延时、循环或动态内存，可用于故障停机路径。
     */
    Y_Motor_Stop();
    W_Motor_Stop();
}

static void Control_UpdateStateBase(bool initialized,
                                    bool enabled,
                                    control_fault_t fault)
{
    control_state.initialized = initialized;
    control_state.enabled = enabled;
    control_state.fault = fault;
}

void Control_Init(void)
{
    y_motor_encoder_data_t encoder;

    memset((void *)&control_state, 0, sizeof(control_state_t));
    memset(&actuator_signs, 0, sizeof(control_actuator_sign_t));

    control_initialized = true;
    control_enabled = false;
    control_fault = CONTROL_FAULT_MANUAL_STOP;

    forward_speed_target = 0.0f;
    lateral_speed_target = 0.0f;
    lateral_speed_feedback = 0.0f;
    turn_output_target = 0.0f;
    pitch_zero = 0.0f;
    roll_zero = 0.0f;

    PID_Init(&fore_aft_speed_pid, &control_fore_aft_config.speed);
    PID_Init(&fore_aft_angle_pid, &control_fore_aft_config.angle);
    PID_Init(&fore_aft_rate_pid, &control_fore_aft_config.angular_rate);

    PID_Init(&left_right_speed_pid, &control_left_right_config.speed);
    PID_Init(&left_right_angle_pid, &control_left_right_config.angle);
    PID_Init(&left_right_rate_pid,
             &control_left_right_config.angular_rate);

    angle_divider_count = 0U;
    speed_divider_count = 0U;

    Y_Motor_GetEncoder(&encoder);
    previous_encoder_total = encoder.total_count;

    Control_UpdateStateBase(true, false, control_fault);
    Control_StopActuators();
}

bool Control_ApplyPidConfig(void)
{
    uint32_t interrupt_state;

    if ((!control_initialized) || control_enabled)
    {
        return false;
    }

    /*
     * 配置结构可能由前台菜单逐字段修改。装载时短暂关闭中断，防止未来控制
     * 调度方式变化后读取到一半新、一半旧的参数。
     */
    interrupt_state = interrupt_global_disable();

    PID_Init(&fore_aft_speed_pid, &control_fore_aft_config.speed);
    PID_Init(&fore_aft_angle_pid, &control_fore_aft_config.angle);
    PID_Init(&fore_aft_rate_pid, &control_fore_aft_config.angular_rate);

    PID_Init(&left_right_speed_pid, &control_left_right_config.speed);
    PID_Init(&left_right_angle_pid, &control_left_right_config.angle);
    PID_Init(&left_right_rate_pid,
             &control_left_right_config.angular_rate);

    interrupt_global_enable(interrupt_state);
    return true;
}

void Control_SetMotionTarget(float forward_speed_counts_per_s,
                             float lateral_speed,
                             float turn_output)
{
    uint32_t interrupt_state = interrupt_global_disable();

    forward_speed_target = forward_speed_counts_per_s;
    lateral_speed_target = lateral_speed;
    turn_output_target =
        Control_Clamp(turn_output, CONTROL_TURN_OUTPUT_LIMIT);

    interrupt_global_enable(interrupt_state);
}

void Control_SetLateralSpeedFeedback(float lateral_speed)
{
    uint32_t interrupt_state = interrupt_global_disable();
    lateral_speed_feedback = lateral_speed;
    interrupt_global_enable(interrupt_state);
}

void Control_SetBalanceZero(float pitch_zero_deg, float roll_zero_deg)
{
    uint32_t interrupt_state;

    /* 运行中改变机械零点会产生目标阶跃，因此只允许在失能状态下修改。 */
    if (control_enabled)
    {
        return;
    }

    interrupt_state = interrupt_global_disable();
    pitch_zero = pitch_zero_deg;
    roll_zero = roll_zero_deg;
    interrupt_global_enable(interrupt_state);
}

bool Control_SetActuatorSigns(const control_actuator_sign_t *signs)
{
    uint32_t interrupt_state;

    if ((signs == NULL) || control_enabled)
    {
        return false;
    }

    if ((!Control_IsSignValid(signs->drive_motor_sign)) ||
        (!Control_IsSignValid(signs->flywheel_balance_sign_1)) ||
        (!Control_IsSignValid(signs->flywheel_balance_sign_2)) ||
        (!Control_IsSignValid(signs->flywheel_turn_sign_1)) ||
        (!Control_IsSignValid(signs->flywheel_turn_sign_2)))
    {
        return false;
    }

    interrupt_state = interrupt_global_disable();
    actuator_signs = *signs;
    interrupt_global_enable(interrupt_state);
    return true;
}

bool Control_Enable(void)
{
    attitude_euler_t attitude;
    y_motor_encoder_data_t encoder;
    uint32_t interrupt_state;

    if (!control_initialized)
    {
        return false;
    }

    if (!Control_AreActuatorSignsValid())
    {
        control_fault = CONTROL_FAULT_NOT_CONFIGURED;
        Control_UpdateStateBase(true, false, control_fault);
        Control_StopActuators();
        return false;
    }

    if ((!Attitude_GetEuler(&attitude)) ||
        (!attitude.ready) ||
        attitude.imu_error)
    {
        control_fault = CONTROL_FAULT_ATTITUDE_INVALID;
        Control_UpdateStateBase(true, false, control_fault);
        Control_StopActuators();
        return false;
    }

    if (!Control_AngleIsSafe(attitude.pitch, attitude.roll))
    {
        control_fault = CONTROL_FAULT_FALL_ANGLE;
        Control_UpdateStateBase(true, false, control_fault);
        Control_StopActuators();
        return false;
    }

    Y_Motor_GetEncoder(&encoder);

    interrupt_state = interrupt_global_disable();
    Control_ClearPidState();
    previous_encoder_total = encoder.total_count;

    /*
     * 速度环第一次执行要等待 10 ms。使能瞬间先把角度目标放在机械零点、
     * 角速度目标放在 0，避免中环在这 10 ms 内使用 memset 后的错误初值。
     */
    control_state.forward_speed_measurement = 0.0f;
    control_state.lateral_speed_measurement = lateral_speed_feedback;
    control_state.pitch_target = pitch_zero;
    control_state.roll_target = roll_zero;
    control_state.pitch_rate_target = 0.0f;
    control_state.roll_rate_target = 0.0f;

    control_fault = CONTROL_FAULT_NONE;
    control_enabled = true;
    Control_UpdateStateBase(true, true, control_fault);
    interrupt_global_enable(interrupt_state);

    return true;
}

void Control_EmergencyStop(control_fault_t fault)
{
    uint32_t interrupt_state = interrupt_global_disable();

    control_enabled = false;
    control_fault = fault;
    Control_ClearPidState();
    Control_UpdateStateBase(control_initialized, false, fault);

    control_state.drive_balance_output = 0.0f;
    control_state.flywheel_balance_output = 0.0f;
    control_state.drive_output = 0;
    control_state.flywheel_output_1 = 0;
    control_state.flywheel_output_2 = 0;

    interrupt_global_enable(interrupt_state);
    Control_StopActuators();
}

void Control_Disable(void)
{
    Control_EmergencyStop(CONTROL_FAULT_MANUAL_STOP);
}

void Control_Timer1ms_ISR(void)
{
    float current_pitch;
    float current_roll;
    float current_pitch_rate;
    float current_roll_rate;
    float drive_balance_output;
    float flywheel_balance_output;
    float turn_output;
    float flywheel_1_mixed;
    float flywheel_2_mixed;
    int32_t drive_output;
    int32_t flywheel_output_1;
    int32_t flywheel_output_2;

    if ((!control_initialized) || (!control_enabled))
    {
        return;
    }

    /*
     * 本函数紧跟在 Attitude_Timer_1ms_ISR() 后执行，因此下列角速度是本周期
     * 最新 IMU 数据；欧拉角每 5 ms 更新一次，正好与角度中环周期一致。
     *
     * 默认轴映射：
     *     pitch / pitch_rate -> 前后平衡；
     *     roll  / roll_rate  -> 左右平衡。
     * 若 IMU 实际安装方向不同，必须先在姿态显示中确认并修改映射，不能靠 PID
     * 正负号掩盖传感器轴错误。
     */
    current_pitch = eulerAngle.pitch;
    current_roll = eulerAngle.roll;
    current_pitch_rate = attitudeRate.pitch_rate;
    current_roll_rate = attitudeRate.roll_rate;

    if ((!eulerAngle.ready) ||
        eulerAngle.imu_error)
    {
        Control_EmergencyStop(CONTROL_FAULT_ATTITUDE_INVALID);
        return;
    }

    if (!Control_AngleIsSafe(current_pitch, current_roll))
    {
        Control_EmergencyStop(CONTROL_FAULT_FALL_ANGLE);
        return;
    }

    /*
     * 速度外环：每 10 ms 运行一次，输出期望车身角度修正量。
     *
     * 前后速度取行进轮累计编码器的 10 ms 差分，避免只使用某一个 5 ms 采样。
     * 左右速度由 Control_SetLateralSpeedFeedback() 注入；当前无对应传感器时为 0。
     */
    speed_divider_count++;
    if (speed_divider_count >=
        (CONTROL_SPEED_PERIOD_MS / CONTROL_RATE_PERIOD_MS))
    {
        y_motor_encoder_data_t encoder;
        int32_t encoder_delta;

        speed_divider_count = 0U;
        Y_Motor_GetEncoder(&encoder);
        encoder_delta = encoder.total_count - previous_encoder_total;
        previous_encoder_total = encoder.total_count;

        control_state.forward_speed_measurement =
            (float)encoder_delta / CONTROL_SPEED_DT_S;
        control_state.lateral_speed_measurement =
            lateral_speed_feedback;

        control_state.pitch_target =
            pitch_zero +
            PID_PositionCalculate(&fore_aft_speed_pid,
                                  control_state.forward_speed_measurement,
                                  forward_speed_target);

        control_state.roll_target =
            roll_zero +
            PID_PositionCalculate(&left_right_speed_pid,
                                  control_state.lateral_speed_measurement,
                                  lateral_speed_target);

        control_state.speed_update_count++;
    }

    /*
     * 角度中环：每 5 ms 运行一次，输出期望角速度。
     * 两个角度环均使用位置式 PID。
     */
    angle_divider_count++;
    if (angle_divider_count >=
        (CONTROL_ANGLE_PERIOD_MS / CONTROL_RATE_PERIOD_MS))
    {
        angle_divider_count = 0U;

        control_state.pitch_rate_target =
            PID_PositionCalculate(&fore_aft_angle_pid,
                                  current_pitch,
                                  control_state.pitch_target);

        control_state.roll_rate_target =
            PID_PositionCalculate(&left_right_angle_pid,
                                  current_roll,
                                  control_state.roll_target);

        control_state.angle_update_count++;
    }

    /*
     * 角速度内环：每 1 ms 运行一次，使用增量式 PID。
     *
     * 增量式 PID 的返回值是已经累计并限幅后的总输出，不是单次增量；
     * 禁止在外部再次累加。
     */
    drive_balance_output =
        PID_IncrementalCalculate(&fore_aft_rate_pid,
                                 current_pitch_rate,
                                 control_state.pitch_rate_target);

    flywheel_balance_output =
        PID_IncrementalCalculate(&left_right_rate_pid,
                                 current_roll_rate,
                                 control_state.roll_rate_target);

    turn_output =
        Control_Clamp(turn_output_target, CONTROL_TURN_OUTPUT_LIMIT);

    /*
     * 执行器混控：
     *
     * 行进轮只承担前后平衡/前进控制；
     * 两个倾斜动量轮同时叠加左右平衡分量和转弯分量。
     *
     * 前后飞轮的安装倾角、旋向可能不同，因此不在这里写死“同向/反向”，而是
     * 通过五个经过实测的 ±1 符号完成分配。
     */
    drive_output =
        Control_FloatToOutput(
            (float)actuator_signs.drive_motor_sign *
                drive_balance_output,
            CONTROL_DRIVE_OUTPUT_LIMIT);

    flywheel_1_mixed =
        (float)actuator_signs.flywheel_balance_sign_1 *
            flywheel_balance_output +
        (float)actuator_signs.flywheel_turn_sign_1 *
            turn_output;

    flywheel_2_mixed =
        (float)actuator_signs.flywheel_balance_sign_2 *
            flywheel_balance_output +
        (float)actuator_signs.flywheel_turn_sign_2 *
            turn_output;

    flywheel_output_1 =
        Control_FloatToOutput(flywheel_1_mixed,
                              CONTROL_FLYWHEEL_OUTPUT_LIMIT);
    flywheel_output_2 =
        Control_FloatToOutput(flywheel_2_mixed,
                              CONTROL_FLYWHEEL_OUTPUT_LIMIT);

    /*
     * 电机输出位于 1 ms 中断末尾，确保本周期三环计算全部完成后再统一更新。
     * CYT2BL3 每帧 7 字节、460800 baud，按 8N1 计算线时约为 152 us，
     * 以 1 kHz 发送时约占串口带宽的 15.2%。现有 Attitude_GetPerformance()
     * 只统计姿态函数本身，并不包含本控制函数；实机上还需用 STM 计时或示波器
     * 单独测量整个 CCU61_CH0 中断时间，确认最坏情况小于 1 ms。
     */
    Y_Motor_SetDuty(drive_output);
    W_Motor_SetDuty(flywheel_output_1, flywheel_output_2);

    control_state.forward_speed_target = forward_speed_target;
    control_state.lateral_speed_target = lateral_speed_target;
    control_state.turn_output_target = turn_output;
    control_state.pitch_measurement = current_pitch;
    control_state.roll_measurement = current_roll;
    control_state.pitch_rate_measurement = current_pitch_rate;
    control_state.roll_rate_measurement = current_roll_rate;
    control_state.drive_balance_output = drive_balance_output;
    control_state.flywheel_balance_output = flywheel_balance_output;
    control_state.drive_output = drive_output;
    control_state.flywheel_output_1 = flywheel_output_1;
    control_state.flywheel_output_2 = flywheel_output_2;
    control_state.rate_update_count++;
}

void Control_GetState(control_state_t *state)
{
    uint32_t interrupt_state;

    if (state == NULL)
    {
        return;
    }

    /*
     * control_state 含多个 float 和计数器，必须作为同一时刻的快照读取。
     * 复制期间短暂关闭中断，避免前台看到由两个控制周期拼接出的状态。
     */
    interrupt_state = interrupt_global_disable();
    *state = control_state;
    interrupt_global_enable(interrupt_state);
}
