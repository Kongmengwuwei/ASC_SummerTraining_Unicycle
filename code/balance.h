#ifndef BALANCE_H_
#define BALANCE_H_

#include "zf_common_headfile.h"
#include "vofa.h"

// 发车状态
typedef enum {
    START_STOP = 0,     // 全停并锁定刹车
    START_DRIVE_ONLY,   // 仅启动行进轮
    START_BALANCE,      // 启动全部平衡控制
} start_state_t;

extern start_state_t start_flag;

// 串级控制波形数据：set=目标，fb=反馈，out=输出，pwm=最终控制量。
typedef struct
{
    // Roll
    float r_rcy_set;
    float r_rcy_fb;
    float r_rcy_out;
    float r_ang_fb;
    float r_ang_out;
    float r_rate_fb;
    float r_pwm;

    // Pitch
    float p_vel_set;
    float p_vel_fb;
    float p_vel_out;
    float p_ang_fb;
    float p_ang_out;
    float p_rate_fb;
    float p_pwm;

    // Yaw
    float y_set;
    float y_fb;
    float y_out;
    float y_rate_fb;
    float y_pwm;
} balance_dbg_t;

extern balance_dbg_t g_bal_dbg;

extern float g_roll_zero, g_pitch_zero;              // 机械零点
extern float g_lean_offset;                          // 压弯偏移
extern float g_pwm_roll, g_pwm_pitch, g_pwm_yaw;     // 三轴输出
extern int16 g_motor_a, g_motor_b, g_motor_c;        // 电机输出
extern int   g_target_distance;                      // 行进速度目标
extern float g_yaw_target;                           // 航向目标

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入零点初值 + 串级各环 PID 参数, 复位状态机为 STOP, 开机调一次
// 参数说明     void
// 返回参数     void
// 使用示例     balance_init();
//-------------------------------------------------------------------------------------------------------------------
void balance_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     平衡主调度, 三轴串级->合成A/B/C->角度超限保护->按状态机限幅死区下发
// 参数说明     void
// 返回参数     void
// 使用示例     balance_run();   // control_loop 内调用
//-------------------------------------------------------------------------------------------------------------------
void balance_run(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动指定轴和指定级联层级的电机测试
// 参数说明     axis/ring       测试轴与最高启用环
// 返回参数     uint8           1=启动成功 0=参数无效
// 使用示例     balance_test_start(TUNE_AXIS_PITCH, TUNE_RING_RATE);
//-------------------------------------------------------------------------------------------------------------------
uint8 balance_test_start(tune_axis_t axis, tune_ring_t ring);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止电机测试并清除全部 PID 状态
// 参数说明     void
// 返回参数     void
// 使用示例     balance_test_stop();
//-------------------------------------------------------------------------------------------------------------------
void balance_test_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询电机测试是否正在运行
// 参数说明     void
// 返回参数     uint8           1=运行中 0=已停止
// 使用示例     if (balance_test_running()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 balance_test_running(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一次独立轴测试控制
// 参数说明     void
// 返回参数     void
// 使用示例     balance_test_run();
//-------------------------------------------------------------------------------------------------------------------
void balance_test_run(void);

#endif
