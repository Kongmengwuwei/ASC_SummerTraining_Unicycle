#ifndef VOFA_H_
#define VOFA_H_

#include "zf_common_headfile.h"

typedef enum
{
    VOFA_OFF = 0,       // 关闭波形输出
    VOFA_ATT,           // att: Roll、Pitch、Yaw，3 通道
} vofa_mode_t;

// 调参轴
typedef enum
{
    TUNE_AXIS_ROLL = 0,     // 横滚轴
    TUNE_AXIS_PITCH,        // 俯仰轴
    TUNE_AXIS_YAW,          // 航向轴
    TUNE_AXIS_MAX
} tune_axis_t;

// 调参环
typedef enum
{
    TUNE_RING_RATE = 0,     // 角速度环
    TUNE_RING_ANGLE,        // 角度环
    TUNE_RING_VEL,          // 速度环或飞轮回收环
    TUNE_RING_MAX
} tune_ring_t;

// 一整行下行命令的处理结果。Run Test 页显示最近一行的分类，
// 用来把"命令没生效"拆成收不到、格式错、没发车三种互不相同的原因。
typedef enum
{
    VOFA_CMD_NONE = 0,      // 还没有提交过整行
    VOFA_CMD_APPLIED,       // 已写入控制目标
    VOFA_CMD_NOT_RUNNING,   // 格式合法但 Run Test 没在跑，目标未写入
    VOFA_CMD_RANGE,         // 数值超出允许范围，被控制层拒绝
    VOFA_CMD_PREFIX,        // 不是 speed: 开头
    VOFA_CMD_FORMAT,        // 缺逗号、字段为空或数字非法
    VOFA_CMD_OVERFLOW,      // 接收环或命令行溢出，整行作废
} vofa_cmd_result_t;

extern volatile vofa_mode_t g_vofa_mode;    // 当前波形模式
extern volatile uint8       g_vofa_div;     // 发送分频
extern volatile tune_axis_t g_tune_axis;    // 当前调参轴
extern volatile tune_ring_t g_tune_ring;    // 当前调参环
extern volatile uint32      g_vofa_rx_bytes;// 无线下行累计接收字节数
extern volatile uint32      g_vofa_cmd_lines;// 已提交解析的整行数
extern volatile uint32      g_vofa_cmd_ok;  // 已写入控制目标的命令数
extern volatile vofa_cmd_result_t g_vofa_cmd_last;  // 最近一行的处理结果
extern volatile float       g_vofa_cmd_turn; // 最近一次解析成功的转向值(°)
extern volatile float       g_vofa_cmd_speed;// 最近一次解析成功的速度值(m/s)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化无线转串口模块与上行发送队列
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_init();
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     搬运一拍串口字节，由 1ms 控制中断调用。只搬字节，不做格式化和解析
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_tick1ms();
//-------------------------------------------------------------------------------------------------------------------
void vofa_tick1ms(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新 Roll、Pitch、Yaw 姿态波形快照
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_snapshot();
//-------------------------------------------------------------------------------------------------------------------
void vofa_snapshot(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按发送分频输出最新波形快照
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_poll();
//-------------------------------------------------------------------------------------------------------------------
void vofa_poll(void);

// 下行只接收 Run Test 遥控命令，以 '\r\n' 或 '\n' 结束：
//   speed:<转向>,<速度>
// 转向是相对发车航向角(°)，速度单位为 m/s。
// speed 命令只在主菜单 Run Test 已启动后生效；无线命令不能发车，返回键始终急停。

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并执行无线串口 Run Test 命令，由主循环调用
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_cmd_poll();
//-------------------------------------------------------------------------------------------------------------------
void vofa_cmd_poll(void);

#endif // VOFA_H_
