#include "Y_Motor.h"

#include <stddef.h>

#include "board_config.h"

// 行进轮 C：DRV8701E 通道 1，PWM = P21_3，DIR = P21_2。
// 编码器是 TIM2 的脉冲方向模式，P33_7 收脉冲、P33_6 收方向。
// 5ms 读一次硬件计数并清零，内部再累计成 20ms 速度窗口供 Pitch 速度环使用。

#define Y_MOTOR_SPEED_WINDOW_SAMPLES  (CTRL_DIV_SPEED / Y_MOTOR_ENCODER_PERIOD_MS)

#if ((CTRL_DIV_SPEED % Y_MOTOR_ENCODER_PERIOD_MS) != 0)
    #error "CTRL_DIV_SPEED 必须是 Y_MOTOR_ENCODER_PERIOD_MS 的整数倍"
#endif

#pragma section all "cpu0_dsram"
static volatile int16 y_motor_count_5ms;        // 最近一次 5ms 采样的计数
static volatile int16 y_motor_count_20ms;       // 最近一个完整 20ms 窗口的计数
static volatile int32 y_motor_total_count;      // 自上次清零以来的累计计数
static int32 y_motor_window_count;              // 当前 20ms 窗口的累加值
static uint8 y_motor_window_samples;            // 当前 20ms 窗口已累加的 5ms 采样数
static uint8 y_motor_dir_level;                 // 当前 DIR 电平，换向时才重写
#pragma section all restore

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将行进轮控制量限制到软件安全范围
// 参数说明     duty            待限制控制量
// 返回参数     int32           限幅后的控制量
// 使用示例     duty = Y_Motor_LimitDuty(duty);
//-------------------------------------------------------------------------------------------------------------------
static int32 Y_Motor_LimitDuty(int32 duty)
{
    return func_limit_ab(duty, -DRIVE_OUT_LIMIT, DRIVE_OUT_LIMIT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化行进轮 C 的方向、PWM 和脉冲方向编码器
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_Init();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_Init(void)
{
    y_motor_count_5ms = 0;
    y_motor_count_20ms = 0;
    y_motor_total_count = 0;
    y_motor_window_count = 0;
    y_motor_window_samples = 0;
    y_motor_dir_level = (uint8)Y_MOTOR_FORWARD_DIR_LEVEL;

    gpio_init(Y_MOTOR_DIR_PIN,
              GPO,
              Y_MOTOR_FORWARD_DIR_LEVEL,
              GPO_PUSH_PULL);
    pwm_init(Y_MOTOR_PWM_PIN, Y_MOTOR_PWM_FREQUENCY_HZ, 0);

    encoder_dir_init(Y_MOTOR_ENCODER_INDEX,
                     Y_MOTOR_ENCODER_A_PIN,
                     Y_MOTOR_ENCODER_B_PIN);
    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置行进轮 C 的有符号占空比并执行方向、限幅和死区补偿
// 参数说明     duty            行进轮控制量，符号决定转向
// 返回参数     void
// 使用示例     Y_Motor_SetDuty(g_motor_c);
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_SetDuty(int32 duty)
{
    uint8 level;

    duty = Y_Motor_LimitDuty(duty);
    duty *= MOTOR_DIR_C;
    if (duty > 0)
        duty += DRIVE_DEAD_ZONE;        // 补掉起转死区，输出直接从 0 跳到 DRIVE_DEAD_ZONE
    else if (duty < 0)
        duty -= DRIVE_DEAD_ZONE;
    duty = Y_Motor_LimitDuty(duty);     // 死区补偿后仍不得越过 C 轮控制安全限幅
    duty = func_limit_ab(duty, -Y_MOTOR_PWM_MAX_DUTY, Y_MOTOR_PWM_MAX_DUTY);

    if (duty == 0)
    {
        pwm_set_duty(Y_MOTOR_PWM_PIN, 0);
        return;                         // 保持当前 DIR 电平，下次同向时不用再翻
    }

    level = (uint8)((duty > 0) ? Y_MOTOR_FORWARD_DIR_LEVEL
                               : ((Y_MOTOR_FORWARD_DIR_LEVEL == GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH));
    if (level != y_motor_dir_level)
    {
        pwm_set_duty(Y_MOTOR_PWM_PIN, 0);   // 只在换向这一拍卸载，避免 H 桥直通
        gpio_set_level(Y_MOTOR_DIR_PIN, level);
        y_motor_dir_level = level;
    }

    pwm_set_duty(Y_MOTOR_PWM_PIN, (uint32)((duty > 0) ? duty : -duty));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将行进轮 C 的 PWM 占空比清零
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_Stop();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_Stop(void)
{
    pwm_set_duty(Y_MOTOR_PWM_PIN, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取并清零硬件编码器，更新 5ms 计数、20ms 速度和累计里程
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_EncoderUpdate5ms();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_EncoderUpdate5ms(void)
{
    int16 count = (int16)(encoder_get_count(Y_MOTOR_ENCODER_INDEX) * ENC_DIR_C);

    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_count_5ms = count;
    y_motor_total_count += (int32)count;
    y_motor_window_count += (int32)count;
    y_motor_window_samples++;

    if (y_motor_window_samples >= Y_MOTOR_SPEED_WINDOW_SAMPLES)
    {
        y_motor_count_20ms =
            (int16)func_limit_ab(y_motor_window_count, -32768, 32767);
        y_motor_window_count = 0;
        y_motor_window_samples = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除行进轮硬件计数、周期速度与累计里程
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_EncoderClear();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_EncoderClear(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_count_5ms = 0;
    y_motor_count_20ms = 0;
    y_motor_total_count = 0;
    y_motor_window_count = 0;
    y_motor_window_samples = 0;
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原子读取行进轮最近 5ms 计数和累计里程
// 参数说明     data            编码器快照输出地址
// 返回参数     void
// 使用示例     Y_Motor_GetEncoder(&encoder);
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_GetEncoder(y_motor_encoder_data_t *data)
{
    uint32 interrupt_state;

    if (data == NULL)
        return;

    interrupt_state = interrupt_global_disable();
    data->count_5ms = y_motor_count_5ms;
    data->total_count = y_motor_total_count;
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取行进轮最近完整 20ms 窗口的编码器增量
// 参数说明     void
// 返回参数     int16           行进轮速度(counts/20ms)
// 使用示例     int16 speed = Y_Motor_GetSpeed20ms();
//-------------------------------------------------------------------------------------------------------------------
int16 Y_Motor_GetSpeed20ms(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    int16 speed = y_motor_count_20ms;

    interrupt_global_enable(interrupt_state);
    return speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取行进轮自上次清零以来的累计编码器计数
// 参数说明     void
// 返回参数     int32           行进轮累计计数
// 使用示例     int32 total = Y_Motor_GetTotalCount();
//-------------------------------------------------------------------------------------------------------------------
int32 Y_Motor_GetTotalCount(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    int32 total = y_motor_total_count;

    interrupt_global_enable(interrupt_state);
    return total;
}
