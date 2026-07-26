#include "Y_Motor.h"

#include <stddef.h>

#include "board_config.h"
#include "pid.h"

#define Y_MOTOR_SPEED_WINDOW_SAMPLES  (CTRL_DIV_SPEED / Y_MOTOR_ENCODER_PERIOD_MS)

#if ((CTRL_DIV_SPEED % Y_MOTOR_ENCODER_PERIOD_MS) != 0)
    #error "CTRL_DIV_SPEED 必须是 Y_MOTOR_ENCODER_PERIOD_MS 的整数倍"
#endif

#pragma section all "cpu0_dsram"
static volatile int16 y_motor_count_5ms;
static volatile int16 y_motor_count_20ms;
static volatile int32 y_motor_total_count;
static int32 y_motor_window_count;
static uint8 y_motor_window_samples;
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
// 参数说明     duty            行进轮控制量
// 返回参数     void
// 使用示例     Y_Motor_SetDuty(g_motor_c);
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_SetDuty(int32 duty)
{
    uint32 pwm_duty;

    duty = Y_Motor_LimitDuty(duty);
    duty *= MOTOR_DIR_C;
    if (duty > 0)
        duty += DRIVE_DEAD_ZONE;
    else if (duty < 0)
        duty -= DRIVE_DEAD_ZONE;
    duty = func_limit_ab(duty, -Y_MOTOR_PWM_MAX_DUTY, Y_MOTOR_PWM_MAX_DUTY);

    pwm_set_duty(Y_MOTOR_PWM_PIN, 0);
    if (duty > 0)
    {
        gpio_set_level(Y_MOTOR_DIR_PIN, Y_MOTOR_FORWARD_DIR_LEVEL);
        pwm_duty = (uint32)duty;
    }
    else if (duty < 0)
    {
        gpio_set_level(Y_MOTOR_DIR_PIN,
                       (uint8)((Y_MOTOR_FORWARD_DIR_LEVEL == GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH));
        pwm_duty = (uint32)(-duty);
    }
    else
    {
        return;
    }

    pwm_set_duty(Y_MOTOR_PWM_PIN, pwm_duty);
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
