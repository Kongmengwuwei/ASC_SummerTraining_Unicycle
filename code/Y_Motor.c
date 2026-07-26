#include "Y_Motor.h"

#include <stddef.h>

#include "zf_common_headfile.h"

/*
 * 单路有刷行进电机接口：
 *
 *   P21.2 -> 方向控制 GPIO
 *   P21.3 -> 单路 PWM 输出
 *
 * P21.4 和 P21.5 不属于这一路电机，本模块不会初始化或占用它们。
 *
 * 注意：下面的“正向”只是软件定义。电机动力线、驱动板方向输入以及
 * 编码器安装方向都会影响最终正负号，首次测试必须架空行进轮并使用
 * 较小占空比确认电机方向与编码器方向是否一致。
 */
#define Y_MOTOR_DIR_PIN               (P21_2)
#define Y_MOTOR_PWM_PIN               (ATOM0_CH1_P21_3)
#define Y_MOTOR_FORWARD_DIR_LEVEL     (GPIO_LOW)
#define Y_MOTOR_REVERSE_DIR_LEVEL     (GPIO_HIGH)

/* 编码器使用 P33.7 脉冲 + P33.6 方向模式。 */
#define Y_MOTOR_ENCODER_INDEX         (TIM2_ENCODER)
#define Y_MOTOR_ENCODER_A_PIN         (TIM2_ENCODER_CH1_P33_7)
#define Y_MOTOR_ENCODER_B_PIN         (TIM2_ENCODER_CH2_P33_6)

#pragma section all "cpu0_dsram"
/* 这两个变量在 5 ms 中断中更新，因此放在 CPU0 的本地 RAM 中。 */
static volatile int16_t y_motor_count_5ms;
static volatile int32_t y_motor_total_count;
#pragma section all restore

/*
 * 将有符号占空比限制在逐飞 PWM 驱动允许的范围内。
 * 先限幅再取绝对值，可以避免直接对 INT32_MIN 取反造成溢出。
 */
static int32_t Y_Motor_LimitDuty(int32_t duty)
{
    if (duty > Y_MOTOR_PWM_MAX_DUTY)
    {
        return Y_MOTOR_PWM_MAX_DUTY;
    }

    if (duty < -Y_MOTOR_PWM_MAX_DUTY)
    {
        return -Y_MOTOR_PWM_MAX_DUTY;
    }

    return duty;
}

void Y_Motor_Init(void)
{
    y_motor_count_5ms = 0;
    y_motor_total_count = 0;

    /*
     * 方向脚先初始化为软件定义的正向，PWM 初始占空比为 0。
     * 因此完成初始化后，驱动板不会向电机输出有效功率。
     */
    gpio_init(Y_MOTOR_DIR_PIN,
              GPO,
              Y_MOTOR_FORWARD_DIR_LEVEL,
              GPO_PUSH_PULL);
    pwm_init(Y_MOTOR_PWM_PIN, Y_MOTOR_PWM_FREQUENCY_HZ, 0U);

    encoder_dir_init(Y_MOTOR_ENCODER_INDEX,
                     Y_MOTOR_ENCODER_A_PIN,
                     Y_MOTOR_ENCODER_B_PIN);
    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
}

void Y_Motor_SetDuty(int32_t duty)
{
    uint32_t pwm_duty;

    duty = Y_Motor_LimitDuty(duty);

    /*
     * 每次改变方向前先撤掉 PWM。
     * 这样从正转命令直接切换到反转命令时，不会在方向脚翻转期间
     * 仍保持原占空比输出。
     */
    pwm_set_duty(Y_MOTOR_PWM_PIN, 0U);

    if (duty > 0)
    {
        gpio_set_level(Y_MOTOR_DIR_PIN, Y_MOTOR_FORWARD_DIR_LEVEL);
        pwm_duty = (uint32_t)duty;
    }
    else if (duty < 0)
    {
        gpio_set_level(Y_MOTOR_DIR_PIN, Y_MOTOR_REVERSE_DIR_LEVEL);
        pwm_duty = (uint32_t)(-duty);
    }
    else
    {
        /* duty 为 0 时保持 PWM 关闭，无需改变方向脚。 */
        return;
    }

    pwm_set_duty(Y_MOTOR_PWM_PIN, pwm_duty);
}

void Y_Motor_Stop(void)
{
    /*
     * 单路 PWM 清零。停止后的机械表现由外接驱动板决定，
     * 本函数不假设它一定是滑行或主动制动。
     */
    pwm_set_duty(Y_MOTOR_PWM_PIN, 0U);
}

void Y_Motor_EncoderUpdate5ms(void)
{
    /*
     * 本函数由 CPU0 的 CCU60_CH0 每 5 ms 调用一次，只执行编码器
     * 读取、清零和累加，不在中断中进行显示、延时或控制参数计算。
     */
    int16_t count =
        (int16_t)encoder_get_count(Y_MOTOR_ENCODER_INDEX);

    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_count_5ms = count;
    y_motor_total_count += (int32_t)count;
}

void Y_Motor_EncoderClear(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    /*
     * 编码器数据会在中断中更新，清零时使用很短的临界区，避免
     * 硬件计数、最近周期计数和累计计数只清除了一部分。
     */
    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_count_5ms = 0;
    y_motor_total_count = 0;
    interrupt_global_enable(interrupt_state);
}

void Y_Motor_GetEncoder(y_motor_encoder_data_t *data)
{
    uint32 interrupt_state;

    if (data == NULL)
    {
        return;
    }

    /*
     * 两个成员必须来自同一次快照，所以读取时短暂关闭全局中断，
     * 防止 5 ms 编码器中断恰好在两个变量之间更新。
     */
    interrupt_state = interrupt_global_disable();
    data->count_5ms = y_motor_count_5ms;
    data->total_count = y_motor_total_count;
    interrupt_global_enable(interrupt_state);
}
