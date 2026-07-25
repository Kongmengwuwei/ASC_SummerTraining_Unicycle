#include "Y_Motor.h"

#include <stddef.h>

#include "zf_common_headfile.h"

#define Y_MOTOR_ENCODER_INDEX (TIM2_ENCODER)
#define Y_MOTOR_ENCODER_A_PIN (TIM2_ENCODER_CH1_P33_7)
#define Y_MOTOR_ENCODER_B_PIN (TIM2_ENCODER_CH2_P33_6)

#pragma section all "cpu0_dsram"
static volatile int16_t y_motor_count_5ms;
static volatile int32_t y_motor_total_count;
#pragma section all restore

void Y_Motor_Init(void)
{
    y_motor_count_5ms = 0;
    y_motor_total_count = 0;

    encoder_dir_init(Y_MOTOR_ENCODER_INDEX,
                     Y_MOTOR_ENCODER_A_PIN,
                     Y_MOTOR_ENCODER_B_PIN);
    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
}

void Y_Motor_EncoderUpdate5ms(void)
{
    int16_t count =
        (int16_t)encoder_get_count(Y_MOTOR_ENCODER_INDEX);

    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_count_5ms = count;
    y_motor_total_count += (int32_t)count;
}

void Y_Motor_EncoderClear(void)
{
    uint32 interrupt_state = interrupt_global_disable();

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

    interrupt_state = interrupt_global_disable();
    data->count_5ms = y_motor_count_5ms;
    data->total_count = y_motor_total_count;
    interrupt_global_enable(interrupt_state);
}
