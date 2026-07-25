#ifndef CODE_Y_MOTOR_H_
#define CODE_Y_MOTOR_H_

#include <stdint.h>

#define Y_MOTOR_ENCODER_PERIOD_MS (5U)

typedef struct
{
    /* Signed A-channel pulses collected during the latest 5 ms period. */
    int16_t count_5ms;
    int32_t total_count;
} y_motor_encoder_data_t;

void Y_Motor_Init(void);
void Y_Motor_EncoderUpdate5ms(void);
void Y_Motor_EncoderClear(void);
void Y_Motor_GetEncoder(y_motor_encoder_data_t *data);

#endif
