#ifndef CODE_ATTITUDE_H_
#define CODE_ATTITUDE_H_

#include <stdbool.h>
#include <stdint.h>

#define ATTITUDE_GYRO_SAMPLE_PERIOD_MS (1U)
#define ATTITUDE_UPDATE_PERIOD_MS (5U)

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    uint32_t update_count;
    bool ready;
    bool imu_error;
} attitude_euler_t;

typedef struct
{
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} attitude_rate_t;

typedef struct
{
    uint32_t last_time_us;
    uint32_t max_time_us;
    float cpu_load_percent;
    float max_cpu_load_percent;
    uint32_t overrun_count;
} attitude_performance_t;

extern volatile attitude_euler_t eulerAngle;
extern volatile attitude_rate_t attitudeRate;

bool Attitude_Init(void);
void Attitude_Calculate(void);
void Attitude_Timer_1ms_ISR(void);
bool Attitude_GetEuler(attitude_euler_t *snapshot);
bool Attitude_GetPerformance(attitude_performance_t *performance);
void Attitude_YawZero(void);
void Attitude_YawSet(float value_degrees);

#endif
