#include "Attitude.h"

#include <math.h>
#include <string.h>

#include "IfxStm.h"
#include "imu.h"
#include "zf_common_headfile.h"

#define ATT_DT (0.005f)
#define ATT_RAD_TO_DEG (57.29577951308232f)
#define ATT_DEG_TO_RAD (0.017453292519943f)

#define MAHONY_KP (2.5f)
#define MAHONY_KI (0.01f)
#define MAHONY_KP_MIN (0.5f)
#define MAHONY_KP_MAX (10.0f)
#define MAHONY_ACC_NORM_MIN (0.75f)
#define MAHONY_ACC_NORM_MAX (1.25f)
#define MAHONY_GYRO_BIAS_MAX (0.2f)
#define MAHONY_ADAPTIVE_KP_GAIN (0.5f)
#define GYRO_YAW_DEAD_ZONE_DPS (1.5f)

#define ATTITUDE_SAMPLES_PER_UPDATE \
    (ATTITUDE_UPDATE_PERIOD_MS / ATTITUDE_GYRO_SAMPLE_PERIOD_MS)

#pragma section all "cpu0_dsram"
volatile attitude_euler_t eulerAngle;
volatile attitude_rate_t attitudeRate;

static float q0;
static float q1;
static float q2;
static float q3;
static float mahony_bias_x;
static float mahony_bias_y;
static float mahony_bias_z;
static float yaw_previous_wrapped;
static float yaw_continuous_raw;
static float yaw_zero_offset;
static bool yaw_has_previous;

static volatile uint32 attitude_sequence;
static uint8 attitude_sample_count;

static volatile uint32 attitude_last_ticks;
static volatile uint32 attitude_max_ticks;
static volatile uint32 attitude_overrun_count;
static uint32 attitude_window_ticks;
static uint32 attitude_stm_frequency;
static uint32 attitude_period_ticks;
#pragma section all restore

static float Attitude_Clamp(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static float Attitude_Normalize180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void Attitude_QuaternionNormalize(void)
{
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (norm < 1.0e-10f)
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }
    else
    {
        float inverse_norm = 1.0f / norm;
        q0 *= inverse_norm;
        q1 *= inverse_norm;
        q2 *= inverse_norm;
        q3 *= inverse_norm;
    }
}

static float Attitude_QuaternionYaw(void)
{
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
           ATT_RAD_TO_DEG;
}

static void Attitude_Publish(float roll, float pitch, float yaw)
{
    attitude_sequence++;
    __dsync();
    eulerAngle.roll = roll;
    eulerAngle.pitch = pitch;
    eulerAngle.yaw = yaw;
    eulerAngle.update_count++;
    eulerAngle.ready = true;
    eulerAngle.imu_error = false;
    __dsync();
    attitude_sequence++;
}

static void Attitude_MahonyUpdate(float gyro_x_dps,
                                  float gyro_y_dps,
                                  float gyro_z_dps)
{
    float ax = imu.acc_x;
    float ay = imu.acc_y;
    float az = imu.acc_z;
    float gx = gyro_x_dps * ATT_DEG_TO_RAD;
    float gy = gyro_y_dps * ATT_DEG_TO_RAD;
    float gz = gyro_z_dps * ATT_DEG_TO_RAD;
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;
    float kp = MAHONY_KP;
    bool acc_valid =
        (acc_norm >= MAHONY_ACC_NORM_MIN) &&
        (acc_norm <= MAHONY_ACC_NORM_MAX);

    if (acc_norm < 1.0e-6f)
    {
        acc_valid = false;
    }
    else
    {
        float inverse_norm = 1.0f / acc_norm;
        ax *= inverse_norm;
        ay *= inverse_norm;
        az *= inverse_norm;
    }

    if (acc_valid)
    {
        float deviation = fabsf(acc_norm - 1.0f);
        float scale =
            1.0f - Attitude_Clamp(deviation * MAHONY_ADAPTIVE_KP_GAIN,
                                  0.0f, 0.9f);
        float predicted_x = 2.0f * (q1 * q3 - q0 * q2);
        float predicted_y = 2.0f * (q0 * q1 + q2 * q3);
        float predicted_z = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

        kp = Attitude_Clamp(MAHONY_KP * scale,
                            MAHONY_KP_MIN, MAHONY_KP_MAX);
        ex = ay * predicted_z - az * predicted_y;
        ey = az * predicted_x - ax * predicted_z;
        ez = ax * predicted_y - ay * predicted_x;

        mahony_bias_x = Attitude_Clamp(
            mahony_bias_x + MAHONY_KI * ex * ATT_DT,
            -MAHONY_GYRO_BIAS_MAX, MAHONY_GYRO_BIAS_MAX);
        mahony_bias_y = Attitude_Clamp(
            mahony_bias_y + MAHONY_KI * ey * ATT_DT,
            -MAHONY_GYRO_BIAS_MAX, MAHONY_GYRO_BIAS_MAX);
        mahony_bias_z = Attitude_Clamp(
            mahony_bias_z + MAHONY_KI * ez * ATT_DT,
            -MAHONY_GYRO_BIAS_MAX, MAHONY_GYRO_BIAS_MAX);
    }
    else
    {
        kp = MAHONY_KP_MIN;
    }

    gx = gx + kp * ex - mahony_bias_x;
    gy = gy + kp * ey - mahony_bias_y;
    gz = gz + kp * ez - mahony_bias_z;

    if (fabsf(gz) * ATT_RAD_TO_DEG < GYRO_YAW_DEAD_ZONE_DPS)
    {
        gz = 0.0f;
    }

    {
        float old_q0 = q0;
        float old_q1 = q1;
        float old_q2 = q2;
        float old_q3 = q3;
        float half_dt = 0.5f * ATT_DT;

        q0 += half_dt * (-old_q1 * gx - old_q2 * gy - old_q3 * gz);
        q1 += half_dt * (old_q0 * gx + old_q2 * gz - old_q3 * gy);
        q2 += half_dt * (old_q0 * gy - old_q1 * gz + old_q3 * gx);
        q3 += half_dt * (old_q0 * gz + old_q1 * gy - old_q2 * gx);
    }

    Attitude_QuaternionNormalize();

    {
        float roll;
        float pitch;
        float yaw_wrapped;
        float yaw_delta = 0.0f;
        float pitch_sine = 2.0f * (q0 * q2 - q1 * q3);

        roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                      1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
               ATT_RAD_TO_DEG;
        pitch = asinf(Attitude_Clamp(pitch_sine, -1.0f, 1.0f)) *
                ATT_RAD_TO_DEG;
        yaw_wrapped = Attitude_QuaternionYaw();

        if (!yaw_has_previous)
        {
            yaw_previous_wrapped = yaw_wrapped;
            yaw_continuous_raw = yaw_wrapped;
            yaw_has_previous = true;
        }
        else
        {
            yaw_delta =
                Attitude_Normalize180(yaw_wrapped - yaw_previous_wrapped);
            yaw_continuous_raw += yaw_delta;
            yaw_previous_wrapped = yaw_wrapped;
        }

        imu_publish_attitude_yaw(yaw_continuous_raw - yaw_zero_offset,
                                 yaw_delta);
        Attitude_Publish(roll, pitch,
                         yaw_continuous_raw - yaw_zero_offset);
    }
}

static void Attitude_ResetMahony(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    mahony_bias_x = 0.0f;
    mahony_bias_y = 0.0f;
    mahony_bias_z = 0.0f;
    yaw_previous_wrapped = 0.0f;
    yaw_continuous_raw = 0.0f;
    yaw_zero_offset = 0.0f;
    yaw_has_previous = false;
    imu_publish_attitude_yaw(0.0f, 0.0f);
}

bool Attitude_Init(void)
{
    memset((void *)&eulerAngle, 0, sizeof(attitude_euler_t));
    memset((void *)&attitudeRate, 0, sizeof(attitude_rate_t));

    attitude_sequence = 0U;
    attitude_sample_count = 0U;
    attitude_last_ticks = 0U;
    attitude_max_ticks = 0U;
    attitude_overrun_count = 0U;
    attitude_window_ticks = 0U;

    attitude_stm_frequency =
        (uint32)IfxStm_getFrequency(IfxStm_getAddress(IfxStm_Index_0));
    attitude_period_ticks =
        (uint32)(((uint64)attitude_stm_frequency *
                  ATTITUDE_UPDATE_PERIOD_MS) /
                 1000U);

    if (imu_init() != 0U)
    {
        eulerAngle.imu_error = true;
        return false;
    }

    if (!imu_calibrate())
    {
        eulerAngle.imu_error = true;
        return false;
    }
    Attitude_ResetMahony();

    imu_update_gyro();
    attitudeRate.roll_rate = imu.gyro_x;
    attitudeRate.pitch_rate = imu.gyro_y;
    attitudeRate.yaw_rate = imu.gyro_z;
    Attitude_Calculate();
    return true;
}

void Attitude_Calculate(void)
{
    float gyro_x_average;
    float gyro_y_average;
    float gyro_z_average;

    imu_update_acc();
    imu_get_gyro_avg(&gyro_x_average,
                     &gyro_y_average,
                     &gyro_z_average);
    Attitude_MahonyUpdate(gyro_x_average,
                          gyro_y_average,
                          gyro_z_average);
}

void Attitude_Timer_1ms_ISR(void)
{
    Ifx_STM *stm = IfxStm_getAddress(IfxStm_Index_0);
    uint32 start_ticks = IfxStm_getLower(stm);
    uint32 elapsed_ticks;
    bool update_complete = false;

    imu_update_gyro();
    attitudeRate.roll_rate = imu.gyro_x;
    attitudeRate.pitch_rate = imu.gyro_y;
    attitudeRate.yaw_rate = imu.gyro_z;

    attitude_sample_count++;
    if (attitude_sample_count >= ATTITUDE_SAMPLES_PER_UPDATE)
    {
        attitude_sample_count = 0U;
        Attitude_Calculate();
        update_complete = true;
    }

    elapsed_ticks = IfxStm_getLower(stm) - start_ticks;
    attitude_window_ticks += elapsed_ticks;

    if (update_complete)
    {
        attitude_last_ticks = attitude_window_ticks;
        if (attitude_window_ticks > attitude_max_ticks)
        {
            attitude_max_ticks = attitude_window_ticks;
        }
        if ((attitude_period_ticks != 0U) &&
            (attitude_window_ticks >= attitude_period_ticks))
        {
            attitude_overrun_count++;
        }
        attitude_window_ticks = 0U;
    }
}

bool Attitude_GetEuler(attitude_euler_t *snapshot)
{
    uint32 sequence_before;
    uint32 sequence_after;
    uint8 attempts;

    if (snapshot == NULL)
    {
        return false;
    }

    for (attempts = 0U; attempts < 3U; attempts++)
    {
        sequence_before = attitude_sequence;
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }

        __dsync();
        snapshot->roll = eulerAngle.roll;
        snapshot->pitch = eulerAngle.pitch;
        snapshot->yaw = eulerAngle.yaw;
        snapshot->update_count = eulerAngle.update_count;
        snapshot->ready = eulerAngle.ready;
        snapshot->imu_error = eulerAngle.imu_error;
        __dsync();

        sequence_after = attitude_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U))
        {
            return snapshot->ready && !snapshot->imu_error;
        }
    }
    return false;
}

bool Attitude_GetPerformance(attitude_performance_t *performance)
{
    uint32 last_ticks;
    uint32 max_ticks;

    if ((performance == NULL) || (attitude_stm_frequency == 0U) ||
        (attitude_period_ticks == 0U))
    {
        return false;
    }

    last_ticks = attitude_last_ticks;
    max_ticks = attitude_max_ticks;
    performance->last_time_us =
        (uint32)(((uint64)last_ticks * 1000000U) /
                 attitude_stm_frequency);
    performance->max_time_us =
        (uint32)(((uint64)max_ticks * 1000000U) /
                 attitude_stm_frequency);
    performance->cpu_load_percent =
        ((float)last_ticks * 100.0f) / (float)attitude_period_ticks;
    performance->max_cpu_load_percent =
        ((float)max_ticks * 100.0f) / (float)attitude_period_ticks;
    performance->overrun_count = attitude_overrun_count;
    return true;
}

void Attitude_YawZero(void)
{
    yaw_zero_offset = yaw_continuous_raw;
    eulerAngle.yaw = 0.0f;
    imu_publish_attitude_yaw(0.0f, 0.0f);
}

void Attitude_YawSet(float value_degrees)
{
    yaw_zero_offset = yaw_continuous_raw - value_degrees;
    eulerAngle.yaw = value_degrees;
    imu_publish_attitude_yaw(value_degrees, 0.0f);
}
