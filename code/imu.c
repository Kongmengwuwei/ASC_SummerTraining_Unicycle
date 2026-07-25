#include "imu.h"

#define IMU_CALIBRATION_WARMUP_MS (1000U)
#define IMU_CALIBRATION_DISCARD_SAMPLES (40U)
#define IMU_CALIBRATION_SAMPLE_COUNT (500U)
#define IMU_CALIBRATION_SAMPLE_PERIOD_MS (5U)
#define IMU_CALIBRATION_MAX_ATTEMPTS (3U)
#define IMU_CALIBRATION_MAX_RANGE_DPS (1.5f)
#define IMU_CALIBRATION_MAX_STDDEV_SQUARED (0.25f)

#pragma section all "cpu0_dsram"
volatile imu_data_t imu;

static float gyro_bias_x;
static float gyro_bias_y;
static float gyro_bias_z;
static float yaw_angle;
static float element_angle;
static float gyro_sum_x;
static float gyro_sum_y;
static float gyro_sum_z;
static uint8 gyro_sum_count;
#pragma section all restore

static void Imu_SensorToBody(float sensor_x,
                             float sensor_y,
                             float sensor_z,
                             float *body_x,
                             float *body_y,
                             float *body_z)
{
    /*
     * Vertical IMU660RB mounting:
     * sensor X points up, sensor Y points forward, sensor Z points left.
     * Body frame: X forward (roll), Y left (pitch), Z up (yaw).
     */
    *body_x = sensor_y;
    *body_y = sensor_z;
    *body_z = sensor_x;
}

static void Imu_ReadGyroBody(float *body_x,
                             float *body_y,
                             float *body_z)
{
    float sensor_x;
    float sensor_y;
    float sensor_z;

    imu660rb_get_gyro();
    sensor_x = imu660rb_gyro_transition(imu660rb_gyro_x);
    sensor_y = imu660rb_gyro_transition(imu660rb_gyro_y);
    sensor_z = imu660rb_gyro_transition(imu660rb_gyro_z);
    Imu_SensorToBody(sensor_x, sensor_y, sensor_z,
                     body_x, body_y, body_z);
}

static bool Imu_CalibrationAxisStable(float sum,
                                      float sum_squared,
                                      float minimum,
                                      float maximum)
{
    float inverse_count =
        1.0f / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    float mean = sum * inverse_count;
    float variance = sum_squared * inverse_count - mean * mean;

    if (variance < 0.0f)
    {
        variance = 0.0f;
    }

    return ((maximum - minimum) <= IMU_CALIBRATION_MAX_RANGE_DPS) &&
           (variance <= IMU_CALIBRATION_MAX_STDDEV_SQUARED);
}

uint8 imu_init(void)
{
    imu.acc_x = 0.0f;
    imu.acc_y = 0.0f;
    imu.acc_z = 0.0f;
    imu.gyro_x = 0.0f;
    imu.gyro_y = 0.0f;
    imu.gyro_z = 0.0f;
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    gyro_sum_count = 0U;
    yaw_angle = 0.0f;
    element_angle = 0.0f;
    return imu660rb_init();
}

bool imu_calibrate(void)
{
    uint16 warmup_samples =
        IMU_CALIBRATION_WARMUP_MS /
        IMU_CALIBRATION_SAMPLE_PERIOD_MS;
    uint8 attempt;
    uint16 i;

    for (i = 0U; i < warmup_samples; i++)
    {
        float body_x;
        float body_y;
        float body_z;

        Imu_ReadGyroBody(&body_x, &body_y, &body_z);
        system_delay_ms(IMU_CALIBRATION_SAMPLE_PERIOD_MS);
    }

    for (attempt = 0U; attempt < IMU_CALIBRATION_MAX_ATTEMPTS; attempt++)
    {
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_z = 0.0f;
        float sum_squared_x = 0.0f;
        float sum_squared_y = 0.0f;
        float sum_squared_z = 0.0f;
        float minimum_x = 0.0f;
        float minimum_y = 0.0f;
        float minimum_z = 0.0f;
        float maximum_x = 0.0f;
        float maximum_y = 0.0f;
        float maximum_z = 0.0f;

        for (i = 0U; i < IMU_CALIBRATION_DISCARD_SAMPLES; i++)
        {
            float body_x;
            float body_y;
            float body_z;

            Imu_ReadGyroBody(&body_x, &body_y, &body_z);
            system_delay_ms(IMU_CALIBRATION_SAMPLE_PERIOD_MS);
        }

        for (i = 0U; i < IMU_CALIBRATION_SAMPLE_COUNT; i++)
        {
            float body_x;
            float body_y;
            float body_z;

            Imu_ReadGyroBody(&body_x, &body_y, &body_z);

            if (i == 0U)
            {
                minimum_x = maximum_x = body_x;
                minimum_y = maximum_y = body_y;
                minimum_z = maximum_z = body_z;
            }
            else
            {
                if (body_x < minimum_x)
                {
                    minimum_x = body_x;
                }
                if (body_x > maximum_x)
                {
                    maximum_x = body_x;
                }
                if (body_y < minimum_y)
                {
                    minimum_y = body_y;
                }
                if (body_y > maximum_y)
                {
                    maximum_y = body_y;
                }
                if (body_z < minimum_z)
                {
                    minimum_z = body_z;
                }
                if (body_z > maximum_z)
                {
                    maximum_z = body_z;
                }
            }

            sum_x += body_x;
            sum_y += body_y;
            sum_z += body_z;
            sum_squared_x += body_x * body_x;
            sum_squared_y += body_y * body_y;
            sum_squared_z += body_z * body_z;
            system_delay_ms(IMU_CALIBRATION_SAMPLE_PERIOD_MS);
        }

        if (Imu_CalibrationAxisStable(sum_x, sum_squared_x,
                                      minimum_x, maximum_x) &&
            Imu_CalibrationAxisStable(sum_y, sum_squared_y,
                                      minimum_y, maximum_y) &&
            Imu_CalibrationAxisStable(sum_z, sum_squared_z,
                                      minimum_z, maximum_z))
        {
            float inverse_count =
                1.0f / (float)IMU_CALIBRATION_SAMPLE_COUNT;

            gyro_bias_x = sum_x * inverse_count;
            gyro_bias_y = sum_y * inverse_count;
            gyro_bias_z = sum_z * inverse_count;
            gyro_sum_x = 0.0f;
            gyro_sum_y = 0.0f;
            gyro_sum_z = 0.0f;
            gyro_sum_count = 0U;
            return true;
        }
    }

    return false;
}

void imu_update_gyro(void)
{
    float body_x;
    float body_y;
    float body_z;

    Imu_ReadGyroBody(&body_x, &body_y, &body_z);

    imu.gyro_x = body_x - gyro_bias_x;
    imu.gyro_y = body_y - gyro_bias_y;
    imu.gyro_z = body_z - gyro_bias_z;

    gyro_sum_x += imu.gyro_x;
    gyro_sum_y += imu.gyro_y;
    gyro_sum_z += imu.gyro_z;
    gyro_sum_count++;
}

void imu_update_acc(void)
{
    float sensor_x;
    float sensor_y;
    float sensor_z;
    float body_x;
    float body_y;
    float body_z;

    imu660rb_get_acc();
    sensor_x = imu660rb_acc_transition(imu660rb_acc_x);
    sensor_y = imu660rb_acc_transition(imu660rb_acc_y);
    sensor_z = imu660rb_acc_transition(imu660rb_acc_z);
    Imu_SensorToBody(sensor_x, sensor_y, sensor_z,
                    &body_x, &body_y, &body_z);
    imu.acc_x = body_x;
    imu.acc_y = body_y;
    imu.acc_z = body_z;
}

void imu_get_gyro_avg(float *gx, float *gy, float *gz)
{
    if ((gx == NULL) || (gy == NULL) || (gz == NULL))
    {
        return;
    }

    if (gyro_sum_count != 0U)
    {
        float inverse_count = 1.0f / (float)gyro_sum_count;
        *gx = gyro_sum_x * inverse_count;
        *gy = gyro_sum_y * inverse_count;
        *gz = gyro_sum_z * inverse_count;
    }
    else
    {
        *gx = imu.gyro_x;
        *gy = imu.gyro_y;
        *gz = imu.gyro_z;
    }

    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    gyro_sum_count = 0U;
}

void imu_publish_attitude_yaw(float yaw_continuous, float yaw_delta)
{
    yaw_angle = yaw_continuous;
    element_angle += yaw_delta;
}

float imu_get_angle_yaw(void)
{
    return yaw_angle;
}

float imu_get_angle_element(void)
{
    return element_angle;
}

void imu_clear_angle_element(void)
{
    element_angle = 0.0f;
}
