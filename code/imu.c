#include "imu.h"

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
    *body_x = sensor_x;
    *body_y = sensor_y;
    *body_z = sensor_z;
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

void imu_calibrate(void)
{
    const uint16 sample_count = 1000U;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16 i;

    for (i = 0U; i < sample_count; i++)
    {
        float sensor_x;
        float sensor_y;
        float sensor_z;
        float body_x;
        float body_y;
        float body_z;

        imu660rb_get_gyro();
        sensor_x = imu660rb_gyro_transition(imu660rb_gyro_x);
        sensor_y = imu660rb_gyro_transition(imu660rb_gyro_y);
        sensor_z = imu660rb_gyro_transition(imu660rb_gyro_z);
        Imu_SensorToBody(sensor_x, sensor_y, sensor_z,
                        &body_x, &body_y, &body_z);
        sum_x += body_x;
        sum_y += body_y;
        sum_z += body_z;
        system_delay_ms(1U);
    }

    gyro_bias_x = sum_x / (float)sample_count;
    gyro_bias_y = sum_y / (float)sample_count;
    gyro_bias_z = sum_z / (float)sample_count;
    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    gyro_sum_count = 0U;
}

void imu_update_gyro(void)
{
    float sensor_x;
    float sensor_y;
    float sensor_z;
    float body_x;
    float body_y;
    float body_z;

    imu660rb_get_gyro();
    sensor_x = imu660rb_gyro_transition(imu660rb_gyro_x);
    sensor_y = imu660rb_gyro_transition(imu660rb_gyro_y);
    sensor_z = imu660rb_gyro_transition(imu660rb_gyro_z);
    Imu_SensorToBody(sensor_x, sensor_y, sensor_z,
                    &body_x, &body_y, &body_z);

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
