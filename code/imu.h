#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_data_t;

extern volatile imu_data_t imu;

uint8 imu_init(void);
bool imu_calibrate(void);
void imu_update_gyro(void);
void imu_update_acc(void);
void imu_get_gyro_avg(float *gx, float *gy, float *gz);
float imu_get_angle_yaw(void);
float imu_get_angle_element(void);
void imu_clear_angle_element(void);
void imu_publish_attitude_yaw(float yaw_continuous, float yaw_delta);

#endif
