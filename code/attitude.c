#include "attitude.h"
#include "imu.h"
#include "board_config.h"
#include "pid.h"                
#include <math.h>

#define ATT_RAD_TO_DEG                 (57.29577951308232f)
#define ATT_DEG_TO_RAD                 (0.017453292519943f)


#define PY_MAHONY_KP                   (2.5f)
#define PY_MAHONY_KI                   (0.01f)
#define PY_MAHONY_KP_MIN               (0.5f)
#define PY_MAHONY_KP_MAX               (10.0f)
#define PY_MAHONY_ACC_NORM_MIN         (0.75f)
#define PY_MAHONY_ACC_NORM_MAX         (1.25f)
#define PY_MAHONY_GYRO_BIAS_MAX        (0.2f)
#define PY_MAHONY_ADAPTIVE_KP_GAIN     (0.5f)
#define PY_GYRO_YAW_DEAD_ZONE_DPS      (1.5f)


#define ATT_BOOST_TICKS                (400u)      
#define ATT_RAMP_TICKS                 (200u)      
#define ATT_BOOST_KP                   (8.0f)      
#define ATT_BOOST_KP_MIN               (4.0f)      

attitude_t att;
att_diag_t att_diag;         

// Mahony 四元数状态
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static float yaw_previous_wrapped = 0.0f;
static float yaw_continuous_raw = 0.0f;
static float yaw_zero_offset = 0.0f;
static uint8 yaw_has_previous = 0;

static uint16 boost_ticks = 0;      
static uint8  quat_diverged = 0;    

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将角度限制到 (-180, 180] 范围
// 参数说明     angle 输入角度，单位 deg
// 返回参数     float           限制后的角度，单位 deg
// 使用示例     float angle = attitude_normalize_180(yaw);
//-------------------------------------------------------------------------------------------------------------------
static float attitude_normalize_180(float angle)
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     归一化 Mahony 四元数并检测发散
// 参数说明     void
// 返回参数     void
// 使用示例     quaternion_normalize();
//-------------------------------------------------------------------------------------------------------------------
static void quaternion_normalize(void)
{
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (norm < 1.0e-10f || norm != norm)
    {
        
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        quat_diverged = 1;
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前四元数转换为航向角
// 参数说明     void
// 返回参数     float           航向角，单位 deg
// 使用示例     float yaw = quaternion_to_yaw();
//-------------------------------------------------------------------------------------------------------------------
static float quaternion_to_yaw(void)
{
    return atan2f(
        2.0f * (q0 * q3 + q1 * q2),
        1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATT_RAD_TO_DEG;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算姿态冷启动收敛权重
// 参数说明     void
// 返回参数     float           收敛权重，范围 0 到 1
// 使用示例     float weight = boost_weight();
//-------------------------------------------------------------------------------------------------------------------
static float boost_weight(void)
{
    if (boost_ticks < ATT_BOOST_TICKS)
        return 1.0f;
    if (boost_ticks >= (ATT_BOOST_TICKS + ATT_RAMP_TICKS))
        return 0.0f;
    return 1.0f - ((float)(boost_ticks - ATT_BOOST_TICKS) / (float)ATT_RAMP_TICKS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一次六轴 Mahony 四元数姿态更新
// 参数说明     gyro_x_dps X 轴角速度；gyro_y_dps Y 轴角速度；gyro_z_dps Z 轴角速度
// 返回参数     void
// 使用示例     mahony_update(gx, gy, gz);
//-------------------------------------------------------------------------------------------------------------------
static void mahony_update(float gyro_x_dps,
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
    float kp;
    uint8 acc_valid;

    
    float w         = boost_weight();
    float kp_base   = PY_MAHONY_KP     + w * (ATT_BOOST_KP     - PY_MAHONY_KP);
    float kp_min    = PY_MAHONY_KP_MIN + w * (ATT_BOOST_KP_MIN - PY_MAHONY_KP_MIN);

    if (boost_ticks < (ATT_BOOST_TICKS + ATT_RAMP_TICKS))
        ++boost_ticks;

    acc_valid = (uint8)(
        (acc_norm >= PY_MAHONY_ACC_NORM_MIN) &&
        (acc_norm <= PY_MAHONY_ACC_NORM_MAX));
    att_diag.acc_norm  = acc_norm;
    att_diag.acc_valid = acc_valid;

    if (acc_norm < 1.0e-6f)
    {
        acc_valid = 0;
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
        float scale = 1.0f - constrain_float(
            deviation * PY_MAHONY_ADAPTIVE_KP_GAIN,
            0.0f,
            0.9f);

        float predicted_x = 2.0f * (q1 * q3 - q0 * q2);
        float predicted_y = 2.0f * (q0 * q1 + q2 * q3);
        float predicted_z = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

        kp = constrain_float(
            kp_base * scale,
            kp_min,
            PY_MAHONY_KP_MAX);

        ex = ay * predicted_z - az * predicted_y;
        ey = az * predicted_x - ax * predicted_z;
        ez = ax * predicted_y - ay * predicted_x;

        if (PY_MAHONY_KI > 0.0f)
        {
            gyro_bias_x = constrain_float(
                gyro_bias_x - PY_MAHONY_KI * ex * ATT_DT,
                -PY_MAHONY_GYRO_BIAS_MAX,
                PY_MAHONY_GYRO_BIAS_MAX);
            gyro_bias_y = constrain_float(
                gyro_bias_y - PY_MAHONY_KI * ey * ATT_DT,
                -PY_MAHONY_GYRO_BIAS_MAX,
                PY_MAHONY_GYRO_BIAS_MAX);
            gyro_bias_z = constrain_float(
                gyro_bias_z - PY_MAHONY_KI * ez * ATT_DT,
                -PY_MAHONY_GYRO_BIAS_MAX,
                PY_MAHONY_GYRO_BIAS_MAX);
        }
    }
    else
    {
        kp = kp_min;
    }

    att_diag.bias_z = gyro_bias_z;

    // Kp 项拉当前误差，减去零偏估计扣掉常值漂移，两项同为负反馈
    gx = gx + kp * ex - gyro_bias_x;
    gy = gy + kp * ey - gyro_bias_y;
    gz = gz + kp * ez - gyro_bias_z;

    
    if (fabsf(gz) * ATT_RAD_TO_DEG < PY_GYRO_YAW_DEAD_ZONE_DPS)
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
        q1 += half_dt * ( old_q0 * gx + old_q2 * gz - old_q3 * gy);
        q2 += half_dt * ( old_q0 * gy - old_q1 * gz + old_q3 * gx);
        q3 += half_dt * ( old_q0 * gz + old_q1 * gy - old_q2 * gx);
    }

    quaternion_normalize();

    att.roll = atan2f(
        2.0f * (q0 * q1 + q2 * q3),
        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATT_RAD_TO_DEG;

    {
        float pitch_sine = 2.0f * (q0 * q2 - q1 * q3);
        pitch_sine = constrain_float(pitch_sine, -1.0f, 1.0f);
        att.pitch = asinf(pitch_sine) * ATT_RAD_TO_DEG;
    }

    {
        float yaw_wrapped = quaternion_to_yaw();
        float yaw_delta = 0.0f;

        if (!yaw_has_previous)
        {
            yaw_previous_wrapped = yaw_wrapped;
            yaw_continuous_raw = yaw_wrapped;
            yaw_has_previous = 1;
        }
        else
        {
            yaw_delta = attitude_normalize_180(
                yaw_wrapped - yaw_previous_wrapped);
            yaw_continuous_raw += yaw_delta;
            yaw_previous_wrapped = yaw_wrapped;
        }

        att.yaw         = yaw_continuous_raw - yaw_zero_offset;
        att.yaw_wrapped = attitude_normalize_180(att.yaw);   
        imu_publish_attitude_yaw(att.yaw, yaw_delta);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据静止重力方向对齐初始四元数
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_align_from_gravity();
//-------------------------------------------------------------------------------------------------------------------
static void attitude_align_from_gravity(void)
{
    float ax, ay, az;
    float norm;

    if (!imu_get_static_acc(&ax, &ay, &az))
        return;                                 

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.5f || norm > 1.5f)
        return;                                 

    {
        float inv    = 1.0f / norm;
        float nx     = ax * inv;
        float ny     = ay * inv;
        float nz     = az * inv;
        float roll0  = atan2f(ny, nz);
        float pitch0 = atan2f(-nx, sqrtf(ny * ny + nz * nz));
        float cr     = cosf(roll0 * 0.5f);
        float sr     = sinf(roll0 * 0.5f);
        float cp     = cosf(pitch0 * 0.5f);
        float sp     = sinf(pitch0 * 0.5f);

        q0 =  cr * cp;
        q1 =  sr * cp;
        q2 =  cr * sp;
        q3 = -sr * sp;
        quaternion_normalize();

        
        att.roll  = roll0  * ATT_RAD_TO_DEG;
        att.pitch = pitch0 * ATT_RAD_TO_DEG;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 Mahony 四元数姿态解算状态
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_init();
//-------------------------------------------------------------------------------------------------------------------
void attitude_init(void)
{
    att.roll = 0.0f;
    att.pitch = 0.0f;
    att.yaw = 0.0f;
    att.yaw_wrapped = 0.0f;
    att.roll_rate = 0.0f;
    att.pitch_rate = 0.0f;
    att.yaw_rate = 0.0f;

    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;

    yaw_previous_wrapped = 0.0f;
    yaw_continuous_raw = 0.0f;
    yaw_zero_offset = 0.0f;
    yaw_has_previous = 0;

    boost_ticks   = 0;                  
    quat_diverged = 0;

    attitude_align_from_gravity();      

    imu_publish_attitude_yaw(0.0f, 0.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查姿态解算冷启动收敛状态
// 参数说明     void
// 返回参数     uint8           1 表示收敛完成，0 表示尚未完成
// 使用示例     uint8 ready = attitude_converged();
//-------------------------------------------------------------------------------------------------------------------
uint8 attitude_converged(void)
{
    return (uint8)(boost_ticks >= (ATT_BOOST_TICKS + ATT_RAMP_TICKS));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查四元数是否发生过发散
// 参数说明     void
// 返回参数     uint8           1 表示发生过发散，0 表示正常
// 使用示例     uint8 fault = attitude_diverged();
//-------------------------------------------------------------------------------------------------------------------
uint8 attitude_diverged(void)
{
    return quat_diverged;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使用 IMU 原始数据更新车体系角速度
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_update_rate();
//-------------------------------------------------------------------------------------------------------------------
void attitude_update_rate(void)
{
    att.roll_rate = imu.gyro_x;
    att.pitch_rate = imu.gyro_y;
    att.yaw_rate = imu.gyro_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新 Mahony 四元数和欧拉角
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_update();
//-------------------------------------------------------------------------------------------------------------------
void attitude_update(void)
{
    float gyro_x_average;
    float gyro_y_average;
    float gyro_z_average;

    imu_update_acc();

    
    if (imu_link_lost())
    {
        imu_get_gyro_avg(&gyro_x_average, &gyro_y_average, &gyro_z_average);  
        return;
    }

    imu_get_gyro_avg(
        &gyro_x_average,
        &gyro_y_average,
        &gyro_z_average);

    mahony_update(
        gyro_x_average,
        gyro_y_average,
        gyro_z_average);
}
