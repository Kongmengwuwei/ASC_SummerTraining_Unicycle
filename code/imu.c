#include "imu.h"
#include "board_config.h"

imu_data_t imu;

static float gyro_bias_x, gyro_bias_y, gyro_bias_z;
static float gyro_sum_x, gyro_sum_y, gyro_sum_z;
static uint8 gyro_sum_count;

static float yaw_angle;
// static float element_angle;          // 元素状态机启用后恢复

static float static_acc_x = 0.0f;
static float static_acc_y = 0.0f;
static float static_acc_z = 1.0f;
static imu_calib_state_t calib_state = IMU_CALIB_NONE;

static uint16 acc_zero_run;
static uint16 gyro_zero_run;
static uint8  link_lost;


#define IMU_SETTLE_MS           (150)


#define IMU_ACC_ZERO_LIMIT      (20)
#define IMU_GYRO_ZERO_LIMIT     (100)
#define IMU_CALIB_SAMPLES       (600)
#define IMU_CALIB_RETRY         (3)
#define IMU_CALIB_MOVE_DPS      (3.0f)
#define IMU_CALIB_MOVE_G        (0.15f)


#define IMU660RB_CTRL8_XL       (0x17)
#define IMU_ODR_XL_416HZ        (0x60)
#define IMU_ODR_G_1667HZ        (0x80)
#define IMU_LPF2_XL_EN          (0x02)


#define IMU_CTRL6_C_FTYPE       (0x01)


#define IMU_CTRL8_XL_HPCF       (0x20)


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RB 传感器坐标转换为车体坐标
// 参数说明     sx/sy/sz 传感器轴输入；bx/by/bz 车体轴输出
// 返回参数     void
// 使用示例     imu_sensor_to_body(sx, sy, sz, &bx, &by, &bz);
//-------------------------------------------------------------------------------------------------------------------
static void imu_sensor_to_body(float sx, float sy, float sz,
                               float *bx, float *by, float *bz)
{
    *bx = sy;
    *by = sz;
    *bz = sx;
}

#if (IMU660RB_USE_SOFT_IIC == 0)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入 IMU660RB 寄存器
// 参数说明     reg 寄存器地址；data 写入数据
// 返回参数     void
// 使用示例     imu_write_reg(IMU660RB_CTRL1_XL, value);
//-------------------------------------------------------------------------------------------------------------------
static void imu_write_reg(uint8 reg, uint8 data)
{
    IMU660RB_CS(0);
    spi_write_8bit_register(IMU660RB_SPI, (uint8)(reg | IMU660RB_SPI_W), data);
    IMU660RB_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU660RB 寄存器
// 参数说明     reg 寄存器地址
// 返回参数     uint8           寄存器数据
// 使用示例     uint8 value = imu_read_reg(IMU660RB_CTRL1_XL);
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu_read_reg(uint8 reg)
{
    uint8 data;
    IMU660RB_CS(0);
    data = spi_read_8bit_register(IMU660RB_SPI, (uint8)(reg | IMU660RB_SPI_R));
    IMU660RB_CS(1);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     配置 IMU660RB 采样率和数字滤波器
// 参数说明     void
// 返回参数     uint8           0 表示配置成功，1 表示回读失败
// 使用示例     uint8 state = imu_tune_sensor();
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu_tune_sensor(void)
{
    uint8 xl = (uint8)((imu_read_reg(IMU660RB_CTRL1_XL) & 0x0Cu) | IMU_ODR_XL_416HZ | IMU_LPF2_XL_EN);
    uint8 g  = (uint8)((imu_read_reg(IMU660RB_CTRL2_G)  & 0x0Fu) | IMU_ODR_G_1667HZ);

    imu_write_reg(IMU660RB_CTRL1_XL, xl);
    imu_write_reg(IMU660RB_CTRL8_XL, IMU_CTRL8_XL_HPCF);
    imu_write_reg(IMU660RB_CTRL2_G,  g);
    imu_write_reg(IMU660RB_CTRL6_C,  IMU_CTRL6_C_FTYPE);
    system_delay_ms(5);

    return (uint8)(imu_read_reg(IMU660RB_CTRL1_XL) != xl ||
                   imu_read_reg(IMU660RB_CTRL2_G)  != g);
}

#else
#error "IMU660RB 必须用硬件 SPI: 软件 IIC 下库不暴露 IIC 句柄, ODR 只能停在 52/208Hz, 1kHz 控制环会一直读到重复样本"
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取车体系三轴角速度
// 参数说明     bx 角速度 X 输出；by 角速度 Y 输出；bz 角速度 Z 输出
// 返回参数     void
// 使用示例     imu_read_gyro_body(&gx, &gy, &gz);
//-------------------------------------------------------------------------------------------------------------------
static void imu_read_gyro_body(float *bx, float *by, float *bz)
{
    imu660rb_get_gyro();
    imu_sensor_to_body(imu660rb_gyro_transition(imu660rb_gyro_x),
                       imu660rb_gyro_transition(imu660rb_gyro_y),
                       imu660rb_gyro_transition(imu660rb_gyro_z), bx, by, bz);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取车体系三轴加速度
// 参数说明     bx 加速度 X 输出；by 加速度 Y 输出；bz 加速度 Z 输出
// 返回参数     void
// 使用示例     imu_read_acc_body(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
static void imu_read_acc_body(float *bx, float *by, float *bz)
{
    imu660rb_get_acc();
    imu_sensor_to_body(imu660rb_acc_transition(imu660rb_acc_x),
                       imu660rb_acc_transition(imu660rb_acc_y),
                       imu660rb_acc_transition(imu660rb_acc_z), bx, by, bz);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RB 并等待滤波器稳定
// 参数说明     void
// 返回参数     uint8           0 表示成功，非 0 表示失败
// 使用示例     uint8 state = imu_init();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_init(void)
{
    uint8 state;

    acc_zero_run  = 0;
    gyro_zero_run = 0;
    link_lost     = 0;

    state = imu660rb_init();
    if (state != 0)
        return state;

    if (imu_tune_sensor() != 0)
        return 1;


    system_delay_ms(IMU_SETTLE_MS);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     静止标定陀螺仪零偏和重力方向
// 参数说明     void
// 返回参数     void
// 使用示例     imu_calibrate();
//-------------------------------------------------------------------------------------------------------------------
void imu_calibrate(void)
{
    uint8 attempt;

    for (attempt = 0; attempt < IMU_CALIB_RETRY; ++attempt)
    {
        float g_sum[3] = {0.0f, 0.0f, 0.0f};
        float a_sum[3] = {0.0f, 0.0f, 0.0f};
        float g_min[3] = {1e30f, 1e30f, 1e30f}, g_max[3] = {-1e30f, -1e30f, -1e30f};
        float a_min[3] = {1e30f, 1e30f, 1e30f}, a_max[3] = {-1e30f, -1e30f, -1e30f};
        uint8 moved = 0;
        uint16 i;
        uint8 k;

        for (i = 0; i < IMU_CALIB_SAMPLES; ++i)
        {
            float g[3], a[3];

            imu_read_gyro_body(&g[0], &g[1], &g[2]);
            imu_read_acc_body(&a[0], &a[1], &a[2]);

            for (k = 0; k < 3u; ++k)
            {
                g_sum[k] += g[k];
                a_sum[k] += a[k];
                if (g[k] < g_min[k]) g_min[k] = g[k];
                if (g[k] > g_max[k]) g_max[k] = g[k];
                if (a[k] < a_min[k]) a_min[k] = a[k];
                if (a[k] > a_max[k]) a_max[k] = a[k];
            }
            system_delay_ms(1);
        }

        {
            float inv = 1.0f / (float)IMU_CALIB_SAMPLES;
            gyro_bias_x = g_sum[0] * inv;
            gyro_bias_y = g_sum[1] * inv;
            gyro_bias_z = g_sum[2] * inv;
            static_acc_x = a_sum[0] * inv;
            static_acc_y = a_sum[1] * inv;
            static_acc_z = a_sum[2] * inv;
        }

        for (k = 0; k < 3u; ++k)
            if ((g_max[k] - g_min[k]) > IMU_CALIB_MOVE_DPS ||
                (a_max[k] - a_min[k]) > IMU_CALIB_MOVE_G)
                moved = 1;

        if (!moved)
        {
            calib_state = IMU_CALIB_OK;
            break;
        }
        calib_state = IMU_CALIB_MOVED;
    }

    gyro_sum_x = gyro_sum_y = gyro_sum_z = 0.0f;
    gyro_sum_count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU 标定状态
// 参数说明     void
// 返回参数     imu_calib_state_t 当前标定状态
// 使用示例     imu_calib_state_t state = imu_calib_state();
//-------------------------------------------------------------------------------------------------------------------
imu_calib_state_t imu_calib_state(void)
{
    return calib_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取标定期间采集的车体系重力方向
// 参数说明     ax 加速度 X 输出；ay 加速度 Y 输出；az 加速度 Z 输出
// 返回参数     uint8           1 表示数据有效，0 表示尚未标定
// 使用示例     uint8 valid = imu_get_static_acc(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_get_static_acc(float *ax, float *ay, float *az)
{
    *ax = static_acc_x;
    *ay = static_acc_y;
    *az = static_acc_z;
    return (uint8)(calib_state != IMU_CALIB_NONE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新车体系角速度并累计平均值
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update_gyro();
//-------------------------------------------------------------------------------------------------------------------
void imu_update_gyro(void)
{
    float bx, by, bz;

    imu_read_gyro_body(&bx, &by, &bz);

    if (imu660rb_gyro_x == 0 && imu660rb_gyro_y == 0 && imu660rb_gyro_z == 0)
    {
        if (gyro_zero_run < IMU_GYRO_ZERO_LIMIT) gyro_zero_run++;
        else link_lost = 1;
    }
    else gyro_zero_run = 0;

    imu.gyro_x = bx - gyro_bias_x;
    imu.gyro_y = by - gyro_bias_y;
    imu.gyro_z = bz - gyro_bias_z;

    gyro_sum_x += imu.gyro_x;
    gyro_sum_y += imu.gyro_y;
    gyro_sum_z += imu.gyro_z;
    ++gyro_sum_count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新车体系加速度
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update_acc();
//-------------------------------------------------------------------------------------------------------------------
void imu_update_acc(void)
{
    imu_read_acc_body(&imu.acc_x, &imu.acc_y, &imu.acc_z);

    if (imu660rb_acc_x == 0 && imu660rb_acc_y == 0 && imu660rb_acc_z == 0)
    {
        if (acc_zero_run < IMU_ACC_ZERO_LIMIT) acc_zero_run++;
        else link_lost = 1;
    }
    else acc_zero_run = 0;
}








//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查 IMU660RB 通信链路状态
// 参数说明     void
// 返回参数     uint8           1 表示链路失效，0 表示正常
// 使用示例     uint8 lost = imu_link_lost();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_link_lost(void)
{
    return link_lost;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取并清除累计的陀螺仪平均值
// 参数说明     gx 角速度 X 输出；gy 角速度 Y 输出；gz 角速度 Z 输出
// 返回参数     void
// 使用示例     imu_get_gyro_avg(&gx, &gy, &gz);
//-------------------------------------------------------------------------------------------------------------------
void imu_get_gyro_avg(float *gx, float *gy, float *gz)
{
    float inv = (gyro_sum_count != 0u) ? (1.0f / (float)gyro_sum_count) : 0.0f;

    *gx = gyro_sum_x * inv;
    *gy = gyro_sum_y * inv;
    *gz = gyro_sum_z * inv;

    gyro_sum_x = gyro_sum_y = gyro_sum_z = 0.0f;
    gyro_sum_count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布姿态解算得到的航向角和本周期增量
// 参数说明     yaw_continuous 连续航向角；yaw_delta 本周期航向增量
// 返回参数     void
// 使用示例     imu_publish_attitude_yaw(att.yaw, yaw_delta);
//-------------------------------------------------------------------------------------------------------------------
void imu_publish_attitude_yaw(float yaw_continuous, float yaw_delta)
{
    yaw_angle = yaw_continuous;
//  element_angle += yaw_delta;
    (void)yaw_delta;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取连续航向角
// 参数说明     void
// 返回参数     float           连续航向角，单位 deg
// 使用示例     float yaw = imu_get_angle_yaw();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_yaw(void)
{
    return yaw_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取元素处理使用的独立累计转角
// 参数说明     void
// 返回参数     float           元素累计转角，单位 deg
// 使用示例     float angle = imu_get_angle_element();
//-------------------------------------------------------------------------------------------------------------------
// float imu_get_angle_element(void)
// {
//     return element_angle;
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清零元素处理使用的独立累计转角
// 参数说明     void
// 返回参数     void
// 使用示例     imu_clear_angle_element();
//-------------------------------------------------------------------------------------------------------------------
// void imu_clear_angle_element(void)
// {
//     element_angle = 0.0f;
// }
