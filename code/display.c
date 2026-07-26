#include "display.h"
#include "board_config.h"
#include "control.h"
#include "balance.h"
#include "attitude.h"
#include "imu.h"
#include "param.h"
#include "vision_core.h"
#include "zf_device_ips200.h"

static uint16 s_text_y0 = DISP_TEXT_Y;
#define DISP_LINE(n)    ((uint16)(s_text_y0 + (n) * 16))

//==================================================模块状态===============================================
static disp_mode_t s_mode      = DISP_MODE_BIN_LINE;    // 显示模式
static uint8       s_frozen    = 0;                     // 冻结标志
// static uint8       s_run_image = 0;                  // Run 页面恢复后启用
static uint8       s_last_mode = 0xFF;                  // 上次模式
static uint8       s_dirty     = 1;                     // 重绘标志
static uint16      s_exposure  = CAM_EXPOSURE_DEFAULT;  // 曝光时间
static uint8       s_image_page_active = 0;             // 图像页标志
static const vision_display_frame_t *s_frame;           // CPU1 锁定快照

#define IMAGE_PAGE_X       (0)
#define IMAGE_PAGE_Y       (16)
#define IMAGE_PAGE_W       (320)
#define IMAGE_PAGE_H       (224)

//==================================================内部函数===============================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在图像区画一个点(自动做屏幕范围保护)
// 参数说明     col/row/color    图像列、图像行和RGB565颜色
// 返回参数     void
// 使用示例     draw_img_point(s_frame->mid_line[i], i, RGB565_RED);
//-------------------------------------------------------------------------------------------------------------------
static void draw_img_point(int col, int row, uint16 color)
{
    if (col < 0 || col >= IMG_W || row < 0 || row >= IMG_H) return;
    ips200_draw_point((uint16)(DISP_IMG_X + col), (uint16)(DISP_IMG_Y + row), color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把算法子区二值图刷到屏幕图像区
// 参数说明     void
// 返回参数     void
// 使用示例     blit_bin_image();
//-------------------------------------------------------------------------------------------------------------------
static void blit_bin_image(void)
{
    if (s_frame == 0) return;
    ips200_show_gray_image((uint16)DISP_IMG_X, (uint16)DISP_IMG_Y,
                           (const uint8 *)s_frame->pixels,
                           (uint16)IMG_W, (uint16)IMG_H,
                           (uint16)IMG_W, (uint16)IMG_H, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在二值图上覆盖左右边线(绿/蓝)、中线(红)、图像中心参考线(黄)、偏差取行区间标记
// 参数说明     void
// 返回参数     void
// 使用示例     overlay_lines();
//-------------------------------------------------------------------------------------------------------------------
static void overlay_lines(void)
{
    int i, top;
    int err_start = ERR_FRONT_ROW;
    int err_end   = ERR_FRONT_ROW + ERR_AVG_ROWS;

    if (s_frame == 0) return;
    top = IMG_H - (int)s_frame->search_stop_line;
    if (top < 0) top = 0;

    for (i = IMG_H - 1; i >= top; i--)
    {
        if (s_frame->left_valid[i])  draw_img_point(s_frame->left_line[i],  i, RGB565_GREEN);
        if (s_frame->right_valid[i]) draw_img_point(s_frame->right_line[i], i, RGB565_BLUE);
        draw_img_point(s_frame->mid_line[i], i, s_frame->mid_valid[i] ? RGB565_RED : RGB565_YELLOW);
    }

    for (i = 0; i < IMG_H; i += 4)
        draw_img_point(IMG_MID_COL, i, RGB565_YELLOW);

    for (i = err_start; i <= err_end && i < IMG_H; i++)
    {
        if (i < 0) continue;
        draw_img_point(0, i, RGB565_RED);
        draw_img_point(IMG_W - 1, i, RGB565_RED);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将算法图像列坐标映射到横屏图像区
// 参数说明     col              算法图像列坐标
// 返回参数     uint16           横屏横坐标
// 使用示例     x = image_page_map_x(s_frame->mid_line[row]);
//-------------------------------------------------------------------------------------------------------------------
static uint16 image_page_map_x(int col)
{
    uint32 x;

    if (col < 0) col = 0;
    if (col >= IMG_W) col = IMG_W - 1;
    x = ((uint32)(col * 2 + 1) * IMAGE_PAGE_W) / (uint32)(IMG_W * 2);
    if (x >= IMAGE_PAGE_W) x = IMAGE_PAGE_W - 1;
    return (uint16)(IMAGE_PAGE_X + x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将算法图像行坐标映射到横屏图像区
// 参数说明     row              算法图像行坐标
// 返回参数     uint16           横屏纵坐标
// 使用示例     y = image_page_map_y(row);
//-------------------------------------------------------------------------------------------------------------------
static uint16 image_page_map_y(int row)
{
    uint32 y;

    if (row < 0) row = 0;
    if (row >= IMG_H) row = IMG_H - 1;
    y = ((uint32)(row * 2 + 1) * IMAGE_PAGE_H) / (uint32)(IMG_H * 2);
    if (y >= IMAGE_PAGE_H) y = IMAGE_PAGE_H - 1;
    return (uint16)(IMAGE_PAGE_Y + y);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在横屏图像区绘制一个已映射的算法点
// 参数说明     col/row/color    算法图像列、行和RGB565颜色
// 返回参数     void
// 使用示例     image_page_draw_point(col, row, RGB565_RED);
//-------------------------------------------------------------------------------------------------------------------
static void image_page_draw_point(int col, int row, uint16 color)
{
    ips200_draw_point(image_page_map_x(col), image_page_map_y(row), color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在横屏图像区绘制相邻算法行之间的线段
// 参数说明     col0/row0/col1/row1/color  起点、终点和RGB565颜色
// 返回参数     void
// 使用示例     image_page_draw_segment(x0, y0, x1, y1, RGB565_GREEN);
//-------------------------------------------------------------------------------------------------------------------
static void image_page_draw_segment(int col0, int row0, int col1, int row1, uint16 color)
{
    ips200_draw_line(image_page_map_x(col0), image_page_map_y(row0),
                     image_page_map_x(col1), image_page_map_y(row1), color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将180x80二值算法图缩放至320x224图像区
// 参数说明     void
// 返回参数     void
// 使用示例     image_page_blit_bin();
//-------------------------------------------------------------------------------------------------------------------
static void image_page_blit_bin(void)
{
    if (s_frame == 0) return;
    ips200_show_gray_image(IMAGE_PAGE_X, IMAGE_PAGE_Y,
                           (const uint8 *)s_frame->pixels,
                           IMG_W, IMG_H, IMAGE_PAGE_W, IMAGE_PAGE_H, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将180x80灰度算法图缩放至320x224图像区
// 参数说明     void
// 返回参数     void
// 使用示例     image_page_blit_gray();
//-------------------------------------------------------------------------------------------------------------------
static void image_page_blit_gray(void)
{
    if (s_frame == 0) return;
    ips200_show_gray_image(IMAGE_PAGE_X, IMAGE_PAGE_Y,
                           (const uint8 *)s_frame->pixels,
                           IMG_W, IMG_H, IMAGE_PAGE_W, IMAGE_PAGE_H, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在横屏二值图上叠加缩放后的左右边线与中线
// 参数说明     void
// 返回参数     void
// 使用示例     image_page_overlay_lines();
//-------------------------------------------------------------------------------------------------------------------
static void image_page_overlay_lines(void)
{
    int row;
    int top;
    int last_left_row = -2;
    int last_right_row = -2;
    int last_mid_row = -2;
    int last_left_col = 0;
    int last_right_col = 0;
    int last_mid_col = 0;

    if (s_frame == 0) return;
    top = IMG_H - (int)s_frame->search_stop_line;
    if (top < 0) top = 0;
    if (top >= IMG_H) top = IMG_H - 1;

    for (row = IMG_H - 1; row >= top; row--)
    {
        if (s_frame->left_valid[row])
        {
            if (last_left_row == row + 1)
                image_page_draw_segment(last_left_col, last_left_row,
                                        s_frame->left_line[row], row, RGB565_GREEN);
            else
                image_page_draw_point(s_frame->left_line[row], row, RGB565_GREEN);
            last_left_col = s_frame->left_line[row];
            last_left_row = row;
        }
        else
        {
            last_left_row = -2;
        }

        if (s_frame->right_valid[row])
        {
            if (last_right_row == row + 1)
                image_page_draw_segment(last_right_col, last_right_row,
                                        s_frame->right_line[row], row, RGB565_BLUE);
            else
                image_page_draw_point(s_frame->right_line[row], row, RGB565_BLUE);
            last_right_col = s_frame->right_line[row];
            last_right_row = row;
        }
        else
        {
            last_right_row = -2;
        }

        if (last_mid_row == row + 1)
            image_page_draw_segment(last_mid_col, last_mid_row,
                                    s_frame->mid_line[row], row,
                                    s_frame->mid_valid[row] ? RGB565_RED : RGB565_YELLOW);
        else
            image_page_draw_point(s_frame->mid_line[row], row,
                                  s_frame->mid_valid[row] ? RGB565_RED : RGB565_YELLOW);
        last_mid_col = s_frame->mid_line[row];
        last_mid_row = row;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制横屏图像页顶部状态栏
// 参数说明     void
// 返回参数     void
// 使用示例     image_page_draw_header();
//-------------------------------------------------------------------------------------------------------------------
static void image_page_draw_header(void)
{
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    switch (s_mode)
    {
    case DISP_MODE_BIN_LINE: ips200_show_string(0, 0, "BIN+L"); break;
    case DISP_MODE_BIN:      ips200_show_string(0, 0, "BIN  "); break;
    case DISP_MODE_GRAY:     ips200_show_string(0, 0, "GRAY "); break;
    default:                 ips200_show_string(0, 0, "BIN+L"); break;
    }

    ips200_show_string(48, 0, "EXP");
    ips200_show_int(80, 0, (int32)s_exposure, 4);
    ips200_show_string(216, 0, "ENT mode");
}

//==================================================外部接口===============================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示模块初始化(复位模式/冻结标志并清屏)
// 参数说明     void
// 返回参数     void
// 使用示例     display_init();
//-------------------------------------------------------------------------------------------------------------------
void display_init(void)
{
    s_mode      = DISP_MODE_BIN_LINE;
    s_frozen    = 0;
//  s_run_image = 0;
    s_last_mode = 0xFF;
    s_dirty     = 1;
    s_image_page_active = 0;
    s_frame = 0;
    display_set_exposure((uint16)CAM_EXPOSURE);
    ips200_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     切到下一个显示模式(循迹页 ENTER 键)
// 参数说明     void
// 返回参数     void
// 使用示例     display_next_mode();
//-------------------------------------------------------------------------------------------------------------------
void display_next_mode(void)
{
    s_mode = (disp_mode_t)((int)s_mode + 1);
    if (s_mode >= DISP_MODE_MAX) s_mode = DISP_MODE_BIN_LINE;
    s_last_mode = 0xFF;
    s_dirty = 1;
    vision_display_request((vision_display_mode_t)s_mode);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     进入独立图像页并切换IPS200横屏方向
// 参数说明     void
// 返回参数     void
// 使用示例     display_image_page_enter();
//-------------------------------------------------------------------------------------------------------------------
void display_image_page_enter(void)
{
    if (s_image_page_active) return;

    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_clear();

    s_image_page_active = 1;
    s_last_mode = 0xFF;
    s_dirty = 1;
    vision_display_request((vision_display_mode_t)s_mode);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     退出独立图像页并立即恢复IPS200竖屏方向
// 参数说明     void
// 返回参数     void
// 使用示例     display_image_page_exit();
//-------------------------------------------------------------------------------------------------------------------
void display_image_page_exit(void)
{
    vision_display_release();
    s_frame = 0;
    s_image_page_active = 0;
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_clear();

    s_last_mode = 0xFF;
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询独立图像页是否处于横屏显示状态
// 参数说明     void
// 返回参数     uint8            1=横屏图像页
// 使用示例     if (display_image_page_active()) display_track_view();
//-------------------------------------------------------------------------------------------------------------------
uint8 display_image_page_active(void)
{
    return s_image_page_active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     冻结/解冻画面
// 参数说明     void
// 返回参数     void
// 使用示例     display_toggle_freeze();
//-------------------------------------------------------------------------------------------------------------------
void display_toggle_freeze(void)
{
    s_frozen = (uint8)(s_frozen ? 0 : 1);
    s_dirty  = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     调整摄像头曝光时间并下发
// 参数说明     step             增量(正=变亮), 内部钳到 [CAM_EXP_MIN, CAM_EXP_MAX]
// 返回参数     void
// 使用示例     display_exposure_step(+CAM_EXP_STEP);
//-------------------------------------------------------------------------------------------------------------------
void display_exposure_step(int step)
{
    int v = (int)s_exposure + step;

    if (v < CAM_EXP_MIN) v = CAM_EXP_MIN;
    if (v > CAM_EXP_MAX) v = CAM_EXP_MAX;
    if ((uint16)v == s_exposure) return;

    display_set_exposure((uint16)v);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前摄像头曝光时间
// 参数说明     void
// 返回参数     uint16           曝光时间
// 使用示例     exposure = display_get_exposure();
//-------------------------------------------------------------------------------------------------------------------
uint16 display_get_exposure(void)
{
    return s_exposure;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置摄像头曝光时间并同步运行参数
// 参数说明     exposure         曝光时间
// 返回参数     void
// 使用示例     display_set_exposure(512);
//-------------------------------------------------------------------------------------------------------------------
void display_set_exposure(uint16 exposure)
{
    int v = (int)exposure;

    if (v < CAM_EXP_MIN) v = CAM_EXP_MIN;
    if (v > CAM_EXP_MAX) v = CAM_EXP_MAX;

    s_exposure = (uint16)v;
    (void)param_set_by_name("cam_exposure", (float)v);
    (void)vision_command_request(VISION_CMD_SET_EXPOSURE, s_exposure);
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取并清"需要重画"标志(按键改过状态但没有新帧时也要刷一次)
// 参数说明     void
// 返回参数     uint8            1=本次需要重画
// 使用示例     if (fresh || display_view_dirty()) display_track_view();
//-------------------------------------------------------------------------------------------------------------------
uint8 display_view_dirty(void)
{
    uint8 d = s_dirty;
    s_dirty = 0;
    return d;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     当前是否冻结
// 参数说明     void
// 返回参数     uint8            1=冻结
// 使用示例     if (!display_is_frozen()) control_vision_debug();
//-------------------------------------------------------------------------------------------------------------------
uint8 display_is_frozen(void)
{
    return s_frozen;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发车运行画面是否刷图像
// 参数说明     on               1=开 0=关
// 返回参数     void
// 使用示例     display_set_run_image(1);
//-------------------------------------------------------------------------------------------------------------------
// void display_set_run_image(uint8 on)
// {
//     s_run_image = (uint8)(on ? 1 : 0);
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读发车运行画面刷图开关
// 参数说明     void
// 返回参数     uint8            1=开
// 使用示例     uint8 en = display_get_run_image();
//-------------------------------------------------------------------------------------------------------------------
// uint8 display_get_run_image(void)
// {
//     return s_run_image;
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     横屏图像页按当前模式刷新整幅图像
// 参数说明     void
// 返回参数     void
// 使用示例     display_track_view();
//-------------------------------------------------------------------------------------------------------------------
void display_track_view(void)
{
    const vision_display_frame_t *frame;

    if (!s_image_page_active) return;
    if (!vision_display_read(&frame))
    {
        vision_display_request((vision_display_mode_t)s_mode);
        image_page_draw_header();
        return;
    }
    if (frame->mode != (uint8)s_mode)
    {
        vision_display_release();
        vision_display_request((vision_display_mode_t)s_mode);
        image_page_draw_header();
        return;
    }
    s_frame = frame;

    if (s_last_mode != (uint8)s_mode)
    {
        ips200_clear();
        s_last_mode = (uint8)s_mode;
    }

    if (s_mode == DISP_MODE_GRAY)
    {
        image_page_blit_gray();
    }
    else
    {
        image_page_blit_bin();
        if (s_mode == DISP_MODE_BIN_LINE)
            image_page_overlay_lines();
    }

    image_page_draw_header();
    vision_display_release();
    s_frame = 0;
    if (!s_frozen)
        vision_display_request((vision_display_mode_t)s_mode);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发车运行画面: 姿态/各轴输出/偏差/循迹有效性/元素/状态机(+可选小图)
// 参数说明     void
// 返回参数     void
// 使用示例     display_run_screen();
//-------------------------------------------------------------------------------------------------------------------
// void display_run_screen(void)
// {
//     static uint8 div;
//     static uint8 text_div;
//
//     if (++text_div < DISP_TEXT_DIV) return;
//     text_div = 0;
//
//     s_text_y0 = DISP_TEXT_Y;
//     ips200_show_string(0, 0,  "== RUN ==       ");
//     ips200_show_string(0, 16, "ST");
//     ips200_show_int   (8 * 3, 16, (int32)start_flag, 1);
//     ips200_show_string(8 * 6, 16, "IMU");
//     ips200_show_int   (8 * 10, 16, (int32)g_imu_ok, 1);
//     ips200_show_string(8 * 13, 16, "VIS");
//     ips200_show_int   (8 * 17, 16, (int32)g_track_valid, 1);
//
//     if (s_run_image)
//     {
//         div++;
//         if (div >= DISP_RUN_DIV)
//         {
//             const vision_display_frame_t *frame;
//             div = 0;
//             vision_display_request(VISION_DISPLAY_BIN_LINE);
//             if (vision_display_read(&frame))
//             {
//                 s_frame = frame;
//                 blit_bin_image();
//                 overlay_lines();
//                 vision_display_release();
//                 s_frame = 0;
//             }
//         }
//     }
//
//     ips200_show_string(0,      DISP_LINE(0), "ROLL");
//     ips200_show_float (8 * 5,  DISP_LINE(0), (double)att.roll, 4, 2);
//     ips200_show_string(8 * 14, DISP_LINE(0), "PWM");
//     ips200_show_int   (8 * 18, DISP_LINE(0), (int32)g_pwm_roll, 6);
//
//     ips200_show_string(0,      DISP_LINE(1), "PTCH");
//     ips200_show_float (8 * 5,  DISP_LINE(1), (double)att.pitch, 4, 2);
//     ips200_show_string(8 * 14, DISP_LINE(1), "PWM");
//     ips200_show_int   (8 * 18, DISP_LINE(1), (int32)g_pwm_pitch, 6);
//
//     ips200_show_string(0,      DISP_LINE(2), "YAW");
//     ips200_show_float (8 * 5,  DISP_LINE(2), (double)att.yaw, 5, 1);
//     ips200_show_string(8 * 14, DISP_LINE(2), "TGT");
//     ips200_show_float (8 * 18, DISP_LINE(2), (double)g_yaw_target, 5, 1);
//
//     ips200_show_string(0,      DISP_LINE(3), "ERR");
//     ips200_show_float (8 * 4,  DISP_LINE(3), (double)g_dbg_error, 4, 1);
//     ips200_show_string(8 * 12, DISP_LINE(3), "LOSTF");
//     ips200_show_int   (8 * 18, DISP_LINE(3), (int32)g_track_lost_frames, 4);
//
//     ips200_show_string(0,      DISP_LINE(4), "SPD T");
//     ips200_show_int   (8 * 6,  DISP_LINE(4), (int32)g_target_distance, 4);
//     ips200_show_string(8 * 12, DISP_LINE(4), "ELEM");
//     ips200_show_int   (8 * 17, DISP_LINE(4), (int32)g_vision_active_elem, 1);
//
//     ips200_show_string(0,      DISP_LINE(5), "MOT A");
//     ips200_show_int   (8 * 6,  DISP_LINE(5), (int32)g_motor_a, 6);
//     ips200_show_string(0,      DISP_LINE(6), "MOT B");
//     ips200_show_int   (8 * 6,  DISP_LINE(6), (int32)g_motor_b, 6);
//     ips200_show_string(8 * 13, DISP_LINE(6), "C");
//     ips200_show_int   (8 * 15, DISP_LINE(6), (int32)g_motor_c, 6);
//
//     ips200_show_string(0, DISP_LINE(7), "ENT:next RET:STOP   ");
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开机自检画面(IMU 失败时长期显示错误, 禁止发车)
// 参数说明     void
// 返回参数     void
// 使用示例     display_imu_fail_screen();
//-------------------------------------------------------------------------------------------------------------------
void display_imu_fail_screen(void)
{
    ips200_show_string(0, 0,      "!! IMU INIT FAIL !! ");
    ips200_show_string(0, 16,     "IMU660RB no answer  ");
    ips200_show_string(0, 16 * 2, "check SPI0 wiring:  ");
    ips200_show_string(0, 16 * 3, "SCK P20_11 MOSI 20_14");
    ips200_show_string(0, 16 * 4, "MISO P20_12 CS 20_13");
    ips200_show_string(0, 16 * 5, "START IS BLOCKED    ");
    ips200_show_string(0, 16 * 6, "brake stays LOCKED  ");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示发车阻止原因
// 参数说明     reason          control_start_block_reason() 的返回值
// 返回参数     void
// 使用示例     display_start_blocked_screen(control_start_block_reason());
//-------------------------------------------------------------------------------------------------------------------
// void display_start_blocked_screen(control_block_t reason)
// {
//     switch (reason)
//     {
//     case CTRL_BLOCK_BLDC_LOST:
//         ips200_show_string(0, 0,      "!! BLDC LINK LOST !!");
//         ips200_show_string(0, 16,     "no speed frame from ");
//         ips200_show_string(0, 16 * 2, "CYT2BL3 dual driver ");
//         ips200_show_string(0, 16 * 3, "check UART3 wiring: ");
//         ips200_show_string(0, 16 * 4, "P15_6 TX -> drv RX  ");
//         ips200_show_string(0, 16 * 5, "P15_7 RX <- drv TX  ");
//         ips200_show_string(0, 16 * 6, "and driver 12V power");
//         break;
//
//     case CTRL_BLOCK_IMU_FAIL:
//         display_imu_fail_screen();
//         break;
//
//     case CTRL_BLOCK_ATT_CONVERGING:
//         ips200_show_string(0, 0,      "ATTITUDE SETTLING   ");
//         ips200_show_string(0, 16,     "cold start converge ");
//         ips200_show_string(0, 16 * 2, "KEEP CAR STILL      ");
//         ips200_show_string(0, 16 * 3, "ready in ~3s        ");
//         ips200_show_string(0, 16 * 4, "then press ENTER    ");
//         ips200_show_string(0, 16 * 5, "                    ");
//         ips200_show_string(0, 16 * 6, "                    ");
//         break;
//
//     case CTRL_BLOCK_IMU_LOST:
//         ips200_show_string(0, 0,      "!! IMU LINK LOST !!(");
//         ips200_show_string(0, 16,     "raw xyz all zero    ");
//         ips200_show_string(0, 16 * 2, "chip stopped answer ");
//         ips200_show_string(0, 16 * 3, "check SPI0 wiring,  ");
//         ips200_show_string(0, 16 * 4, "3V3 supply & ground ");
//         ips200_show_string(0, 16 * 5, "shorten IMU cable   ");
//         ips200_show_string(0, 16 * 6, "POWER CYCLE REQUIRED");
//         break;
//
//     case CTRL_BLOCK_ATT_DIVERGED:
//         ips200_show_string(0, 0,      "!! ATTITUDE LOST !! ");
//         ips200_show_string(0, 16,     "quaternion diverged ");
//         ips200_show_string(0, 16 * 2, "angles NOT trusted  ");
//         ips200_show_string(0, 16 * 3, "POWER CYCLE REQUIRED");
//         ips200_show_string(0, 16 * 4, "START IS BLOCKED    ");
//         ips200_show_string(0, 16 * 5, "brake stays LOCKED  ");
//         ips200_show_string(0, 16 * 6, "                    ");
//         break;
//
//     case CTRL_BLOCK_NONE:
//     default:
//         break;
//     }
// }
