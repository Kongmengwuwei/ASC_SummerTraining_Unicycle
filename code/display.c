#include "display.h"
#include "board_config.h"
#include "control.h"
#include "element.h"
#include "param.h"
#include "vision_core.h"
#include "zf_device_ips200.h"

#define IMAGE_PAGE_X       (40)         // 图像区在横屏 320 像素宽屏幕内水平居中
#define IMAGE_PAGE_Y       (74)         // 图像区在状态栏下方垂直居中
#define IMAGE_PAGE_W       (240)        // 横屏显示区
#define IMAGE_PAGE_H       (107)        // 保持原有图像页布局

static disp_mode_t s_mode = DISP_MODE_BIN_LINE;         // 当前显示模式
static uint8       s_last_mode = 0xFF;                  // 上次已绘制的模式，变了要整屏清一次
static uint8       s_dirty = 1;                         // 需要重画标志
static uint16      s_exposure = CAM_EXPOSURE_DEFAULT;   // 当前曝光时间
static uint8       s_image_page_active = 0;             // IPS200 处于横屏图像页
static const vision_display_frame_t *s_frame;           // 本次绘制期间锁定的 CPU1 快照

static uint8       s_ipm_pick = IPM_PICK_NO_TRACK;      // 上一帧取角点的结果，ipm_pick_t
static uint8       s_ipm_row_near;                      // 上一帧选中的近端行
static uint8       s_ipm_row_far;                       // 上一帧选中的远端行
static uint8       s_ipm_col[4];                        // 四个角点列：近左/近右/远左/远右

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把算法图像列坐标映射到横屏图像区
// 参数说明     col             算法图像列坐标
// 返回参数     uint16          屏幕横坐标
// 使用示例     x = image_page_map_x(s_frame->mid_line[row]);
//-------------------------------------------------------------------------------------------------------------------
static uint16 image_page_map_x(int col)
{
    uint32 x;

    if (col < 0) col = 0;
    if (col >= IMG_W) col = IMG_W - 1;
    x = ((uint32)(col * 2 + 1) * IMAGE_PAGE_W) / (uint32)(IMG_W * 2);
    return (uint16)(IMAGE_PAGE_X + x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把算法图像行坐标映射到横屏图像区
// 参数说明     row             算法图像行坐标
// 返回参数     uint16          屏幕纵坐标
// 使用示例     y = image_page_map_y(row);
//-------------------------------------------------------------------------------------------------------------------
static uint16 image_page_map_y(int row)
{
    uint32 y;

    if (row < 0) row = 0;
    if (row >= IMG_H) row = IMG_H - 1;
    y = ((uint32)(row * 2 + 1) * IMAGE_PAGE_H) / (uint32)(IMG_H * 2);
    return (uint16)(IMAGE_PAGE_Y + y);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在横屏图像区画一个映射后的算法点
// 参数说明     col/row/color   算法图像列、行和 RGB565 颜色
// 返回参数     void
// 使用示例     image_page_draw_point(col, row, RGB565_RED);
//-------------------------------------------------------------------------------------------------------------------
static void image_page_draw_point(int col, int row, uint16 color)
{
    ips200_draw_point(image_page_map_x(col), image_page_map_y(row), color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在横屏图像区画相邻两算法行之间的线段
// 参数说明     col0/row0/col1/row1/color 起点、终点和 RGB565 颜色
// 返回参数     void
// 使用示例     image_page_draw_segment(x0, y0, x1, y1, RGB565_GREEN);
//-------------------------------------------------------------------------------------------------------------------
static void image_page_draw_segment(int col0, int row0, int col1, int row1, uint16 color)
{
    ips200_draw_line(image_page_map_x(col0), image_page_map_y(row0),
                     image_page_map_x(col1), image_page_map_y(row1), color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把 180x80 算法帧等比例放大到图像区
// 参数说明     void
// 返回参数     void
// 使用示例     image_page_blit();
//-------------------------------------------------------------------------------------------------------------------
static void image_page_blit(void)
{
    if (s_frame == 0) return;
    ips200_show_gray_image(IMAGE_PAGE_X, IMAGE_PAGE_Y,
                           (const uint8 *)s_frame->pixels,
                           IMG_W, IMG_H, IMAGE_PAGE_W, IMAGE_PAGE_H, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     叠加左边线(绿)、右边线(蓝)、中线(有效红/补线黄)，只画前瞻区内的行
// 参数说明     void
// 返回参数     void
// 使用示例     image_page_overlay_lines();
//-------------------------------------------------------------------------------------------------------------------
static void image_page_overlay_lines(void)
{
    int row;
    int top;
    int last_left_row = -2;             // 上一行画过左边线的行号，连续才连线
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

        if (s_frame->mid_valid[row])
        {
            if (last_mid_row == row + 1)
                image_page_draw_segment(last_mid_col, last_mid_row,
                                        s_frame->mid_line[row], row, RGB565_RED);
            else
                image_page_draw_point(s_frame->mid_line[row], row, RGB565_RED);
            last_mid_col = s_frame->mid_line[row];
            last_mid_row = row;
        }
        else
        {
            last_mid_row = -2;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画图像页顶部状态栏：显示模式、曝光、阈值、循迹有效性和元素编号
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
    case DISP_MODE_BIN:      ips200_show_string(0, 0, "BIN  "); break;
    case DISP_MODE_GRAY:     ips200_show_string(0, 0, "GRAY "); break;
    default:                 ips200_show_string(0, 0, "BIN+L"); break;
    }

    ips200_show_string(48, 0, "E");
    ips200_show_int(56, 0, (int32)s_exposure, 4);
    ips200_show_string(96, 0, "T");
    ips200_show_int(104, 0, (int32)((s_frame != 0) ? s_frame->threshold : 0), 3);
    ips200_show_string(136, 0, "V");
    ips200_show_int(144, 0, (int32)((s_frame != 0) ? s_frame->track_valid : 0), 1);

    ips200_show_string(0, 16,
        element_name((s_frame != 0) ? s_frame->active_elem : (uint8)ELEM_NONE));

    ips200_show_string(48, 16, "F");
    ips200_show_int(56, 16, (int32)(g_vision_fps + 0.5f), 3);

    ips200_show_string(96, 16, "L");
    ips200_show_int(104, 16, (int32)((s_frame != 0) ? s_frame->search_stop_line : 0), 3);

    ips200_set_color(g_vision_ipm_ok ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    ips200_show_string(136, 16, "P");
    ips200_show_int(144, 16, (int32)g_vision_ipm_ok, 1);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在图像上画出逆透视标定的两条采样行与四个采样点
// 参数说明     void
// 返回参数     uint8            1=两行都有完整的左右边线，可以标定
// 使用示例     if (display_ipm_overlay()) menu_status("READY");
//-------------------------------------------------------------------------------------------------------------------
uint8 display_ipm_overlay(void)
{
    int rows[2], k;

    if (!s_image_page_active) return (uint8)IPM_PICK_NO_TRACK;
    if (s_ipm_pick != (uint8)IPM_PICK_OK) return s_ipm_pick;

    rows[0] = s_ipm_row_near;
    rows[1] = s_ipm_row_far;

    for (k = 0; k < 2; k++)
    {
        int r = rows[k];
        int cl = s_ipm_col[2 * k];
        int cr = s_ipm_col[2 * k + 1];

        if (r <= 0 || r >= IMG_H) return (uint8)IPM_PICK_NO_TRACK;

        // 只画两个角点之间那一段，横线的两端就是梯形的边
        image_page_draw_segment(cl, r, cr, r, RGB565_GREEN);
        image_page_draw_segment(cl - 3, r, cl + 3, r, RGB565_YELLOW);
        image_page_draw_segment(cl, r - 3, cl, r + 3, RGB565_YELLOW);
        image_page_draw_segment(cr - 3, r, cr + 3, r, RGB565_YELLOW);
        image_page_draw_segment(cr, r - 3, cr, r + 3, RGB565_YELLOW);
    }
    // 梯形的两条斜边，摆正了它应该左右对称
    image_page_draw_segment(s_ipm_col[0], rows[0], s_ipm_col[2], rows[1], RGB565_GREEN);
    image_page_draw_segment(s_ipm_col[1], rows[0], s_ipm_col[3], rows[1], RGB565_GREEN);
    return (uint8)IPM_PICK_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读上一帧逆透视标定选中的近端行号
// 参数说明     void
// 返回参数     uint8            行号，0=没选到
// 使用示例     row = display_ipm_row_near();
//-------------------------------------------------------------------------------------------------------------------
uint8 display_ipm_row_near(void)
{
    return s_ipm_row_near;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读上一帧逆透视标定选中的远端行号
// 参数说明     void
// 返回参数     uint8            行号，0=没选到
// 使用示例     row = display_ipm_row_far();
//-------------------------------------------------------------------------------------------------------------------
uint8 display_ipm_row_far(void)
{
    return s_ipm_row_far;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读上一帧逆透视标定两个采样行上的赛道像素宽度
// 参数说明     near             1=近端行，0=远端行
// 返回参数     int              宽度(像素)，没选到时是 0
// 使用示例     w = display_ipm_width(0);
//-------------------------------------------------------------------------------------------------------------------
int display_ipm_width(uint8 near)
{
    return near ? ((int)s_ipm_col[1] - (int)s_ipm_col[0])
                : ((int)s_ipm_col[3] - (int)s_ipm_col[2]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位显示模式并按运行参数下发一次曝光
// 参数说明     void
// 返回参数     void
// 使用示例     display_init();
//-------------------------------------------------------------------------------------------------------------------
void display_init(void)
{
    s_mode = DISP_MODE_BIN_LINE;
    s_last_mode = 0xFF;
    s_dirty = 1;
    s_image_page_active = 0;
    s_frame = 0;
    display_set_exposure((uint16)CAM_EXPOSURE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     切到下一个显示模式并请求 CPU1 按新模式出快照
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
// 函数简介     进入图像页并把 IPS200 切成横屏
// 参数说明     void
// 返回参数     void
// 使用示例     display_image_page_enter();
//-------------------------------------------------------------------------------------------------------------------
void display_image_page_enter(void)
{
    if (s_image_page_active) return;

    ips200_set_dir(IPS200_CROSSWISE_180);
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
// 函数简介     退出图像页，释放快照并把 IPS200 切回竖屏
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
// 函数简介     按步长调整曝光时间，内部钳到 [CAM_EXP_MIN, CAM_EXP_MAX]
// 参数说明     step            曝光增量，正数变亮
// 返回参数     void
// 使用示例     display_exposure_step(CAM_EXP_STEP);
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
// 函数简介     设置曝光时间，同步写回运行参数并下发给 CPU1
// 参数说明     exposure        曝光时间
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
// 函数简介     取并清"需要重画"标志
// 参数说明     void
// 返回参数     uint8           1=本次需要重画
// 使用示例     if (display_view_dirty()) display_track_view();
//-------------------------------------------------------------------------------------------------------------------
uint8 display_view_dirty(void)
{
    uint8 d = s_dirty;

    s_dirty = 0;
    return d;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按当前模式刷新一次整幅图像
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

    image_page_blit();
    if (s_mode == DISP_MODE_BIN_LINE)
        image_page_overlay_lines();
    image_page_draw_header();

    s_ipm_pick = s_frame->ipm_pick;
    s_ipm_row_near = s_frame->ipm_row_near;
    s_ipm_row_far = s_frame->ipm_row_far;
    memcpy(s_ipm_col, s_frame->ipm_col, sizeof(s_ipm_col));

    vision_display_release();
    s_frame = 0;
    vision_display_request((vision_display_mode_t)s_mode);
}
