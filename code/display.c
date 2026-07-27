#include "display.h"
#include "board_config.h"
#include "element.h"
#include "param.h"
#include "vision_core.h"
#include "zf_device_ips200.h"

#define IMAGE_PAGE_X       (0)          // 横屏图像区左上角横坐标
#define IMAGE_PAGE_Y       (57)         // 320x142 图像在状态栏下方 224 像素区域内垂直居中
#define IMAGE_PAGE_W       (320)        // 横屏图像区宽度
#define IMAGE_PAGE_H       (142)        // 保持 180x80 原始宽高比，避免赛道纵向拉伸

static disp_mode_t s_mode = DISP_MODE_BIN_LINE;         // 当前显示模式
static uint8       s_last_mode = 0xFF;                  // 上次已绘制的模式，变了要整屏清一次
static uint8       s_dirty = 1;                         // 需要重画标志
static uint16      s_exposure = CAM_EXPOSURE_DEFAULT;   // 当前曝光时间
static uint8       s_image_page_active = 0;             // IPS200 处于横屏图像页
static const vision_display_frame_t *s_frame;           // 本次绘制期间锁定的 CPU1 快照

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把算法图像列坐标映射到横屏图像区
// 参数说明     col             算法图像列坐标
// 返回参数     uint16          横屏横坐标
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
// 函数简介     把算法图像行坐标映射到横屏图像区
// 参数说明     row             算法图像行坐标
// 返回参数     uint16          横屏纵坐标
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
// 函数简介     把 180x80 算法帧放大填满图像区，二值图与灰度图走同一条路径
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
    // 元素显示名字而不是编号，跑车时一眼就能看出识别成了什么
    ips200_show_string(160, 0,
        element_name((s_frame != 0) ? s_frame->active_elem : (uint8)ELEM_NONE));
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

    // 快照还没准备好，或者是上一个模式生成的，先重新请求，本次只刷状态栏
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

    vision_display_release();
    s_frame = 0;
    vision_display_request((vision_display_mode_t)s_mode);
}
