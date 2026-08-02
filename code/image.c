#include "image.h"
#include "element.h"
#include "perspective.h"
#include <math.h>

#pragma section all "cpu1_dsram"

Image my_image;                                 // 图像处理结果
int   Road_Half_Wide[IMG_H];                    // 自适应半赛宽表(像素)，见下方说明
int   g_err_front_row = ERR_FRONT_ROW_DEFAULT;  // 前瞻行，拟合线在这一行求值，由 CPU0 的运行参数刷新
float g_mid_error = 0.0f;                       // 前瞻行像素偏差，正=赛道位于车体左侧
float g_track_lateral = 0.0f;                   // 横向偏差，正=赛道位于车体左侧
float g_track_heading = 0.0f;                   // 航向偏差，正=赛道朝向车体左侧
float g_track_curvature = 0.0f;                 // 曲率，正=左弯
float g_track_quality = 0.0f;                   // 循迹质量

static uint8 img_gray[IMG_H][IMG_W];            // 灰度帧缓冲区
static int s_seed_col = IMG_MID_COL;             // 下一帧近端起点种子列
static int s_last_near_mid = IMG_MID_COL;        // 上一帧近端中线
static uint8 s_road_learn_count[IMG_H];          // 每行半赛宽学习次数
static float s_row_depth[IMG_H];                 // 每行在俯视平面里的纵深，逐行限幅用
static uint8 s_row_depth_ok[IMG_H];              // 上面那个值是否有效
static int s_last_threshold = (OTSU_TH_MIN + OTSU_TH_MAX) / 2;

static float s_fit_u[IMG_H];
static float s_fit_z[IMG_H];
static float s_fit_w[IMG_H];
static float s_fit_base_w[IMG_H];
static int s_mid_filter[IMG_H];

static uint8 s_allow_width_learning;

#define IMAGE_OTSU_ROI_TOP          4
#define IMAGE_OTSU_THRESHOLD_SLEW   6
#define IMAGE_EDGE_BLACK_RUN        1
#define IMAGE_SEED_SEARCH_RADIUS    (EN_EDGE_TRACK + EN_SEED_RECOVER)
#define IMAGE_EDGE_RECOVER_RADIUS   (EN_EDGE_TRACK + 2 * EN_SEED_RECOVER)
#define IMAGE_EDGE_WIDTH_MIN_PCT    55
#define IMAGE_EDGE_GAP_MIN          4           // 跨窄黑条扫描允许跨过的最小黑段(列)
#define IMAGE_EDGE_GAP_MAX          20          // 同上的上限，再宽就当真边界
#define IMAGE_ENVELOPE_MIN_PCT      40
#define IMAGE_ENVELOPE_MAX_PCT      240
#define IMAGE_MID_HOLE_MAX          5
#define IMAGE_FIT_MIN_POINTS        10
#define IMAGE_FIT_RESIDUAL_MIN      (0.08f)
#define IMAGE_FIT_RESIDUAL_GAIN     (2.5f)
#define IMAGE_ROAD_ROW_SAMPLES      12
#define IMAGE_ROAD_FIT_MIN_ROWS     16          // 够这么多行学熟才敢往量不到的行外推
#define IMAGE_ROAD_FIT_MIN_SPAN     (20.0f)     // 学熟的行跨度下限
#define IMAGE_ROAD_READY_ROWS       16
#define IMAGE_ROAD_LEARN_MIN_DUAL   24
#define IMAGE_ROAD_LEARN_CURVATURE  (0.08f)
#define IMAGE_RAD_TO_DEG            (57.2957795f)

#define IMAGE_BEV_SCALE             ((float)IPM_HALF_WIDTH)
#define IMAGE_U_MIN                 (-3.00f)    // 近端可以到标定近行后面这么远(负号=更靠近车)
#define IMAGE_U_MAX                 (8.00f)     // 远端超过这里的点畸变太大，不要
#define IMAGE_SINGLE_WEIGHT         (0.45f)     // 单边补出来的中线点在转向拟合里的权重
#define IMAGE_MID_MIN_POINTS        8           // 候选点少于这么多就判本帧循迹无效
#define IMAGE_PREVIEW_SPAN          (3.00f)     // 转向拟合窗口的纵向跨度(半赛宽)
#define IMAGE_U_SPAN_FULL           (2.00f)     // 拟合覆盖度满分对应的 u 跨度
#define MID_SLEW_MAX                6           // 相邻有效行中线的最大跳变(列)，没标定时的兜底
#define MID_SLOPE_MAX               (1.20f)     // 俯视平面里赛道倾角上限 dz/du
#define MID_SLEW_Z_BASE             (0.06f)     // 倾角限幅的常数余量(半赛宽)
#define IMAGE_SLOPE_SPAN            8           // 求边线俯视斜率的最大行跨度
#define IMAGE_SLOPE_CORR_MAX        (3.00f)     // 沿行半赛宽的斜率修正上限(1/cos)
#define IMAGE_FOOT_SPAN             40          // 找垂足时在参考行上下各看这么多行
#define IMAGE_BISECT_STEPS          9           // 单边行沿行二分的次数
#define IMAGE_MID_EDGE_KEEP         2           // 中线离画面边缘少于这么多列就当出画
#define IMAGE_EXTEND_FIT_ROWS       20          // 近端续线取最近这么多有效行定斜率
#define IMAGE_EXTEND_SLOPE_MAX      (2.50f)     // 近端续线斜率上限(列/行)
static int iclip(int x, int lo, int hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     自适应半宽表是否已经学熟，所有"超宽"判据都要先问它
// 参数说明     void
// 返回参数     uint8            1=可以拿 Road_Half_Wide 当基准比了
// 使用示例     if (!image_road_wide_ready()) wide_rows = 0;
//-------------------------------------------------------------------------------------------------------------------
uint8 image_road_wide_ready(void)
{
    int row;
    int ready_rows = 0;

    // 标定过之后半赛宽是由矩阵算出来的，不存在"还没学熟"这回事
    if (ipm_ready()) return 1u;
    for (row = ERR_FIT_ROW_TOP; row < IMG_H; row++)
        if (s_road_learn_count[row] >= IMAGE_ROAD_ROW_SAMPLES) ready_rows++;

    return (ready_rows >= IMAGE_ROAD_READY_ROWS) ? 1u : 0u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取指定行可供元素判断使用的道路包络
// 参数说明     row/left/right   图像行、左边界输出、右边界输出
// 返回参数     uint8            1=包络可信 0=本行没有可信包络
// 使用示例     if (image_get_track_envelope(row, &left, &right)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 image_get_track_envelope(int row, int *left, int *right)
{
    int width;
    int expected_width;

    if (row < 0 || row >= IMG_H || left == 0 || right == 0) return 0;
    if (my_image.Left_Source[row] == IMAGE_POINT_NONE ||
        my_image.Right_Source[row] == IMAGE_POINT_NONE) return 0;
    if (my_image.Left_Line[row] < EN_COL_MIN_LIMIT ||
        my_image.Right_Line[row] > EN_COL_MAX_LIMIT ||
        my_image.Left_Line[row] >= my_image.Right_Line[row]) return 0;

    width = my_image.Right_Line[row] - my_image.Left_Line[row] + 1;
    expected_width = iclip(2 * Road_Half_Wide[row], EN_MIN_RUN, IMG_W - 2);
    if (width * 100 < expected_width * IMAGE_ENVELOPE_MIN_PCT ||
        width * 100 > expected_width * IMAGE_ENVELOPE_MAX_PCT) return 0;

    *left = my_image.Left_Line[row];
    *right = my_image.Right_Line[row];
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将半赛宽表复位为编译期初值
// 参数说明     void
// 返回参数     void
// 使用示例     image_reset_road_wide();
//-------------------------------------------------------------------------------------------------------------------
static void image_reset_road_wide(void)
{
    int i;

    float near_half = 0.5f * (float)ROAD_WIDE_NEAR_INIT;
    float span = (float)(IMG_H - 1 - ROAD_WIDE_HORIZON_ROW);

    memset(s_road_learn_count, 0, sizeof(s_road_learn_count));
    if (span < 1.0f) span = 1.0f;
    for (i = 0; i < IMG_H; i++)
        Road_Half_Wide[i] = iclip((int)(near_half * (float)(i - ROAD_WIDE_HORIZON_ROW) / span
                                        + 0.5f), ROAD_HALF_MIN, IMG_W);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化图像数据与标准赛宽表
// 参数说明     void
// 返回参数     void
// 使用示例     image_init();
//-------------------------------------------------------------------------------------------------------------------
void image_init(void)
{
    int i;

    image_reset_road_wide();

    for (i = 0; i < IMG_H; i++)
    {
        my_image.Left_Line[i]       = 0;
        my_image.Right_Line[i]      = IMG_W - 1;
        my_image.Mid_Line[i]        = IMG_MID_COL;
        my_image.Road_Wide[i]       = IMG_W - 1;
        my_image.Road_Perp_Wide[i]  = 0;
        my_image.Left_Lost_Flag[i]  = 1;
        my_image.Right_Lost_Flag[i] = 1;
        my_image.Mid_Lost_Flag[i]   = 1;
        my_image.Left_Source[i]     = IMAGE_POINT_NONE;
        my_image.Right_Source[i]    = IMAGE_POINT_NONE;
        my_image.Mid_Source[i]      = IMAGE_POINT_NONE;
    }
    my_image.Search_Stop_Line = 0;
    my_image.Edge_Row_Bottom = -1;
    my_image.Left_Row_Bottom = -1;
    my_image.Right_Row_Bottom = -1;
    my_image.Boundry_Start_Left = -1;
    my_image.Boundry_Start_Right = -1;
    my_image.Mid_Valid_Rows   = 0;
    my_image.Valid_Row_Bottom = -1;
    my_image.Valid_Row_Top    = -1;
    my_image.Track_Valid      = 0;
    g_mid_error               = 0.0f;
    s_seed_col                = IMG_MID_COL;
    s_last_near_mid           = IMG_MID_COL;
    s_last_threshold          = (OTSU_TH_MIN + OTSU_TH_MAX) / 2;
    s_allow_width_learning    = 0;
    g_track_lateral           = 0.0f;
    g_track_heading           = 0.0f;
    g_track_curvature         = 0.0f;
    g_track_quality           = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复制摄像头新帧到图像处理缓冲区
// 参数说明     void
// 返回参数     uint8            1=获取成功 0=无新帧
// 使用示例     if (!image_grab()) return 0;
//-------------------------------------------------------------------------------------------------------------------
uint8 image_grab(void)
{
    int r;

    if (!mt9v03x_finish_flag) return 0;

    for (r = 0; r < IMG_H; r++)
        memcpy(img_gray[r], &mt9v03x_image[r + IMG_ROW_OFFSET][IMG_COL_OFFSET], IMG_W);

    __dsync();
    mt9v03x_finish_flag = 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算大津阈值并完成整帧二值化
// 参数说明     void
// 返回参数     void
// 使用示例     image_binarize();
//-------------------------------------------------------------------------------------------------------------------
static void image_binarize(void)
{
    uint16 hist[256];
    uint32 i, total = 0;
    uint32 sum_all = 0;
    uint32 sum_b = 0;
    uint32 w_b = 0;
    uint32 w_f;
    int    r, c, threshold, th_first = 0, th_last = 0;
    int    gmin = 255, gmax = 0;
    float  num, var, var_max = -1.0f;

    memset(hist, 0, sizeof(hist));

    for (r = IMAGE_OTSU_ROI_TOP; r < IMG_H; r += 2)
    {
        const uint8 *src = img_gray[r];
        for (c = 0; c < IMG_W; c += 2)
        {
            uint8 g = src[c];
            hist[g]++;
            total++;
            sum_all += g;
            if (g < gmin) gmin = g;
            if (g > gmax) gmax = g;
        }
    }
    my_image.Contrast = gmax - gmin;

    threshold = s_last_threshold;
    if (total > 0u && my_image.Contrast >= OTSU_CONTRAST_MIN)
    {
        for (i = (uint32)gmin; i <= (uint32)gmax; i++)
        {
            w_b += hist[i];
            sum_b += i * hist[i];
            if (w_b == 0u) continue;
            w_f = total - w_b;
            if (w_f == 0u) break;
            if (hist[i] == 0u) continue;

            num = (float)sum_b * (float)w_f
                - (float)(sum_all - sum_b) * (float)w_b;
            var = num * num / ((float)w_b * (float)w_f);
            if (var > var_max)
            {
                var_max = var;
                th_first = (int)i;
                th_last = (int)i;
            }
            else if (fabsf(var - var_max) <= 1e-6f * (fabsf(var_max) + 1.0f))
            {
                th_last = (int)i;
            }
        }

        threshold = iclip((th_first + th_last) / 2, OTSU_TH_MIN, OTSU_TH_MAX);
        if (threshold > s_last_threshold + IMAGE_OTSU_THRESHOLD_SLEW)
            threshold = s_last_threshold + IMAGE_OTSU_THRESHOLD_SLEW;
        else if (threshold < s_last_threshold - IMAGE_OTSU_THRESHOLD_SLEW)
            threshold = s_last_threshold - IMAGE_OTSU_THRESHOLD_SLEW;
        s_last_threshold = threshold;
    }
    my_image.Threshold = threshold;

    for (r = 0; r < IMG_H; r++)
    {
        const uint8 *src = img_gray[r];
        uint8       *dst = my_image.image_two_value[r];

        if (r < 2)
        {
            memset(dst, IMG_BLACK, IMG_W);
            continue;
        }

        dst[0] = IMG_BLACK;
        dst[1] = IMG_BLACK;
        for (c = 2; c < IMG_W - 2; c++)
            dst[c] = (src[c] >= threshold) ? IMG_WHITE : IMG_BLACK;
        dst[IMG_W - 2] = IMG_BLACK;
        dst[IMG_W - 1] = IMG_BLACK;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在近端行区间里找一个落在赛道白区内的种子列
// 参数说明     hint             起点提示列，一般是上一帧的近端中线
// 返回参数     int              种子列，-1 表示近端没有够宽的白区
// 使用示例     seed = en_find_seed(s_seed_col);
//-------------------------------------------------------------------------------------------------------------------
static int en_find_seed(int hint)
{
    int row, col, lx, rx;
    int run_left, run_right, run_width;
    int best_col = -1;
    int best_score = -32767;

    hint = iclip(hint, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT);

    // 优先沿用上一帧的近端中线，正常行驶时不扫描整行
    for (row = EN_START_ROW_BOTTOM; row >= EN_START_ROW_TOP; row--)
    {
        if (my_image.image_two_value[row][hint] != IMG_WHITE) continue;

        for (lx = hint; lx > EN_COL_MIN_LIMIT &&
                        my_image.image_two_value[row][lx - 1] == IMG_WHITE; lx--) { }
        for (rx = hint; rx < EN_COL_MAX_LIMIT &&
                        my_image.image_two_value[row][rx + 1] == IMG_WHITE; rx++) { }

        if (rx - lx + 1 >= EN_MIN_ROAD_WIDE) return (lx + rx) / 2;
    }

    // 提示列不可用时在近端区找最宽的白色连通段，仅作为恢复兜底
    for (row = EN_START_ROW_BOTTOM; row >= EN_START_ROW_TOP; row--)
    {
        col = EN_COL_MIN_LIMIT;
        while (col <= EN_COL_MAX_LIMIT)
        {
            while (col <= EN_COL_MAX_LIMIT &&
                   my_image.image_two_value[row][col] != IMG_WHITE)
                col++;
            run_left = col;
            while (col <= EN_COL_MAX_LIMIT &&
                   my_image.image_two_value[row][col] == IMG_WHITE)
                col++;
            run_right = col - 1;
            run_width = run_right - run_left + 1;

            if (run_width >= EN_MIN_ROAD_WIDE)
            {
                int center = (run_left + run_right) / 2;
                int score = 2 * run_width - 3 * func_abs(center - hint);
                if (score > best_score)
                {
                    best_score = score;
                    best_col = center;
                }
            }
        }
        if (best_col >= 0) break;
    }
    return best_col;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取包含种子列的那一段白区，种子落在黑区时先就近拉回
// 参数说明     pixels/seed      本行二值数据与种子列
// 参数说明     left/right       输出白区左右端列
// 返回参数     uint8            1=本行在种子附近有够宽的赛道白区
// 使用示例     if (!image_row_run(pixels, seed, &lx, &rx)) continue;
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_row_run(const uint8 *pixels, int seed, int *left, int *right)
{
    int lx, rx, step, found = -1;

    seed = iclip(seed, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT);
    if (pixels[seed] == IMG_WHITE) found = seed;
    else
        for (step = 1; step <= IMAGE_SEED_SEARCH_RADIUS; step++)
        {
            if (seed - step >= EN_COL_MIN_LIMIT && pixels[seed - step] == IMG_WHITE)
            { found = seed - step; break; }
            if (seed + step <= EN_COL_MAX_LIMIT && pixels[seed + step] == IMG_WHITE)
            { found = seed + step; break; }
        }
    if (found < 0) return 0;

    for (lx = found; lx > EN_COL_MIN_LIMIT && pixels[lx - 1] == IMG_WHITE; lx--) { }
    for (rx = found; rx < EN_COL_MAX_LIMIT && pixels[rx + 1] == IMG_WHITE; rx++) { }
    if (rx - lx + 1 < EN_MIN_RUN) return 0;
    *left = lx;
    *right = rx;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从赛道内部白点向左找第一个"白黑黑"跳变，即左边线
// 参数说明     pixels/seed      本行二值数据与赛道内部起点列
// 参数说明     edge             输出边线列
// 返回参数     uint8            1=边线在画面内量到 0=白到画面外，本行左边线无观测
// 使用示例     left_ok = image_edge_left(pixels, inside, &left_col);
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_edge_left(const uint8 *pixels, int seed, int *edge)
{
    int j;

    for (j = iclip(seed, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT); j >= EN_COL_MIN_LIMIT + 2; j--)
    {
        if (pixels[j] != IMG_WHITE) continue;
        if (pixels[j - 1] == IMG_BLACK && pixels[j - 2] == IMG_BLACK)
        {
            *edge = j;
            return 1;
        }
    }
    *edge = EN_COL_MIN_LIMIT;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从赛道内部白点向右找第一个"白黑黑"跳变，即右边线
// 参数说明     pixels/seed      本行二值数据与赛道内部起点列
// 参数说明     edge             输出边线列
// 返回参数     uint8            1=边线在画面内量到 0=白到画面外，本行右边线无观测
// 使用示例     right_ok = image_edge_right(pixels, inside, &right_col);
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_edge_right(const uint8 *pixels, int seed, int *edge)
{
    int j;

    for (j = iclip(seed, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT); j <= EN_COL_MAX_LIMIT - 2; j++)
    {
        if (pixels[j] != IMG_WHITE) continue;
        if (pixels[j + 1] == IMG_BLACK && pixels[j + 2] == IMG_BLACK)
        {
            *edge = j;
            return 1;
        }
    }
    *edge = EN_COL_MAX_LIMIT;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     允许跨过窄黑条的左边线扫描，用于斑马线和赛道上的污渍
// 参数说明     pixels/seed      本行二值数据与赛道内部起点列
// 参数说明     limit            黑段短于等于这么多列就当噪声跨过去
// 参数说明     stop_col         最远只找到这一列，防止跨出赛道
// 参数说明     edge             输出边线列
// 返回参数     uint8            1=边线在画面内量到
// 使用示例     left_ok = image_edge_left_gap(pixels, inside, limit, stop, &left_col);
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_edge_left_gap(const uint8 *pixels, int seed, int limit,
                                 int stop_col, int *edge)
{
    int j;
    int last_white = iclip(seed, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT);
    int black = 0;

    stop_col = iclip(stop_col, EN_COL_MIN_LIMIT, last_white);
    for (j = last_white; j >= stop_col; j--)
    {
        if (pixels[j] == IMG_WHITE) { last_white = j; black = 0; continue; }
        if (++black > limit)
        {
            *edge = last_white;
            return (uint8)(last_white > EN_COL_MIN_LIMIT);
        }
    }
    *edge = last_white;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     允许跨过窄黑条的右边线扫描，用于斑马线和赛道上的污渍
// 参数说明     pixels/seed      本行二值数据与赛道内部起点列
// 参数说明     limit            黑段短于等于这么多列就当噪声跨过去
// 参数说明     stop_col         最远只找到这一列，防止跨出赛道
// 参数说明     edge             输出边线列
// 返回参数     uint8            1=边线在画面内量到
// 使用示例     right_ok = image_edge_right_gap(pixels, inside, limit, stop, &right_col);
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_edge_right_gap(const uint8 *pixels, int seed, int limit,
                                  int stop_col, int *edge)
{
    int j;
    int last_white = iclip(seed, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT);
    int black = 0;

    stop_col = iclip(stop_col, last_white, EN_COL_MAX_LIMIT);
    for (j = last_white; j <= stop_col; j++)
    {
        if (pixels[j] == IMG_WHITE) { last_white = j; black = 0; continue; }
        if (++black > limit)
        {
            *edge = last_white;
            return (uint8)(last_white < EN_COL_MAX_LIMIT);
        }
    }
    *edge = last_white;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     沿白色赛道连通区逐行提取左右边线
// 参数说明     void
// 返回参数     void
// 使用示例     image_get_edge();
//-------------------------------------------------------------------------------------------------------------------
static void image_get_edge(void)
{
    int i, row;
    int seed;
    int scan_top_row = IMG_H;
    int blank_run = 0;
    int left_last = 0, left_last_row = -1;
    int right_last = 0, right_last_row = -1;
    uint8 started = 0;

    for (i = 0; i < IMG_H; i++)
    {
        my_image.Left_Line[i] = EN_COL_MIN_LIMIT;
        my_image.Right_Line[i] = EN_COL_MAX_LIMIT;
        my_image.Left_Lost_Flag[i] = 1;
        my_image.Right_Lost_Flag[i] = 1;
        my_image.Left_Source[i] = IMAGE_POINT_NONE;
        my_image.Right_Source[i] = IMAGE_POINT_NONE;
        my_image.Road_Perp_Wide[i] = 0;
        my_image.Road_Wide[i] = 0;
    }
    my_image.Search_Stop_Line = 0;
    my_image.Edge_Row_Bottom = -1;
    my_image.Left_Row_Bottom = -1;
    my_image.Right_Row_Bottom = -1;
    my_image.Boundry_Start_Left = -1;
    my_image.Boundry_Start_Right = -1;
    my_image.Left_Lost_Counter = IMG_H;
    my_image.Right_Lost_Counter = IMG_H;
    my_image.Both_Lost_Counter = IMG_H;

    seed = en_find_seed(s_seed_col);
    if (seed < 0)
    {
        s_seed_col = IMG_MID_COL;
        return;
    }
    seed = iclip(seed, EN_COL_MIN_LIMIT + 1, EN_COL_MAX_LIMIT - 1);

    for (row = IMG_H - 1; row >= EN_ROW_TOP_LIMIT; row--)
    {
        const uint8 *pixels = my_image.image_two_value[row];
        int lx = 0, rx = 0;
        int inside;
        int left_col = EN_COL_MIN_LIMIT;
        int right_col = EN_COL_MAX_LIMIT;
        int next_seed;
        uint8 left_ok, right_ok;

        if (!image_row_run(pixels, seed, &lx, &rx))
        {
            // 本行在种子附近没有赛道白区：远端到头，或近端还没进赛道
            if (started)
            {
                if (++blank_run > EN_BOTH_LOST_MAX) break;
            }
            else if (row < EN_START_ROW_TOP)
            {
                break;                              // 近端 20 行都没找到赛道，这帧没得看
            }
            continue;
        }
        blank_run = 0;
        started = 1;
        scan_top_row = row;
        inside = (lx + rx) / 2;

        left_ok = image_edge_left(pixels, inside, &left_col);
        right_ok = image_edge_right(pixels, inside, &right_col);

        // 白区明显窄于本行预计赛宽 = 赛道内部被条纹或污渍切开了，改用能跨过窄黑条的扫描
        {
            int expected = iclip(2 * Road_Half_Wide[row], EN_MIN_RUN, 2 * IMG_W);

            if ((right_col - left_col + 1) * 100 < expected * IMAGE_EDGE_WIDTH_MIN_PCT)
            {
                int gap = iclip(expected / 5, IMAGE_EDGE_GAP_MIN, IMAGE_EDGE_GAP_MAX);

                left_ok = image_edge_left_gap(pixels, inside, gap,
                                              inside - expected, &left_col);
                right_ok = image_edge_right_gap(pixels, inside, gap,
                                                inside + expected, &right_col);
            }
        }

        if (left_ok && left_last_row >= 0 && left_last_row - row <= 4 &&
            func_abs(left_col - left_last) > EN_EDGE_TRACK * (left_last_row - row))
            left_ok = 0;
        if (right_ok && right_last_row >= 0 && right_last_row - row <= 4 &&
            func_abs(right_col - right_last) > EN_EDGE_TRACK * (right_last_row - row))
            right_ok = 0;
        if (left_ok && right_ok && right_col - left_col + 1 < EN_MIN_RUN)
        {
            left_ok = 0;
            right_ok = 0;
        }

        if (left_ok)
        {
            my_image.Left_Line[row] = left_col;
            my_image.Left_Lost_Flag[row] = 0;
            my_image.Left_Source[row] = IMAGE_POINT_MEASURED;
            left_last = left_col;
            left_last_row = row;
        }
        else
        {
            my_image.Left_Line[row] = iclip(left_col, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT - 1);
        }
        if (right_ok)
        {
            my_image.Right_Line[row] = right_col;
            my_image.Right_Lost_Flag[row] = 0;
            my_image.Right_Source[row] = IMAGE_POINT_MEASURED;
            right_last = right_col;
            right_last_row = row;
        }
        else
        {
            my_image.Right_Line[row] = iclip(right_col, my_image.Left_Line[row] + 1,
                                             EN_COL_MAX_LIMIT);
        }

        // 种子交给下一行：双边取中，单边按半赛宽偏，两边都量不到就原地不动
        if (left_ok && right_ok)      next_seed = (left_col + right_col) / 2;
        else if (left_ok)             next_seed = left_col + Road_Half_Wide[row];
        else if (right_ok)            next_seed = right_col - Road_Half_Wide[row];
        else                          next_seed = seed;
        next_seed = iclip(next_seed, seed - EN_SEED_STEP_MAX, seed + EN_SEED_STEP_MAX);
        next_seed = iclip(next_seed, lx, rx);
        seed = iclip(next_seed, EN_COL_MIN_LIMIT + 1, EN_COL_MAX_LIMIT - 1);
    }

    if (!started)
    {
        s_seed_col = IMG_MID_COL;
        return;
    }

    scan_top_row = iclip(scan_top_row, 0, IMG_H - 1);
    my_image.Search_Stop_Line = IMG_H - scan_top_row;

    for (i = IMG_H - 1; i >= 0 && my_image.Left_Lost_Flag[i]; i--) { }
    my_image.Left_Row_Bottom = i;
    my_image.Boundry_Start_Left = i;
    for (i = IMG_H - 1; i >= 0 && my_image.Right_Lost_Flag[i]; i--) { }
    my_image.Right_Row_Bottom = i;
    my_image.Boundry_Start_Right = i;
    for (i = IMG_H - 1; i >= 0; i--)
        if (!my_image.Left_Lost_Flag[i] && !my_image.Right_Lost_Flag[i])
        {
            my_image.Edge_Row_Bottom = i;
            break;
        }

    // 垂直于赛道方向的宽度，只有双边都实测且上下都实测的行才算得出来，其余保持 0
    for (i = IMG_H - 1; i >= scan_top_row; i--)
    {
        int near_row = i + EN_SLOPE_SPAN;
        int far_row = i - EN_SLOPE_SPAN;
        float slope;

        if (near_row >= IMG_H || far_row < scan_top_row) continue;
        if (my_image.Left_Lost_Flag[i] || my_image.Right_Lost_Flag[i] ||
            my_image.Left_Lost_Flag[near_row] || my_image.Right_Lost_Flag[near_row] ||
            my_image.Left_Lost_Flag[far_row] || my_image.Right_Lost_Flag[far_row]) continue;

        slope = (float)((my_image.Left_Line[far_row] + my_image.Right_Line[far_row]) -
                        (my_image.Left_Line[near_row] + my_image.Right_Line[near_row])) /
                (float)(4 * EN_SLOPE_SPAN);
        my_image.Road_Perp_Wide[i] =
            (int)((float)(my_image.Right_Line[i] - my_image.Left_Line[i]) /
                  sqrtf(1.0f + slope * slope) + 0.5f);
    }

    {
        int left_bottom = (my_image.Left_Row_Bottom >= 0)
                        ? my_image.Left_Row_Bottom : IMG_H - 1;
        int right_bottom = (my_image.Right_Row_Bottom >= 0)
                         ? my_image.Right_Row_Bottom : IMG_H - 1;
        int both_bottom = (left_bottom < right_bottom) ? left_bottom : right_bottom;

        my_image.Left_Lost_Counter = 0;
        my_image.Right_Lost_Counter = 0;
        my_image.Both_Lost_Counter = 0;
        for (i = IMG_H - 1; i >= scan_top_row; i--)
        {
            my_image.Road_Wide[i] = my_image.Right_Line[i] - my_image.Left_Line[i];
            if (i <= left_bottom && my_image.Left_Lost_Flag[i])
                my_image.Left_Lost_Counter++;
            if (i <= right_bottom && my_image.Right_Lost_Flag[i])
                my_image.Right_Lost_Counter++;
            if (i <= both_bottom &&
                my_image.Left_Lost_Flag[i] && my_image.Right_Lost_Flag[i])
                my_image.Both_Lost_Counter++;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点限幅
// 参数说明     value/low/high  输入值、下限、上限
// 返回参数     float           限幅结果
// 使用示例     value = image_fclip(value, 0.0f, 1.0f);
//-------------------------------------------------------------------------------------------------------------------
static float image_fclip(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查拟合用浮点数
// 参数说明     value           待检查数值
// 返回参数     uint8           1=有效
// 使用示例     if (!image_float_valid(value)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_float_valid(float value)
{
    return (uint8)(value == value && fabsf(value) < 1.0e6f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询边线来源权重
// 参数说明     source          image_point_source_t
// 返回参数     float           拟合权重
// 使用示例     weight = image_source_weight(source);
//-------------------------------------------------------------------------------------------------------------------
static float image_source_weight(uint8 source)
{
    if (source == IMAGE_POINT_MEASURED) return 1.0f;
    if (source == IMAGE_POINT_REPAIRED) return 0.65f;
    return 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算中线二次曲线值
// 参数说明     coefficient/u   系数与归一化纵向坐标
// 返回参数     float           归一化横向坐标
// 使用示例     z = image_poly_value(coefficient, u);
//-------------------------------------------------------------------------------------------------------------------
static float image_poly_value(const float coefficient[3], float u)
{
    return coefficient[0] + coefficient[1] * u + coefficient[2] * u * u;
}

static float s_left_u[IMG_H];
static float s_left_z[IMG_H];
static float s_right_u[IMG_H];
static float s_right_z[IMG_H];
static uint8 s_left_bev_valid[IMG_H];
static uint8 s_right_bev_valid[IMG_H];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按逆透视矩阵重算每行的图像半赛宽
// 参数说明     void
// 返回参数     void
// 使用示例     image_update_half_width();
//-------------------------------------------------------------------------------------------------------------------
//
// 标定过之后半赛宽是算得出来的，不必再靠双边实测行慢慢学：
// 俯视平面上赛道宽恒为 2*IPM_HALF_WIDTH，把 ±IPM_HALF_WIDTH 投回本行即可。
// 近端赛道宽出画面的机位上这一步尤其重要 —— 那几十行永远学不到表。
static void image_update_half_width(void)
{
    int row;

    for (row = 0; row < IMG_H; row++) s_row_depth_ok[row] = 0u;
    if (!ipm_ready()) return;
    for (row = 0; row < IMG_H; row++)
    {
        float x, y, cl, rl, cr, rr;

        if (!ipm_to_bev((float)IMG_MID_COL, (float)row, &x, &y)) continue;
        // 顺便记下这一行在俯视平面里的纵深，逐行限幅要用它来判断"这一行跨了多远地面"
        s_row_depth[row] = -y / IMAGE_BEV_SCALE;
        s_row_depth_ok[row] = (uint8)image_float_valid(s_row_depth[row]);
        if (!ipm_to_img(-(float)IPM_HALF_WIDTH, y, &cl, &rl)) continue;
        if (!ipm_to_img((float)IPM_HALF_WIDTH, y, &cr, &rr)) continue;
        if (!image_float_valid(cl) || !image_float_valid(cr) || cr <= cl) continue;
        Road_Half_Wide[row] = iclip((int)(0.5f * (cr - cl) + 0.5f), ROAD_HALF_MIN, IMG_W);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把一侧边线的某一行换到归一化坐标
// 参数说明     row/left        行号与侧别(1=左)
// 参数说明     u/z             输出纵向、横向归一化坐标
// 返回参数     uint8           1=这一行本侧有可用实测点
// 使用示例     if (image_edge_normalize(row, 1, &u, &z)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_edge_normalize(int row, uint8 left, float *u, float *z)
{
    int lost = left ? my_image.Left_Lost_Flag[row] : my_image.Right_Lost_Flag[row];
    uint8 source = left ? my_image.Left_Source[row] : my_image.Right_Source[row];
    int col = left ? my_image.Left_Line[row] : my_image.Right_Line[row];

    if (lost || image_source_weight(source) <= 0.0f) return 0;
    if (ipm_ready())
    {
        float x, y;

        if (!ipm_to_bev((float)col, (float)row, &x, &y)) return 0;
        *u = -y / IMAGE_BEV_SCALE;
        *z = x / IMAGE_BEV_SCALE;
    }
    else
    {
        int half = Road_Half_Wide[row];

        if (half < ROAD_HALF_MIN) return 0;
        *u = (float)(IMG_H - 1 - row) / (float)(IMG_H - 1);
        *z = (float)(col - IMG_MID_COL) / (float)half;
    }
    if (!image_float_valid(*u) || !image_float_valid(*z)) return 0;
    return (uint8)(*u > IMAGE_U_MIN && *u < IMAGE_U_MAX && fabsf(*z) < 4.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整帧缓存左右边线的归一化坐标
// 参数说明     void
// 返回参数     void
// 使用示例     image_prepare_normalized_edges();
//-------------------------------------------------------------------------------------------------------------------
static void image_prepare_normalized_edges(void)
{
    int row;

    for (row = 0; row < IMG_H; row++)
    {
        s_left_bev_valid[row] = image_edge_normalize(row, 1u, &s_left_u[row], &s_left_z[row]);
        s_right_bev_valid[row] = image_edge_normalize(row, 0u, &s_right_u[row], &s_right_z[row]);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     求某一行某侧边线在俯视平面里的斜率 dz/du
// 参数说明     row/left        行号与侧别(1=左)
// 参数说明     slope           输出斜率
// 返回参数     uint8           1=求到了
// 使用示例     if (image_edge_slope(row, 1, &s)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_edge_slope(int row, uint8 left, float *slope)
{
    const float *edge_u = left ? s_left_u : s_right_u;
    const float *edge_z = left ? s_left_z : s_right_z;
    const uint8 *valid = left ? s_left_bev_valid : s_right_bev_valid;
    int near_row = -1, far_row = -1, step;
    float du;

    for (step = IMAGE_SLOPE_SPAN; step >= 2; step--)
    {
        if (near_row < 0 && row + step < IMG_H && valid[row + step]) near_row = row + step;
        if (far_row < 0 && row - step >= 0 && valid[row - step]) far_row = row - step;
    }
    if (near_row < 0 && valid[row]) near_row = row;
    if (far_row < 0 && valid[row]) far_row = row;
    if (near_row < 0 || far_row < 0 || near_row == far_row) return 0;

    du = edge_u[far_row] - edge_u[near_row];
    if (fabsf(du) < 1.0e-4f) return 0;
    *slope = (edge_z[far_row] - edge_z[near_row]) / du;
    return (uint8)image_float_valid(*slope);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     俯视平面里某点到一侧边线折线的垂直距离
// 参数说明     u/z             点的俯视坐标
// 参数说明     row/left        参考行与侧别(1=左)，只在这一行附近找垂足
// 返回参数     float           距离(以半赛宽为单位)
// 使用示例     d = image_edge_distance(u, z, row, 1);
//-------------------------------------------------------------------------------------------------------------------
static float image_edge_distance(float u, float z, int row, uint8 left)
{
    const float *edge_u = left ? s_left_u : s_right_u;
    const float *edge_z = left ? s_left_z : s_right_z;
    const uint8 *valid = left ? s_left_bev_valid : s_right_bev_valid;
    int lo = iclip(row - IMAGE_FOOT_SPAN, 0, IMG_H - 1);
    int hi = iclip(row + IMAGE_FOOT_SPAN, 0, IMG_H - 1);
    int i, previous = -1;
    float best = 1.0e9f;

    for (i = lo; i <= hi; i++)
    {
        float du, dz, d;

        if (!valid[i]) continue;
        du = u - edge_u[i];
        dz = z - edge_z[i];
        d = du * du + dz * dz;
        if (d < best) best = d;
        if (previous >= 0)                          // 再算一次到线段的距离，远端顶点稀疏
        {
            float su = edge_u[i] - edge_u[previous];
            float sz = edge_z[i] - edge_z[previous];
            float len = su * su + sz * sz;

            if (len > 1.0e-8f)
            {
                float t = ((u - edge_u[previous]) * su + (z - edge_z[previous]) * sz) / len;

                if (t > 0.0f && t < 1.0f)
                {
                    du = u - (edge_u[previous] + t * su);
                    dz = z - (edge_z[previous] + t * sz);
                    d = du * du + dz * dz;
                    if (d < best) best = d;
                }
            }
        }
        previous = i;
    }
    return (best > 1.0e8f) ? -1.0f : sqrtf(best);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单边行：沿本行二分，找到离边线正好一个半赛宽的那一列
// 参数说明     row/left        行号与可见的那一侧(1=左)
// 参数说明     column          输出中线列
// 返回参数     uint8           1=找到了
// 使用示例     if (image_single_col(row, 1u, &col)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_single_col(int row, uint8 left, float *column)
{
    float low = (float)(left ? my_image.Left_Line[row] : my_image.Right_Line[row]);
    float high = low + (left ? 1.0f : -1.0f) *
                 (float)Road_Half_Wide[row] * IMAGE_SLOPE_CORR_MAX;
    int i;

    high = image_fclip(high, -(float)IMG_W, 2.0f * (float)IMG_W);
    {
        // 先确认区间外端确实已经超过一个半宽，否则二分无解，退回系数法
        float x, y, distance;

        if (!ipm_to_bev(high, (float)row, &x, &y)) return 0;
        distance = image_edge_distance(-y / IMAGE_BEV_SCALE, x / IMAGE_BEV_SCALE, row, left);
        if (distance < 1.0f) return 0;
    }
    for (i = 0; i < IMAGE_BISECT_STEPS; i++)
    {
        float middle = 0.5f * (low + high);
        float x, y, distance;

        if (!ipm_to_bev(middle, (float)row, &x, &y)) return 0;
        distance = image_edge_distance(-y / IMAGE_BEV_SCALE, x / IMAGE_BEV_SCALE, row, left);
        if (distance < 0.0f) return 0;
        if (distance < 1.0f) low = middle;
        else high = middle;
    }
    *column = 0.5f * (low + high);
    return (uint8)image_float_valid(*column);
}

static float s_row_corr[IMG_H];
static uint8 s_row_corr_ok[IMG_H];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把修正系数数组补洞并平滑
// 参数说明     void
// 返回参数     void
// 使用示例     image_corr_fill();
//-------------------------------------------------------------------------------------------------------------------
static void image_corr_fill(void)
{
    int row, last = -1;
    float tmp[IMG_H];

    for (row = IMG_H - 1; row >= 0; row--)          // 由近到远，用近端的值往远端填
    {
        if (s_row_corr_ok[row]) { last = row; continue; }
        if (last >= 0) s_row_corr[row] = s_row_corr[last];
    }
    last = -1;
    for (row = 0; row < IMG_H; row++)               // 再由远到近补一遍开头
    {
        if (s_row_corr_ok[row]) { last = row; continue; }
        if (last >= 0) s_row_corr[row] = s_row_corr[last];
    }

    memcpy(tmp, s_row_corr, sizeof(tmp));
    for (row = 2; row < IMG_H - 2; row++)
        s_row_corr[row] = (tmp[row - 2] + 2.0f * tmp[row - 1] + 3.0f * tmp[row] +
                           2.0f * tmp[row + 1] + tmp[row + 2]) * (1.0f / 9.0f);
    for (row = 0; row < IMG_H; row++)
        s_row_corr[row] = image_fclip(s_row_corr[row], 1.0f, IMAGE_SLOPE_CORR_MAX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     由左右边线的俯视斜率生成逐行修正系数
// 参数说明     void
// 返回参数     void
// 使用示例     image_build_corr();
//-------------------------------------------------------------------------------------------------------------------
static void image_build_corr(void)
{
    int row;

    for (row = 0; row < IMG_H; row++)
    {
        float slope, sum = 0.0f, column;
        int n = 0;

        s_row_corr[row] = 1.0f;
        s_row_corr_ok[row] = 0u;

        if (ipm_ready())
        {
            if (s_left_bev_valid[row] && image_single_col(row, 1u, &column))
            {
                float factor = (column - (float)my_image.Left_Line[row]) /
                               (float)Road_Half_Wide[row];

                if (factor >= 1.0f && factor <= IMAGE_SLOPE_CORR_MAX) { sum += factor; n++; }
            }
            if (s_right_bev_valid[row] && image_single_col(row, 0u, &column))
            {
                float factor = ((float)my_image.Right_Line[row] - column) /
                               (float)Road_Half_Wide[row];

                if (factor >= 1.0f && factor <= IMAGE_SLOPE_CORR_MAX) { sum += factor; n++; }
            }
        }
        // 兜底：按边线的俯视斜率算 1/cos。大曲率下偏小，但总比没有强
        if (n == 0)
        {
            if (s_left_bev_valid[row] && image_edge_slope(row, 1u, &slope))
            { sum += sqrtf(1.0f + slope * slope); n++; }
            if (s_right_bev_valid[row] && image_edge_slope(row, 0u, &slope))
            { sum += sqrtf(1.0f + slope * slope); n++; }
        }
        if (n > 0) { s_row_corr[row] = sum / (float)n; s_row_corr_ok[row] = 1u; }
    }
    image_corr_fill();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐行构建中线
// 参数说明     measured_dual   输出双边实测行数
// 返回参数     int             有效中线行数
// 使用示例     rows = image_build_rows(&dual);
//-------------------------------------------------------------------------------------------------------------------
static int image_build_rows(int *measured_dual)
{
    int row;
    int count = 0;
    int top = IMG_H - iclip(my_image.Search_Stop_Line, 0, IMG_H);

    *measured_dual = 0;
    image_prepare_normalized_edges();
    image_build_corr();
    for (row = IMG_H - 1; row >= top; row--)
    {
        uint8 left_ok = s_left_bev_valid[row];
        uint8 right_ok = s_right_bev_valid[row];
        float middle;

        if (left_ok && right_ok)
        {
            if (my_image.Right_Line[row] <= my_image.Left_Line[row] + 1) continue;
            middle = 0.5f * (float)(my_image.Left_Line[row] + my_image.Right_Line[row]);
            my_image.Mid_Source[row] = IMAGE_POINT_MEASURED;
            (*measured_dual)++;
        }
        else if (left_ok)
        {
            middle = (float)my_image.Left_Line[row] +
                     (float)Road_Half_Wide[row] * s_row_corr[row];
            my_image.Mid_Source[row] = IMAGE_POINT_REPAIRED;
        }
        else if (right_ok)
        {
            middle = (float)my_image.Right_Line[row] -
                     (float)Road_Half_Wide[row] * s_row_corr[row];
            my_image.Mid_Source[row] = IMAGE_POINT_REPAIRED;
        }
        else continue;

        if (!image_float_valid(middle) ||
            middle < (float)IMAGE_MID_EDGE_KEEP ||
            middle > (float)(IMG_W - 1 - IMAGE_MID_EDGE_KEEP))
        {
            my_image.Mid_Source[row] = IMAGE_POINT_NONE;
            continue;
        }
        my_image.Mid_Line[row] = iclip((int)(middle + 0.5f), 0, IMG_W - 1);
        my_image.Mid_Lost_Flag[row] = 0;
        count++;
    }
    return count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐行限幅，削掉双边区换到单边区之类的台阶
// 参数说明     void
// 返回参数     void
// 使用示例     image_slew_midline();
//-------------------------------------------------------------------------------------------------------------------
static void image_slew_midline(void)
{
    int row, last_row = -1;

    for (row = IMG_H - 1; row >= 0; row--)
    {
        int limit, low, high;

        if (my_image.Mid_Lost_Flag[row]) continue;
        if (last_row < 0) { last_row = row; continue; }

        limit = MID_SLEW_MAX * (last_row - row);
        if (ipm_ready() && s_row_depth_ok[row] && s_row_depth_ok[last_row])
        {
            float du = fabsf(s_row_depth[row] - s_row_depth[last_row]);
            float allow_z = MID_SLOPE_MAX * du + MID_SLEW_Z_BASE;
            float allow_col = allow_z * (float)Road_Half_Wide[row];

            if (image_float_valid(allow_col) && allow_col > (float)limit)
                limit = (int)allow_col;
        }
        if (limit > IMG_W) limit = IMG_W;
        low = my_image.Mid_Line[last_row] - limit;
        high = my_image.Mid_Line[last_row] + limit;
        my_image.Mid_Line[row] = iclip(my_image.Mid_Line[row], low, high);
        last_row = row;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把中线从最近的有效行沿局部斜率续到画面底部
// 参数说明     void
// 返回参数     void
// 使用示例     image_extend_near_rows();
//-------------------------------------------------------------------------------------------------------------------
static void image_extend_near_rows(void)
{
    int row, bottom = -1, n = 0;
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, det, slope, base;

    for (row = IMG_H - 1; row >= 0; row--)
        if (!my_image.Mid_Lost_Flag[row]) { bottom = row; break; }
    if (bottom < 0 || bottom >= IMG_H - 1) return;

    for (row = bottom; row >= 0 && n < IMAGE_EXTEND_FIT_ROWS; row--)
    {
        if (my_image.Mid_Lost_Flag[row]) continue;
        sx += (float)row;
        sy += (float)my_image.Mid_Line[row];
        sxx += (float)row * (float)row;
        sxy += (float)row * (float)my_image.Mid_Line[row];
        n++;
    }
    if (n < 4) return;
    det = (float)n * sxx - sx * sx;
    if (fabsf(det) < 1.0f) return;
    slope = ((float)n * sxy - sx * sy) / det;
    base = (sy - slope * sx) / (float)n;
    if (!image_float_valid(slope) || !image_float_valid(base)) return;
    slope = image_fclip(slope, -IMAGE_EXTEND_SLOPE_MAX, IMAGE_EXTEND_SLOPE_MAX);

    for (row = bottom + 1; row < IMG_H; row++)
    {
        float middle = slope * (float)row + base;

        if (!image_float_valid(middle)) break;
        my_image.Mid_Line[row] = iclip((int)(middle + 0.5f), 0, IMG_W - 1);
        my_image.Mid_Lost_Flag[row] = 0;
        my_image.Mid_Source[row] = IMAGE_POINT_PREDICTED;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把近端一段中线换到俯视平面，供转向标量的最小二乘用
// 参数说明     window_dual     输出窗口内的双边实测行数
// 返回参数     int             进入拟合的点数
// 使用示例     n = image_fill_preview_window(&dual);
//-------------------------------------------------------------------------------------------------------------------
static int image_fill_preview_window(int *window_dual)
{
    int row, n = 0;
    float u_near = 0.0f;
    uint8 have_near = 0;

    *window_dual = 0;
    for (row = IMG_H - 1; row >= 0 && n < IMG_H; row--)
    {
        float u, z;

        if (my_image.Mid_Lost_Flag[row]) continue;
        if (ipm_ready())
        {
            float x, y;

            if (!ipm_to_bev((float)my_image.Mid_Line[row], (float)row, &x, &y)) continue;
            u = -y / IMAGE_BEV_SCALE;
            z = x / IMAGE_BEV_SCALE;
        }
        else
        {
            int half = Road_Half_Wide[row];

            if (half < ROAD_HALF_MIN) continue;
            u = (float)(IMG_H - 1 - row) / (float)(IMG_H - 1);
            z = (float)(my_image.Mid_Line[row] - IMG_MID_COL) / (float)half;
        }
        if (!image_float_valid(u) || !image_float_valid(z)) continue;
        if (!have_near) { u_near = u; have_near = 1u; }
        if (u > u_near + IMAGE_PREVIEW_SPAN) break;

        s_fit_u[n] = u;
        s_fit_z[n] = z;
        s_fit_w[n] = (my_image.Mid_Source[row] == IMAGE_POINT_MEASURED)
                   ? 1.0f : IMAGE_SINGLE_WEIGHT;
        s_fit_base_w[n] = s_fit_w[n];
        if (my_image.Mid_Source[row] == IMAGE_POINT_MEASURED) (*window_dual)++;
        n++;
    }
    return n;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在两个有效中线行之间补齐短空洞
// 参数说明     void
// 返回参数     void
// 使用示例     image_fill_mid_gaps();
//-------------------------------------------------------------------------------------------------------------------
static void image_fill_mid_gaps(void)
{
    int row = 0;

    while (row < IMG_H)
    {
        int gap_start, gap_end, far_row, near_row, fill_row;

        if (!my_image.Mid_Lost_Flag[row]) { row++; continue; }
        gap_start = row;
        while (row < IMG_H && my_image.Mid_Lost_Flag[row]) row++;
        gap_end = row - 1;
        far_row = gap_start - 1;
        near_row = row;
        if (gap_end - gap_start + 1 > IMAGE_MID_HOLE_MAX ||
            far_row < 0 || near_row >= IMG_H) continue;

        for (fill_row = gap_start; fill_row <= gap_end; fill_row++)
        {
            int numerator = (my_image.Mid_Line[near_row] - my_image.Mid_Line[far_row]) *
                            (fill_row - far_row);

            my_image.Mid_Line[fill_row] = iclip(my_image.Mid_Line[far_row] +
                                                numerator / (near_row - far_row),
                                                0, IMG_W - 1);
            my_image.Mid_Lost_Flag[fill_row] = 0;
            my_image.Mid_Source[fill_row] = IMAGE_POINT_REPAIRED;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行加权二次最小二乘
// 参数说明     count/coefficient 候选数量与输出系数
// 参数说明     used/u_min/u_max  输出点数与纵向范围
// 返回参数     uint8           1=拟合成功
// 使用示例     if (image_fit_solve(count, c, &used, &u0, &u1)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_fit_solve(int count, float coefficient[3],
                             int *used, float *u_min, float *u_max)
{
    float matrix[3][4] = {{0.0f, 0.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 0.0f, 0.0f}};
    int i;
    int row;
    int col;
    float shift = 0.0f;
    float weight_sum = 0.0f;

    *used = 0;
    *u_min = 1.0e6f;
    *u_max = -1.0e6f;

    for (i = 0; i < count; i++)
        if (s_fit_w[i] > 0.0f) { shift += s_fit_w[i] * s_fit_u[i]; weight_sum += s_fit_w[i]; }
    if (weight_sum > 0.0f) shift /= weight_sum;

    for (i = 0; i < count; i++)
    {
        float w = s_fit_w[i];
        float u = s_fit_u[i] - shift;
        float z = s_fit_z[i];
        float u2;

        if (w <= 0.0f) continue;
        u2 = u * u;
        matrix[0][0] += w;           matrix[0][1] += w * u;
        matrix[0][2] += w * u2;      matrix[0][3] += w * z;
        matrix[1][1] += w * u2;      matrix[1][2] += w * u2 * u;
        matrix[1][3] += w * u * z;   matrix[2][2] += w * u2 * u2;
        matrix[2][3] += w * u2 * z;
        if (u < *u_min) *u_min = u;
        if (u > *u_max) *u_max = u;
        (*used)++;
    }

    if (*used < IMAGE_FIT_MIN_POINTS || *u_max - *u_min < 0.08f) return 0;
    matrix[1][0] = matrix[0][1]; matrix[2][0] = matrix[0][2]; matrix[2][1] = matrix[1][2];

    for (col = 0; col < 3; col++)
    {
        int pivot = col;
        float pivot_abs = fabsf(matrix[col][col]);

        for (row = col + 1; row < 3; row++)
            if (fabsf(matrix[row][col]) > pivot_abs)
            { pivot = row; pivot_abs = fabsf(matrix[row][col]); }
        if (pivot_abs < 1.0e-6f) return 0;
        if (pivot != col)
        {
            int k;
            for (k = col; k < 4; k++)
            { float temp = matrix[col][k]; matrix[col][k] = matrix[pivot][k]; matrix[pivot][k] = temp; }
        }
        {
            float inverse = 1.0f / matrix[col][col];
            int k;
            for (k = col; k < 4; k++) matrix[col][k] *= inverse;
        }
        for (row = 0; row < 3; row++)
        {
            float factor;
            int k;
            if (row == col) continue;
            factor = matrix[row][col];
            for (k = col; k < 4; k++) matrix[row][k] -= factor * matrix[col][k];
        }
    }

    // 从平移坐标展开回原坐标
    coefficient[0] = matrix[0][3] - matrix[1][3] * shift + matrix[2][3] * shift * shift;
    coefficient[1] = matrix[1][3] - 2.0f * matrix[2][3] * shift;
    coefficient[2] = matrix[2][3];
    *u_min += shift;
    *u_max += shift;
    return (uint8)(image_float_valid(coefficient[0]) &&
                   image_float_valid(coefficient[1]) && image_float_valid(coefficient[2]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算拟合加权平均绝对残差
// 参数说明     count/coefficient 候选数量与曲线系数
// 返回参数     float           平均绝对残差
// 使用示例     residual = image_fit_residual(count, coefficient);
//-------------------------------------------------------------------------------------------------------------------
static float image_fit_residual(int count, const float coefficient[3])
{
    float sum = 0.0f;
    float weight_sum = 0.0f;
    int i;

    for (i = 0; i < count; i++)
        if (s_fit_w[i] > 0.0f)
        {
            sum += s_fit_w[i] * fabsf(s_fit_z[i] - image_poly_value(coefficient, s_fit_u[i]));
            weight_sum += s_fit_w[i];
        }
    return weight_sum > 0.0f ? sum / weight_sum : 1.0e6f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一次残差剔除的鲁棒二次拟合
// 参数说明     count/dual      候选总数与双侧实测数
// 参数说明     coefficient/range/quality 输出系数、范围和质量
// 返回参数     uint8           1=拟合成功
// 使用示例     if (image_fit_curve(count, dual, c, &u0, &u1, &q)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_fit_curve(int count, int dual, float coefficient[3],
                             float *u_min, float *u_max, float *quality)
{
    float first[3];
    float first_min;
    float first_max;
    float residual;
    float limit;
    int first_used;
    int used;
    int i;
    int rejected = 0;

    if (!image_fit_solve(count, first, &first_used, &first_min, &first_max)) return 0;
    residual = image_fit_residual(count, first);
    limit = IMAGE_FIT_RESIDUAL_GAIN * residual;
    if (limit < IMAGE_FIT_RESIDUAL_MIN) limit = IMAGE_FIT_RESIDUAL_MIN;
    for (i = 0; i < count; i++)
        if (s_fit_w[i] > 0.0f &&
            fabsf(s_fit_z[i] - image_poly_value(first, s_fit_u[i])) > limit)
        { s_fit_w[i] = 0.0f; rejected++; }

    if (rejected > 0 && image_fit_solve(count, coefficient, &used, u_min, u_max))
        residual = image_fit_residual(count, coefficient);
    else
    {
        memcpy(s_fit_w, s_fit_base_w, sizeof(float) * (uint32)count);
        coefficient[0] = first[0]; coefficient[1] = first[1]; coefficient[2] = first[2];
        used = first_used; *u_min = first_min; *u_max = first_max;
        residual = image_fit_residual(count, coefficient);
    }

    {
        float support = image_fclip((float)used / 32.0f, 0.0f, 1.0f);
        float coverage = image_fclip((*u_max - *u_min) / IMAGE_U_SPAN_FULL, 0.0f, 1.0f);
        float clean = image_fclip(1.0f - residual / 0.30f, 0.0f, 1.0f);
        float dual_ratio = image_fclip((float)dual / (float)(used > 0 ? used : 1), 0.0f, 1.0f);
        *quality = support * coverage * clean * (0.55f + 0.45f * dual_ratio);
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将中线列限制在当前可信左右边界之间
// 参数说明     row/column      图像行与待限制列
// 返回参数     int             限制后的中线列
// 使用示例     middle = image_clamp_mid(row, middle);
//-------------------------------------------------------------------------------------------------------------------
static int image_clamp_mid(int row, int column)
{
    int low = 1;
    int high = IMG_W - 2;

    if (!my_image.Left_Lost_Flag[row]) low = my_image.Left_Line[row] + 1;
    if (!my_image.Right_Lost_Flag[row]) high = my_image.Right_Line[row] - 1;
    low = iclip(low, 1, IMG_W - 2);
    high = iclip(high, 1, IMG_W - 2);
    if (low > high) return -1;
    return iclip(column, low, high);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对连续中线执行 1-2-3-2-1 平滑
// 参数说明     void
// 返回参数     void
// 使用示例     image_filter_midline();
//-------------------------------------------------------------------------------------------------------------------
static void image_filter_midline(void)
{
    int row;

    memcpy(s_mid_filter, my_image.Mid_Line, sizeof(s_mid_filter));
    for (row = 2; row < IMG_H - 2; row++)
    {
        if (my_image.Mid_Lost_Flag[row - 2] || my_image.Mid_Lost_Flag[row - 1] ||
            my_image.Mid_Lost_Flag[row] || my_image.Mid_Lost_Flag[row + 1] ||
            my_image.Mid_Lost_Flag[row + 2]) continue;
        s_mid_filter[row] = (my_image.Mid_Line[row - 2] +
                             2 * my_image.Mid_Line[row - 1] +
                             3 * my_image.Mid_Line[row] +
                             2 * my_image.Mid_Line[row + 1] +
                             my_image.Mid_Line[row + 2] + 4) / 9;
    }
    for (row = 0; row < IMG_H; row++)
        if (!my_image.Mid_Lost_Flag[row])
        {
            int clipped = image_clamp_mid(row, s_mid_filter[row]);

            if (clipped >= 0) my_image.Mid_Line[row] = clipped;
            else
            {
                my_image.Mid_Lost_Flag[row] = 1;
                my_image.Mid_Source[row] = IMAGE_POINT_NONE;
            }
        }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用双侧实测边线逐行学习半赛宽，再线性外推到量不到的行
// 参数说明     void
// 返回参数     void
// 使用示例     image_update_road_width();
//-------------------------------------------------------------------------------------------------------------------
static void image_update_road_width(void)
{
    int row;
    int learned = 0;
    float sr = 0.0f, srr = 0.0f, sh = 0.0f, srh = 0.0f;
    float row_lo = -1.0f, row_hi = -1.0f;

    for (row = ERR_FIT_ROW_TOP; row < IMG_H; row++)
    {
        int half;
        int current;
        int difference;

        if (my_image.Left_Source[row] != IMAGE_POINT_MEASURED ||
            my_image.Right_Source[row] != IMAGE_POINT_MEASURED ||
            my_image.Left_Lost_Flag[row] || my_image.Right_Lost_Flag[row] ||
            my_image.Road_Perp_Wide[row] <= 0) continue;
        half = my_image.Road_Perp_Wide[row] / 2;
        current = Road_Half_Wide[row];
        if ((float)half < (float)current * ROAD_WIDE_MIN_RATIO ||
            (float)half > (float)current * ROAD_WIDE_MAX_RATIO) continue;
        difference = half - current;
        if (difference > 0) current += (difference + 7) / 8;
        else if (difference < 0) current -= (-difference + 7) / 8;
        Road_Half_Wide[row] = iclip(current, ROAD_HALF_MIN, IMG_W);
        if (s_road_learn_count[row] < 255u) s_road_learn_count[row]++;
    }

    for (row = 0; row < IMG_H; row++)
    {
        float r, h;

        if (s_road_learn_count[row] < IMAGE_ROAD_ROW_SAMPLES) continue;
        r = (float)row;
        h = (float)Road_Half_Wide[row];
        if (row_lo < 0.0f) row_lo = r;
        row_hi = r;
        sr += r; srr += r * r; sh += h; srh += r * h;
        learned++;
    }
    if (learned < IMAGE_ROAD_FIT_MIN_ROWS || row_hi - row_lo < IMAGE_ROAD_FIT_MIN_SPAN) return;

    {
        float det = (float)learned * srr - sr * sr;
        float slope, base;

        if (det < 1.0f) return;
        slope = ((float)learned * srh - sr * sh) / det;
        base = (sh - slope * sr) / (float)learned;
        if (slope <= 0.0f) return;                  // 近端必须比远端宽，否则这组样本不可信
        for (row = 0; row < IMG_H; row++)
            if (s_road_learn_count[row] < IMAGE_ROAD_ROW_SAMPLES)
                Road_Half_Wide[row] = iclip((int)(slope * (float)row + base + 0.5f),
                                            ROAD_HALF_MIN, IMG_W);
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     汇总中线并生成转向几何量
// 参数说明     coefficient/range 曲线系数与有效范围
// 参数说明     quality         当前拟合质量
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void image_update_track_output(const float coefficient[3],
                                      float u_min, float u_max, float quality)
{
    int row;
    int reference_row = iclip(g_err_front_row, 0, IMG_H - 1);
    int selected_row = -1;
    int distance;
    float u_reference;
    float z_reference;
    float derivative;
    float heading_scale;
    float curvature_den;
    float ring_offset = 0.0f;

    my_image.Mid_Valid_Rows = 0;
    my_image.Valid_Row_Bottom = -1;
    my_image.Valid_Row_Top = -1;
    for (row = 0; row < IMG_H; row++)
        if (!my_image.Mid_Lost_Flag[row])
        {
            if (my_image.Valid_Row_Top < 0) my_image.Valid_Row_Top = row;
            my_image.Valid_Row_Bottom = row;
            my_image.Mid_Valid_Rows++;
        }

    my_image.Track_Valid = (my_image.Mid_Valid_Rows >= TRACK_MIN_VALID_ROWS);
    g_track_quality = image_fclip(quality, 0.0f, 1.0f);
    if (!my_image.Track_Valid)
    {
        g_mid_error = 0.0f;
        g_track_lateral = 0.0f; g_track_heading = 0.0f; g_track_curvature = 0.0f;
        return;
    }

    for (distance = 0; distance < IMG_H; distance++)
    {
        int far_row = reference_row - distance;
        int near_row = reference_row + distance;
        if (far_row >= 0 && !my_image.Mid_Lost_Flag[far_row]) { selected_row = far_row; break; }
        if (near_row < IMG_H && !my_image.Mid_Lost_Flag[near_row]) { selected_row = near_row; break; }
    }
    if (selected_row < 0)
    {
        my_image.Track_Valid = 0;
        g_mid_error = 0.0f;
        g_track_lateral = 0.0f;
        g_track_heading = 0.0f;
        g_track_curvature = 0.0f;
        return;
    }

    /* 保持既有转向约定：图像目标位于左侧时误差为正。 */
    g_mid_error = (float)(IMG_MID_COL - my_image.Mid_Line[selected_row]);
    if (ipm_ready())
    {
        float x;
        float y;

        u_reference = ipm_to_bev((float)IMG_MID_COL, (float)reference_row, &x, &y)
                    ? -y / IMAGE_BEV_SCALE : 0.5f * (u_min + u_max);
    }
    else u_reference = (float)(IMG_H - 1 - reference_row) / (float)(IMG_H - 1);

    u_reference = image_fclip(u_reference, u_min, u_max);
    z_reference = image_poly_value(coefficient, u_reference);
    derivative = coefficient[1] + 2.0f * coefficient[2] * u_reference;
    // 标定过的归一化平面横纵同尺度，斜率就是真实斜率，不用再折算
    heading_scale = 1.0f;
    g_track_lateral = image_fclip(-z_reference, -2.0f, 2.0f);
    g_track_heading = image_fclip(-atanf(derivative / heading_scale) *
                                  IMAGE_RAD_TO_DEG, -90.0f, 90.0f);
    curvature_den = 1.0f + derivative * derivative;
    curvature_den *= sqrtf(curvature_den);
    if (curvature_den < 1.0e-4f) curvature_den = 1.0e-4f;
    g_track_curvature = image_fclip(-2.0f * coefficient[2] / curvature_den, -1.0f, 1.0f);

    if (g_island.detect != 0 && g_island.island_state == 3)
    {
        ring_offset = (g_island.detect == 1)
                    ? (float)g_elem_action.ring_side_offset
                    : -(float)g_elem_action.ring_side_offset;
        g_mid_error -= ring_offset;
        g_track_lateral = image_fclip(g_track_lateral -
                                      ring_offset / (float)Road_Half_Wide[selected_row],
                                      -2.0f, 2.0f);
    }

    s_last_near_mid = my_image.Mid_Line[my_image.Valid_Row_Bottom];
    s_seed_col = iclip(s_last_near_mid, EN_COL_MIN_LIMIT + 1, EN_COL_MAX_LIMIT - 1);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     近端窗口点数不够时，直接由逐行中线生成转向标量
// 参数说明     void
// 返回参数     void
// 使用示例     image_update_track_output_local();
//-------------------------------------------------------------------------------------------------------------------
static void image_update_track_output_local(void)
{
    int row;
    int reference_row = iclip(g_err_front_row, 0, IMG_H - 1);
    int selected_row = -1;
    int far_row = -1;
    int near_row = -1;
    int distance;

    my_image.Mid_Valid_Rows = 0;
    my_image.Valid_Row_Bottom = -1;
    my_image.Valid_Row_Top = -1;
    for (row = 0; row < IMG_H; row++)
    {
        if (my_image.Mid_Lost_Flag[row]) continue;
        if (my_image.Valid_Row_Top < 0) my_image.Valid_Row_Top = row;
        my_image.Valid_Row_Bottom = row;
        my_image.Mid_Valid_Rows++;
    }

    my_image.Track_Valid = (my_image.Mid_Valid_Rows >= TRACK_MIN_VALID_ROWS);
    g_track_quality = image_fclip((float)my_image.Mid_Valid_Rows /
                                  (float)(my_image.Search_Stop_Line > 0
                                          ? my_image.Search_Stop_Line : IMG_H),
                                  0.0f, 1.0f) * 0.5f;
    if (!my_image.Track_Valid)
    {
        g_mid_error = 0.0f;
        g_track_lateral = 0.0f;
        g_track_heading = 0.0f;
        g_track_curvature = 0.0f;
        return;
    }

    for (distance = 0; distance < IMG_H; distance++)
    {
        int far = reference_row - distance;
        int near = reference_row + distance;

        if (far >= 0 && !my_image.Mid_Lost_Flag[far]) { selected_row = far; break; }
        if (near < IMG_H && !my_image.Mid_Lost_Flag[near]) { selected_row = near; break; }
    }
    if (selected_row < 0)
    {
        my_image.Track_Valid = 0;
        g_mid_error = 0.0f;
        g_track_lateral = 0.0f;
        g_track_heading = 0.0f;
        g_track_curvature = 0.0f;
        return;
    }
    for (distance = 6; distance < IMG_H; distance++)
    {
        if (far_row < 0 && selected_row - distance >= 0 &&
            !my_image.Mid_Lost_Flag[selected_row - distance])
            far_row = selected_row - distance;
        if (near_row < 0 && selected_row + distance < IMG_H &&
            !my_image.Mid_Lost_Flag[selected_row + distance])
            near_row = selected_row + distance;
        if (far_row >= 0 && near_row >= 0) break;
    }

    g_mid_error = (float)(IMG_MID_COL - my_image.Mid_Line[selected_row]);
    g_track_lateral = image_fclip(g_mid_error /
                                  (float)(Road_Half_Wide[selected_row] > 0
                                          ? Road_Half_Wide[selected_row] : 1),
                                  -2.0f, 2.0f);
    if (far_row >= 0 && near_row >= 0)
        g_track_heading = image_fclip(atanf((float)(my_image.Mid_Line[near_row] -
                                                    my_image.Mid_Line[far_row]) /
                                            (float)(near_row - far_row)) * IMAGE_RAD_TO_DEG,
                                          -90.0f, 90.0f);
    else g_track_heading = 0.0f;
    g_track_curvature = 0.0f;

    if (g_island.detect != 0 && g_island.island_state == 3)
    {
        float ring_offset = (g_island.detect == 1)
                          ? (float)g_elem_action.ring_side_offset
                          : -(float)g_elem_action.ring_side_offset;
        g_mid_error -= ring_offset;
        g_track_lateral = image_fclip(g_track_lateral -
                                      ring_offset /
                                      (float)(Road_Half_Wide[selected_row] > 0
                                              ? Road_Half_Wide[selected_row] : 1),
                                      -2.0f, 2.0f);
    }

    s_last_near_mid = my_image.Mid_Line[my_image.Valid_Row_Bottom];
    s_seed_col = iclip(s_last_near_mid, EN_COL_MIN_LIMIT + 1, EN_COL_MAX_LIMIT - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐行构建中线并生成转向几何量
// 参数说明     void
// 返回参数     void
// 使用示例     Image_Build_Mid_Line();
//-------------------------------------------------------------------------------------------------------------------
void Image_Build_Mid_Line(void)
{
    float coefficient[3];
    float u_min = 0.0f;
    float u_max = 0.0f;
    float quality = 0.0f;
    uint8 previous_track_valid = (uint8)my_image.Track_Valid;
    int measured_dual = 0;
    int window_dual = 0;
    int count;
    int window_count;
    int row;

    image_update_half_width();
    for (row = 0; row < IMG_H; row++)
    {
        my_image.Mid_Line[row] = s_last_near_mid;
        my_image.Mid_Lost_Flag[row] = 1;
        my_image.Mid_Source[row] = IMAGE_POINT_NONE;
    }

    count = image_build_rows(&measured_dual);
    if (count < IMAGE_MID_MIN_POINTS || my_image.Contrast < OTSU_CONTRAST_MIN / 2)
    {
        my_image.Track_Valid = 0;
        my_image.Mid_Valid_Rows = 0;
        my_image.Valid_Row_Bottom = -1;
        my_image.Valid_Row_Top = -1;
        g_mid_error = 0.0f;
        g_track_lateral = 0.0f;
        g_track_heading = 0.0f;
        g_track_curvature = 0.0f;
        g_track_quality = 0.0f;
        return;
    }

    image_fill_mid_gaps();
    image_slew_midline();
    image_filter_midline();
    image_extend_near_rows();

    // 转向标量：近端窗口的加权鲁棒最小二乘，二次曲线在这个尺度上是好模型
    window_count = image_fill_preview_window(&window_dual);
    if (window_count >= IMAGE_FIT_MIN_POINTS &&
        image_fit_curve(window_count, window_dual, coefficient, &u_min, &u_max, &quality))
        image_update_track_output(coefficient, u_min, u_max, quality);
    // 窗口点数不够就退回差分：位置照旧，航向由中线的局部斜率给，曲率给 0
    else
        image_update_track_output_local();

    if (!ipm_ready() && s_allow_width_learning && previous_track_valid &&
        my_image.Track_Valid && measured_dual >= IMAGE_ROAD_LEARN_MIN_DUAL &&
        fabsf(g_track_curvature) <= IMAGE_ROAD_LEARN_CURVATURE &&
        g_elem_action.active_elem == ELEM_NONE) image_update_road_width();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在两个已知点之间补一段边线
// 参数说明     line/lost/source 边线、丢线标志和来源数组
// 参数说明     row0/row1        两个端点的行号，谁大谁小都行
// 返回参数     void
// 使用示例     image_line_fill(line, lost, source, row0, row1);
//-------------------------------------------------------------------------------------------------------------------
static void image_line_fill(int *line, int *lost, uint8 *source, int row0, int row1)
{
    int i, t, c0, c1;
    float k;

    row0 = iclip(row0, 0, IMG_H - 1);
    row1 = iclip(row1, 0, IMG_H - 1);
    if (row1 < row0) { t = row1; row1 = row0; row0 = t; }
    if (row1 == row0) return;

    c0 = line[row0];
    c1 = line[row1];
    k  = (float)(c1 - c0) / (float)(row1 - row0);

    for (i = row0; i <= row1; i++)
    {
        line[i] = iclip((int)((float)(i - row0) * k + (float)c0 + 0.5f), 0, IMG_W - 1);
        lost[i] = 0;
        source[i] = IMAGE_POINT_REPAIRED;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对一段边线做最小二乘拟合，再沿拟合直线把它延伸到近端
// 参数说明     line/lost/source 边线、丢线标志和来源数组
// 参数说明     fit_top/fit_bot  参与拟合的行区间，fit_top 更远
// 参数说明     to_row           延伸到的近端行号
// 返回参数     uint8            1=拟合成功并已延伸 0=有效点不足，什么都没做
// 使用示例     image_line_fit_down(line, lost, source, top, bottom, to_row);
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_line_fit_down(int *line, int *lost, uint8 *source,
                                 int fit_top, int fit_bot, int to_row)
{
    int i, t, n = 0;
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, den, k, b;

    fit_top = iclip(fit_top, 0, IMG_H - 1);
    fit_bot = iclip(fit_bot, 0, IMG_H - 1);
    if (fit_bot < fit_top) { t = fit_bot; fit_bot = fit_top; fit_top = t; }
    to_row = iclip(to_row, 0, IMG_H - 1);

    for (i = fit_top; i <= fit_bot; i++)
    {
        if (lost[i]) continue;
        sx  += (float)i;
        sy  += (float)line[i];
        sxx += (float)i * (float)i;
        sxy += (float)i * (float)line[i];
        n++;
    }
    if (n < 4) return 0;

    den = (float)n * sxx - sx * sx;
    if (den > -1.0f && den < 1.0f) return 0;
    k = ((float)n * sxy - sx * sy) / den;
    b = (sy - k * sx) / (float)n;

    for (i = fit_bot; i <= to_row; i++)
    {
        line[i] = iclip((int)(k * (float)i + b + 0.5f), 0, IMG_W - 1);
        lost[i] = 0;
        source[i] = IMAGE_POINT_REPAIRED;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在指定边线上找一个角点，从近端往远端找，只返回第一个
// 参数说明     line/lost        边线数组/丢线标志数组
// 参数说明     start/end        近端起始行/远端终止行
// 参数说明     up_side          1=找上角点 0=找下角点
// 参数说明     sign             +1=左边线 -1=右边线，用来把左右两侧统一成一套判据
// 返回参数     int              角点行号，找不到返回 0
// 使用示例     row = find_corner(my_image.Left_Line, my_image.Left_Lost_Flag, IMG_H-1, top, 1, 1);
static int find_corner(const int *line, const int *lost, int start, int end,
                       uint8 up_side, int sign)
{
    int i, t;

    if (start < end) { t = start; start = end; end = t; }
    if (start > IMG_H - 6) start = IMG_H - 6;
    if (end   < 5)         end   = 5;
    if (start < end) return 0;

    for (i = start; i >= end; i--)
    {
        int flat_a, flat_b, flat_c, jump2, jump3, jump4;

        if (lost[i] || lost[i - 1] || lost[i - 2] || lost[i - 3] ||
            lost[i + 1] || lost[i + 2] || lost[i + 3] || lost[i + 4])
            continue;

        if (up_side)
        {
            flat_a = func_abs(line[i]     - line[i - 1]);
            flat_b = func_abs(line[i - 1] - line[i - 2]);
            flat_c = func_abs(line[i - 2] - line[i - 3]);
            jump2  = sign * (line[i] - line[i + 2]);
            jump3  = sign * (line[i] - line[i + 3]);
            jump4  = sign * (line[i] - line[i + 4]);
        }
        else
        {
            flat_a = func_abs(line[i]     - line[i + 1]);
            flat_b = func_abs(line[i + 1] - line[i + 2]);
            flat_c = func_abs(line[i + 2] - line[i + 3]);
            jump2  = sign * (line[i] - line[i - 2]);
            jump3  = sign * (line[i] - line[i - 3]);
            jump4  = sign * (line[i] - line[i - 4]);
        }

        if (flat_a <= CORNER_FLAT && flat_b <= CORNER_FLAT && flat_c <= CORNER_FLAT &&
            jump2 >= CORNER_JUMP && jump3 >= CORNER_JUMP2 && jump4 >= CORNER_JUMP2)
            return i;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一条边线在给定行区间内直线拟合的平均绝对残差，判断这条边线直不直
// 参数说明     line/lost       边线数组/丢线标志数组
// 参数说明     row_bottom      这一侧真正量到边线的最下面一行(Left_Row_Bottom / Right_Row_Bottom)
// 返回参数     float           平均残差(像素)，有效行少于 EDGE_RES_MIN_ROWS 返回 -1
// 使用示例     res = image_edge_residual(my_image.Right_Line, my_image.Right_Lost_Flag,
//                                        my_image.Right_Row_Bottom);
float image_edge_residual(const int *line, const int *lost, int row_bottom)
{
    int   i, n = 0;
    int   top = IMG_H - my_image.Search_Stop_Line;
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    float den, k, b, res = 0.0f;

    if (top < 0) top = 0;
    if (row_bottom > IMG_H - 1) row_bottom = IMG_H - 1;
    if (row_bottom < top) return -1.0f;

    for (i = row_bottom; i >= top; i--)
    {
        if (lost[i]) continue;
        sx += (float)i;            sy  += (float)line[i];
        sxx += (float)i * (float)i; sxy += (float)i * (float)line[i];
        n++;
    }
    if (n < EDGE_RES_MIN_ROWS) return -1.0f;

    den = (float)n * sxx - sx * sx;
    if (den > -1.0f && den < 1.0f) return -1.0f;
    k = ((float)n * sxy - sx * sy) / den;
    b = (sy - k * sx) / (float)n;

    for (i = row_bottom; i >= top; i--)
    {
        if (lost[i]) continue;
        res += fabsf((float)line[i] - (k * (float)i + b));
    }
    return res / (float)n;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一次求出左右边线的上下四个角点
// 参数说明     corner          输出，顺序 [左下, 左上, 右下, 右上]，找不到的置 0
// 返回参数     void
// 使用示例     Image_Find_Corners(corner);
//-------------------------------------------------------------------------------------------------------------------
void Image_Find_Corners(int corner[4])
{
    int top = IMG_H - my_image.Search_Stop_Line;
    int search_rows = iclip(my_image.Search_Stop_Line, 0, IMG_H);
    int lost_limit = (search_rows * 4) / 5;

    corner[0] = corner[1] = corner[2] = corner[3] = 0;
    if (search_rows < 12) return;
    if (top < 5) top = 5;

    // 大部分行都丢线时角点没有判断意义
    if (my_image.Left_Lost_Counter < lost_limit)
    {
        corner[0] = find_corner(my_image.Left_Line, my_image.Left_Lost_Flag,
                                IMG_H - 1, top, 0u, 1);
        corner[1] = find_corner(my_image.Left_Line, my_image.Left_Lost_Flag,
                                IMG_H - 1, top, 1u, 1);
    }
    if (my_image.Right_Lost_Counter < lost_limit)
    {
        corner[2] = find_corner(my_image.Right_Line, my_image.Right_Lost_Flag,
                                IMG_H - 1, top, 0u, -1);
        corner[3] = find_corner(my_image.Right_Line, my_image.Right_Lost_Flag,
                                IMG_H - 1, top, 1u, -1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用上下两个角点直线补左边线
// 参数说明     up_row/down_row  上角点行/下角点行
// 返回参数     void
// 使用示例     Image_Fill_Left(l_up, l_down);
//-------------------------------------------------------------------------------------------------------------------
void Image_Fill_Left(int up_row, int down_row)
{
    image_line_fill(my_image.Left_Line, my_image.Left_Lost_Flag,
                    my_image.Left_Source, up_row, down_row);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用上下两个角点直线补右边线
// 参数说明     up_row/down_row  上角点行/下角点行
// 返回参数     void
// 使用示例     Image_Fill_Right(r_up, r_down);
//-------------------------------------------------------------------------------------------------------------------
void Image_Fill_Right(int up_row, int down_row)
{
    image_line_fill(my_image.Right_Line, my_image.Right_Lost_Flag,
                    my_image.Right_Source, up_row, down_row);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     拟合上角点以远的左边线，顺着方向补到近端
// 参数说明     up_row           上角点行
// 返回参数     uint8            1=补线成功
// 使用示例     Image_Extend_Left(l_up);
//-------------------------------------------------------------------------------------------------------------------
uint8 Image_Extend_Left(int up_row)
{
    return image_line_fit_down(my_image.Left_Line, my_image.Left_Lost_Flag,
                               my_image.Left_Source, up_row - CROSS_FIT_ROWS,
                               up_row - CROSS_FIT_SKIP, IMG_H - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     拟合上角点以远的右边线，顺着方向补到近端
// 参数说明     up_row           上角点行
// 返回参数     uint8            1=补线成功
// 使用示例     Image_Extend_Right(r_up);
//-------------------------------------------------------------------------------------------------------------------
uint8 Image_Extend_Right(int up_row)
{
    return image_line_fit_down(my_image.Right_Line, my_image.Right_Lost_Flag,
                               my_image.Right_Source, up_row - CROSS_FIT_ROWS,
                               up_row - CROSS_FIT_SKIP, IMG_H - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完成大津二值化与逐行提边
// 参数说明     allow_width_learning  1=允许更新赛宽表 0=冻结赛宽表
// 返回参数     void
// 使用示例     image_process_edges(1);
//-------------------------------------------------------------------------------------------------------------------
void image_process_edges(uint8 allow_width_learning)
{
    s_allow_width_learning = allow_width_learning ? 1u : 0u;
    image_binarize();
    image_get_edge();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据当前边线构建中线并更新转向几何量
// 参数说明     void
// 返回参数     void
// 使用示例     image_process_finish();
//-------------------------------------------------------------------------------------------------------------------
void image_process_finish(void)
{
    Image_Build_Mid_Line();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完成单帧二值化、提边、中线和转向几何计算
// 参数说明     allow_width_learning  1=允许更新赛宽表 0=冻结赛宽表
// 返回参数     void
// 使用示例     image_process(1);
//-------------------------------------------------------------------------------------------------------------------
void image_process(uint8 allow_width_learning)
{
    image_process_edges(allow_width_learning);
    image_process_finish();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复制当前整帧灰度图
// 参数说明     destination     目标缓冲区，至少 IMG_W*IMG_H 字节
// 返回参数     void
// 使用示例     image_copy_gray(display_pixels);
//-------------------------------------------------------------------------------------------------------------------
void image_copy_gray(uint8 *destination)
{
    if (destination != 0)
        memcpy(destination, img_gray, sizeof(img_gray));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从当前帧算出逆透视标定要的四个角点
// 参数说明     pts              输出，[0..3] = 近左/近右/远左/远右 的列坐标
// 参数说明     rn/rf            输出，近端行与远端行行号
// 参数说明     len_ratio        输出，两个采样行之间的地面距离 / 赛道宽度
// 返回参数     ipm_pick_t       IPM_PICK_OK 或具体的失败原因
// 使用示例     if (image_ipm_pick_points(pts, &rn, &rf, &ratio) == IPM_PICK_OK) { ... }
// 双侧实测边线先做直线拟合，再选择宽度充分的近、远端采样行。
//-------------------------------------------------------------------------------------------------------------------
ipm_pick_t image_ipm_pick_points(float pts[4], int *rn, int *rf, float *len_ratio)
{
    int   i, top, bottom;
    int   n = 0, row_lo = -1, row_hi = -1;      // row_lo=最近的一行 row_hi=最远的一行
    float sr = 0.0f, srr = 0.0f;
    float sl = 0.0f, srl = 0.0f, sg = 0.0f, srg = 0.0f;
    float det, al, bl, ar, br, res = 0.0f, wide_near, dwide, near_row, far_row;

    *rn = 0; *rf = 0;
    *len_ratio = IPM_LEN_RATIO;
    pts[0] = pts[1] = pts[2] = pts[3] = 0.0f;

    top = IMG_H - my_image.Search_Stop_Line;
    if (top < 0) top = 0;
    bottom = my_image.Edge_Row_Bottom;
    if (bottom > IMG_H - 1) bottom = IMG_H - 1;
    if (bottom < top) return IPM_PICK_NO_TRACK;

    for (i = bottom; i >= top; i--)
    {
        float r = (float)i;

        if (my_image.Left_Lost_Flag[i] || my_image.Right_Lost_Flag[i]) continue;
        if (row_lo < 0) row_lo = i;
        row_hi = i;
        n++;
        sr += r;   srr += r * r;
        sl += (float)my_image.Left_Line[i];   srl += r * (float)my_image.Left_Line[i];
        sg += (float)my_image.Right_Line[i];  srg += r * (float)my_image.Right_Line[i];
    }

    if (n < IPM_FIT_MIN_ROWS)                 return IPM_PICK_FEW_ROWS;
    if (row_lo - row_hi < IPM_MIN_ROW_GAP)    return IPM_PICK_SHORT_SPAN;

    det = (float)n * srr - sr * sr;
    if (det < 1.0f) return IPM_PICK_SHORT_SPAN;
    al = ((float)n * srl - sr * sl) / det;  bl = (sl - al * sr) / (float)n;
    ar = ((float)n * srg - sr * sg) / det;  br = (sg - ar * sr) / (float)n;

    // 平均绝对残差，两条边线一起算。直道摆正时应该在 1 像素以内
    for (i = bottom; i >= top; i--)
    {
        float r = (float)i;
        if (my_image.Left_Lost_Flag[i] || my_image.Right_Lost_Flag[i]) continue;
        res += fabsf((float)my_image.Left_Line[i]  - (al * r + bl));
        res += fabsf((float)my_image.Right_Line[i] - (ar * r + br));
    }
    res /= (float)(2 * n);
    if (res > IPM_FIT_TOL) return IPM_PICK_NOT_STRAIGHT;

    // 近端行取实测区最下面那一行，宽度不够就是压根没对准赛道
    near_row  = (float)row_lo;
    wide_near = (ar * near_row + br) - (al * near_row + bl);
    if (wide_near < (float)IPM_MIN_WIDTH) return IPM_PICK_NARROW;

    // 远端行：拟合宽度随行号线性变化，直接反解宽度等于 IPM_FAR_MIN_WIDTH 的那一行，
    // 再夹进实测区。这样远端点永远有足够宽度，四点不会退化成共线
    dwide = ar - al;                                    // 每往下一行赛道宽多少像素
    far_row = (float)row_hi;
    if (dwide > 0.01f)
    {
        float r = ((float)IPM_FAR_MIN_WIDTH - br + bl) / dwide;
        if (r > far_row) far_row = r;
    }
    if (far_row > near_row - (float)IPM_MIN_ROW_GAP) return IPM_PICK_NARROW;

    *rn = (int)(near_row + 0.5f);
    *rf = (int)(far_row + 0.5f);
    pts[0] = al * near_row + bl;  pts[1] = ar * near_row + br;
    pts[2] = al * far_row  + bl;  pts[3] = ar * far_row  + br;

    {
        float focal = IPM_FOCAL_PIX;
        float center_row = (float)IMG_H * 0.5f;
        float horizon = (al - ar > 1.0e-4f || ar - al > 1.0e-4f)
                      ? (br - bl) / (al - ar) : 1.0e6f;
        float tangent, scale, depth_near, depth_far, ratio;

        if (focal >= 40.0f && focal <= 600.0f && dwide > 0.01f &&
            horizon < far_row - 1.0f && horizon > -1.0e5f)
        {
            tangent = (center_row - horizon) / focal;
            scale = dwide * sqrtf(1.0f + tangent * tangent);
            depth_near = (focal - (near_row - center_row) * tangent) /
                         (scale * (near_row - horizon));
            depth_far = (focal - (far_row - center_row) * tangent) /
                        (scale * (far_row - horizon));
            ratio = depth_far - depth_near;
            if (ratio == ratio && ratio > IPM_LEN_RATIO_MIN && ratio < IPM_LEN_RATIO_MAX)
                *len_ratio = ratio;
        }
    }
    return IPM_PICK_OK;
}

#pragma section all restore
