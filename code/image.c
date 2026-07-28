#include "image.h"
#include "element.h"
#include "IfxStm.h"

#pragma section all "cpu1_dsram"

Image my_image;                                 // 图像处理结果
int   Standard_Road_Wide[IMG_H];                // 标准赛宽表
float g_mid_error = 0.0f;                       // 当前中线偏差
image_profile_t g_image_profile;                // 最近一帧图像主链耗时

typedef struct
{
    int8  black_dx;
    int8  black_dy;
    int8  white_dx;
    int8  white_dy;
    uint8 blocked_dir;
    uint8 next_dir;
} en_step_t;

// 左右边线采用镜像优先级，方向约束用于阻止爬虫立即回头
static const en_step_t s_en_steps_left[7] =
{
    {-1, -1,  0, -1, 2, 7},
    { 1, -1,  1,  0, 3, 6},
    { 0, -1,  1, -1, 0xFFu, 0},
    {-1,  0, -1, -1, 5, 4},
    { 1,  0,  1,  1, 4, 5},
    {-1,  1, -1,  0, 6, 3},
    { 1,  1,  0,  1, 7, 2},
};

static const en_step_t s_en_steps_right[7] =
{
    { 1, -1,  0, -1, 3, 6},
    {-1, -1, -1,  0, 2, 7},
    { 0, -1, -1, -1, 0xFFu, 0},
    { 1,  0,  1, -1, 4, 5},
    {-1,  0, -1,  1, 5, 4},
    {-1,  1,  0,  1, 6, 3},
    { 1,  1,  1,  0, 7, 2},
};

#define EN_VISITED_BYTES ((IMG_W + 7) / 8)
static uint8 s_en_visited[IMG_H][EN_VISITED_BYTES];

static uint8 img_gray[IMG_H][IMG_W];            // 灰度帧缓冲区
static int s_seed_col = IMG_MID_COL;             // 下一帧近端起点种子列
static int s_last_near_mid = IMG_MID_COL;        // 上一帧近端中线

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对整数执行限幅
// 参数说明     x/lo/hi          值/下限/上限
// 返回参数     int              限幅结果
// 使用示例     v = iclip(v, 0, IMG_W-1);
//-------------------------------------------------------------------------------------------------------------------
static int iclip(int x, int lo, int hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 CPU1 STM 时钟差换算为微秒
// 参数说明     ticks            STM1 计数差
// 返回参数     uint32           经过四舍五入的微秒数
// 使用示例     elapsed_us = image_ticks_to_us(now - start);
//-------------------------------------------------------------------------------------------------------------------
static inline uint32 image_ticks_to_us(uint32 ticks)
{
    return (ticks + 50u) / 100u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按近端和远端赛宽重建标准赛宽表，行 0 是远端，行 IMG_H-1 是近端，中间线性插值
// 参数说明     near_wide/far_wide 近端与远端标准赛道宽度(像素)，内部钳到 [4, IMG_W-4]
// 返回参数     void
// 使用示例     image_set_road_wide(133, 30);
//-------------------------------------------------------------------------------------------------------------------
void image_set_road_wide(int near_wide, int far_wide)
{
    int i;

    near_wide = iclip(near_wide, 4, IMG_W - 4);
    far_wide  = iclip(far_wide,  4, IMG_W - 4);

    for (i = 0; i < IMG_H; i++)
        Standard_Road_Wide[i] = iclip(far_wide + (near_wide - far_wide) * i / (IMG_H - 1),
                                      4, IMG_W - 4);
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

    memset(&g_image_profile, 0, sizeof(g_image_profile));
    image_set_road_wide(ROAD_WIDE_NEAR_DEFAULT, ROAD_WIDE_FAR_DEFAULT);

    for (i = 0; i < IMG_H; i++)
    {
        my_image.Left_Line[i]       = 0;
        my_image.Right_Line[i]      = IMG_W - 1;
        my_image.Mid_Line[i]        = IMG_MID_COL;
        my_image.Road_Wide[i]       = IMG_W - 1;
        my_image.Left_Lost_Flag[i]  = 1;
        my_image.Right_Lost_Flag[i] = 1;
        my_image.Mid_Lost_Flag[i]   = 1;
    }
    my_image.Search_Stop_Line = 0;
    my_image.Boundry_Start_Left = 0;
    my_image.Boundry_Start_Right = 0;
    my_image.Mid_Valid_Rows   = 0;
    my_image.Valid_Row_Bottom = -1;
    my_image.Valid_Row_Top    = -1;
    my_image.Track_Valid      = 0;
    g_mid_error               = 0.0f;
    s_seed_col                = IMG_MID_COL;
    s_last_near_mid           = IMG_MID_COL;
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
    int    r, c, threshold = 0, th_first = 0, th_last = 0;
    int    gmin = 255, gmax = 0;
    float  num, var, var_max = -1.0f;

    memset(hist, 0, sizeof(hist));

    // 隔行隔列统计灰度直方图
    for (r = 0; r < IMG_H; r += 2)
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

    // 只在本帧实际灰度范围内搜索阈值，避免固定扫描 0~255。
    for (i = (uint32)gmin; i <= (uint32)gmax; i++)
    {
        w_b += hist[i];
        sum_b += i * hist[i];
        if (w_b == 0u) continue;
        w_f = total - w_b;
        if (w_f == 0u) break;
        if (hist[i] == 0u) continue;

        // 使用等价公式计算类间方差
        num = (float)sum_b * (float)w_f
            - (float)(sum_all - sum_b) * (float)w_b;
        var = num * num / ((float)w_b * (float)w_f);

        // 记录最大方差平台首尾并取中值
        if (var > var_max)       { var_max = var; th_first = (int)i; th_last = (int)i; }
        else if (var == var_max) { th_last = (int)i; }
    }
    threshold = iclip((th_first + th_last) / 2, OTSU_TH_MIN, OTSU_TH_MAX);
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
// 函数简介     搜索八邻域左右边线起点
// 参数说明     seed_col/sl/sr   中心种子列/左起点/右起点
// 返回参数     uint8            bit0=左有效 bit1=右有效
// 使用示例     side = en_get_start(seed, sl, sr);
//-------------------------------------------------------------------------------------------------------------------
static uint8 en_get_start(int seed_col, int *sl, int *sr)
{
    int row, lx, rx;
    int col, run_left, run_right, run_width;
    int best_row = -1;
    int best_left = 0;
    int best_right = 0;
    int best_width = 0;
    uint8 side;

    seed_col = iclip(seed_col, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT);

    // 优先沿用上一帧近端中线，正常行驶时不扫描整行
    for (row = EN_START_ROW_BOTTOM; row >= EN_START_ROW_TOP; row--)
    {
        if (my_image.image_two_value[row][seed_col] != IMG_WHITE) continue;

        for (lx = seed_col; lx > 0         && my_image.image_two_value[row][lx] == IMG_WHITE; lx--) { }
        for (rx = seed_col; rx < IMG_W - 1 && my_image.image_two_value[row][rx] == IMG_WHITE; rx++) { }

        side = 0;
        if (lx > 1)         side |= 1u;     // 排除左侧黑框
        if (rx < IMG_W - 2) side |= 2u;
        if (side == 0u) continue;                                       // 未检测到有效边线
        if (side == 3u && (rx - lx) < EN_MIN_ROAD_WIDE) continue;       // 排除过窄边线

        sl[0] = lx; sl[1] = row;
        sr[0] = rx; sr[1] = row;
        return side;
    }

    // 起点丢失时寻找底部区域最宽白色连通段，仅作为恢复兜底
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

            if (run_left > EN_COL_MIN_LIMIT &&
                run_right < EN_COL_MAX_LIMIT &&
                run_width >= EN_MIN_ROAD_WIDE &&
                run_width > best_width)
            {
                best_row = row;
                best_left = run_left - 1;
                best_right = run_right + 1;
                best_width = run_width;
            }
        }
    }

    if (best_row >= 0)
    {
        sl[0] = best_left;
        sl[1] = best_row;
        sr[0] = best_right;
        sr[1] = best_row;
        return 3u;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     沿单侧黑白边界执行带方向约束的八邻域跟踪
// 参数说明     sx/sy/left_side 起点列/起点行/1=左边线 0=右边线
// 返回参数     uint16           边界点数量
// 使用示例     n = en_trace(sl[0], sl[1], 1);
//-------------------------------------------------------------------------------------------------------------------
static uint16 en_trace(int sx, int sy, uint8 left_side)
{
    uint16 cnt = 0;
    int cx = sx, cy = sy;
    int i, nx, ny, wx, wy, line_col;
    uint8 last_dir = 0;
    uint8 found;
    const en_step_t *steps = left_side ? s_en_steps_left : s_en_steps_right;

    memset(s_en_visited, 0, sizeof(s_en_visited));

    while (cnt < EN_MAX_PTS)
    {
        uint8 visited_mask = (uint8)(1u << (cx & 7));

        if (s_en_visited[cy][cx >> 3] & visited_mask) break;
        s_en_visited[cy][cx >> 3] |= visited_mask;

        if (left_side)
        {
            line_col = iclip(cx + 1, 0, IMG_W - 1);
            if (my_image.Left_Lost_Flag[cy] || line_col > my_image.Left_Line[cy])
            {
                my_image.Left_Line[cy] = line_col;
                my_image.Left_Lost_Flag[cy] = 0;
            }
        }
        else
        {
            line_col = iclip(cx - 1, 0, IMG_W - 1);
            if (my_image.Right_Lost_Flag[cy] || line_col < my_image.Right_Line[cy])
            {
                my_image.Right_Line[cy] = line_col;
                my_image.Right_Lost_Flag[cy] = 0;
            }
        }
        cnt++;

        if (cy <= EN_ROW_TOP_LIMIT) break;
        if (cy >= IMG_H - 1) break;
        if (cx <= EN_COL_MIN_LIMIT || cx >= EN_COL_MAX_LIMIT) break;

        found = 0;
        for (i = 0; i < 7; i++)
        {
            if (last_dir == steps[i].blocked_dir) continue;

            nx = cx + steps[i].black_dx;
            ny = cy + steps[i].black_dy;
            wx = cx + steps[i].white_dx;
            wy = cy + steps[i].white_dy;

            if (s_en_visited[ny][nx >> 3] & (uint8)(1u << (nx & 7))) continue;
            if (my_image.image_two_value[ny][nx] != IMG_BLACK) continue;
            if (my_image.image_two_value[wy][wx] != IMG_WHITE) continue;

            cx = nx;
            cy = ny;
            last_dir = steps[i].next_dir;
            found = 1;
            break;
        }
        if (!found) break;
    }
    return cnt;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     提取八邻域左右边线并统计丢线状态
// 参数说明     void
// 返回参数     void
// 使用示例     image_get_edge();
//-------------------------------------------------------------------------------------------------------------------
static void image_get_edge(void)
{
    int sl[2], sr[2];
    int i, top = IMG_H, last;
    uint8 side;

    for (i = 0; i < IMG_H; i++)
    {
        my_image.Left_Line[i]       = 0;
        my_image.Right_Line[i]      = IMG_W - 1;
        my_image.Left_Lost_Flag[i]  = 1;
        my_image.Right_Lost_Flag[i] = 1;
    }
    my_image.Search_Stop_Line = 0;
    my_image.Boundry_Start_Left = 0;
    my_image.Boundry_Start_Right = 0;

    side = en_get_start(s_seed_col, sl, sr);
    if (side)
    {
        if (side & 1u)                          // 左边线取每行最大列
        {
            (void)en_trace(sl[0], sl[1], 1u);
        }

        if (side & 2u)                          // 右边线取每行最小列
        {
            (void)en_trace(sr[0], sr[1], 0u);
        }

        // 记录两侧向远端跟踪到的最小行号，供环岛状态机判断边界延伸位置
        for (i = 0; i < IMG_H && my_image.Left_Lost_Flag[i]; i++) { }
        if (i < IMG_H) my_image.Boundry_Start_Left = i;

        for (i = 0; i < IMG_H && my_image.Right_Lost_Flag[i]; i++) { }
        if (i < IMG_H) my_image.Boundry_Start_Right = i;

        // 使用最近有效边线补齐起点以下各行
        for (last = IMG_H - 1; last >= 0 && my_image.Left_Lost_Flag[last]; last--) { }
        for (i = last + 1; last >= 0 && i < IMG_H; i++)
        { my_image.Left_Line[i] = my_image.Left_Line[last]; my_image.Left_Lost_Flag[i] = 0; }

        for (last = IMG_H - 1; last >= 0 && my_image.Right_Lost_Flag[last]; last--) { }
        for (i = last + 1; last >= 0 && i < IMG_H; i++)
        { my_image.Right_Line[i] = my_image.Right_Line[last]; my_image.Right_Lost_Flag[i] = 0; }

        // 计算有效前瞻行数
        for (i = 0; i < IMG_H; i++)
            if (!my_image.Left_Lost_Flag[i] || !my_image.Right_Lost_Flag[i]) { top = i; break; }
        if (top >= IMG_H) top = IMG_H - 1;
        my_image.Search_Stop_Line = IMG_H - top;
    }
    else
    {
        my_image.Left_Lost_Counter = IMG_H;
        my_image.Right_Lost_Counter = IMG_H;
        my_image.Both_Lost_Counter = IMG_H;
        s_seed_col = IMG_MID_COL;
        return;
    }

    if (g_island.island_state == 3 && my_image.Search_Stop_Line > 70)
        my_image.Search_Stop_Line = 70;

    // 汇总前瞻区赛宽与丢线状态
    my_image.Left_Lost_Counter = 0; my_image.Right_Lost_Counter = 0; my_image.Both_Lost_Counter = 0;
    top = IMG_H - iclip(my_image.Search_Stop_Line, 0, IMG_H);

    for (i = IMG_H - 1; i >= top; i--)
    {
        my_image.Road_Wide[i] = my_image.Right_Line[i] - my_image.Left_Line[i];
        if (my_image.Left_Lost_Flag[i])  my_image.Left_Lost_Counter++;
        if (my_image.Right_Lost_Flag[i]) my_image.Right_Lost_Counter++;
        if (my_image.Left_Lost_Flag[i] && my_image.Right_Lost_Flag[i]) my_image.Both_Lost_Counter++;
    }

    // 更新下一帧起点种子列
    i = IMG_H - 1;
    s_seed_col = (my_image.Search_Stop_Line > 0 && !my_image.Left_Lost_Flag[i] && !my_image.Right_Lost_Flag[i])
               ? ((my_image.Left_Line[i] + my_image.Right_Line[i]) / 2) : IMG_MID_COL;
    s_seed_col = iclip(s_seed_col, EN_COL_MIN_LIMIT + 1, EN_COL_MAX_LIMIT - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据左右边线与标准赛宽构建中线
// 参数说明     void
// 返回参数     void
// 使用示例     Image_Build_Mid_Line();
//-------------------------------------------------------------------------------------------------------------------
void Image_Build_Mid_Line(void)
{
    int i, top, half, mid, wide, lok, rok;
    int last_mid = s_last_near_mid;

    my_image.Mid_Valid_Rows   = 0;
    my_image.Valid_Row_Bottom = -1;
    my_image.Valid_Row_Top    = -1;
    top = IMG_H - iclip(my_image.Search_Stop_Line, 0, IMG_H);

    for (i = IMG_H - 1; i >= 0; i--)            // 自近端向远端处理
    {
        if (i < top) { my_image.Mid_Line[i] = last_mid; my_image.Mid_Lost_Flag[i] = 1; continue; }

        lok  = !my_image.Left_Lost_Flag[i];
        rok  = !my_image.Right_Lost_Flag[i];
        half = Standard_Road_Wide[i] / 2;

        if (lok && rok)
        {
            wide = my_image.Right_Line[i] - my_image.Left_Line[i];
            if (wide < (int)((float)Standard_Road_Wide[i] * ROAD_WIDE_MIN_RATIO) ||
                wide > (int)((float)Standard_Road_Wide[i] * ROAD_WIDE_MAX_RATIO))
            {
                // 依据上一行中线选择有效单边
                int mid_l = my_image.Left_Line[i]  + half;
                int mid_r = my_image.Right_Line[i] - half;
                mid = (func_abs(mid_l - last_mid) <= func_abs(mid_r - last_mid)) ? mid_l : mid_r;
            }
            else
                mid = (my_image.Left_Line[i] + my_image.Right_Line[i]) / 2;
        }
        else if (lok) mid = my_image.Left_Line[i]  + half;   // 左边线单边补偿
        else if (rok) mid = my_image.Right_Line[i] - half;   // 右边线单边补偿
        else { my_image.Mid_Line[i] = last_mid; my_image.Mid_Lost_Flag[i] = 1; continue; }

        mid = iclip(mid, 0, IMG_W - 1);
        my_image.Mid_Line[i]      = mid;
        my_image.Mid_Lost_Flag[i] = 0;
        last_mid = mid;

        my_image.Mid_Valid_Rows++;
        if (my_image.Valid_Row_Bottom < 0) my_image.Valid_Row_Bottom = i;
        my_image.Valid_Row_Top = i;
    }

    my_image.Track_Valid = (my_image.Mid_Valid_Rows >= TRACK_MIN_VALID_ROWS &&
                            my_image.Contrast >= OTSU_CONTRAST_MIN) ? 1 : 0;
    if (my_image.Valid_Row_Bottom >= 0)
        s_last_near_mid = my_image.Mid_Line[my_image.Valid_Row_Bottom];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按局部斜率延长指定边线
// 参数说明     line/lost/start/end  边线/丢线标志/起始行/终止行
// 返回参数     void
// 使用示例     lengthen_boundry(my_image.Left_Line, my_image.Left_Lost_Flag, s, e);
//-------------------------------------------------------------------------------------------------------------------
static void lengthen_boundry(int *line, int *lost, int start, int end)
{
    int i, t, x0, x1;
    float k;

    start = iclip(start, 0, IMG_H - 1);
    end   = iclip(end,   0, IMG_H - 1);
    if (end < start) { t = end; end = start; start = t; }
    if (end == start) return;

    if (start <= 5)                             // 远端起点采用两点直线
    {
        x0 = line[start]; x1 = line[end];
        k  = (float)(x1 - x0) / (float)(end - start);
    }
    else
    {
        x0 = line[start];
        k  = (float)(line[start] - line[start - 4]) / 4.0f;     // 四行局部斜率
    }

    for (i = start; i <= end; i++)
    {
        line[i] = iclip((int)((float)(i - start) * k + (float)x0), 0, IMG_W - 1);
        lost[i] = 0;                            // 标记补线有效
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按局部斜率延长左边线
// 参数说明     start/end        延长起始/终止行
// 返回参数     void
// 使用示例     Lengthen_Left_Boundry(start, end);
//-------------------------------------------------------------------------------------------------------------------
void Lengthen_Left_Boundry(int start, int end)
{
    lengthen_boundry(my_image.Left_Line, my_image.Left_Lost_Flag, start, end);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按局部斜率延长右边线
// 参数说明     start/end        延长起始/终止行
// 返回参数     void
// 使用示例     Lengthen_Right_Boundry(start, end);
//-------------------------------------------------------------------------------------------------------------------
void Lengthen_Right_Boundry(int start, int end)
{
    lengthen_boundry(my_image.Right_Line, my_image.Right_Lost_Flag, start, end);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     搜索左右边线的上角点
// 参数说明     start/end        近端起始行/远端终止行
// 返回参数     void
// 使用示例     Find_Up_Point(IMG_H-1, 0);
//-------------------------------------------------------------------------------------------------------------------
void Find_Up_Point(int start, int end)
{
    int i, t;
    my_image.Left_Up_Find = 0;
    my_image.Right_Up_Find = 0;

    if (start < end) { t = start; start = end; end = t; }
    if (end   <= IMG_H - my_image.Search_Stop_Line) end = IMG_H - my_image.Search_Stop_Line;
    if (end   <= 5) end = 5;
    if (start >= IMG_H - 6) start = IMG_H - 6;
    if (start < end) return;

    for (i = start; i >= end; i--)
    {
        if (my_image.Left_Up_Find == 0 &&
            !my_image.Left_Lost_Flag[i] &&
            !my_image.Left_Lost_Flag[i - 1] &&
            !my_image.Left_Lost_Flag[i - 2] &&
            !my_image.Left_Lost_Flag[i - 3] &&
            !my_image.Left_Lost_Flag[i + 2] &&
            !my_image.Left_Lost_Flag[i + 3] &&
            !my_image.Left_Lost_Flag[i + 4] &&
            func_abs(my_image.Left_Line[i]     - my_image.Left_Line[i - 1]) <= 5 &&
            func_abs(my_image.Left_Line[i - 1] - my_image.Left_Line[i - 2]) <= 5 &&
            func_abs(my_image.Left_Line[i - 2] - my_image.Left_Line[i - 3]) <= 5 &&
            (my_image.Left_Line[i] - my_image.Left_Line[i + 2]) >= CORNER_JUMP &&
            (my_image.Left_Line[i] - my_image.Left_Line[i + 3]) >= 15 &&
            (my_image.Left_Line[i] - my_image.Left_Line[i + 4]) >= 15)
            my_image.Left_Up_Find = i;

        if (my_image.Right_Up_Find == 0 &&
            !my_image.Right_Lost_Flag[i] &&
            !my_image.Right_Lost_Flag[i - 1] &&
            !my_image.Right_Lost_Flag[i - 2] &&
            !my_image.Right_Lost_Flag[i - 3] &&
            !my_image.Right_Lost_Flag[i + 2] &&
            !my_image.Right_Lost_Flag[i + 3] &&
            !my_image.Right_Lost_Flag[i + 4] &&
            func_abs(my_image.Right_Line[i]     - my_image.Right_Line[i - 1]) <= 5 &&
            func_abs(my_image.Right_Line[i - 1] - my_image.Right_Line[i - 2]) <= 5 &&
            func_abs(my_image.Right_Line[i - 2] - my_image.Right_Line[i - 3]) <= 5 &&
            (my_image.Right_Line[i] - my_image.Right_Line[i + 2]) <= -CORNER_JUMP &&
            (my_image.Right_Line[i] - my_image.Right_Line[i + 3]) <= -15 &&
            (my_image.Right_Line[i] - my_image.Right_Line[i + 4]) <= -15)
            my_image.Right_Up_Find = i;

        if (my_image.Left_Up_Find && my_image.Right_Up_Find) break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算指定行区间的平均中线偏差
// 参数说明     start_point/end_point  远端起始行/近端终止行
// 返回参数     float            中线平均偏差
// 使用示例     err = err_sum_average(ERR_FRONT_ROW, ERR_FRONT_ROW + ERR_AVG_ROWS);
//-------------------------------------------------------------------------------------------------------------------
float err_sum_average(int start_point, int end_point)
{
    int i, t, n;
    float err = 0.0f;

    if (!my_image.Track_Valid || my_image.Valid_Row_Top < 0) { g_mid_error = 0.0f; return 0.0f; }

    if (end_point < start_point) { t = end_point; end_point = start_point; start_point = t; }
    if (start_point < my_image.Valid_Row_Top)    start_point = my_image.Valid_Row_Top;
    if (end_point   > my_image.Valid_Row_Bottom) end_point   = my_image.Valid_Row_Bottom;
    start_point = iclip(start_point, 0, IMG_H - 1);
    end_point   = iclip(end_point,   0, IMG_H - 1);

    if (end_point <= start_point)               // 使用最小有效区间
    {
        end_point   = my_image.Valid_Row_Bottom;
        start_point = end_point - 2;
        if (start_point < my_image.Valid_Row_Top) start_point = my_image.Valid_Row_Top;
        if (end_point <= start_point) { g_mid_error = 0.0f; return 0.0f; }
    }
    n = end_point - start_point;

    if (g_island.detect && g_island.island_state != 0 && g_island.island_state != 5)
    {
        int inner = (g_island.island_state == 3) ? g_elem_action.ring_side_offset : 0;
        if ((g_island.detect == 1) == (g_island.island_state == 3))
            for (i = start_point; i < end_point; i++)
                err += (float)(IMG_MID_COL - Standard_Road_Wide[i] / 2 - my_image.Left_Line[i] - inner);
        else
            for (i = start_point; i < end_point; i++)
                err += (float)(IMG_MID_COL + Standard_Road_Wide[i] / 2 - my_image.Right_Line[i] + inner);
        g_mid_error = err / (float)n;
        return g_mid_error;
    }

    for (i = start_point; i < end_point; i++)
        err += (float)(IMG_MID_COL - my_image.Mid_Line[i]);

    g_mid_error = err / (float)n;
    return g_mid_error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完成大津二值化、八邻域提边与中线构建
// 参数说明     void
// 返回参数     void
// 使用示例     image_process();
//-------------------------------------------------------------------------------------------------------------------
void image_process(void)
{
    uint32 start;
    uint32 stage;

    start = MODULE_STM1.TIM0.U;
    stage = start;
    image_binarize();
    g_image_profile.binarize_us = image_ticks_to_us(MODULE_STM1.TIM0.U - stage);
    g_image_profile.border_us = 0;

    stage = MODULE_STM1.TIM0.U;
    image_get_edge();
    g_image_profile.edge_us = image_ticks_to_us(MODULE_STM1.TIM0.U - stage);

    stage = MODULE_STM1.TIM0.U;
    Image_Build_Mid_Line();
    g_image_profile.midline_us = image_ticks_to_us(MODULE_STM1.TIM0.U - stage);
    g_image_profile.total_us = image_ticks_to_us(MODULE_STM1.TIM0.U - start);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复制当前 180x80 灰度算法帧
// 参数说明     destination     目标缓冲区，至少 IMG_W*IMG_H 字节
// 返回参数     void
// 使用示例     image_copy_gray(display_pixels);
//-------------------------------------------------------------------------------------------------------------------
void image_copy_gray(uint8 *destination)
{
    if (destination != 0)
        memcpy(destination, img_gray, sizeof(img_gray));
}

#pragma section all restore
