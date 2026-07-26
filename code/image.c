#include "image.h"
// #include "element.h"                 // 元素识别暂不接入

#pragma section all "cpu1_dsram"

Image my_image;                                 // 图像处理结果
int   Standard_Road_Wide[IMG_H];                // 标准赛宽表
float g_mid_error = 0.0f;                       // 当前中线偏差

// 八邻域搜索顺序: 左边线顺时针，右边线逆时针
static const int8 en_seed_l[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{ 1,-1},{ 1,0},{ 1,1} };
static const int8 en_seed_r[8][2] = { {0,1},{ 1,1},{ 1,0},{ 1,-1},{0,-1},{-1,-1},{-1,0},{-1,1} };

static uint16 en_pts_l[EN_MAX_PTS][2];          // 左边线生长点
static uint16 en_pts_r[EN_MAX_PTS][2];          // 右边线生长点

static uint8 img_gray[IMG_H][IMG_W];            // 灰度帧缓冲区

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
// 函数简介     读取二值图像像素并处理越界坐标
// 参数说明     x/y              列/行
// 返回参数     uint8            IMG_WHITE / IMG_BLACK
// 使用示例     if (img_pixel(x, y) == IMG_WHITE) ...
//-------------------------------------------------------------------------------------------------------------------
static uint8 img_pixel(int x, int y)
{
    if (x < 0 || x >= IMG_W || y < 0 || y >= IMG_H) return IMG_BLACK;
    return my_image.image_two_value[y][x];
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

    for (i = 0; i < IMG_H; i++)
    {
        Standard_Road_Wide[i] = iclip(ROAD_WIDE_FAR + (ROAD_WIDE_NEAR - ROAD_WIDE_FAR) * i / (IMG_H - 1),
                                      4, IMG_W - 4);
        my_image.Left_Line[i]       = 0;
        my_image.Right_Line[i]      = IMG_W - 1;
        my_image.Mid_Line[i]        = IMG_MID_COL;
        my_image.Road_Wide[i]       = IMG_W - 1;
        my_image.Left_Lost_Flag[i]  = 1;
        my_image.Right_Lost_Flag[i] = 1;
        my_image.Mid_Lost_Flag[i]   = 1;
    }
    my_image.Search_Stop_Line = 0;
    my_image.Mid_Valid_Rows   = 0;
    my_image.Valid_Row_Bottom = -1;
    my_image.Valid_Row_Top    = -1;
    my_image.Track_Valid      = 0;
    g_mid_error               = 0.0f;
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
    uint32 hist[256];
    uint32 i, total = 0;
    int    r, c, threshold = 0, th_first = 0, th_last = 0;
    int    gmin = 255, gmax = 0;
    float  sum_all = 0.0f, sum_b = 0.0f, w_b = 0.0f, w_f, num, var, var_max = -1.0f;

    for (i = 0; i < 256; i++) hist[i] = 0;

    // 隔行隔列统计灰度直方图
    for (r = 0; r < IMG_H; r += 2)
    {
        const uint8 *src = img_gray[r];
        for (c = 0; c < IMG_W; c += 2)
        {
            uint8 g = src[c];
            hist[g]++; total++;
            if (g < gmin) gmin = g;
            if (g > gmax) gmax = g;
        }
    }
    my_image.Contrast = gmax - gmin;

    for (i = 0; i < 256; i++) sum_all += (float)i * (float)hist[i];
    for (i = 0; i < 256; i++)
    {
        w_b += (float)hist[i];
        if (w_b <= 0.0f) continue;
        w_f = (float)total - w_b;
        if (w_f <= 0.0f) break;
        sum_b += (float)i * (float)hist[i];
        // 使用等价公式计算类间方差
        num = sum_b * w_f - (sum_all - sum_b) * w_b;
        var = num * num / (w_b * w_f);

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
        for (c = 0; c < IMG_W; c++)
            dst[c] = (src[c] >= threshold) ? IMG_WHITE : IMG_BLACK;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行八邻域孤立点滤波并设置图像边框
// 参数说明     void
// 返回参数     void
// 使用示例     image_filter_frame();
//-------------------------------------------------------------------------------------------------------------------
static void image_filter_frame(void)
{
    int i, j;
    uint32 num;

    // 使用相邻三行指针完成原地滤波
    for (i = 1; i < IMG_H - 1; i++)
    {
        uint8 *p0 = my_image.image_two_value[i - 1];
        uint8 *p1 = my_image.image_two_value[i];
        uint8 *p2 = my_image.image_two_value[i + 1];

        for (j = 1; j < IMG_W - 1; j++)
        {
            uint8 v = p1[j];

            num = (uint32)p0[j-1] + p0[j] + p0[j+1]
                + (uint32)p1[j-1]         + p1[j+1]
                + (uint32)p2[j-1] + p2[j] + p2[j+1];

            if      (num >= 255u * 5u) { if (v == IMG_BLACK) p1[j] = IMG_WHITE; }
            else if (num <= 255u * 2u) { if (v == IMG_WHITE) p1[j] = IMG_BLACK; }
        }
    }

    for (i = 0; i < IMG_H; i++)
    {
        my_image.image_two_value[i][0]         = IMG_BLACK;
        my_image.image_two_value[i][1]         = IMG_BLACK;
        my_image.image_two_value[i][IMG_W - 2] = IMG_BLACK;
        my_image.image_two_value[i][IMG_W - 1] = IMG_BLACK;
    }
    for (j = 0; j < IMG_W; j++)
    {
        my_image.image_two_value[0][j] = IMG_BLACK;
        my_image.image_two_value[1][j] = IMG_BLACK;
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
    uint8 side;

    seed_col = iclip(seed_col, EN_COL_MIN_LIMIT, EN_COL_MAX_LIMIT);

    for (row = EN_START_ROW_BOTTOM; row >= EN_START_ROW_TOP; row--)
    {
        if (row < 0 || row >= IMG_H) continue;
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
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     沿单侧黑白边界执行八邻域跟踪
// 参数说明     sx/sy/seed/pts   起点列/起点行/搜索顺序/输出点集
// 返回参数     uint16           边界点数量
// 使用示例     n = en_trace(sl[0], sl[1], en_seed_l, en_pts_l);
//-------------------------------------------------------------------------------------------------------------------
static uint16 en_trace(int sx, int sy, const int8 (*seed)[2], uint16 (*pts)[2])
{
    uint16 cnt = 0;
    int cx = sx, cy = sy;
    int i, ax, ay, best_x = 0, best_y = 0, found;

    while (cnt < EN_MAX_PTS)
    {
        pts[cnt][0] = (uint16)cx;
        pts[cnt][1] = (uint16)cy;
        cnt++;

        if (cy <= EN_ROW_TOP_LIMIT) break;
        if (cy >= IMG_H - 1) break;         // 防止越过图像底部
        if (cx <= EN_COL_MIN_LIMIT || cx >= EN_COL_MAX_LIMIT) break;

        found = 0;
        for (i = 0; i < 8; i++)
        {
            ax = cx + seed[i][0];
            ay = cy + seed[i][1];
            if (img_pixel(ax, ay) != IMG_BLACK) continue;
            if (img_pixel(cx + seed[(i + 1) & 7][0], cy + seed[(i + 1) & 7][1]) != IMG_WHITE) continue;
            if (!found || ay < best_y) { best_x = ax; best_y = ay; found = 1; }
        }
        if (!found) break;
        if (best_x == cx && best_y == cy) break;
        if (cnt >= 2 && (int)pts[cnt - 2][0] == best_x && (int)pts[cnt - 2][1] == best_y) break;

        cx = best_x; cy = best_y;
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
    static int s_seed_col = IMG_MID_COL;        // 当前起点种子列
    int sl[2], sr[2];
    uint16 n, k;
    int i, x, y, top = IMG_H, last;
    uint8 side;

    for (i = 0; i < IMG_H; i++)
    {
        my_image.Left_Line[i]       = 0;
        my_image.Right_Line[i]      = IMG_W - 1;
        my_image.Left_Lost_Flag[i]  = 1;
        my_image.Right_Lost_Flag[i] = 1;
    }
    my_image.Search_Stop_Line = 0;

    side = en_get_start(s_seed_col, sl, sr);
    if (side)
    {
        if (side & 1u)                          // 左边线取每行最大列
        {
            n = en_trace(sl[0], sl[1], en_seed_l, en_pts_l);
            for (k = 0; k < n; k++)
            {
                x = (int)en_pts_l[k][0]; y = (int)en_pts_l[k][1];
                if (y < 0 || y >= IMG_H) continue;
                if (my_image.Left_Lost_Flag[y] || x > my_image.Left_Line[y])
                {
                    my_image.Left_Line[y] = iclip(x + 1, 0, IMG_W - 1); // 记录边界白侧
                    my_image.Left_Lost_Flag[y] = 0;
                }
            }
        }

        if (side & 2u)                          // 右边线取每行最小列
        {
            n = en_trace(sr[0], sr[1], en_seed_r, en_pts_r);
            for (k = 0; k < n; k++)
            {
                x = (int)en_pts_r[k][0]; y = (int)en_pts_r[k][1];
                if (y < 0 || y >= IMG_H) continue;
                if (my_image.Right_Lost_Flag[y] || x < my_image.Right_Line[y])
                {
                    my_image.Right_Line[y] = iclip(x - 1, 0, IMG_W - 1);// 记录边界白侧
                    my_image.Right_Lost_Flag[y] = 0;
                }
            }
        }

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

//  if (g_island.island_state == 3 && my_image.Search_Stop_Line > 70)
//      my_image.Search_Stop_Line = 70;         // 环岛启用后恢复

    // 汇总前瞻区赛宽与丢线状态
    my_image.Left_Lost_Counter = 0; my_image.Right_Lost_Counter = 0; my_image.Both_Lost_Counter = 0;
    my_image.Boundry_Start_Left = 0; my_image.Boundry_Start_Right = 0;
    top = IMG_H - iclip(my_image.Search_Stop_Line, 0, IMG_H);

    for (i = IMG_H - 1; i >= top; i--)
    {
        my_image.Road_Wide[i] = my_image.Right_Line[i] - my_image.Left_Line[i];
        if (my_image.Left_Lost_Flag[i])  my_image.Left_Lost_Counter++;
        if (my_image.Right_Lost_Flag[i]) my_image.Right_Lost_Counter++;
        if (my_image.Left_Lost_Flag[i] && my_image.Right_Lost_Flag[i]) my_image.Both_Lost_Counter++;
        if (my_image.Boundry_Start_Left  == 0 && !my_image.Left_Lost_Flag[i])  my_image.Boundry_Start_Left  = i;
        if (my_image.Boundry_Start_Right == 0 && !my_image.Right_Lost_Flag[i]) my_image.Boundry_Start_Right = i;
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
    int last_mid = IMG_MID_COL;

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
            func_abs(my_image.Left_Line[i]     - my_image.Left_Line[i - 1]) <= 5 &&
            func_abs(my_image.Left_Line[i - 1] - my_image.Left_Line[i - 2]) <= 5 &&
            func_abs(my_image.Left_Line[i - 2] - my_image.Left_Line[i - 3]) <= 5 &&
            (my_image.Left_Line[i] - my_image.Left_Line[i + 2]) >= CORNER_JUMP &&
            (my_image.Left_Line[i] - my_image.Left_Line[i + 3]) >= 15 &&
            (my_image.Left_Line[i] - my_image.Left_Line[i + 4]) >= 15)
            my_image.Left_Up_Find = i;

        if (my_image.Right_Up_Find == 0 &&
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

//  环岛单边偏差计算暂不接入，后续恢复元素识别时再启用。
//  if (g_island.detect && g_island.island_state != 0 && g_island.island_state != 5)
//  {
//      int inner = (g_island.island_state == 3) ? g_elem_action.ring_side_offset : 0;
//      if ((g_island.detect == 1) == (g_island.island_state == 3))
//          for (i = start_point; i < end_point; i++)
//              err += (float)(IMG_MID_COL - Standard_Road_Wide[i] / 2 - my_image.Left_Line[i] - inner);
//      else
//          for (i = start_point; i < end_point; i++)
//              err += (float)(IMG_MID_COL + Standard_Road_Wide[i] / 2 - my_image.Right_Line[i] + inner);
//      g_mid_error = err / (float)n;
//      return g_mid_error;
//  }

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
    image_binarize();
    image_filter_frame();
    image_get_edge();
    Image_Build_Mid_Line();
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
