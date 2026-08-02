#include "perspective.h"
#include <math.h>
#include <string.h>

#pragma section all "cpu1_dsram"

// 图像 -> 俯视 的单应矩阵，行优先。s_ok=0 表示还没标定
static float s_h[9];
static float s_hinv[9];
static uint8 s_ok;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断单精度数是否为有限值
// 参数说明     value           待检查数值
// 返回参数     uint8           1=有限值 0=NaN 或无穷
// 使用示例     if (!perspective_finite(value)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static float s_w_img = 0.0f;        // s_h 在赛道上一点的齐次分母符号(图像 -> 俯视)
static float s_w_bev = 0.0f;        // s_hinv 在同一点的齐次分母符号(俯视 -> 图像)

static uint8 perspective_finite(float value)
{
    union
    {
        float  f;
        uint32 u;
    } bits;

    bits.f = value;
    return (uint8)((bits.u & 0x7F800000u) != 0x7F800000u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两个 3x3 双精度矩阵的乘积
// 参数说明     a/b/out         左矩阵、右矩阵与结果
// 返回参数     void
// 使用示例     mat3_mul(a, b, out);
//-------------------------------------------------------------------------------------------------------------------
static void mat3_mul(const double a[9], const double b[9], double out[9])
{
    int r, c, k;

    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++)
        {
            double sum = 0.0;
            for (k = 0; k < 3; k++) sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对四个二维点做 Hartley 归一化
// 参数说明     src/dst         输入点与归一化点
// 参数说明     t/tinv          归一化矩阵及其逆矩阵
// 返回参数     uint8           1=归一化成功 0=点集退化
// 使用示例     if (!normalize4(src, norm, t, tinv)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 normalize4(float src[4][2], double dst[4][2],
                        double t[9], double tinv[9])
{
    int i;
    double cx = 0.0, cy = 0.0, distance = 0.0;
    double scale;

    for (i = 0; i < 4; i++)
    {
        cx += (double)src[i][0];
        cy += (double)src[i][1];
    }
    cx *= 0.25;
    cy *= 0.25;

    for (i = 0; i < 4; i++)
    {
        double dx = (double)src[i][0] - cx;
        double dy = (double)src[i][1] - cy;
        distance += sqrt(dx * dx + dy * dy);
    }
    distance *= 0.25;
    if (distance < 1e-9) return 0;
    scale = 1.4142135623730951 / distance;

    memset(t, 0, sizeof(double) * 9u);
    memset(tinv, 0, sizeof(double) * 9u);
    t[0] = scale; t[2] = -scale * cx;
    t[4] = scale; t[5] = -scale * cy;
    t[8] = 1.0;
    tinv[0] = 1.0 / scale; tinv[2] = cx;
    tinv[4] = 1.0 / scale; tinv[5] = cy;
    tinv[8] = 1.0;

    for (i = 0; i < 4; i++)
    {
        dst[i][0] = scale * ((double)src[i][0] - cx);
        dst[i][1] = scale * ((double)src[i][1] - cy);
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     高斯消元解 8 元线性方程组，标定时才调用，不进控制周期
// 参数说明     a                系数矩阵，行优先 8x8，函数内会被破坏
// 参数说明     b                常数向量，函数内会被破坏
// 参数说明     x                输出解向量
// 返回参数     uint8            1=有解 0=矩阵奇异
// 使用示例     if (!solve8(a, b, x)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 solve8(double a[64], double b[8], double x[8])
{
    int k, i, j, max_row;
    double max_val, factor, sum, tmp;

    for (k = 0; k < 8; k++)
    {
        max_val = fabs(a[k * 8 + k]);
        max_row = k;
        for (i = k + 1; i < 8; i++)
            if (fabs(a[i * 8 + k]) > max_val) { max_val = fabs(a[i * 8 + k]); max_row = i; }
        if (max_val < 1e-12) return 0;                  // 奇异，四点共线或重合

        if (max_row != k)
        {
            for (j = k; j < 8; j++)
            { tmp = a[k * 8 + j]; a[k * 8 + j] = a[max_row * 8 + j]; a[max_row * 8 + j] = tmp; }
            tmp = b[k]; b[k] = b[max_row]; b[max_row] = tmp;
        }
        for (i = k + 1; i < 8; i++)
        {
            factor = a[i * 8 + k] / a[k * 8 + k];
            for (j = k; j < 8; j++) a[i * 8 + j] -= factor * a[k * 8 + j];
            b[i] -= factor * b[k];
        }
    }
    for (i = 7; i >= 0; i--)
    {
        sum = 0.0;
        for (j = i + 1; j < 8; j++) sum += a[i * 8 + j] * x[j];
        x[i] = (b[i] - sum) / a[i * 8 + i];
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     由四对点解单应矩阵，src -> dst
// 参数说明     src/dst          各 4 个点，[i][0]=x [i][1]=y
// 参数说明     h                输出，行优先 9 个系数，h[8] 归一化为 1
// 返回参数     uint8            1=求解成功
// 使用示例     if (!homography(src, dst, h)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 homography(float src[4][2], float dst[4][2], float h[9])
{
    double a[64], b[8], sol[8];
    double src_n[4][2], dst_n[4][2];
    double ts[9], ts_inv[9], td[9], td_inv[9];
    double hn[9], temp[9], hd[9];
    double scale;
    int i, row;

    if (!normalize4(src, src_n, ts, ts_inv)) return 0;
    if (!normalize4(dst, dst_n, td, td_inv)) return 0;
    (void)ts_inv;
    (void)td;

    memset(a, 0, sizeof(a));
    for (i = 0; i < 4; i++)
    {
        double sx = src_n[i][0], sy = src_n[i][1];
        double dx = dst_n[i][0], dy = dst_n[i][1];

        row = 2 * i;
        a[row * 8 + 0] = sx; a[row * 8 + 1] = sy; a[row * 8 + 2] = 1.0;
        a[row * 8 + 6] = -dx * sx; a[row * 8 + 7] = -dx * sy;
        b[row] = dx;

        row = 2 * i + 1;
        a[row * 8 + 3] = sx; a[row * 8 + 4] = sy; a[row * 8 + 5] = 1.0;
        a[row * 8 + 6] = -dy * sx; a[row * 8 + 7] = -dy * sy;
        b[row] = dy;
    }
    if (!solve8(a, b, sol)) return 0;

    for (i = 0; i < 8; i++) hn[i] = sol[i];
    hn[8] = 1.0;

    mat3_mul(hn, ts, temp);
    mat3_mul(td_inv, temp, hd);
    if (fabs(hd[8]) < 1e-12) return 0;
    scale = 1.0 / hd[8];
    for (i = 0; i < 9; i++)
    {
        h[i] = (float)(hd[i] * scale);
        if (!perspective_finite(h[i]) || fabsf(h[i]) > 1e8f) return 0;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     求 3x3 矩阵的逆，用于俯视坐标反算回图像
// 参数说明     m                输入矩阵，行优先
// 参数说明     inv              输出逆矩阵，行优先
// 返回参数     uint8            1=可逆
// 使用示例     if (!inv3(s_h, s_hinv)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 inv3(const float m[9], float inv[9])
{
    float c0 = m[4] * m[8] - m[5] * m[7];
    float c1 = m[5] * m[6] - m[3] * m[8];
    float c2 = m[3] * m[7] - m[4] * m[6];
    float det = m[0] * c0 + m[1] * c1 + m[2] * c2;
    float s;

    if (det > -1e-9f && det < 1e-9f) return 0;
    s = 1.0f / det;

    inv[0] = c0 * s;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * s;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * s;
    inv[3] = c1 * s;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * s;
    inv[5] = (m[2] * m[3] - m[0] * m[5]) * s;
    inv[6] = c2 * s;
    inv[7] = (m[1] * m[6] - m[0] * m[7]) * s;
    inv[8] = (m[0] * m[4] - m[1] * m[3]) * s;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用一个 3x3 单应矩阵变换一个点
// 参数说明     m                矩阵，行优先
// 参数说明     u/v              输入点
// 参数说明     ox/oy            输出点
// 返回参数     uint8            1=有效 0=齐次分母接近 0(消隐线上)
// 使用示例     if (!apply(s_h, col, row, &x, &y)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 apply(const float m[9], float u, float v, float *ox, float *oy, float w_expect)
{
    float w = m[6] * u + m[7] * v + m[8];
    if (w > -1e-6f && w < 1e-6f) return 0;
    if (w_expect != 0.0f && ((w > 0.0f) != (w_expect > 0.0f))) return 0;
    w = 1.0f / w;
    *ox = (m[0] * u + m[1] * v + m[2]) * w;
    *oy = (m[3] * u + m[4] * v + m[5]) * w;
    if (!perspective_finite(*ox) || !perspective_finite(*oy)) return 0;
    if (fabsf(*ox) > 1e6f || fabsf(*oy) > 1e6f) return 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询逆透视矩阵是否已标定可用
// 参数说明     void
// 返回参数     uint8            1=可用
// 使用示例     if (ipm_ready()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 ipm_ready(void)
{
    return s_ok;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除当前逆透视矩阵
// 参数说明     void
// 返回参数     void
// 使用示例     ipm_clear();
//-------------------------------------------------------------------------------------------------------------------
void ipm_clear(void)
{
    memset(s_h, 0, sizeof(s_h));
    memset(s_hinv, 0, sizeof(s_hinv));
    s_w_img = 0.0f;
    s_w_bev = 0.0f;
    s_ok = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入一组逆透视系数并做可用性检查
// 参数说明     h                行优先的 9 个系数
// 返回参数     uint8            1=载入成功
// 使用示例     ipm_load(g_param.ipm_h);
//-------------------------------------------------------------------------------------------------------------------
uint8 ipm_load(const float h[9])
{
    int i;
    float h_new[9];
    float hinv_new[9];
    float tx, ty, col, row;

    if (h == 0) return 0;
    if (h[8] < 0.5f || h[8] > 1.5f) return 0;
    for (i = 0; i < 9; i++)
    {
        if (!perspective_finite(h[i]) || fabsf(h[i]) > 1e8f) return 0;
        h_new[i] = h[i];
    }
    if (!inv3(h_new, hinv_new)) return 0;

    for (i = 0; i < 9; i++)
        if (!perspective_finite(hinv_new[i]) || fabsf(hinv_new[i]) > 1e8f) return 0;

    if (!apply(h_new, (float)IMG_MID_COL, (float)(IMG_H - 4), &tx, &ty, 0.0f)) return 0;
    if (!apply(hinv_new, tx, ty, &col, &row, 0.0f)) return 0;
    if (fabsf(col - (float)IMG_MID_COL) > 0.5f || fabsf(row - (float)(IMG_H - 4)) > 0.5f) return 0;

    memcpy(s_h, h_new, sizeof(s_h));
    memcpy(s_hinv, hinv_new, sizeof(s_hinv));
    // 用画面正中偏近的一点定两个方向的参考符号，那里一定在赛道上、一定在相机前面
    s_w_img = h_new[6] * (float)IMG_MID_COL + h_new[7] * (float)(IMG_H - 4) + h_new[8];
    s_w_bev = hinv_new[6] * tx + hinv_new[7] * ty + hinv_new[8];
    if (s_w_img > -1e-6f && s_w_img < 1e-6f) s_w_img = 1.0f;
    if (s_w_bev > -1e-6f && s_w_bev < 1e-6f) s_w_bev = 1.0f;
    s_ok = 1;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读回当前逆透视系数
// 参数说明     h                输出，行优先的 9 个系数
// 返回参数     void
// 使用示例     ipm_store(g_param.ipm_h);
//-------------------------------------------------------------------------------------------------------------------
void ipm_store(float h[9])
{
    int i;
    if (h == 0) return;
    for (i = 0; i < 9; i++) h[i] = s_ok ? s_h[i] : 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把图像坐标映射到俯视平面
// 参数说明     col/row          图像列/行
// 参数说明     x/y              输出，俯视平面坐标
// 返回参数     uint8            1=映射有效
// 使用示例     if (ipm_to_bev(col, row, &x, &y)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 ipm_to_bev(float col, float row, float *x, float *y)
{
    if (!s_ok) return 0;
    return apply(s_h, col, row, x, y, s_w_img);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把俯视平面坐标映射回图像
// 参数说明     x/y              俯视平面坐标
// 参数说明     col/row          输出，图像列/行
// 返回参数     uint8            1=映射有效
// 使用示例     if (ipm_to_img(x, y, &col, &row)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 ipm_to_img(float x, float y, float *col, float *row)
{
    if (!s_ok) return 0;
    return apply(s_hinv, x, y, col, row, s_w_bev);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用一帧直道图像现场标定逆透视矩阵
// 参数说明     l_near/r_near/row_near  近端采样行的左右边线列与行号
// 参数说明     l_far/r_far/row_far     远端采样行的左右边线列与行号
// 参数说明     len_ratio        两个采样行之间的地面距离 / 赛道宽度
// 返回参数     uint8            1=标定成功
// 使用示例     ipm_calibrate(l0, r0, IMG_H-6, l1, r1, 40, IPM_LEN_RATIO);
//-------------------------------------------------------------------------------------------------------------------
uint8 ipm_calibrate(float l_near, float r_near, float row_near,
                    float l_far,  float r_far,  float row_far, float len_ratio)
{
    float src[4][2], dst[4][2];
    float h[9];
    float hinv[9];
    float half = (float)IPM_HALF_WIDTH;
    float depth;
    int i;

    if (r_near - l_near < (float)IPM_MIN_WIDTH) return 0;
    if (r_far  - l_far  < (float)IPM_FAR_MIN_WIDTH * 0.5f) return 0;
    if (row_near - row_far < (float)IPM_MIN_ROW_GAP) return 0;
    if (!(len_ratio > 0.05f) || !(len_ratio < 20.0f)) return 0;

    depth = 2.0f * half * len_ratio;

    // 图像上的梯形：近端两点 + 远端两点
    src[0][0] = l_near; src[0][1] = row_near;
    src[1][0] = r_near; src[1][1] = row_near;
    src[2][0] = l_far;  src[2][1] = row_far;
    src[3][0] = r_far;  src[3][1] = row_far;

    // 俯视上的矩形：x 向右，y 向远端为负，近端那条边落在 y=0
    dst[0][0] = -half; dst[0][1] = 0.0f;
    dst[1][0] =  half; dst[1][1] = 0.0f;
    dst[2][0] = -half; dst[2][1] = -depth;
    dst[3][0] =  half; dst[3][1] = -depth;

    if (!homography(src, dst, h)) return 0;
    if (!inv3(h, hinv)) return 0;

    for (i = 0; i < 4; i++)
    {
        float x, y, col, row;

        if (!apply(h, src[i][0], src[i][1], &x, &y, 0.0f)) return 0;
        if (fabsf(x - dst[i][0]) > 0.25f || fabsf(y - dst[i][1]) > 0.25f)
            return 0;
        if (!apply(hinv, x, y, &col, &row, 0.0f)) return 0;
        if (fabsf(col - src[i][0]) > 0.25f || fabsf(row - src[i][1]) > 0.25f)
            return 0;
    }

    for (i = 1; i <= 3; i++)
    {
        float t = (float)i * 0.25f;
        float lc = l_near + (l_far - l_near) * t;
        float rc = r_near + (r_far - r_near) * t;
        float rr = row_near + (row_far - row_near) * t;
        float lx, ly, rx, ry;

        if (!apply(h, lc, rr, &lx, &ly, 0.0f) || !apply(h, rc, rr, &rx, &ry, 0.0f))
            return 0;
        if (fabsf(lx + half) > 1.0f || fabsf(rx - half) > 1.0f ||
            ly > 1.0f || ly < -depth - 1.0f || ry > 1.0f || ry < -depth - 1.0f)
            return 0;
    }
    return ipm_load(h);
}

#pragma section all restore
