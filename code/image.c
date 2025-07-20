/*
 * image.c
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */
//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "zf_common_headfile.h"
#include "image.h"
#include "motor.h"

// 宏定义
#define white_pixel 255
#define black_pixel 0
#define standard_foot_roadwidth 35 // 可能受光线等二值化影响,逆透视改变则需要对应改变
#define bin_jump_num 1             // 跳过的点数
#define border_max image_w - 2     // 边界最大值
#define border_min 1               // 边界最小值
#define USE_num image_h * 3        // 定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点
#define lost_width 15              // 处理区两侧单边延展列数，基于底线宽（底线丢时可能处理区不完全包含于视野）
#define hightest_least 15          // 最终处理上边忽略行数之最少值（寻线可能比其更大）

#define GrayScale 256 // 大津法中

/*变量声明*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold;                   // 图像分割阈值
uint8 bin_image[image_h][image_w];        // 图像数组
uint8 bin_image_circlr[image_h][image_w]; // 图像数组
corner_inline *l_corner_point = NULL;
corner_inline *r_corner_point = NULL;

// 存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = {{0}};       // 左线八邻域
uint16 points_r[(uint16)USE_num][2] = {{0}};       // 右线八邻域
uint16 dir_r[(uint16)USE_num] = {0};               // 用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = {0};               // 用来存储左边生长方向
uint16 data_stastics_l = 0;                        // 统计左边找到点的个数
uint16 data_stastics_r = 0;                        // 统计右边找到点的个数
uint8 hightest = 0;                                // 八邻域寻线得最高点，后与hightest_least比较取大值
uint16 trans_l[(uint16)USE_num][2] = {{0}};        // 左线变换
uint16 trans_r[(uint16)USE_num][2] = {{0}};        // 右线
uint16 beyond_trans_l[(uint16)USE_num][2] = {{0}}; // 为寻上角而过度逆变
uint16 beyond_trans_r[(uint16)USE_num][2] = {{0}};

uint8 l_border[image_h];    // 左线数组(内容, 序号)
uint8 r_border[image_h];    // 右线数组
uint8 center_line[image_h]; // 中线数组
uint8 cent_line_high;

IfxCpu_mutexLock screen_mutex;
IfxCpu_mutexLock param_mutex;
IfxCpu_mutexLock dspeed_mutex;

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
float Maxmin(float a, float min, float max)
{
    if (a < min)
        a = min;
    if (a > max)
        a = max;
    return a;
}

int my_abs(int value)
{
    if (value >= 0)
        return value;
    else
        return -value;
}

int16 limit_a_b(int16 x, int16 a, int16 b)
{
    if (x < a)
        x = a;
    if (x > b)
        x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)
        return y;
    else if (x < -y)
        return -y;
    else
        return x;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8 (*mt9v03x_image)[image_w])
{
#define use_num 1 // 1就是不压缩，2就是压缩一倍
    uint8 row = 0, line = 0;
    for (uint8 i = 0; i < image_h; i += use_num) //
    {
        for (uint8 j = 0; j < image_w; j += use_num) //
        {
            original_image[row][line] = mt9v03x_image[i][j]; // 这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
    uint16 Image_Width = col;
    uint16 Image_Height = row;
    int X;
    uint16 Y;
    uint8 *data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack = 0, OmegaFore = 0, MicroBack = 0, MicroFore = 0, SigmaB = 0, Sigma = 0; // 类间方差;
    uint8 MinValue = 0, MaxValue = 0;
    uint8 Threshold = 0;

    for (Y = 0; Y < Image_Height; Y++) // Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        // Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y * Image_Width + X]]++; // 统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ; // 获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--)
        ; // 获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue; // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue; // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y]; //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y; // 灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];                                               // 前景像素点数
        PixelFore = Amount - PixelBack;                                                    // 背景像素点数
        OmegaBack = (double)PixelBack / Amount;                                            // 前景像素百分比
        OmegaFore = (double)PixelFore / Amount;                                            // 背景像素百分比
        PixelIntegralBack += HistGram[Y] * Y;                                              // 前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;                             // 背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;                                 // 前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;                                 // 背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // g
        if (Sigma > SigmaB)                                                                // 遍历最大的类间方差g
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 threshold_add = -10;
void turn_to_bin(void)
{
    uint8 i, j;
    image_thereshold = otsuThreshold(original_image[0], image_w, image_h) + threshold_add; ///////////////根据反光情况修改
    for (i = 0; i < image_h; i++)
    {
        for (j = 0; j < image_w; j++)
        {
            if (original_image[i][j] > image_thereshold)
                    bin_image[i][j] = white_pixel;
                else
                    bin_image[i][j] = black_pixel;
        }
    }
}
/*----------------------------------------------------------------------
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
-------------------------------------------------------------------------*/
static uint8 start_point_l[2] = {0}; // 左边起点的x，y值
static uint8 start_point_r[2] = {0}; // 右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0, l_found = 0, r_found = 0;
    // 清零
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y

    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y

    // 从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;         // x
        start_point_l[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            // printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;         // x
        start_point_r[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            // printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if (l_found && r_found)
        return 1;
    else
    { // 未找到起点
        return 0;
    }
}

/*-------------------------------------------------------------------------------------------
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
--------------------------------------------------------------------------------------------*/
void search_l_r(uint16 break_flag, uint8 (*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{

    uint8 i = 0, j = 0;
    // 左边变量
    uint8 search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    uint8 temp_l[8][2] = {{0}};
    uint8 center_point_l[2] = {0};
    uint16 l_data_statics; // 统计左边
    // 定义八个邻域
    static int8 seeds_l[8][2] = {
        {0, 1},
        {-1, 1},
        {-1, 0},
        {-1, -1},
        {0, -1},
        {1, -1},
        {1, 0},
        {1, 1},
    };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    // 这个是顺时针

    // 右边变量
    uint8 search_filds_r[8][2] = {{0}};
    uint8 center_point_r[2] = {0}; // 中心坐标点
    uint8 index_r = 0;             // 索引下标
    uint8 temp_r[8][2] = {{0}};
    uint16 r_data_statics; // 统计右边
    // 定义八个邻域
    static int8 seeds_r[8][2] = {
        {0, 1},
        {1, 1},
        {1, 0},
        {1, -1},
        {0, -1},
        {-1, -1},
        {-1, 0},
        {-1, 1},
    };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    // 这个是逆时针

    l_data_statics = *l_stastic; // 统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic; // 统计找到了多少个点，方便后续把点全部画出来

    // 第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x; // x
    center_point_l[1] = l_start_y; // y
    center_point_r[0] = r_start_x; // x
    center_point_r[1] = r_start_y; // y

    // 开启邻域循环
    while (break_flag--)
    {
        // 左边
        for (i = 0; i < 8; i++) // 传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0]; // x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1]; // y
        }
        // 中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0]; // x
        points_l[l_data_statics][1] = center_point_l[1]; // y
        l_data_statics++;                                // 索引加一

        // 右边
        for (i = 0; i < 8; i++) // 传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0]; // x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1]; // y
        }
        // 中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0]; // x
        points_r[r_data_statics][1] = center_point_r[1]; // y

        index_l = 0; // 先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0; // 先清零，后使用
            temp_l[i][1] = 0; // 先清零，后使用
        }

        // 左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0 && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i); // 记录生长方向
            }

            if (index_l)
            {
                // 更新坐标点
                center_point_l[0] = temp_l[0][0]; // x
                center_point_l[1] = temp_l[0][1]; // y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0]; // x
                        center_point_l[1] = temp_l[j][1]; // y
                    }
                }
            }
        }
        if (l_data_statics > 2 && r_data_statics > 2)
        {
            if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] && points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) || (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
            {
                // printf("三次进入同一个点，退出\n");
                break;
            }
        }

        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2 && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2))
        {
            // printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1; // 取出最高点
            // printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            // printf("\n如果左边比右边高了，左边等待右边\n");
            continue; // 如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7 && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1])) // 左边比右边高且已经向下生长了
        {
            // printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0]; // x
            center_point_l[1] = points_l[l_data_statics - 1][1]; // y
            l_data_statics--;
        }
        r_data_statics++; // 索引加一

        index_r = 0; // 先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0; // 先清零，后使用
            temp_r[i][1] = 0; // 先清零，后使用
        }

        // 右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0 && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;                       // 索引加一
                dir_r[r_data_statics - 1] = (i); // 记录生长方向
                // printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                // 更新坐标点
                center_point_r[0] = temp_r[0][0]; // x
                center_point_r[1] = temp_r[0][1]; // y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0]; // x
                        center_point_r[1] = temp_r[j][1]; // y
                    }
                }
            }
        }
    }
    // 取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}
/*--------------------------------------------------
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
-------------------------------------------------------*/
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    // 初始化
    for (i = 0; i < image_h; i++)
    {
        l_border[i] = border_min;
    }
    h = image_h - 2;
    // 左边
    for (j = 0; j < total_L; j++)
    {
        if ((points_l[j][1] == h)) // && ((h < l_corner_point->f_cp_y)||(h > l_corner_point->h_cp_y)))
        {
            l_border[h] = trans_l[j][0] + 1; // 内一像素
        }
        else
            continue; // 每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break; // 到最后一行退出
        }
    }
}
/*------------------------------------------------------------------
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
---------------------------------------------------------------------*/
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < image_h; i++)
    {
        r_border[i] = border_max; // 右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    // 右边
    for (j = 0; j < total_R; j++)
    {
        if ((points_r[j][1] == h)) // && ((h < r_corner_point->f_cp_y)||(h > r_corner_point->h_cp_y)))
        {
            r_border[h] = trans_r[j][0] - 1; // 外一像素
        }
        else
            continue; // 每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
            break; // 到最后一行退出
    }
}

// 定义膨胀和腐蚀的阈值区间
#define threshold_max 255 * 5                  // 此参数可根据自己的需求调节
#define threshold_min 255 * 2                  // 此参数可根据自己的需求调节
void image_filter(uint8 (*bin_image)[image_w]) // 形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint32 num = 0;
    for (uint16 i = 1; i < image_h - 1; i++)
    {
        for (uint16 j = 1; j < (image_w - 1); j++)
        {
            // 统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1] + bin_image[i][j - 1] + bin_image[i][j + 1] + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = white_pixel; // 白
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = black_pixel; // 黑
            }
        }
    }
}

/*-------------------------------------------------------
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框, 方便八邻域
参数说明：uint8(*image)[image_w] 图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
----------------------------------------------------------*/
void image_draw_rectan(uint8 (*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;
    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        // image[image_h-1][i] = 0;
    }
}

////-------逆变换边线------------------------------------------------------------------------------
double normal_matrix[3][3] = {{-1.11048, -4.06186, 186.592}, {-0.149566, -3.88910, 136.969}, {-0.00236018, -0.0406898, 1}};
double beyond_matrix[3][3] = {{-1.11048, -4.06186, 186.592}, {-0.149566, -3.88910, 136.969}, {-0.00236018, -0.0406898, 1}};
void EdgeLinePerspective(uint16 *in_line, uint8 num, uint16 *out_line, double *change_inverse_Mat)
{
    for (uint8 count = 0; count < num; count++)
    {
        float i = in_line[count * 2 + 0];
        float j = in_line[count * 2 + 1];
        uint16 solve_x = (int)((change_inverse_Mat[0] * i + change_inverse_Mat[1] * j + change_inverse_Mat[2]) / (change_inverse_Mat[6] * i + change_inverse_Mat[7] * j + change_inverse_Mat[8]));
        uint16 solve_y = (int)((change_inverse_Mat[3] * i + change_inverse_Mat[4] * j + change_inverse_Mat[5]) / (change_inverse_Mat[6] * i + change_inverse_Mat[7] * j + change_inverse_Mat[8]));
        out_line[count * 2 + 0] = solve_x;
        out_line[count * 2 + 1] = solve_y;
    }
}

bool is_valid(int x, int y)
{
    return (x >= 0 && x < image_w && y >= TU_CIRCLE_Y_MIN && y < TU_CIRCLE_Y_MAX);
}

int tu_min(int a, int b)
{
    return (a < b) ? a : b;
}

void bfs(uint8 *image, int start_x, int start_y, int *sum_x, int *sum_y, int *count, int *x_min, int *x_max, int *y_min, int *y_max)
{
    int queue_x[TU_QUEUE_SIZE], queue_y[TU_QUEUE_SIZE];
    int front = 0, rear = 0; // 队列的头尾索引

    queue_x[rear] = start_x;
    queue_y[rear] = start_y;
    rear++;

    image[start_y * image_w + start_x] = TU_VISITED; // 标记访问
    *sum_x = start_x;
    *sum_y = start_y;
    *count = 1;

    int min_x = start_x, max_x = start_x;
    int min_y = start_y, max_y = start_y;

    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};

    while (front < rear)
    {
        int x = queue_x[front];
        int y = queue_y[front];
        front++;

        for (int d = 0; d < 4; d++)
        {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (is_valid(nx, ny) && image[ny * image_w + nx] == 0)
            {

                image[ny * image_w + nx] = TU_VISITED;
                queue_x[rear] = nx;
                queue_y[rear] = ny;
                rear++;

                *sum_x += nx;
                *sum_y += ny;
                (*count)++;

                if (nx < min_x)
                    min_x = nx;
                if (nx > max_x)
                    max_x = nx;
                if (ny < min_y)
                    min_y = ny;
                if (ny > max_y)
                    max_y = ny;
            }
        }
    }
    *x_min = min_x;
    *x_max = max_x;
    *y_min = min_y;
    *y_max = max_y;
    int width = max_x - min_x;
    int height = max_y - min_y;

    // 从中心点开始是否被截断
    bool l_t_valid = true;
    bool r_t_valid = true;
    bool l_b_valid = true;
    bool r_b_valid = true;

    int l_t_x = min_x;
    int l_t_y = min_y;
    int r_t_x = max_x;
    int r_t_y = min_y;
    int l_b_x = min_x;
    int l_b_y = max_y;
    int r_b_x = max_x;
    int r_b_y = max_y;

    int center_x = (min_x + max_x) / 2;
    int center_y = (min_y + max_y) / 2;

    int num = tu_min(max_x - min_x, max_y - min_y) * 2;
    // 统计从中心点到四个角点的线段上有多少TU_VISITED的点
    int l_t_count = 0, r_t_count = 0, l_b_count = 0, r_b_count = 0;
    for (int i = 0; i < num; i++)
    {
        int x1 = center_x + (l_t_x - center_x) * i / num;
        int y1 = center_y + (l_t_y - center_y) * i / num;
        if (is_valid(x1, y1) && l_t_valid)
        {
            if (image[y1 * image_w + x1] == TU_VISITED || image[y1 * image_w + x1] == TU_BOX_COLOR)
            {
                image[y1 * image_w + x1] = TU_BOX_COLOR; // 画框
                l_t_count++;
            }
            else
            {
                l_t_valid = false;
            }
        }
        else
        {
            l_t_valid = false;
        }
        x1 = center_x + (r_t_x - center_x) * i / num;
        y1 = center_y + (r_t_y - center_y) * i / num;
        if (is_valid(x1, y1) && r_t_valid)
        {
            if (image[y1 * image_w + x1] == TU_VISITED || image[y1 * image_w + x1] == TU_BOX_COLOR)
            {
                image[y1 * image_w + x1] = TU_BOX_COLOR; // 画框
                r_t_count++;
            }
            else
            {
                r_t_valid = false;
            }
        }
        else
        {
            r_t_valid = false;
        }

        x1 = center_x + (l_b_x - center_x) * i / num;
        y1 = center_y + (l_b_y - center_y) * i / num;
        if (is_valid(x1, y1) && l_b_valid)
        {
            if (image[y1 * image_w + x1] == TU_VISITED || image[y1 * image_w + x1] == TU_BOX_COLOR)
            {
                image[y1 * image_w + x1] = TU_BOX_COLOR; // 画框
                l_b_count++;
            }
            else
            {
                l_b_valid = false;
            }
        }
        else
        {
            l_b_valid = false;
        }

        x1 = center_x + (r_b_x - center_x) * i / num;
        y1 = center_y + (r_b_y - center_y) * i / num;
        if (is_valid(x1, y1) && r_b_valid)
        {
            if (image[y1 * image_w + x1] == TU_VISITED || image[y1 * image_w + x1] == TU_BOX_COLOR)
            {
                image[y1 * image_w + x1] = TU_BOX_COLOR; // 画框
                r_b_count++;
            }
            else
            {
                r_b_valid = false;
            }
        }
        else
        {
            r_b_valid = false;
        }
    }

    int l_diff = abs(l_t_count - l_b_count);
    int r_diff = abs(r_t_count - r_b_count);
    float l_ratio = (float)l_diff / *count;
    float r_ratio = (float)r_diff / *count;
    int min_diff = tu_min(l_diff, r_diff);
    int diff_1 = abs(l_t_count - r_b_count);
    int diff_2 = abs(r_t_count - l_b_count);
    //    printf("count=%d\n", *count);
    //    printf("l_ratio=%f\n", l_ratio);
    //    printf("r_ratio=%f\n", r_ratio);

    // 检查是否是有效区域
    // if (l_ratio > TU_MIN_DIFF_RATIO && r_ratio > TU_MIN_DIFF_RATIO && min_diff != 0 && abs(l_diff - r_diff) / min_diff < TU_MAX_L_R_DIFF_RATIO)
    if (diff_1 < 2 && diff_2 < 2 && min_diff != 0 && abs(l_diff - r_diff) / min_diff < TU_MAX_L_R_DIFF_RATIO)

    {
        // 满足条件，标记为有效区域
    }
    else
    {
        *count = 0; // 直接标记此区域为无效
    }
    // 检查是否和图像边界相交
    if (min_x == 0 || max_x == image_w - 1 || min_y == 0 || max_y == image_h - 1)
    {
        *count = 0; // 直接标记此区域为无效
    }
}

void draw_box(uint8 *image, int x_min, int x_max, int y_min, int y_max)
{
    int left = (x_min < 0) ? 0 : x_min;
    int right = (x_max >= image_w) ? image_w - 1 : x_max;
    int top = (y_min < 0) ? 0 : y_min;
    int bottom = (y_max >= image_h) ? image_h - 1 : y_max;
    for (int i = left; i <= right; i++)
    {
        image[top * image_w + i] = TU_BOX_COLOR;    // 上边
        image[bottom * image_w + i] = TU_BOX_COLOR; // 下边
    }
    for (int j = top; j <= bottom; j++)
    {
        image[j * image_w + left] = TU_BOX_COLOR;  // 左边
        image[j * image_w + right] = TU_BOX_COLOR; // 右边
    }
}

void draw_circle_range_line(uint8 *image)
{
    for (uint16 i = 0; i < image_w; i++)
    {
        image[TU_CIRCLE_Y_MIN * image_w + i] = TU_BOX_COLOR; // 上边
        image[TU_CIRCLE_Y_MAX * image_w + i] = TU_BOX_COLOR; // 下边
    }
}
/**
 * @brief 寻找所有的区域
 * @param image
 */
bool img_update;
bool find_circle(uint8 *image)
{
    bool find_circle = false;
    for (int y = TU_CIRCLE_Y_MIN; y < TU_CIRCLE_Y_MAX; y++)
    {
        for (int x = 0; x < image_w; x++)
        {
            if (image[y * image_w + x] == 0)
            {
                int sum_x = 0, sum_y = 0, count = 0;
                int x_min = 0, x_max = 0, y_min = 0, y_max = 0;
                bfs(image, x, y, &sum_x, &sum_y, &count, &x_min, &x_max, &y_min, &y_max);
                if (count != 0)
                if (count > TU_MIN_CIRCLE_COUNT && count < TU_MAX_CIRCLE_COUNT)
                {
                    // printf("count=%d\n", count);
                    int center_x = sum_x / count;
                    int center_y = sum_y / count;
                    draw_box(image, x_min, x_max, y_min, y_max); // 画框
                    find_circle = true;
                }
            }
        }
    }
    return find_circle;
}

bool circle_flag = false; // 是否找到环岛
float thre = 20; //标准差阈值
//判断左边界是不是直道
bool left_continue(void)
{
    int sum = 0;
    int count = 0;
    for (int i = 0; i < data_stastics_l; i++)
    {
        if(trans_l[i][1] >= 10 && trans_l[i][1] <= 60)
        {
            sum += trans_l[i][0];
            count++;
        }
    }
    float average = sum / count;
    sum = 0;
    for (int i = 0; i < data_stastics_l; i++)
    {
        if(trans_l[i][1] >= 10 && trans_l[i][1] <= 60)
        {
            sum += (trans_l[i][0] - average) * (trans_l[i][0] - average);
        }
    }
    sum /= count;
    float std = sqrt(sum);
    return (std > thre ? 0 : 1);
}

bool right_continue(void)
{
    int sum = 0;
    int count = 0;
    for (int i = 0; i < data_stastics_r; i++)
    {
        if(trans_r[i][1] >= 15 && trans_r[i][1] <= 60)
        {
            sum += trans_r[i][0];
            count++;
        }
    }
    float average = sum / count;
    sum = 0;
    for (int i = 0; i < data_stastics_r; i++)
    {
        if(trans_r[i][1] >= 15 && trans_r[i][1] <= 60)
        {
            sum += (trans_r[i][0] - average) * (trans_r[i][0] - average);
        }
    }
    sum /= count;
    float std = sqrt(sum);
    return (std > thre ? 0 : 1);
}

bool find_circle_area(void)
{
    if (img_update)
    {
        // 把mt9v03x_image二值化后并保存到bin_image_circlr，此处手动设置阈值
        
        for (int i = 0; i < image_h; i++)
        {
            for (int j = 0; j < image_w; j++)
            {
                bin_image_circlr[i][j] = bin_image[i][j];
            }
        }
        bool find = find_circle(bin_image_circlr[0]);
        img_update = false;
        if (find)
        {
            circle_flag = true;
            return true;
        }
        else
        {
            circle_flag = false;
            return false;
        }
    }
    else
    {
        if (circle_flag)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

int y1 = 53;
int y2 = 67;
int turn_thre = 5;

bool is_right_area(void){
    int x1 = 0;
    int x2 = 0;
    int points_got =0;
    for (int i = 0; i < data_stastics_r; i++)
    {
       if(trans_r[i][1] == y1 && points_got != 1)
       {
           x1 = trans_r[i][0];
           points_got = (points_got == 0) ? 1 : 3;
       }

       else if(trans_r[i][1] == y2 && points_got != 2)
       {
           x2 = trans_r[i][0];
           points_got = (points_got == 0) ? 2 : 3;
       }

       else if(points_got == 3)
           break;
    }
    return ((x2 - x1) >= turn_thre ? 1 : 0);
}

bool is_left_area(void){
    int x1 = 0;
    int x2 = 0;
    int points_got =0;
    for (int i = 0; i < data_stastics_l; i++)
    {
       if(trans_l[i][1] == y1 && points_got != 1)
       {
           x1 = trans_l[i][0];
           points_got = (points_got == 0) ? 1 : 3;
       }

       else if(trans_l[i][1] == y2 && points_got != 2)
       {
           x2 = trans_l[i][0];
           points_got = (points_got == 0) ? 2 : 3;
       }

       else if(points_got == 3)
           break;
    }
    return ((x1 - x2) >= turn_thre ? 1 : 0);
}
/*---------------------------------------
函数名称：error_calculate(void)
功能说明：计算原始误差，并计算左右轮参数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
--------------------------------------------*/
float error = 0;
float error_test = 0;
float error_last = 0;
float err_kp = 33.5;
float err_kd = 2.4;
int d_speed = 0;       // 定义左右两轮的差速
int e_calcu_lenth = 0; // 动态的误差计算范围
int dspeed_now = 0;
void error_calculate(void)
{
    float err_kp_now = err_kp;
    float err_kd_now = err_kd;

    e_calcu_lenth = 0;
    for (int i = image_h - 20; i > 80; i--)
    {
        if (center_line[i])
        {
            error += (image_w / 2.0f - center_line[i]);
            e_calcu_lenth++;
        }
        else
        {
            break;
        }
    }
    if (e_calcu_lenth > 0)
    {
        error /= e_calcu_lenth;
    }

    // 3) 用 P+D 计算输出差速
    float delta_error = error - error_last;
    float p_term = err_kp_now * error;
    float d_term = err_kd_now * delta_error;
    float dspeed_f = p_term + d_term;

    // 限幅并写回全局 d_speed
    int dspeed_i = Maxmin(dspeed_f, -150.0f, 150.0f);
    if (IfxCpu_acquireMutex(&dspeed_mutex))
    {
        d_speed = dspeed_i;
        IfxCpu_releaseMutex(&dspeed_mutex);
    }

    // 4) 保存本次误差，用于下次 D 项计算
    error_last = error;
}

void show_star(uint8 x, uint8 y)
{
    if ((x > 0) && (x < ips200_width_max - 1) && (y > 0) && (y < ips200_height_max - 1))
        ips200_draw_point((uint16)x + 10, (uint16)y + 125, uesr_RED);
    if ((x > -1) && ((x + 2) < ips200_width_max) && (y > 0) && (y < ips200_height_max - 1))
        ips200_draw_point((uint16)x + 1 + 10, (uint16)y + 125, uesr_RED);
    if ((x > 0) && (x < ips200_width_max - 1) && (y > -1) && ((y + 2) < ips200_height_max))
        ips200_draw_point((uint16)x + 10, (uint16)y + 1 + 125, uesr_RED);
    if ((x > 1) && ((x) < ips200_width_max) && (y > 0) && (y < ips200_height_max - 1))
        ips200_draw_point((uint16)x - 1 + 10, (uint16)y + 125, uesr_RED);
    if ((x > 0) && (x < ips200_width_max - 1) && (y > 1) && (y < ips200_height_max))
        ips200_draw_point((uint16)x + 10, (uint16)y - 1 + 125, uesr_RED);
}
/***********************************************
 * @name  : corner_4points
 * @brief : 一边线以曲率半径寻角点,如不配合逆透视则上角点效果不好
 * @param : uint16* in_line: 输入八邻域边线二维数组:[i][0]=x,[i][1]=y
 *          int num: 输入边线的长度
 *          int dist: 三个点求曲率的两点之间的间隔的数量
 * @return: 第一个寻得角点,若为0则未寻得
 ************************************************/
int corner_4points(uint16 *in_line, int num, int dist, bool orient)
{
    if (orient == 0)
        for (int i = dist - 1; i < num - dist; i += dist) // 从小至大寻，对于八邻域边线即从底向上，找低角
        {
            if ((in_line[i * 2] > 10) && (in_line[i * 2] < 178) && (in_line[i * 2 + 1] > hightest + 20) && (in_line[i * 2 + 1] < 90))
            { // 处理画面中心之点，忽略左右上下屏幕边缘
                float d1 = (in_line[i * 2 + 1] - in_line[i * 2 + 1 - dist * 2]) / (in_line[i * 2] - in_line[i * 2 - dist * 2]);
                float d2 = (in_line[i * 2 + 1 + dist * 2] - in_line[i * 2 + 1]) / (in_line[i * 2 + dist * 2] - in_line[i * 2]);
                float dd = (d2 - d1) / (in_line[i * 2 + dist * 2] - in_line[i * 2 - dist * 2]);
                if (!dd)
                    continue;
                float rr = (1 + (d1 + d2) * (d1 + d2) / 4) * (1 + (d1 + d2) * (d1 + d2) / 4) * (1 + (d1 + d2) * (d1 + d2) / 4) / dd / dd;
                int t_point_x = (int)((4 * in_line[i * 2] - in_line[i * 2 - dist * 2] - in_line[i * 2 + dist * 2]) / 2);
                int t_point_y = (int)((4 * in_line[i * 2 + 1] - in_line[i * 2 - dist * 2 + 1] - in_line[i * 2 + dist * 2 + 1]) / 2);
                if ((rr < 100))
                {
                    return i;
                }
            }
        }
    if (orient == 1)
        for (int i = num - dist; i > dist - 1; i -= dist) // 从大至小寻，对于八邻域边线即从顶向下，找高角
        {
            if ((in_line[i * 2] > 10) && (in_line[i * 2] < 178) && (in_line[i * 2 + 1] > hightest + 20) && (in_line[i * 2 + 1] < 90))
            { // 处理画面中心之点，忽略左右上下屏幕边缘
                float d1 = (in_line[i * 2 + 1] - in_line[i * 2 + 1 - dist * 2]) / (in_line[i * 2] - in_line[i * 2 - dist * 2]);
                float d2 = (in_line[i * 2 + 1 + dist * 2] - in_line[i * 2 + 1]) / (in_line[i * 2 + dist * 2] - in_line[i * 2]);
                float dd = (d2 - d1) / (in_line[i * 2 + dist * 2] - in_line[i * 2 - dist * 2]);
                if (!dd)
                    continue;
                float rr = (1 + (d1 + d2) * (d1 + d2) / 4) * (1 + (d1 + d2) * (d1 + d2) / 4) * (1 + (d1 + d2) * (d1 + d2) / 4) / dd / dd;
                int t_point_x = (int)((4 * in_line[i * 2] - in_line[i * 2 - dist * 2] - in_line[i * 2 + dist * 2]) / 2);
                int t_point_y = (int)((4 * in_line[i * 2 + 1] - in_line[i * 2 - dist * 2 + 1] - in_line[i * 2 + dist * 2 + 1]) / 2);
                if ((rr < 100))
                {
                    return i;
                }
            }
        }
    return 0;
}
/*------------------------------------------------------------------------------------------
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
----------------------------------------------------------------------------------------------*/
int xflg_now = 0;
int foot_roadwidth = 0;
bool left_ctn = 0;
bool right_ctn = 0;
void image_process(void)
{
    img_update = true;
    circle_flag = false;
    Get_image(mt9v03x_image);
    turn_to_bin();
    /*提取赛道边界*/
    image_filter(bin_image);      // 滤波
    image_draw_rectan(bin_image); // 预处理
                                  // 清零
    memset(center_line, 0, sizeof(center_line));
    memset(l_border, 0, sizeof(l_border));
    memset(r_border, 0, sizeof(r_border));
    memset(points_l, 0, sizeof(points_l));
    memset(points_r, 0, sizeof(points_r));
    memset(trans_l, 0, sizeof(trans_l));
    memset(trans_r, 0, sizeof(trans_r));
    memset(trans_r, 0, sizeof(beyond_trans_l));
    memset(trans_r, 0, sizeof(beyond_trans_r));
    data_stastics_l = 0;
    data_stastics_r = 0;
//    left_ctn = 0;
    uint16 lbc_x = 0;
    uint16 lbc_y = 0;
    uint16 rbc_x = 0;
    uint16 rbc_y = 0;
    int lbc, rbc;

    if (get_start_point(image_h - 2)) // 找到起点了，再执行八领域，没找到就一直找
    {
        //    printf("正在开始八领域\n");
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        if (hightest < hightest_least)
            hightest = hightest_least; // 最高边判定
        //    printf("八邻域已结束\n");
        lbc = corner_4points(&points_l, data_stastics_l, 5, 0); // 左下角点
        rbc = corner_4points(&points_r, data_stastics_r, 5, 0);
        if (lbc && (points_l[lbc][1] > 30)&&(points_l[lbc][1]<100))
                    {
                        lbc_x = points_l[lbc][0];
                        lbc_y = points_l[lbc][1];
                        //show_star(lbc_x, lbc_y);
                    }
                    if (rbc && (points_r[rbc][1] > 30)&&(points_r[rbc][1]<100))
                    {
                        rbc_x = points_r[rbc][0];
                        rbc_y = points_r[rbc][1];
                       // show_star(rbc_x, rbc_y);
                    }
        if (IfxCpu_acquireMutex(&screen_mutex))
        {
            ips200_show_int(0, 290, lbc_y, 3);
            ips200_show_int(40, 290, rbc_y, 3);
            IfxCpu_releaseMutex(&screen_mutex);

        }
        // 边线逆透视
        EdgeLinePerspective(&points_l, data_stastics_l, &trans_l, &normal_matrix);
        EdgeLinePerspective(&points_r, data_stastics_r, &trans_r, &normal_matrix);
        // 从爬取的边界线并逆透视后内提取单行边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);
        left_ctn = left_continue();
        right_ctn = right_continue();
        // 求中线
        bool a = 0;
        int diff_mid = 0;
        for (int i = image_h - lowest; i > hightest; i--) // 从图底向上求
        {
            if (((l_border[i] > 5) && (r_border[i] < 180))) // ||my_abs((l_border[i]-r_border[i]) > 10))//避免回头弯中线求错
            {
                if(is_ready_to_turn_right){
                    if(!a){
                        diff_mid = my_abs(r_border[i] - image_w / 2);
                        a = 1;
                        center_line[i] = image_w / 2;
                    }
                    else{
                        center_line[i] = r_border[i] - diff_mid;
                    }
                }
                else if(is_ready_to_turn_left){
                    if(!a){
                        diff_mid = my_abs(l_border[i] - image_w / 2);
                        a = 1;
                        center_line[i] = image_w / 2;
                    }
                    else{
                        center_line[i] = l_border[i] + diff_mid;
                    }
                }
                else {
                    center_line[i] = (l_border[i] + r_border[i]) >> 1; // 均分求中线
                }
            }
        }
    }
    // 元素的处理全部放在这里
    // 判别是否失去底边
    //    e_calcu_lenth = (image_h - 40)-(60);
    foot_roadwidth = r_border[image_h - 2] - l_border[image_h - 2]; // 此为逆变换后路宽
    if (foot_roadwidth > standard_foot_roadwidth && xflg_now < 6)
        xflg_now++;
    else if (xflg_now > 0)
        xflg_now--;

    if((rbc_y>70) && (lbc_y>70) && (my_abs(rbc_y-lbc_y)<8))
    {
        if (IfxCpu_acquireMutex(&dspeed_mutex))
        {
            d_speed = 0;
            IfxCpu_releaseMutex(&dspeed_mutex);
        }
        system_delay_ms(500);
    }
    else
    update_status();
}

/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/
