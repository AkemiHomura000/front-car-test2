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
#define standard_foot_roadwidth 170 // 可能受光线等二值化影响
#define bin_jump_num 1              // 跳过的点数
#define border_max image_w - 2      // 边界最大值
#define border_min 1                // 边界最小值
#define USE_num image_h * 3         // 定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点
#define lost_width 15               // 处理区两侧单边延展列数，基于底线宽（底线丢时可能处理区不完全包含于视野）
#define hightest_least 15           // 最终处理上边忽略行数之最少值（寻线可能比其更大）

#define GrayScale 256 // 大津法中

/*变量声明*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold;            // 图像分割阈值
uint8 bin_image[image_h][image_w]; // 图像数组
corner_inline *l_corner_point = NULL;
corner_inline *r_corner_point = NULL;

// 存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = {{0}}; // 左线八邻域
uint16 points_r[(uint16)USE_num][2] = {{0}}; // 右线八邻域
uint16 dir_r[(uint16)USE_num] = {0};         // 用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = {0};         // 用来存储左边生长方向
uint16 data_stastics_l = 0;                  // 统计左边找到点的个数
uint16 data_stastics_r = 0;                  // 统计右边找到点的个数
uint8 hightest = 0;                          // 八邻域寻线得最高点，后与hightest_least比较取大值
uint16 trans_l[(uint16)USE_num][2] = {{0}};  // 左线变换
uint16 trans_r[(uint16)USE_num][2] = {{0}};  // 右线

uint8 l_border[image_h];    // 左线数组(内容, 序号)
uint8 r_border[image_h];    // 右线数组
uint8 center_line[image_h]; // 中线数组
uint8 cent_line_high;

IfxCpu_mutexLock screen_mutex;

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
void turn_to_bin(void)
{
    uint8 i, j;
    image_thereshold = otsuThreshold(original_image[0], image_w, image_h); ///////////////根据反光情况修改
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
float error_last = 0;
float err_kp = 4;
float err_kd = 1.2;
int d_speed = 0; // 定义左右两轮的差速

void error_calculate(void)
{
    float kp = 0; // 采用动态kp
    error = 0;
    d_speed = 0;
    for (int i = image_h / 2; i < image_h / 2 + 5; i++)
        error += (image_w / 2 - center_line[i]); // 预瞄点计算原始误差

    error /= 5;

    kp = Maxmin(0.00025 * error * error + err_kp, 0, 2.5);

    d_speed = Maxmin((kp * error + err_kd * (error - error_last)), -150, 150);
    error_last = error;
}

////-------逆变换边线------------------------------------------------------------------------------
void EdgeLinePerspective(uint16 *in_line, uint8 num, uint16 *out_line)
{
    double change_inverse_Mat[3][3] = {{-1.11048, -4.06186, 186.592}, {-0.149566, -3.88910, 136.969}, {-0.00236018, -0.0406898, 1}};
    for (uint8 count = 0; count < num; count++)
    {
        float i = in_line[count * 2 + 0];
        float j = in_line[count * 2 + 1];
        uint16 solve_x = (int)((change_inverse_Mat[0][0] * i + change_inverse_Mat[0][1] * j + change_inverse_Mat[0][2]) / (change_inverse_Mat[2][0] * i + change_inverse_Mat[2][1] * j + change_inverse_Mat[2][2]));
        uint16 solve_y = (int)((change_inverse_Mat[1][0] * i + change_inverse_Mat[1][1] * j + change_inverse_Mat[1][2]) / (change_inverse_Mat[2][0] * i + change_inverse_Mat[2][1] * j + change_inverse_Mat[2][2]));
        if ((solve_x < image_w) && (solve_x > 0))
            out_line[count * 2 + 0] = solve_x;
        if ((solve_y < image_h) && (solve_y > 0))
            out_line[count * 2 + 1] = solve_y;
    }
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
int head_roadwidth = 0;
void image_process(void)
{
    Get_image(mt9v03x_image);
    turn_to_bin();
    /*提取赛道边界*/
    image_filter(bin_image);      // 滤波
    image_draw_rectan(bin_image); // 预处理
    // 清零
    data_stastics_l = 0;
    data_stastics_r = 0;
    if (get_start_point(image_h - 2)) // 找到起点了，再执行八领域，没找到就一直找
    {
        //    printf("正在开始八领域\n");
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        if (hightest < hightest_least)
            hightest = hightest_least; // 最高边判定
        //    printf("八邻域已结束\n");
        EdgeLinePerspective(&points_l, data_stastics_l, &trans_l);
        EdgeLinePerspective(&points_r, data_stastics_r, &trans_r);
        // 寻找角点补全边线
        //        l_corner_point = corner_4points(&points_l, data_stastics_l, 3);
        //        r_corner_point = corner_4points(&points_r, data_stastics_r, 3);
        // 从爬取的边界线内提取边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);
        // 求中线
        uint8 center_ipt = 0;
        float itplt[3] = {0}, d1ipt[2] = {0}, d2ipt = 0; //[3]={0}, d3ipt[2]={0}, d4ipt=0;
        float stp = 0, ipt_x0 = 0;
        for (int i = image_h - lowest; i > hightest; i--)
        {
            //            if((l_border[i]>l_border[image_h-2]-lost_width) && (r_border[i]<r_border[image_h-2]+lost_width))//无丢线
            if ((l_border[i] > 5) && (r_border[i] < 180))
            {
                center_line[i] = (l_border[i] + r_border[i]) >> 1; // 均分求中线
            }
        }
    }
    // 元素对左右边线的处理全部放在这里
    // 判别是否有岔口（十字或环）
    foot_roadwidth = r_border[image_h - 2] - l_border[image_h - 2];
    head_roadwidth = r_border[hightest] - l_border[hightest];
    if (foot_roadwidth > standard_foot_roadwidth && xflg_now < 6)
        xflg_now++;
    else if (xflg_now > 0)
        xflg_now--;
    if (xflg_now > 3) // xflg_now>3即三次连续超标，表明底线丢线
    {
        // 如只能捕一个前角则为环，可捕左右两角为十字，无角则为大弯过度
    }
    else
    {
        // 底边未丢线，分辨前方是否出现丢线

        // 前方无丢线，可能为直道或小弯
    }

    error_calculate();
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
