/*
 * image.c
 *
 *  Created on: 2025��3��1��
 *      Author: Night
 */
//-------------------------------------------------------------------------------------------------------------------
//  ���:������ͼ����

//------------------------------------------------------------------------------------------------------------------
#include "zf_common_headfile.h"
#include "image.h"
#include "motor.h"

// �궨��
#define white_pixel 255
#define black_pixel 0
#define standard_foot_roadwidth 170 // �����ܹ��ߵȶ�ֵ��Ӱ��
#define bin_jump_num 1              // �����ĵ���
#define border_max image_w - 2      // �߽����ֵ
#define border_min 1                // �߽���Сֵ
#define USE_num image_h * 3         // �����ҵ�������Ա��������˵300�����ܷ��£�������Щ�������ȷʵ�Ѷ����ඨ����һ��
#define lost_width 15               // ���������൥����չ���������ڵ��߿����߶�ʱ���ܴ���������ȫ��������Ұ��
#define hightest_least 15           // ���մ����ϱߺ�������֮����ֵ��Ѱ�߿��ܱ������

#define GrayScale 256 // �����

/*��������*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold;                   // ͼ��ָ���ֵ
uint8 bin_image[image_h][image_w];        // ͼ������
uint8 bin_image_circlr[image_h][image_w]; // ͼ������
corner_inline *l_corner_point = NULL;
corner_inline *r_corner_point = NULL;

// ��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = {{0}}; // ���߰�����
uint16 points_r[(uint16)USE_num][2] = {{0}}; // ���߰�����
uint16 dir_r[(uint16)USE_num] = {0};         // �����洢�ұ���������
uint16 dir_l[(uint16)USE_num] = {0};         // �����洢�����������
uint16 data_stastics_l = 0;                  // ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;                  // ͳ���ұ��ҵ���ĸ���
uint8 hightest = 0;                          // ������Ѱ�ߵ���ߵ㣬����hightest_least�Ƚ�ȡ��ֵ
uint16 trans_l[(uint16)USE_num][2] = {{0}};  // ���߱任
uint16 trans_r[(uint16)USE_num][2] = {{0}};  // ����

uint8 l_border[image_h];    // ��������(����, ���)
uint8 r_border[image_h];    // ��������
uint8 center_line[image_h]; // ��������
uint8 cent_line_high;

IfxCpu_mutexLock screen_mutex;

/*
�������ƣ�int my_abs(int value)
����˵���������ֵ
����˵����
�������أ�����ֵ
�޸�ʱ�䣺2022��9��8��
��    ע��
example��  my_abs( x)��
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
�������ƣ�int16 limit(int16 x, int16 y)
����˵������x,y�е���Сֵ
����˵����
�������أ�������ֵ�е���Сֵ
�޸�ʱ�䣺2022��9��8��
��    ע��
example��  limit( x,  y)
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
//  @brief      ���һ���Ҷ�ͼ��
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8 (*mt9v03x_image)[image_w])
{
#define use_num 1 // 1���ǲ�ѹ����2����ѹ��һ��
    uint8 row = 0, line = 0;
    for (uint8 i = 0; i < image_h; i += use_num) //
    {
        for (uint8 j = 0; j < image_w; j += use_num) //
        {
            original_image[row][line] = mt9v03x_image[i][j]; // ����Ĳ�����д�������ͷ�ɼ�����ͼ��
            line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     ��̬��ֵ
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
    double OmegaBack = 0, OmegaFore = 0, MicroBack = 0, MicroFore = 0, SigmaB = 0, Sigma = 0; // ��䷽��;
    uint8 MinValue = 0, MaxValue = 0;
    uint8 Threshold = 0;

    for (Y = 0; Y < Image_Height; Y++) // Y<Image_Height��ΪY =Image_Height���Ա���� �ж�ֵ��
    {
        // Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y * Image_Width + X]]++; // ͳ��ÿ���Ҷ�ֵ�ĸ�����Ϣ
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ; // ��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--)
        ; // ��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
    {
        return MaxValue; // ͼ����ֻ��һ����ɫ
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue; // ͼ����ֻ�ж�����ɫ
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y]; //  ��������
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y; // �Ҷ�ֵ����
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];                                               // ǰ�����ص���
        PixelFore = Amount - PixelBack;                                                    // �������ص���
        OmegaBack = (double)PixelBack / Amount;                                            // ǰ�����ذٷֱ�
        OmegaFore = (double)PixelFore / Amount;                                            // �������ذٷֱ�
        PixelIntegralBack += HistGram[Y] * Y;                                              // ǰ���Ҷ�ֵ
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;                             // �����Ҷ�ֵ
        MicroBack = (double)PixelIntegralBack / PixelBack;                                 // ǰ���ҶȰٷֱ�
        MicroFore = (double)PixelIntegralFore / PixelFore;                                 // �����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // g
        if (Sigma > SigmaB)                                                                // ����������䷽��g
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      ͼ���ֵ���������õ��Ǵ�򷨶�ֵ����
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
void turn_to_bin(void)
{
    uint8 i, j;
    image_thereshold = otsuThreshold(original_image[0], image_w, image_h); ///////////////���ݷ�������޸�
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
�������ƣ�void get_start_point(uint8 start_row)
����˵����Ѱ�������߽�ı߽����Ϊ������ѭ������ʼ��
����˵����������������
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example��  get_start_point(image_h-2)
-------------------------------------------------------------------------*/
static uint8 start_point_l[2] = {0}; // �������x��yֵ
static uint8 start_point_r[2] = {0}; // �ұ�����x��yֵ
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0, l_found = 0, r_found = 0;
    // ����
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y

    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y

    // ���м�����ߣ��������
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;         // x
        start_point_l[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            // printf("�ҵ�������image[%d][%d]\n", start_row,i);
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
            // printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if (l_found && r_found)
        return 1;
    else
    { // δ�ҵ����
        return 0;
    }
}

/*-------------------------------------------------------------------------------------------
�������ƣ�void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

����˵������������ʽ��ʼ���ұߵ�ĺ�������������е�࣬���õ�ʱ��Ҫ©�ˣ������������һ�������ꡣ
����˵����
break_flag_r            �������Ҫѭ���Ĵ���
(*image)[image_w]       ����Ҫ�����ҵ��ͼ�����飬�����Ƕ�ֵͼ,�����������Ƽ���
                       �ر�ע�⣬��Ҫ�ú궨��������Ϊ����������������ݿ����޷����ݹ���
*l_stastic              ��ͳ��������ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
*r_stastic              ��ͳ���ұ����ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
l_start_x               �������������
l_start_y               ��������������
r_start_x               ���ұ���������
r_start_y               ���ұ����������
hightest                ��ѭ���������õ�����߸߶�
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example��
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
--------------------------------------------------------------------------------------------*/
void search_l_r(uint16 break_flag, uint8 (*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{

    uint8 i = 0, j = 0;
    // ��߱���
    uint8 search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    uint8 temp_l[8][2] = {{0}};
    uint8 center_point_l[2] = {0};
    uint16 l_data_statics; // ͳ�����
    // ����˸�����
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
    // �����˳ʱ��

    // �ұ߱���
    uint8 search_filds_r[8][2] = {{0}};
    uint8 center_point_r[2] = {0}; // ���������
    uint8 index_r = 0;             // �����±�
    uint8 temp_r[8][2] = {{0}};
    uint16 r_data_statics; // ͳ���ұ�
    // ����˸�����
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
    // �������ʱ��

    l_data_statics = *l_stastic; // ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    r_data_statics = *r_stastic; // ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������

    // ��һ�θ��������  ���ҵ������ֵ������
    center_point_l[0] = l_start_x; // x
    center_point_l[1] = l_start_y; // y
    center_point_r[0] = r_start_x; // x
    center_point_r[1] = r_start_y; // y

    // ��������ѭ��
    while (break_flag--)
    {
        // ���
        for (i = 0; i < 8; i++) // ����8F����
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0]; // x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1]; // y
        }
        // �����������䵽�Ѿ��ҵ��ĵ���
        points_l[l_data_statics][0] = center_point_l[0]; // x
        points_l[l_data_statics][1] = center_point_l[1]; // y
        l_data_statics++;                                // ������һ

        // �ұ�
        for (i = 0; i < 8; i++) // ����8F����
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0]; // x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1]; // y
        }
        // �����������䵽�Ѿ��ҵ��ĵ���
        points_r[r_data_statics][0] = center_point_r[0]; // x
        points_r[r_data_statics][1] = center_point_r[1]; // y

        index_l = 0; // �����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0; // �����㣬��ʹ��
            temp_l[i][1] = 0; // �����㣬��ʹ��
        }

        // ����ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0 && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i); // ��¼��������
            }

            if (index_l)
            {
                // ���������
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
                // printf("���ν���ͬһ���㣬�˳�\n");
                break;
            }
        }

        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2 && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2))
        {
            // printf("\n���������˳�\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1; // ȡ����ߵ�
            // printf("\n��y=%d���˳�\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            // printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
            continue; // �����߱��ұ߸��ˣ���ߵȴ��ұ�
        }
        if (dir_l[l_data_statics - 1] == 7 && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1])) // ��߱��ұ߸����Ѿ�����������
        {
            // printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0]; // x
            center_point_l[1] = points_l[l_data_statics - 1][1]; // y
            l_data_statics--;
        }
        r_data_statics++; // ������һ

        index_r = 0; // �����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0; // �����㣬��ʹ��
            temp_r[i][1] = 0; // �����㣬��ʹ��
        }

        // �ұ��ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0 && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;                       // ������һ
                dir_r[r_data_statics - 1] = (i); // ��¼��������
                // printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                // ���������
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
    // ȡ��ѭ������
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}
/*--------------------------------------------------
�������ƣ�void get_left(uint16 total_L)
����˵�����Ӱ�����߽�����ȡ��Ҫ�ı���
����˵����
total_L ���ҵ��ĵ������
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example�� get_left(data_stastics_l );
-------------------------------------------------------*/
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    // ��ʼ��
    for (i = 0; i < image_h; i++)
    {
        l_border[i] = border_min;
    }
    h = image_h - 2;
    // ���
    for (j = 0; j < total_L; j++)
    {
        if ((points_l[j][1] == h)) // && ((h < l_corner_point->f_cp_y)||(h > l_corner_point->h_cp_y)))
        {
            l_border[h] = trans_l[j][0] + 1; // ��һ����
        }
        else
            continue; // ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)
        {
            break; // �����һ���˳�
        }
    }
}
/*------------------------------------------------------------------
�������ƣ�void get_right(uint16 total_R)
����˵�����Ӱ�����߽�����ȡ��Ҫ�ı���
����˵����
total_R  ���ҵ��ĵ������
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example��get_right(data_stastics_r);
---------------------------------------------------------------------*/
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < image_h; i++)
    {
        r_border[i] = border_max; // �ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
    }
    h = image_h - 2;
    // �ұ�
    for (j = 0; j < total_R; j++)
    {
        if ((points_r[j][1] == h)) // && ((h < r_corner_point->f_cp_y)||(h > r_corner_point->h_cp_y)))
        {
            r_border[h] = trans_r[j][0] - 1; // ��һ����
        }
        else
            continue; // ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)
            break; // �����һ���˳�
    }
}
// �������ͺ͸�ʴ����ֵ����
#define threshold_max 255 * 5                  // �˲����ɸ����Լ����������
#define threshold_min 255 * 2                  // �˲����ɸ����Լ����������
void image_filter(uint8 (*bin_image)[image_w]) // ��̬ѧ�˲�������˵�������ͺ͸�ʴ��˼��
{
    uint32 num = 0;
    for (uint16 i = 1; i < image_h - 1; i++)
    {
        for (uint16 j = 1; j < (image_w - 1); j++)
        {
            // ͳ�ư˸����������ֵ
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1] + bin_image[i][j - 1] + bin_image[i][j + 1] + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = white_pixel; // ��
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = black_pixel; // ��
            }
        }
    }
}

/*-------------------------------------------------------
�������ƣ�void image_draw_rectan(uint8(*image)[image_w])
����˵������ͼ��һ���ڿ�, ���������
����˵����uint8(*image)[image_w] ͼ���׵�ַ
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example�� image_draw_rectan(bin_image);
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
�������ƣ�error_calculate(void)
����˵��������ԭʼ�������������ֲ���
����˵������
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example�� image_process();
--------------------------------------------*/
float error = 0;
float error_last = 0;
float err_kp = 4;
float err_kd = 1.2;
int d_speed = 0; // �����������ֵĲ���

void error_calculate(void)
{
    float kp = 0; // ���ö�̬kp
    error = 0;
    d_speed = 0;
    for (int i = image_h / 2; i < image_h / 2 + 5; i++)
        error += (image_w / 2 - center_line[i]); // Ԥ������ԭʼ���

    error /= 5;

    kp = Maxmin(0.00025 * error * error + err_kp, 0, 2.5);

    d_speed = Maxmin((kp * error + err_kd * (error - error_last)), -150, 150);
    error_last = error;
}

////-------��任����------------------------------------------------------------------------------
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
/**
 * @brief ��������Ƿ���ͼ��Χ��
 * @param x
 * @param y
 * @return true ��ͼ��Χ��
 * @return false ����ͼ��Χ��
 */
bool is_valid(int x, int y)
{
    return (x >= 0 && x < image_w && y >= TU_CIRCLE_Y_MIN && y < TU_CIRCLE_Y_MAX);
}
int tu_min(int a, int b)
{
    return (a < b) ? a : b;
}
/**
 * @brief ������ͨ�������ҵ�����ΪTU_VISITED�����ҽ���ֻ�������Խ��ߵ��������е�ʱ������Ϊ����Ч���������ͼ��߽��ཻ������Ϊ����Ч����
 * @param image
 * @param start_x
 * @param start_y
 * @param sum_x x����ĺ�
 * @param sum_y y����ĺ�
 * @param count ��������
 * @param x_min x������Сֵ
 * @param x_max x�������ֵ
 * @param y_min y������Сֵ
 * @param y_max y�������ֵ
 */
void bfs(uint8 *image, int start_x, int start_y, int *sum_x, int *sum_y, int *count, int *x_min, int *x_max, int *y_min, int *y_max)
{
    int queue_x[TU_QUEUE_SIZE], queue_y[TU_QUEUE_SIZE];
    int front = 0, rear = 0; // ���е�ͷβ����

    queue_x[rear] = start_x;
    queue_y[rear] = start_y;
    rear++;

    image[start_y * image_w + start_x] = TU_VISITED; // ��Ƿ���
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

    // �����ĵ㿪ʼ�Ƿ񱻽ض�
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
    // ͳ�ƴ����ĵ㵽�ĸ��ǵ���߶����ж���TU_VISITED�ĵ�
    int l_t_count = 0, r_t_count = 0, l_b_count = 0, r_b_count = 0;
    for (int i = 0; i < num; i++)
    {
        int x1 = center_x + (l_t_x - center_x) * i / num;
        int y1 = center_y + (l_t_y - center_y) * i / num;
        if (is_valid(x1, y1) && l_t_valid)
        {
            if (image[y1 * image_w + x1] == TU_VISITED || image[y1 * image_w + x1] == TU_BOX_COLOR)
            {
                image[y1 * image_w + x1] = TU_BOX_COLOR; // ����
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
                image[y1 * image_w + x1] = TU_BOX_COLOR; // ����
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
                image[y1 * image_w + x1] = TU_BOX_COLOR; // ����
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
                image[y1 * image_w + x1] = TU_BOX_COLOR; // ����
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

    //    printf("count=%d\n", *count);
    //    printf("l_ratio=%f\n", l_ratio);
    //    printf("r_ratio=%f\n", r_ratio);
    // std::cout << "count=" << *count << std::endl;
    // std::cout << "l_ratio=" << l_ratio << std::endl;
    // std::cout << "r_ratio=" << r_ratio << std::endl;

    // ����Ƿ�����Ч����
    if (l_ratio > TU_MIN_DIFF_RATIO && r_ratio > TU_MIN_DIFF_RATIO && min_diff != 0 && abs(l_diff - r_diff) / min_diff < TU_MAX_L_R_DIFF_RATIO)
    {
        // �������������Ϊ��Ч����
    }
    else
    {
        *count = 0; // ֱ�ӱ�Ǵ�����Ϊ��Ч
    }
    // ����Ƿ��ͼ��߽��ཻ
    if (min_x == 0 || max_x == image_w - 1 || min_y == 0 || max_y == image_h - 1)
    {
        *count = 0; // ֱ�ӱ�Ǵ�����Ϊ��Ч
    }
}
/**
 * @brief �������껭��
 * @param image
 * @param x_min
 * @param x_max
 * @param y_min
 * @param y_max
 */
void draw_box(uint8 *image, int x_min, int x_max, int y_min, int y_max)
{
    int left = (x_min < 0) ? 0 : x_min;
    int right = (x_max >= image_w) ? image_w - 1 : x_max;
    int top = (y_min < 0) ? 0 : y_min;
    int bottom = (y_max >= image_h) ? image_h - 1 : y_max;
    for (int i = left; i <= right; i++)
    {
        image[top * image_w + i] = TU_BOX_COLOR;    // �ϱ�
        image[bottom * image_w + i] = TU_BOX_COLOR; // �±�
    }
    for (int j = top; j <= bottom; j++)
    {
        image[j * image_w + left] = TU_BOX_COLOR;  // ���
        image[j * image_w + right] = TU_BOX_COLOR; // �ұ�
    }
}
void draw_circle_range_line(uint8 *image)
{
    for (uint16 i = 0; i < image_w; i++)
    {
        image[TU_CIRCLE_Y_MIN * image_w + i] = TU_BOX_COLOR; // �ϱ�
        image[TU_CIRCLE_Y_MAX * image_w + i] = TU_BOX_COLOR; // �±�
    }
}
/**
 * @brief Ѱ�����е�����
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
                if (count > TU_MIN_CIRCLE_COUNT && count < TU_MAX_CIRCLE_COUNT)
                {
                    printf("count=%d\n", count);
                    int center_x = sum_x / count;
                    int center_y = sum_y / count;
                    draw_box(image, x_min, x_max, y_min, y_max); // ����
                    find_circle = true;
                }
            }
        }
    }
    if (find_circle)
    {
        // printf("find_circle \n");
        return true;
    }
    else
    {
        // printf("not find_circle \n");
        return false;
    }
    // printf("-\n");
    // draw_circle_range_line(image);
}
bool circle_flag = false; // �Ƿ��ҵ�����
bool find_circle_area(void)
{
    if (img_update)
    {
        // ��mt9v03x_image��ֵ���󲢱��浽bin_image_circlr���˴��ֶ�������ֵ
        for (int i = 0; i < image_h; i++)
        {
            for (int j = 0; j < image_w; j++)
            {
                bin_image_circlr[i][j] = mt9v03x_image[i][j] > 100 ? 255 : 0;
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
/*------------------------------------------------------------------------------------------
�������ƣ�void image_process(void)
����˵�������մ�����
����˵������
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example�� image_process();
----------------------------------------------------------------------------------------------*/
int xflg_now = 0;
int foot_roadwidth = 0;
int head_roadwidth = 0;
void image_process(void)
{
    img_update = true;
    circle_flag = false;
    Get_image(mt9v03x_image);
    turn_to_bin();
    /*��ȡ�����߽�*/
    image_filter(bin_image);      // �˲�
    image_draw_rectan(bin_image); // Ԥ����
                                  // ����
    data_stastics_l = 0;
    data_stastics_r = 0;
    if (get_start_point(image_h - 2)) // �ҵ�����ˣ���ִ�а�����û�ҵ���һֱ��
    {
        //    printf("���ڿ�ʼ������\n");
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        if (hightest < hightest_least)
            hightest = hightest_least; // ��߱��ж�
        //    printf("�������ѽ���\n");
        EdgeLinePerspective(&points_l, data_stastics_l, &trans_l);
        EdgeLinePerspective(&points_r, data_stastics_r, &trans_r);
        // Ѱ�ҽǵ㲹ȫ����
        //        l_corner_point = corner_4points(&points_l, data_stastics_l, 3);
        //        r_corner_point = corner_4points(&points_r, data_stastics_r, 3);
        // ����ȡ�ı߽�������ȡ����
        get_left(data_stastics_l);
        get_right(data_stastics_r);
        // ������
        uint8 center_ipt = 0;
        float itplt[3] = {0}, d1ipt[2] = {0}, d2ipt = 0; //[3]={0}, d3ipt[2]={0}, d4ipt=0;
        float stp = 0, ipt_x0 = 0;
        for (int i = image_h - lowest; i > hightest; i--)
        {
            //            if((l_border[i]>l_border[image_h-2]-lost_width) && (r_border[i]<r_border[image_h-2]+lost_width))//�޶���
            if ((l_border[i] > 5) && (r_border[i] < 180))
            {
                center_line[i] = (l_border[i] + r_border[i]) >> 1; // ����������
            }
        }
    }

    // Ԫ�ض����ұ��ߵĴ���ȫ����������
    // �б��Ƿ��в�ڣ�ʮ�ֻ򻷣�
    foot_roadwidth = r_border[image_h - 2] - l_border[image_h - 2];
    head_roadwidth = r_border[hightest] - l_border[hightest];
    if (foot_roadwidth > standard_foot_roadwidth && xflg_now < 6)
        xflg_now++;
    else if (xflg_now > 0)
        xflg_now--;
    if (xflg_now > 3) // xflg_now>3�������������꣬�������߶���
    {
        // ��ֻ�ܲ�һ��ǰ����Ϊ�����ɲ���������Ϊʮ�֣��޽���Ϊ�������
    }
    else
    {
        // �ױ�δ���ߣ��ֱ�ǰ���Ƿ���ֶ���

        // ǰ���޶��ߣ�����Ϊֱ����С��
    }

    error_calculate();
}

/*

��������㣨0.0��***************����>*************xֵ���
************************************************************
************************************************************
************************************************************
************************************************************
******************��������һ��ͼ��*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
yֵ���*******************************************(188.120)

*/
