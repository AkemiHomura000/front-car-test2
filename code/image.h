/*
 * image.h
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#pragma section all "cpu0_dsram"
//颜色定义
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色
//图像定义
#define image_h 120//图像高度
#define image_w 188//图像宽度
#define lowest 4//最终处理下边忽略行数之最少值
//--------datatype declaration---------
typedef struct corner_inline
{
      uint8 f_cp_x;
      uint8 f_cp_y;
      uint8 h_cp_x;
      uint8 h_cp_y;
}corner_inline;
//---------function declaration----------
extern void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了
extern int16 limit_a_b(int16 x, int16 a, int16 b);
//---------data declaration------------
extern IfxCpu_mutexLock screen_mutex;
extern uint8 original_image[image_h][image_w];//原图数组
extern uint8 bin_image[image_h][image_w];//二值化图像数组
extern uint8 l_border[image_h];//左线数组(内容, 序号)
extern uint8 r_border[image_h];//右线数组
extern uint8 center_line[image_h];//中线数组
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r;//统计右边找到点的个数
extern int d_speed;
extern float error;          // 误差值
extern float err_kp;
extern float err_kd;
extern int foot_roadwidth;
extern int head_roadwidth;
extern uint8 hightest;//八邻域寻线得最高点，后与hightest_least比较取大值
extern uint8 cent_line_high;//中线寻得最高点
extern int xflg_now;
extern corner_inline* l_corner_point;
extern corner_inline* r_corner_point;

#endif /* CODE_IMAGE_H_ */
