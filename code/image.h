/*
 * image.h
 *
 *  Created on: 2025��3��1��
 *      Author: Night
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#pragma section all "cpu0_dsram"
#define TU_M_PI 3.1415926
#define TU_VISITED 127
#define TU_QUEUE_SIZE (image_w * image_h)
#define TU_BOX_COLOR 0
#define TU_MIN_CIRCLE_COUNT 600     // ��С��������
#define TU_MAX_CIRCLE_COUNT 2000    // �����������
#define TU_CIRCLE_Y_MIN 30          // ��⻷���ķ�Χ
#define TU_CIRCLE_Y_MAX 90          // ��⻷���ķ�Χ
#define TU_MIN_DIFF_RATIO 0.012f    // ��С��ֵ����
#define TU_MAX_L_R_DIFF_RATIO 0.35f // ���Ҳ������
// ��ɫ����
#define uesr_RED 0XF800   // ��ɫ
#define uesr_GREEN 0X07E0 // ��ɫ
#define uesr_BLUE 0X001F  // ��ɫ
// ͼ����
#define image_h 120 // ͼ��߶�
#define image_w 188 // ͼ����
#define lowest 4    // ���մ����±ߺ�������֮����ֵ
//--------datatype declaration---------
typedef struct corner_inline
{
      uint8 f_cp_x;
      uint8 f_cp_y;
      uint8 h_cp_x;
      uint8 h_cp_y;
} corner_inline;
//---------function declaration----------
extern void image_process(void); // ֱ�����жϻ�ѭ������ô˳���Ϳ���ѭ��ִ����
extern bool find_circle_area(void);
extern int16 limit_a_b(int16 x, int16 a, int16 b);
//---------data declaration------------
extern IfxCpu_mutexLock screen_mutex;
extern IfxCpu_mutexLock param_mutex;
extern IfxCpu_mutexLock dspeed_mutex;
extern uint8 threshold_add;
extern uint8 original_image[image_h][image_w];   // ԭͼ����
extern uint8 bin_image[image_h][image_w];        // ��ֵ��ͼ������
extern uint8 bin_image_circlr[image_h][image_w]; // ͼ������
extern uint8 l_border[image_h];                  // ��������(����, ���)
extern uint8 r_border[image_h];                  // ��������
extern uint8 center_line[image_h];               // ��������
extern uint16 data_stastics_l;                   // ͳ������ҵ���ĸ���
extern uint16 data_stastics_r;                   // ͳ���ұ��ҵ���ĸ���
extern int d_speed;
extern float error; // ���ֵ
extern float err_kp;
extern float err_kd;
extern int foot_roadwidth;
extern uint8 hightest;       // ������Ѱ�ߵ���ߵ㣬����hightest_least�Ƚ�ȡ��ֵ
extern uint8 cent_line_high; // ����Ѱ����ߵ�
extern int xflg_now;
extern corner_inline *l_corner_point;
extern corner_inline *r_corner_point;

#endif /* CODE_IMAGE_H_ */
