/*********************************************************************************************************************
 * TC377 Opensourec Library ����TC377 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC377 ��Դ���һ����
 *
 * TC377 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          cpu2_main
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.10.2
 * ����ƽ̨          TC377TP
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "motor.h"
#include "image.h"
#define image_xmove 10
#pragma section all "cpu2_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
// void show_star(uint8 x, uint8 y)
//{
//    if ((x > 0) && (x < ips200_width_max-1) && (y > 0) && (y < ips200_height_max-1))
//    ips200_draw_point((uint16)x, (uint16)y, uesr_GREEN);
//    if ((x > -1) && ((x+2) < ips200_width_max) && (y > 0) && (y < ips200_height_max-1))
//    ips200_draw_point((uint16)x + 1, (uint16)y, uesr_GREEN);
//    if ((x > 0) && (x < ips200_width_max-1) && (y > -1) && ((y+2) < ips200_height_max))
//    ips200_draw_point((uint16)x, (uint16)y + 1, uesr_GREEN);
//    if ((x > 1) && ((x) < ips200_width_max) && (y > 0) && (y < ips200_height_max-1))
//    ips200_draw_point((uint16)x - 1, (uint16)y, uesr_GREEN);
//    if ((x > 0) && (x < ips200_width_max-1) && (y > 1) && (y < ips200_height_max))
//    ips200_draw_point((uint16)x, (uint16)y - 1, uesr_GREEN);
//}
void screen_show(void)
{
    // �˴�չʾ��Ļ
    ips200_show_gray_image(0 + image_xmove, 0, original_image[0], image_w, image_h, image_w, image_h, 0);
    ips200_show_gray_image(0 + image_xmove, 125, bin_image[0], image_w, image_h, image_w, image_h, 0);

    ips200_show_string(100, 270, "left:");
    ips200_show_int(160,270,left_ctn,3);

    ips200_show_string(0, 270, "right:");
    ips200_show_int(50, 270, right_ctn, 3);

    ips200_show_string(0, 250, "circle");
    ips200_show_int(140, 250, left_circle_find, 3);
    ips200_show_int(160, 250, is_ready_to_turn_left, 3);

    // ips200_show_string(0, 250, "ifcirc:");
    // if (xflg_now > 3) // xflg_now>3�������������꣬�������߶���
    // {
    //     ips200_show_string(100, 250, "losfoot");
    // }
    // else
    // {
    //     // �ױ�δ���ߣ��ֱ�ǰ���Ƿ���ֶ���
    //     ips200_show_string(100, 250, "getfoot");
    // }

    ips200_draw_line(0 + image_xmove, (uint16)hightest, image_w + image_xmove, (uint16)hightest, RGB565_RED);                             // ͼƬ������ߴ���1��
    ips200_draw_line(0 + image_xmove, (uint16)hightest + 125, image_w + image_xmove, (uint16)hightest + 125, RGB565_RED);                 // ��ߴ���2��
    ips200_draw_line(0 + image_xmove, (uint16)image_h - lowest, image_w + image_xmove, (uint16)image_h - lowest, RGB565_RED);             // ��ʹ���1��
    ips200_draw_line(0 + image_xmove, (uint16)image_h - lowest + 125, image_w + image_xmove, (uint16)image_h - lowest + 125, RGB565_RED); // ��ʹ���2��

    ips200_draw_line(0 + image_xmove, (uint16)TU_CIRCLE_Y_MIN + 125, image_w + image_xmove, (uint16)TU_CIRCLE_Y_MIN + 125, RGB565_BLUE);
    ips200_draw_line(0 + image_xmove, (uint16)TU_CIRCLE_Y_MAX + 125, image_w + image_xmove, (uint16)TU_CIRCLE_Y_MAX + 125, RGB565_BLUE);
    for (int i = hightest; i < image_h - 1; i++)
    {
        if ((l_border[i] > 0) && ((l_border[i] + image_xmove) < (ips200_width_max - 1)))
        {
            ips200_draw_point((uint16)l_border[i] + image_xmove, (uint16)(i), uesr_RED); /*����*/
            ips200_draw_point((uint16)l_border[i] + 1 + image_xmove, (uint16)(i), uesr_RED);
            ips200_draw_point((uint16)l_border[i] - 1 + image_xmove, (uint16)(i), uesr_RED);
        }
        if ((r_border[i] > 0) && ((r_border[i] + image_xmove) < (ips200_width_max - 1)))
        {
            ips200_draw_point((uint16)r_border[i] + image_xmove, (uint16)(i), uesr_RED); /*����*/
            ips200_draw_point((uint16)r_border[i] + 1 + image_xmove, (uint16)(i), uesr_RED);
            ips200_draw_point((uint16)r_border[i] - 1 + image_xmove, (uint16)(i), uesr_RED);
        }
        if ((center_line[i] > 0) && ((center_line[i] + image_xmove) < (ips200_width_max - 1)))
        {
            ips200_draw_point((uint16)center_line[i] + image_xmove, (uint16)(i), uesr_BLUE); /*����*/
            ips200_draw_point((uint16)center_line[i] + 1 + image_xmove, (uint16)(i), uesr_BLUE);
            ips200_draw_point((uint16)center_line[i] - 1 + image_xmove, (uint16)(i), uesr_BLUE);
        }
    }
}
void core2_main(void) // ������Ļ��ʾ
{
    disable_Watchdog();         // �رտ��Ź�
    interrupt_global_enable(0); // ��ȫ���ж�
    // �˴���д�û����� ���������ʼ�������
    // pit_ms_init(CCU60_CH0, 10);
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        if (IfxCpu_acquireMutex(&screen_mutex))
        {
            screen_show();
            IfxCpu_releaseMutex(&screen_mutex);
        }
        system_delay_ms(100);

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
