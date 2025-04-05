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
void show_star(uint8 x, uint8 y)
{
    if ((x > 1) && (x < ips200_width_max) && (y > 1) && (y < ips200_height_max))
        ips200_draw_point((uint16)x, (uint16)y, uesr_GREEN); // �ǵ����ʰ뾶С��������Ե���ߴ�+�������
    ips200_draw_point((uint16)x + 1, (uint16)y, uesr_GREEN);
    ips200_draw_point((uint16)x, (uint16)y + 1, uesr_GREEN);
    ips200_draw_point((uint16)x - 1, (uint16)y, uesr_GREEN);
    ips200_draw_point((uint16)x, (uint16)y - 1, uesr_GREEN);
}
void screen_show(void)
{
    // �˴�չʾ��Ļ
    ips200_show_gray_image(0 + image_xmove, 0, original_image[0], image_w, image_h, image_w, image_h, 0);
    ips200_show_gray_image(0 + image_xmove, 125, bin_image[0], image_w, image_h, image_w, image_h, 0);
    //
    //    ips200_show_string(0, 180, "ifstop:");
    //    ips200_show_int(60,180,stop,3);
    //
    //    ips200_show_string(0, 200, "lspeed:");
    //    ips200_show_int(60,200,speed_l,3);
    //    ips200_show_string(100, 200, "rspeed:");
    //    ips200_show_int(160,200,speed_r,3);
    //
    //    ips200_show_string(0, 220, "error:");
    //    ips200_show_int(60,220,error,3);
    //    ips200_show_string(100, 220, "dspeed:");
    //    ips200_show_int(160,220,d_speed,3);
    //
    //    ips200_show_int(0,260,head_roadwidth,3);
    //    ips200_show_int(40,260,foot_roadwidth,3);
    //    if(xflg_now > 3)//xflg_now>3�������������꣬�������߶���
    //        {
    //            ips200_show_string(80, 260, "losfoot");
    //        }
    //        else
    //        {
    //        //�ױ�δ���ߣ��ֱ�ǰ���Ƿ���ֶ���
    //            ips200_show_string(80, 260, "getfoot");
    //        }
    ips200_draw_line(0 + image_xmove, (uint16)hightest, image_w + image_xmove, (uint16)hightest, RGB565_RED);
    ips200_draw_line(0 + image_xmove, (uint16)hightest + 125, image_w + image_xmove, (uint16)hightest + 125, RGB565_RED);
    ips200_draw_line(0 + image_xmove, (uint16)image_h - lowest, image_w + image_xmove, (uint16)image_h - lowest, RGB565_RED);
    ips200_draw_line(0 + image_xmove, (uint16)image_h - lowest + 125, image_w + image_xmove, (uint16)image_h - lowest + 125, RGB565_RED);
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
        system_delay_ms(50);

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
