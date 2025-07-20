/*********************************************************************************************************************
 * TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC377 开源库的一部分
 *
 * TC377 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu2_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC377TP
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "motor.h"
#include "image.h"
#define image_xmove 10
#pragma section all "cpu2_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
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
    // 此处展示屏幕
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
    // if (xflg_now > 3) // xflg_now>3即三次连续超标，表明底线丢线
    // {
    //     ips200_show_string(100, 250, "losfoot");
    // }
    // else
    // {
    //     // 底边未丢线，分辨前方是否出现丢线
    //     ips200_show_string(100, 250, "getfoot");
    // }

    ips200_draw_line(0 + image_xmove, (uint16)hightest, image_w + image_xmove, (uint16)hightest, RGB565_RED);                             // 图片处理最高处（1）
    ips200_draw_line(0 + image_xmove, (uint16)hightest + 125, image_w + image_xmove, (uint16)hightest + 125, RGB565_RED);                 // 最高处（2）
    ips200_draw_line(0 + image_xmove, (uint16)image_h - lowest, image_w + image_xmove, (uint16)image_h - lowest, RGB565_RED);             // 最低处（1）
    ips200_draw_line(0 + image_xmove, (uint16)image_h - lowest + 125, image_w + image_xmove, (uint16)image_h - lowest + 125, RGB565_RED); // 最低处（2）

    ips200_draw_line(0 + image_xmove, (uint16)TU_CIRCLE_Y_MIN + 125, image_w + image_xmove, (uint16)TU_CIRCLE_Y_MIN + 125, RGB565_BLUE);
    ips200_draw_line(0 + image_xmove, (uint16)TU_CIRCLE_Y_MAX + 125, image_w + image_xmove, (uint16)TU_CIRCLE_Y_MAX + 125, RGB565_BLUE);
    for (int i = hightest; i < image_h - 1; i++)
    {
        if ((l_border[i] > 0) && ((l_border[i] + image_xmove) < (ips200_width_max - 1)))
        {
            ips200_draw_point((uint16)l_border[i] + image_xmove, (uint16)(i), uesr_RED); /*左线*/
            ips200_draw_point((uint16)l_border[i] + 1 + image_xmove, (uint16)(i), uesr_RED);
            ips200_draw_point((uint16)l_border[i] - 1 + image_xmove, (uint16)(i), uesr_RED);
        }
        if ((r_border[i] > 0) && ((r_border[i] + image_xmove) < (ips200_width_max - 1)))
        {
            ips200_draw_point((uint16)r_border[i] + image_xmove, (uint16)(i), uesr_RED); /*右线*/
            ips200_draw_point((uint16)r_border[i] + 1 + image_xmove, (uint16)(i), uesr_RED);
            ips200_draw_point((uint16)r_border[i] - 1 + image_xmove, (uint16)(i), uesr_RED);
        }
        if ((center_line[i] > 0) && ((center_line[i] + image_xmove) < (ips200_width_max - 1)))
        {
            ips200_draw_point((uint16)center_line[i] + image_xmove, (uint16)(i), uesr_BLUE); /*中线*/
            ips200_draw_point((uint16)center_line[i] + 1 + image_xmove, (uint16)(i), uesr_BLUE);
            ips200_draw_point((uint16)center_line[i] - 1 + image_xmove, (uint16)(i), uesr_BLUE);
        }
    }
}
void core2_main(void) // 负责屏幕显示
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    // pit_ms_init(CCU60_CH0, 10);
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready(); // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        if (IfxCpu_acquireMutex(&screen_mutex))
        {
            screen_show();
            IfxCpu_releaseMutex(&screen_mutex);
        }
        system_delay_ms(100);

        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
