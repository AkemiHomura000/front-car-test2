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
* 文件名称          cpu1_main
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
#include "all_init.h"
#include "motor.h"
#include "image.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
void core1_main(void)//负责电机控制和上位机交流
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    all_init();
    gpio_init(P21_5, GPO, GPIO_HIGH, GPO_PUSH_PULL);         // 初始化 默认高电平 推挽输出模式 5对应左轮方向
    gpio_init(P21_3, GPO, GPIO_LOW, GPO_PUSH_PULL);         // 初始化 默认高电平 推挽输出模式 3对应右轮方向
    pit_ms_init(CCU60_CH0, 10);
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    printf("init_uart\r\n");
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] = speed_l;
    oscilloscope_data.data[1] = speed_r;
    oscilloscope_data.data[2] = target_speed - d_speed;
    oscilloscope_data.data[3] = target_speed + d_speed;
    oscilloscope_data.data[4] = error;
    oscilloscope_data.data[5] = d_speed;
    oscilloscope_data.channel_num = 6;
    while (TRUE)
    {
        // 滴答客解析接收到的数据
        seekfree_assistant_data_analysis();
        // 遍历
        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
        {
            // 更新标志位
            if(seekfree_assistant_parameter_update_flag[i])
            {
                seekfree_assistant_parameter_update_flag[i] = 0;
                //--------------更新内部参数
                target_speed = seekfree_assistant_parameter[0];
                err_kp       = seekfree_assistant_parameter[1];
                err_kd       = seekfree_assistant_parameter[2];
                sptr_r.D     = seekfree_assistant_parameter[3];
                stop         = seekfree_assistant_parameter[4];
                //--------------通过DEBBUG串口发送信息
//                printf("receive data channel : %d ", i);
//                printf("data : %f ", seekfree_assistant_parameter[i]);
//                printf("\r\n");
            }
        }
        oscilloscope_data.data[0] = speed_l;
        oscilloscope_data.data[1] = speed_r;
        oscilloscope_data.data[2] = target_speed - d_speed;
        oscilloscope_data.data[3] = target_speed + d_speed;
        oscilloscope_data.data[4] = error;
        oscilloscope_data.data[5] = d_speed;
        seekfree_assistant_oscilloscope_send(&oscilloscope_data);
        system_delay_ms(5);
        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
