/*
 * all_init.c
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */
#include "zf_common_headfile.h"
#include "all_init.h"
#include "image.h"

#define LED1 (P33_9) // 驱动红外灯
#define SWITCH2 (P33_12)
void PWM_Init()
{
    // gpio_init(P21_2, GPO, 0, GPI_FLOATING_IN);//电机转向初始化
    pwm_init(ATOM0_CH2_P21_4, 20000, 1000); // 左边
    //    pwm_init (ATOM0_CH3_P21_5, 17*1000, 0);//正转pwm
    // gpio_init(P21_4, GPO, 0, GPI_FLOATING_IN);
    pwm_init(ATOM0_CH0_P21_2, 20000, 1000);
    //    pwm_init (ATOM0_CH1_P21_3, 17*1000, 0);//正转pwm
}
void encoder_Init()
{
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6); // 左边
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9); // 右边
}
void imu_init()
{
    while (1)
    {
        if (imu660ra_init())
            printf("\r\n IMU660RA init error."); // IMU660RA 初始化失败
        else
            break;
        gpio_toggle_level(LED1); // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
}
void all_init()
{
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 SWITCH2 输入 默认高电平 上拉输入

    PWM_Init();
    imu_init();
    encoder_Init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}
