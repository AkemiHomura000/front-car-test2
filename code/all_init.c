/*
 * all_init.c
 *
 *  Created on: 2025��3��1��
 *      Author: Night
 */
#include "zf_common_headfile.h"
#include"all_init.h"
#include "image.h"

#define LED1 (P33_9) //���������
PID_Datatypedef sptr_l,sptr_r;
void PWM_Init()
{
    //gpio_init(P21_2, GPO, 0, GPI_FLOATING_IN);//���ת���ʼ��
    pwm_init (ATOM0_CH2_P21_4, 20000, 1000);//���
//    pwm_init (ATOM0_CH3_P21_5, 17*1000, 0);//��תpwm
    //gpio_init(P21_4, GPO, 0, GPI_FLOATING_IN);
    pwm_init (ATOM0_CH0_P21_2, 20000, 1000);
//    pwm_init (ATOM0_CH1_P21_3, 17*1000, 0);//��תpwm
    PID_Init(&sptr_l);
    PID_Init(&sptr_r);
    sptr_l.P = 50;
    sptr_l.I = 0.002;
    sptr_l.D = 1.5;

    sptr_r.P = sptr_l.P;
    sptr_r.I = sptr_l.I;
    sptr_r.D = sptr_l.D;
}
void encoder_Init()
{
    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);//���
    encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);//�ұ�
}
void imu_init()
{
     while (1)
    {
        if (imu660ra_init())
            printf("\r\n IMU660RA init error."); // IMU660RA ��ʼ��ʧ��
        else
            break;
        gpio_toggle_level(LED1); // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
    }
}
void all_init()
{
    PWM_Init();
    imu_init();
    encoder_Init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}
