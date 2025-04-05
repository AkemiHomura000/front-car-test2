/*
 * motor.c
 *
 *  Created on: 2025��3��1��
 *      Author: Night
 */
//1111
#include "zf_common_headfile.h"
#include "motor.h"
#include "all_init.h"
#include "image.h"
#define dead_least_r 780
#define dead_least_l 600
// ȫ�ֱ�������
int16 pwm_r = dead_least_r;             // �ҵ��PWMֵ
int16 pwm_l = dead_least_l;             // ����PWMֵ
int32 speed_r;  // �ҵ���ٶ�
int32 speed_l;  // �����ٶ�
int32 target_speed = 50;
uint8 stop = 1;

// ������ƺ���
void motor_control() {
    if(1)
    {
        int16 pwm_r_add = 0;  // �ҵ��PWM����
        int16 pwm_l_add = 0;  // ����PWM����
            int level = 1;

            if (error >= 60 || error <= -60){
                    target_speed = 700;
                    level = 1.15;
                }
            else{
                target_speed = 900;
                level = 1;
            }

            // �����ٶ�������PWMֵ//level * 0.13
        pwm_l_add = MotorPID_Output(&sptr_l, speed_l, 0.13 * level * target_speed - d_speed);
        pwm_r_add = MotorPID_Output(&sptr_r, speed_r, 0.13 * level * target_speed + d_speed);

            // ����PWMֵ
        pwm_l += pwm_l_add;
        pwm_r += pwm_r_add;

            // ����PWMֵ�ķ�Χ
        pwm_l = limit_a_b(pwm_l, -6000, 6000);
        pwm_r = limit_a_b(pwm_r, -6000, 6000);
        if(speed_l == 0) pwm_l=1000;
        if(speed_r == 0) pwm_r=1300;
    }
    else
    {
        pwm_l=0;
        pwm_r=0;
    }

    // �����ҵ��PWM���
    if (pwm_r >= 0) {
        gpio_set_level(P21_3, GPIO_LOW);  // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH0_P21_2, pwm_r);  // ����PWMռ�ձ�
    }
    else {
        gpio_set_level(P21_3, GPIO_HIGH);  // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH0_P21_2, -pwm_r);  // ����PWMռ�ձ�
    }

    // ��������PWM���
    if (pwm_l >= 0) {
        gpio_set_level(P21_5, GPIO_LOW);  // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH2_P21_4, pwm_l);  // ����PWMռ�ձ�
    }
    else {
        gpio_set_level(P21_5, GPIO_HIGH);  // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH2_P21_4, -pwm_l);  // ����PWMռ�ձ�
    }
}

// ��������ȡ����
void encoder_Read() {
    speed_l = encoder_get_count(TIM5_ENCODER);  // ��ȡ����������ֵ
    encoder_clear_count(TIM5_ENCODER);  // �����������������
    speed_r = -encoder_get_count(TIM6_ENCODER);  // ��ȡ�ҵ��������ֵ������
    encoder_clear_count(TIM6_ENCODER);  // ����ҵ������������
}

// PID��ʼ������
void PID_Init(PID_Datatypedef* sptr) {
    sptr->P = 0;  // ��ʼ������ϵ��
    sptr->I = 0;  // ��ʼ������ϵ��
    sptr->D = 0;  // ��ʼ��΢��ϵ��
    sptr->LastError = 0;  // ��ʼ����һ�����
    sptr->PrevError = 0;  // ��ʼ�����ϴ����
}

// PID�����������
int MotorPID_Output(PID_Datatypedef* sptr, int NowSpeed, int ExpectSpeed) {
    int Increase;  // PID�������
    float iError;    // ��ǰ���

    iError = (float)(ExpectSpeed - NowSpeed); // ���㵱ǰ���
    Increase = (int)(sptr->P * (iError - sptr->LastError)
            + sptr->I * iError
            + sptr->D * (iError - 2 * sptr->LastError + sptr->PrevError));

    sptr->PrevError = sptr->LastError;  // �������ϴ����
    sptr->LastError = iError;  // ������һ�����

    return Increase;  // ����PID�������
}
