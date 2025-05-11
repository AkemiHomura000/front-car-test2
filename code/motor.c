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
int16 speed_r;  // �ҵ���ٶ�
int16 speed_l;  // �����ٶ�
int dspeed_here = 0;
int16 target_speed = 80;
int16 t_speed = 80;
bool stop = 0;

float angle_yaw=0; // �Ƕ�
float zero_point = 0; // imu���
int8 zero_point_count = 0; // imu������
#define IMU_ZERO_COUNT 50 // imu������
// ������ƺ���
void motor_control() {
    bool stop_now;
    if(IfxCpu_acquireMutex(&param_mutex))
    {
        stop_now = stop;
        target_speed = t_speed;
        IfxCpu_releaseMutex(&param_mutex);
    }
    if(!stop_now)
    {
        int16 pwm_r_add = 0;  // �ҵ��PWM����
        int16 pwm_l_add = 0;  // ����PWM����
        float level = 1.0;

//        if (error >= 20 || error <= -20)
//        {
//            target_speed = 80;
//            level = 1.15;
//        }
//        else
//        {
//            target_speed = 120;
//            level = 1.0;
//        }
        if(IfxCpu_acquireMutex(&dspeed_mutex))
        {
            dspeed_here = d_speed;
            IfxCpu_releaseMutex(&dspeed_mutex);
        }
            // �����ٶ�������PWMֵ//level * 0.13
        pwm_l_add = MotorPID_Output(&sptr_l, speed_l, (int16)(level * (float)target_speed - (float)dspeed_here));
        pwm_r_add = MotorPID_Output(&sptr_r, speed_r, (int16)(level * (float)target_speed + (float)dspeed_here));

            // ����PWMֵ
        pwm_l += pwm_l_add;
        pwm_r += pwm_r_add;

            // ����PWMֵ�ķ�Χ
        pwm_l = limit_a_b(pwm_l, -6000, 6000);
        pwm_r = limit_a_b(pwm_r, -6000, 6000);
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
    speed_l = encoder_get_count(TIM2_ENCODER);  // ��ȡ����������ֵ
    encoder_clear_count(TIM2_ENCODER);  // �����������������
    speed_r = -encoder_get_count(TIM4_ENCODER);  // ��ȡ�ҵ��������ֵ������
    encoder_clear_count(TIM4_ENCODER);  // ����ҵ������������
}
void imu_Read() {
    imu660ra_get_acc();  // ��ȡ IMU660RA �ļ��ٶȲ�����ֵ
    imu660ra_get_gyro(); // ��ȡ IMU660RA �Ľ��ٶȲ�����ֵ
   float data = imu660ra_gyro_transition(imu660ra_gyro_z);
    if(zero_point_count<IMU_ZERO_COUNT)
    {
        zero_point += data;
        zero_point_count++;
    }
    else if(zero_point_count==IMU_ZERO_COUNT)
    {
        zero_point /= IMU_ZERO_COUNT;
        zero_point_count++;
    }
    else
    {
        data -= zero_point;
    }
    angle_yaw += data * 0.01;
    // printf("angle_yaw:%f\r\n",angle_yaw);
}

void print_angle()
{
    printf("angle_yaw:%f\r\n",angle_yaw);
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
