/*
 * motor.c
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */
//1111
#include "zf_common_headfile.h"
#include "motor.h"
#include "all_init.h"
#include "image.h"
#define dead_least_r 780
#define dead_least_l 600
// 全局变量声明
int16 pwm_r = dead_least_r;             // 右电机PWM值
int16 pwm_l = dead_least_l;             // 左电机PWM值
int32 speed_r;  // 右电机速度
int32 speed_l;  // 左电机速度
int32 target_speed = 50;
uint8 stop = 1;

// 电机控制函数
void motor_control() {
    if(1)
    {
        int16 pwm_r_add = 0;  // 右电机PWM增量
        int16 pwm_l_add = 0;  // 左电机PWM增量
            int level = 1;

            if (error >= 60 || error <= -60){
                    target_speed = 700;
                    level = 1.15;
                }
            else{
                target_speed = 900;
                level = 1;
            }

            // 根据速度误差调整PWM值//level * 0.13
        pwm_l_add = MotorPID_Output(&sptr_l, speed_l, 0.13 * level * target_speed - d_speed);
        pwm_r_add = MotorPID_Output(&sptr_r, speed_r, 0.13 * level * target_speed + d_speed);

            // 更新PWM值
        pwm_l += pwm_l_add;
        pwm_r += pwm_r_add;

            // 限制PWM值的范围
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

    // 设置右电机PWM输出
    if (pwm_r >= 0) {
        gpio_set_level(P21_3, GPIO_LOW);  // 设置电机方向为正转
        pwm_set_duty(ATOM0_CH0_P21_2, pwm_r);  // 设置PWM占空比
    }
    else {
        gpio_set_level(P21_3, GPIO_HIGH);  // 设置电机方向为反转
        pwm_set_duty(ATOM0_CH0_P21_2, -pwm_r);  // 设置PWM占空比
    }

    // 设置左电机PWM输出
    if (pwm_l >= 0) {
        gpio_set_level(P21_5, GPIO_LOW);  // 设置电机方向为正转
        pwm_set_duty(ATOM0_CH2_P21_4, pwm_l);  // 设置PWM占空比
    }
    else {
        gpio_set_level(P21_5, GPIO_HIGH);  // 设置电机方向为反转
        pwm_set_duty(ATOM0_CH2_P21_4, -pwm_l);  // 设置PWM占空比
    }
}

// 编码器读取函数
void encoder_Read() {
    speed_l = encoder_get_count(TIM5_ENCODER);  // 读取左电机编码器值
    encoder_clear_count(TIM5_ENCODER);  // 清除左电机编码器计数
    speed_r = -encoder_get_count(TIM6_ENCODER);  // 读取右电机编码器值（反向）
    encoder_clear_count(TIM6_ENCODER);  // 清除右电机编码器计数
}

// PID初始化函数
void PID_Init(PID_Datatypedef* sptr) {
    sptr->P = 0;  // 初始化比例系数
    sptr->I = 0;  // 初始化积分系数
    sptr->D = 0;  // 初始化微分系数
    sptr->LastError = 0;  // 初始化上一次误差
    sptr->PrevError = 0;  // 初始化上上次误差
}

// PID控制输出函数
int MotorPID_Output(PID_Datatypedef* sptr, int NowSpeed, int ExpectSpeed) {
    int Increase;  // PID输出增量
    float iError;    // 当前误差

    iError = (float)(ExpectSpeed - NowSpeed); // 计算当前误差
    Increase = (int)(sptr->P * (iError - sptr->LastError)
            + sptr->I * iError
            + sptr->D * (iError - 2 * sptr->LastError + sptr->PrevError));

    sptr->PrevError = sptr->LastError;  // 更新上上次误差
    sptr->LastError = iError;  // 更新上一次误差

    return Increase;  // 返回PID输出增量
}
