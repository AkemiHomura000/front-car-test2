#include "zf_common_headfile.h"
#include "motor.h"
#include "all_init.h"
#include "image.h"

/* ---------------------------------- imu��� --------------------------------- */
float angle_yaw = 0;       // �Ƕ�
float zero_point = 0;      // imu���
int8 zero_point_count = 0; // imu������
#define IMU_ZERO_COUNT 50  // imu������
/* ---------------------------------- ��������� --------------------------------- */
#define ENCODER_LINE_NUM 2340 //  ȷ������������
#define WHEEL_RADIUS 0.0325   // ȷ�� ���Ӱ뾶
int16 encoder_count_l = 0;    // ��������������
int16 encoder_count_r = 0;    // �ҵ������������
float encoder_distance = 0.0; // ��������¼��ʻ����
/* ---------------------------------- PID��� --------------------------------- */
PID_Datatypedef sptr_l, sptr_r;
float debug_p = 10.0, debug_i = 5.0, debug_d = 3.0; // PID���Բ���
#define PWM_PID_P 10.0
#define PWM_PID_I 5.0
#define PWM_PID_D 3.0
RUNNING_STATE running_state = START; // ����״̬
/* --------------------------------- ���������� --------------------------------- */
#define WHEEL_BASE 0.155 // ȷ�� �־�
#define dead_least_r 780
#define dead_least_l 600
/* ---------------------------------- Ԫ�ش��� ---------------------------------- */
CIRCLE_STATE circle_state = CIRCLE_NOT_FIND; // ����״̬
float start_distance = 0.0;                  // ��ʼ����
float start_angle = 0.0;                     // ��ʼ�Ƕ�
#define IN_CIRCLE_DISTANCE 0.2               // ���뻷���ľ���
#define IN_CIRCLE_ANGLE 45.0                 // ���뻷���ĽǶ�
// ȫ�ֱ�������
int16 pwm_r = dead_least_r; // �ҵ��PWMֵ
int16 pwm_l = dead_least_l; // ����PWMֵ
float speed_r;              // �ҵ���ٶ�
float speed_l;              // �����ٶ�
float target_speed = 70.0;
float debug_t_speed = 70.0;
bool stop = 0;
float debug_diff_speed = 0.0;

// ������ƺ���
void motor_control()
{
    // �ڴ˴����г�ʼ������all_init.c�о���ֻ����Ӳ����ʼ��
    static bool initialized = false; // ֻ�ڵ�һ�ν���ʱΪ false
    if (!initialized)
    {
        PID_Init(&sptr_l);
        PID_Init(&sptr_r);
        sptr_l.P = PWM_PID_P;
        sptr_l.I = PWM_PID_I;
        sptr_l.D = PWM_PID_D;
        sptr_r.P = sptr_l.P;
        sptr_r.I = sptr_l.I;
        sptr_r.D = sptr_l.D;
        initialized = true;
    }
    bool stop_now;
    if (IfxCpu_acquireMutex(&param_mutex))
    {
        stop_now = stop;
        target_speed = debug_t_speed;
        sptr_l.P = debug_p;
        sptr_l.I = debug_i;
        sptr_l.D = debug_d;
        sptr_r.P = sptr_l.P;
        sptr_r.I = sptr_l.I;
        sptr_r.D = sptr_l.D;
        IfxCpu_releaseMutex(&param_mutex);
    }
    bool switch2 = gpio_get_level(SWITCH2); // ��ȡ����״̬
    if (!switch2)
    {
        int16 pwm_r_add = 0; // �ҵ��PWM����
        int16 pwm_l_add = 0; // ����PWM����

        // todo ����error����ת��뾶
        int image_error = 0;
        if (IfxCpu_acquireMutex(&dspeed_mutex))
        {
            image_error = d_speed;
            IfxCpu_releaseMutex(&dspeed_mutex);
        }
        // float turn_radius = turn_radius_caculate(image_error); // ת��뾶
        // float diff_speed = diff_speed_caculate(turn_radius)*2 ;   // ���ֲ��٣�
        // if (IfxCpu_acquireMutex(&param_mutex))
        // {
        //     // diff_speed = debug_diff_speed;
        //     // turn_radius = debug_diff_speed;
        //     // diff_speed = diff_speed_caculate(turn_radius);
        //     // diff_speed = 0.0;
        //     IfxCpu_releaseMutex(&param_mutex);
        // }
        float diff_speed = image_error * 0.5;
        pwm_l_add = MotorPID_Output(&sptr_l, speed_l, target_speed - diff_speed);
        pwm_r_add = MotorPID_Output(&sptr_r, speed_r, target_speed + diff_speed);
        // ����PWMֵ
        pwm_l += pwm_l_add;
        pwm_r += pwm_r_add;

        running_state_update(); // ��������״̬
        if (running_state == START)
        {
            // ����PWMֵ�ķ�Χ
            pwm_l = limit_a_b(pwm_l, -2100, 2100);
            pwm_r = limit_a_b(pwm_r, -2100, 2100);
        }
        else
        {
            // ����PWMֵ�ķ�Χ
            pwm_l = limit_a_b(pwm_l, -3000, 3000);
            pwm_r = limit_a_b(pwm_r, -3000, 3000);
        }

        seekfree_assistant_oscilloscope_struct oscilloscope_data;
        oscilloscope_data.channel_num = 7;
        oscilloscope_data.data[0] = pwm_l;
        oscilloscope_data.data[1] = pwm_r;
        oscilloscope_data.data[2] = speed_l;
        oscilloscope_data.data[3] = speed_r;
        oscilloscope_data.data[4] = diff_speed;
        oscilloscope_data.data[5] = target_speed;
        oscilloscope_data.data[6] = image_error;
        seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    }
    else
    {
        pwm_l = 0;
        pwm_r = 0;
    }
    pwm_out_put();
}
float diff_speed_caculate(float turn_radius)
{
    // todo �������v1-v2
    float result = 0.0; // ���ֲ��٣�����Ϊ��������Ϊ��
    result = target_speed * WHEEL_BASE / turn_radius;
    return result / 2;
}
float turn_radius_caculate(float image_error)
{
    // todo ����ת��뾶
    float result = 0.0; // ת��뾶
    result = 28.0 / image_error;
    return result;
}
void pwm_out_put()
{
    // �����ҵ��PWM���
    if (pwm_r >= 0)
    {
        gpio_set_level(P21_3, GPIO_LOW);      // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH0_P21_2, pwm_r); // ����PWMռ�ձ�
    }
    else
    {
        gpio_set_level(P21_3, GPIO_HIGH);      // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH0_P21_2, -pwm_r); // ����PWMռ�ձ�
    }

    // ��������PWM���
    if (pwm_l >= 0)
    {
        gpio_set_level(P21_5, GPIO_LOW);      // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH2_P21_4, pwm_l); // ����PWMռ�ձ�
    }
    else
    {
        gpio_set_level(P21_5, GPIO_HIGH);      // ���õ������Ϊ��ת
        pwm_set_duty(ATOM0_CH2_P21_4, -pwm_l); // ����PWMռ�ձ�
    }
}
// ��������ȡ����
void encoder_Read()
{
    encoder_count_l = encoder_get_count(TIM2_ENCODER); // ��ȡ����������ֵ
    encoder_clear_count(TIM2_ENCODER);                 // �����������������
    // printf("encoder_count_l:%d\r\n", encoder_count_l);
    encoder_count_r = -encoder_get_count(TIM4_ENCODER); // ��ȡ�ҵ��������ֵ������
    encoder_clear_count(TIM4_ENCODER);                  // ����ҵ������������
    // ��������
    speed_l = 100.0 * (float)encoder_count_l / ENCODER_LINE_NUM * 2 * 3.14 / (PIT_60_0_PERIOD * 0.001) * WHEEL_RADIUS; // �����ٶ�
    speed_r = 100.0 * (float)encoder_count_r / ENCODER_LINE_NUM * 2 * 3.14 / (PIT_60_0_PERIOD * 0.001) * WHEEL_RADIUS; // �����ٶ�
    // ������ʻ����
    encoder_distance += (speed_l + speed_r) / 2 * (PIT_60_0_PERIOD * 0.001); // ƽ���ٶ�
}
void imu_Read()
{
    imu660ra_get_acc();  // ��ȡ IMU660RA �ļ��ٶȲ�����ֵ
    imu660ra_get_gyro(); // ��ȡ IMU660RA �Ľ��ٶȲ�����ֵ
    float data = imu660ra_gyro_transition(imu660ra_gyro_z);
    if (zero_point_count < IMU_ZERO_COUNT)
    {
        zero_point += data;
        zero_point_count++;
    }
    else if (zero_point_count == IMU_ZERO_COUNT)
    {
        zero_point /= IMU_ZERO_COUNT;
        zero_point_count++;
    }
    else
    {
        data -= zero_point;
    }
    angle_yaw += data * 0.01;
    if (angle_yaw > 180)
        angle_yaw -= 360;
    else if (angle_yaw < -180)
        angle_yaw += 360;
    // printf("angle_yaw:%f\r\n",angle_yaw);
}

void print_angle()
{
    printf("angle_yaw:%f\r\n", angle_yaw);
}
// PID��ʼ������
void PID_Init(PID_Datatypedef *sptr)
{
    sptr->P = 0;         // ��ʼ������ϵ��
    sptr->I = 0;         // ��ʼ������ϵ��
    sptr->D = 0;         // ��ʼ��΢��ϵ��
    sptr->LastError = 0; // ��ʼ����һ�����
    sptr->PrevError = 0; // ��ʼ�����ϴ����
}

float MotorPID_Output(PID_Datatypedef *sptr, float NowSpeed, float ExpectSpeed)
{
    float Increase; // PID �������
    float iError;   // ��ǰ���
    iError = ExpectSpeed - NowSpeed;
    Increase = sptr->P * (iError - sptr->LastError) + sptr->I * iError + sptr->D * (iError - 2.0f * sptr->LastError + sptr->PrevError);

    // ���������ʷ
    sptr->PrevError = sptr->LastError;
    sptr->LastError = iError;

    return Increase;
}

void update_status(void) // ���³��뻷״̬��
{
    // switch (circle_state)
    // {
    // case CIRCLE_NOT_FIND:
    //     if () // todo find circle
    //     {
    //         start_distance = encoder_distance;
    //         start_angle = angle_yaw;
    //         circle_state = CIRCLE_FIND;
    //     }
    //     break;
    // case CIRCLE_FIND:
    // {
    //     float distance = encoder_distance - start_distance;
    //     float angle = angle_yaw - start_angle;
    //     // todo ������̬
    //     if (distance > IN_CIRCLE_DISTANCE && angle > IN_CIRCLE_ANGLE)
    //     {
    //     }
    // }
    // break;
    // case CIRCLE_IN:
    // {
    //     float angle = angle_yaw - start_angle;
    //     if (angle > 180) // todo ������̬
    // }
    // break;
    // case CIRCLE_OUT:
    //     circle_state = CIRCLE_NOT_FIND;
    //     break;
    // }
}

float get_distance(void) // ��ȡ����
{
    return encoder_distance;
}

float get_angle(void) // ��ȡ�Ƕ�
{
    return angle_yaw;
}

void running_state_update(void)
{
    if (running_state == START)
    {
        if (speed_l > 0.8 * target_speed || speed_r > 0.8 * target_speed)
        {
            running_state = NORMAL;
        }
    }
}
