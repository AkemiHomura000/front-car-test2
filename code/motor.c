#include "zf_common_headfile.h"
#include "motor.h"
#include "all_init.h"
#include "image.h"

/* ---------------------------------- imu��� --------------------------------- */
float angle_yaw = 0;       // �Ƕ�
float zero_point = 0;      // imu���
int8 zero_point_count = 0; // imu������
float yaw_speed = 0;       // yaw���ٶ� ��/s
#define IMU_ZERO_COUNT 50  // imu������
/* ---------------------------------- ��������� --------------------------------- */
#define ENCODER_LINE_NUM 2340 //  ȷ������������
#define WHEEL_RADIUS 0.0325   // ȷ�� ���Ӱ뾶
int16 encoder_count_l = 0;    // ��������������
int16 encoder_count_r = 0;    // �ҵ������������
float encoder_distance = 0.0; // ��������¼��ʻ����
/* ---------------------------------- PID��� --------------------------------- */
#define dead_least_r 780
#define dead_least_l 600
int16 pwm_r = dead_least_r; // �ҵ��PWMֵ
int16 pwm_l = dead_least_l; // ����PWMֵ
bool stop = 0;
// �ٶȻ�
PID_Datatypedef sptr_line;
float debug_p = 10.0, debug_i = 5.0, debug_d = 3.0; // PID���Բ���
float speed_r;                                      // �ҵ���ٶ�
float speed_l;                                      // �����ٶ�
float line_speed = 0.0;                             // �������ٶ�
float target_speed = 100.0;
float debug_t_speed = 130.0;
#define PWM_PID_P 30.0
#define PWM_PID_I 2.0
#define PWM_PID_D 20.0
RUNNING_STATE running_state = START; // ����״̬
// ���ٶȻ�
PID_Datatypedef sptr_angular;
float debug_angular_p = 0.0, debug_angular_i = 0.0, debug_angular_d = 0.0; // PID���Բ���
float debug_angular_speed = 0.0;
float t_angular_speed = 0.0; // Ŀ����ٶ�
#define ANGULAR_PID_P 60.0
#define ANGULAR_PID_I 2.1
#define ANGULAR_PID_D 20.0
/* --------------------------------- ���������� --------------------------------- */
#define WHEEL_BASE 0.155 // ȷ�� �־�
/* ---------------------------------- Ԫ�ش��� ---------------------------------- */
CIRCLE_STATE circle_state = CIRCLE_NOT_FIND; // ����״̬
float start_distance = 0.0;                  // ��ʼ����
float start_angle = 0.0;                     // ��ʼ�Ƕ�
#define IN_CIRCLE_DISTANCE 0.2               // ���뻷���ľ���
#define IN_CIRCLE_ANGLE 45.0                 // ���뻷���ĽǶ�

        int error_image = 0; // ���
// ������ƺ���
void motor_control()
{
    // �ڴ˴����г�ʼ������all_init.c�о���ֻ����Ӳ����ʼ��
    static bool initialized = false; // ֻ�ڵ�һ�ν���ʱΪ false
    if (!initialized)
    {
        PID_Init(&sptr_line);
        PID_Init(&sptr_angular);
        // ���ٶȻ�
        sptr_line.P = PWM_PID_P;
        sptr_line.I = PWM_PID_I;
        sptr_line.D = PWM_PID_D;
        // ���ٶȻ�
        sptr_angular.P = ANGULAR_PID_P;
        sptr_angular.I = ANGULAR_PID_I;
        sptr_angular.D = ANGULAR_PID_D;
        initialized = true;
    }
    bool stop_now;
    // ��ȡ��λ������
    if (IfxCpu_acquireMutex(&param_mutex))
    {
        stop_now = stop;
        target_speed = debug_t_speed;
        // sptr_line.P = debug_angular_p;
        // sptr_line.I = debug_angular_i;
        // sptr_line.D = debug_angular_d;
        IfxCpu_releaseMutex(&param_mutex);
    }
    bool switch2 = gpio_get_level(SWITCH2); // ��ȡ����״̬
    if (!switch2)
    {
        int16 pwm_r_add = 0; // �ҵ��PWM����
        int16 pwm_l_add = 0; // ����PWM����


        if (IfxCpu_acquireMutex(&dspeed_mutex))
        {
            error_image = d_speed;
            IfxCpu_releaseMutex(&dspeed_mutex);
        }
        // ��ȡ��λ������
        if (IfxCpu_acquireMutex(&param_mutex))
        {
            t_angular_speed = debug_angular_speed;
            target_speed = debug_t_speed;
            IfxCpu_releaseMutex(&param_mutex);
        }
        angular_speed_control(error_image, yaw_speed); // ���ٶȻ�
        line_speed_control(target_speed, line_speed);      // ���ٶȻ�

        running_state_update(); // ��������״̬,���ƴ��㿪ʼ����ʱ�ı���ֵ����ֹ����
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

//        seekfree_assistant_oscilloscope_struct oscilloscope_data;
//        oscilloscope_data.channel_num = 8;
//        oscilloscope_data.data[0] = pwm_l;
//        oscilloscope_data.data[1] = pwm_r;
//        oscilloscope_data.data[2] = speed_l;
//        oscilloscope_data.data[3] = speed_r;
//        oscilloscope_data.data[4] = line_speed;
//        oscilloscope_data.data[5] = target_speed;
//        oscilloscope_data.data[6] = yaw_speed;
//        oscilloscope_data.data[7] = t_angular_speed;
//        seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    }
    else
    {
        pwm_l = 0;
        pwm_r = 0;
    }
    pwm_out_put();
}
void line_speed_control(float target_line_speed, float current_line_speed)
{
    // �ٶȻ�
    int16 pwm_line_add = MotorPID_Output(&sptr_line, current_line_speed, target_line_speed);
    // ����PWMֵ
    pwm_l += pwm_line_add;
    pwm_r += pwm_line_add;
}
void angular_speed_control(float target_angular_speed, float current_angular_speed)
{
    // ���ٶȻ�
    int16 pwm_angular_add = MotorPID_Output(&sptr_angular, current_angular_speed, target_angular_speed );
    // ����PWMֵ
    pwm_l -= pwm_angular_add;
    pwm_r += pwm_angular_add;
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
    line_speed = (speed_l + speed_r) / 2;                                                                              // �������ٶ�
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
    yaw_speed = data; // ���ٶ�
    angle_yaw += data * 0.001 * PIT_60_1_PERIOD;
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
     switch (circle_state)
     {
         case CIRCLE_NOT_FIND:
         {
             if (left_ctn&&circle_flag) // todo find circle
             {

                 start_angle = angle_yaw;
                 if (IfxCpu_acquireMutex(&dspeed_mutex))
                     {
                         d_speed = 0;
                         IfxCpu_releaseMutex(&dspeed_mutex);
                     }
                     system_delay_ms(550);
                 circle_state = CIRCLE_FIND;
             }
//             else if (!left_ctn && circle_flag)
//             {
//                 start_angle = angle_yaw;
//                 if (IfxCpu_acquireMutex(&dspeed_mutex))
//                 {
//                      d_speed = 0;
//                      IfxCpu_releaseMutex(&dspeed_mutex);
//                  }
//                  system_delay_ms(500);
//                  circle_state = CROSS_FIND;
//             }
             else
                 error_calculate();
         }
             break;
         case CIRCLE_FIND:
         {
             if (IfxCpu_acquireMutex(&dspeed_mutex))
            {
                d_speed = -150;
                IfxCpu_releaseMutex(&dspeed_mutex);
            }
             system_delay_ms(500);
             circle_state = CIRCLE_IN;
         }
         break;
         case CIRCLE_IN:
         {
             error_calculate();
                if ((angle_yaw < (start_angle+43)) && (angle_yaw > (start_angle+48)))
                {
                    start_distance = encoder_distance;
                    circle_state = CIRCLE_OUT;
                }
         }
         break;
         case CIRCLE_END:
         {
             float angle = angle_yaw - start_angle;
             if (IfxCpu_acquireMutex(&dspeed_mutex))
            {
                d_speed = 1.5*angle;
                IfxCpu_releaseMutex(&dspeed_mutex);
            }
             if (encoder_distance - start_distance > 100)
                 circle_state = CIRCLE_OUT;
         }
         break;
         case CIRCLE_OUT:
         {
             if (IfxCpu_acquireMutex(&dspeed_mutex))
             {
                    d_speed = -150;
                    IfxCpu_releaseMutex(&dspeed_mutex);
                         }
             if ((angle_yaw < (start_angle+3)) && (angle_yaw > (start_angle-3)))
            {
                    circle_state = CIRCLE_END;
            }

         }
             break;
         case CROSS_FIND:
         {
             error_calculate();
             if ((angle_yaw < ((int)start_angle + 105) % 180) && (angle_yaw > ((int)start_angle + 95) % 180))
            {
                 start_distance = encoder_distance;
                 circle_state = CROSS_OUT;
              }
          }
           break;
         case CROSS_OUT:
         {
             float angle = angle_yaw - start_angle;
             if (IfxCpu_acquireMutex(&dspeed_mutex))
              {
                   d_speed = 1.5*angle;
                   IfxCpu_releaseMutex(&dspeed_mutex);
               }
             if (encoder_distance - start_distance > 20)
                 circle_state = CIRCLE_NOT_FIND;
         }
         break;
     }
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
