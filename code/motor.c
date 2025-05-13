#include "zf_common_headfile.h"
#include "motor.h"
#include "all_init.h"
#include "image.h"

/* ---------------------------------- imu相关 --------------------------------- */
float angle_yaw = 0;       // 角度
float zero_point = 0;      // imu零点
int8 zero_point_count = 0; // imu零点计数
#define IMU_ZERO_COUNT 50  // imu零点计数
/* ---------------------------------- 编码器相关 --------------------------------- */
#define ENCODER_LINE_NUM 2340 //  确定编码器线数
#define WHEEL_RADIUS 0.0325   // 确定 轮子半径
int16 encoder_count_l = 0;    // 左电机编码器计数
int16 encoder_count_r = 0;    // 右电机编码器计数
float encoder_distance = 0.0; // 编码器记录行驶距离
/* ---------------------------------- PID相关 --------------------------------- */
PID_Datatypedef sptr_l, sptr_r;
float debug_p = 10.0, debug_i = 5.0, debug_d = 3.0; // PID调试参数
#define PWM_PID_P 10.0
#define PWM_PID_I 5.0
#define PWM_PID_D 3.0
RUNNING_STATE running_state = START; // 运行状态
/* --------------------------------- 电机控制相关 --------------------------------- */
#define WHEEL_BASE 0.155 // 确定 轮距
#define dead_least_r 780
#define dead_least_l 600
/* ---------------------------------- 元素处理 ---------------------------------- */
CIRCLE_STATE circle_state = CIRCLE_NOT_FIND; // 环岛状态
float start_distance = 0.0;                  // 起始距离
float start_angle = 0.0;                     // 起始角度
#define IN_CIRCLE_DISTANCE 0.2               // 进入环岛的距离
#define IN_CIRCLE_ANGLE 45.0                 // 进入环岛的角度
// 全局变量声明
int16 pwm_r = dead_least_r; // 右电机PWM值
int16 pwm_l = dead_least_l; // 左电机PWM值
float speed_r;              // 右电机速度
float speed_l;              // 左电机速度
float target_speed = 70.0;
float debug_t_speed = 70.0;
bool stop = 0;
float debug_diff_speed = 0.0;

// 电机控制函数
void motor_control()
{
    // 在此处进行初始化，在all_init.c中尽量只进行硬件初始化
    static bool initialized = false; // 只在第一次进入时为 false
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
    bool switch2 = gpio_get_level(SWITCH2); // 获取开关状态
    if (!switch2)
    {
        int16 pwm_r_add = 0; // 右电机PWM增量
        int16 pwm_l_add = 0; // 左电机PWM增量

        // todo 根据error计算转弯半径
        int image_error = 0;
        if (IfxCpu_acquireMutex(&dspeed_mutex))
        {
            image_error = d_speed;
            IfxCpu_releaseMutex(&dspeed_mutex);
        }
        // float turn_radius = turn_radius_caculate(image_error); // 转弯半径
        // float diff_speed = diff_speed_caculate(turn_radius)*2 ;   // 两轮差速，
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
        // 更新PWM值
        pwm_l += pwm_l_add;
        pwm_r += pwm_r_add;

        running_state_update(); // 更新运行状态
        if (running_state == START)
        {
            // 限制PWM值的范围
            pwm_l = limit_a_b(pwm_l, -2100, 2100);
            pwm_r = limit_a_b(pwm_r, -2100, 2100);
        }
        else
        {
            // 限制PWM值的范围
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
    // todo 计算差速v1-v2
    float result = 0.0; // 两轮差速，往左为正，往右为负
    result = target_speed * WHEEL_BASE / turn_radius;
    return result / 2;
}
float turn_radius_caculate(float image_error)
{
    // todo 计算转弯半径
    float result = 0.0; // 转弯半径
    result = 28.0 / image_error;
    return result;
}
void pwm_out_put()
{
    // 设置右电机PWM输出
    if (pwm_r >= 0)
    {
        gpio_set_level(P21_3, GPIO_LOW);      // 设置电机方向为正转
        pwm_set_duty(ATOM0_CH0_P21_2, pwm_r); // 设置PWM占空比
    }
    else
    {
        gpio_set_level(P21_3, GPIO_HIGH);      // 设置电机方向为反转
        pwm_set_duty(ATOM0_CH0_P21_2, -pwm_r); // 设置PWM占空比
    }

    // 设置左电机PWM输出
    if (pwm_l >= 0)
    {
        gpio_set_level(P21_5, GPIO_LOW);      // 设置电机方向为正转
        pwm_set_duty(ATOM0_CH2_P21_4, pwm_l); // 设置PWM占空比
    }
    else
    {
        gpio_set_level(P21_5, GPIO_HIGH);      // 设置电机方向为反转
        pwm_set_duty(ATOM0_CH2_P21_4, -pwm_l); // 设置PWM占空比
    }
}
// 编码器读取函数
void encoder_Read()
{
    encoder_count_l = encoder_get_count(TIM2_ENCODER); // 读取左电机编码器值
    encoder_clear_count(TIM2_ENCODER);                 // 清除左电机编码器计数
    // printf("encoder_count_l:%d\r\n", encoder_count_l);
    encoder_count_r = -encoder_get_count(TIM4_ENCODER); // 读取右电机编码器值（反向）
    encoder_clear_count(TIM4_ENCODER);                  // 清除右电机编码器计数
    // 计算轮速
    speed_l = 100.0 * (float)encoder_count_l / ENCODER_LINE_NUM * 2 * 3.14 / (PIT_60_0_PERIOD * 0.001) * WHEEL_RADIUS; // 左轮速度
    speed_r = 100.0 * (float)encoder_count_r / ENCODER_LINE_NUM * 2 * 3.14 / (PIT_60_0_PERIOD * 0.001) * WHEEL_RADIUS; // 右轮速度
    // 计算行驶距离
    encoder_distance += (speed_l + speed_r) / 2 * (PIT_60_0_PERIOD * 0.001); // 平均速度
}
void imu_Read()
{
    imu660ra_get_acc();  // 获取 IMU660RA 的加速度测量数值
    imu660ra_get_gyro(); // 获取 IMU660RA 的角速度测量数值
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
// PID初始化函数
void PID_Init(PID_Datatypedef *sptr)
{
    sptr->P = 0;         // 初始化比例系数
    sptr->I = 0;         // 初始化积分系数
    sptr->D = 0;         // 初始化微分系数
    sptr->LastError = 0; // 初始化上一次误差
    sptr->PrevError = 0; // 初始化上上次误差
}

float MotorPID_Output(PID_Datatypedef *sptr, float NowSpeed, float ExpectSpeed)
{
    float Increase; // PID 输出增量
    float iError;   // 当前误差
    iError = ExpectSpeed - NowSpeed;
    Increase = sptr->P * (iError - sptr->LastError) + sptr->I * iError + sptr->D * (iError - 2.0f * sptr->LastError + sptr->PrevError);

    // 更新误差历史
    sptr->PrevError = sptr->LastError;
    sptr->LastError = iError;

    return Increase;
}

void update_status(void) // 更新出入环状态机
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
    //     // todo 控制姿态
    //     if (distance > IN_CIRCLE_DISTANCE && angle > IN_CIRCLE_ANGLE)
    //     {
    //     }
    // }
    // break;
    // case CIRCLE_IN:
    // {
    //     float angle = angle_yaw - start_angle;
    //     if (angle > 180) // todo 控制姿态
    // }
    // break;
    // case CIRCLE_OUT:
    //     circle_state = CIRCLE_NOT_FIND;
    //     break;
    // }
}

float get_distance(void) // 获取距离
{
    return encoder_distance;
}

float get_angle(void) // 获取角度
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
