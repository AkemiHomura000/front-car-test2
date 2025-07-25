/*
 * motor.h
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
//--------datatype declaration---------
typedef struct
{
        float P;
        float I;
        float D;

        float LastError;
        float PrevError;

} PID_Datatypedef;
typedef enum
{
        CIRCLE_NOT_FIND,
        LEFT_CIRCLE_IN,
        RIGHT_CIRCLE_IN,
        LEFT_CIRCLE_OUT,
        RIGHT_CIRCLE_OUT
} CIRCLE_STATE;
typedef enum
{
        START,   // 从零开始起步
        NORMAL   // 正常行驶
} RUNNING_STATE; // 运行状态
//---------function declaration----------
extern void encoder_Read(void);
extern void imu_Read(void);
extern void motor_control(void);
extern float MotorPID_Output(PID_Datatypedef *sptr, float NowSpeed, float ExpectSpeed);
extern void PID_Init(PID_Datatypedef *sptr);
extern void pwm_out_put();
extern void print_angle(void);
extern void update_status(void); // 更新出入环状态机
extern float get_distance(void); // 获取距离
extern float get_angle(void);    // 获取角度
extern void running_state_update(void); // 更新运行状态
extern void line_speed_control(float target_line_speed, float current_line_speed);
extern void angular_speed_control(float target_angular_speed, float current_angular_speed);
//---------data declaration------------
extern float target_speed_l;
extern float target_speed_r;
extern float target_speed;
extern float debug_t_speed;
extern float debug_angular_speed;
extern float debug_p, debug_i, debug_d;
extern float debug_angular_p, debug_angular_i, debug_angular_d;
extern float speed_l; // 单位 cm/s
extern float speed_r; // 单位 cm/s
extern bool stop;
extern float angle_yaw;
extern bool right_circle_find; // 是否找到右环岛
extern bool left_circle_find; // 是否找到右环岛
extern bool is_ready_to_turn_left; // 是否准备转向左环岛
extern bool is_ready_to_turn_right; // 是否准备转向右环岛
#endif /* CODE_MOTOR_H_ */
