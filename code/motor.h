/*
 * motor.h
 *
 *  Created on: 2025Äê3ÔÂ1ÈÕ
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

}PID_Datatypedef;
//---------function declaration----------
extern void encoder_Read(void);
extern void imu_Read(void);
extern void motor_control(void);
extern int MotorPID_Output(PID_Datatypedef*sptr,int NowSpeed,int ExpectSpeed);
extern void PID_Init(PID_Datatypedef*sptr);

extern void print_angle(void);
//---------data declaration------------
extern int32 target_speed_l;
extern int32 target_speed_r;
extern int32 target_speed;
extern int32 speed_l;
extern int32 speed_r;
extern uint8 stop;
extern float angle_yaw;
#endif /* CODE_MOTOR_H_ */
