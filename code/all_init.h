/*
 * all_init.h
 *
 *  Created on: 2025��3��1��
 *      Author: Night
 */

#ifndef CODE_ALL_INIT_H_
#define CODE_ALL_INIT_H_
#include "motor.h"

#define PIT_60_0_PERIOD 50 // 50ms �����������
#define PIT_60_1_PERIOD 10 // 10ms imu
#define PIT_61_0_PERIOD 50 // 50ms ��ȡ����ֵ
//--------datatype declaration---------
#define LED1 (P33_9) // ���������
#define SWITCH2 (P33_12)
//---------function declaration----------
extern void all_init(void);


#endif /* CODE_ALL_INIT_H_ */
