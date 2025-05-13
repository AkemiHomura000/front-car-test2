/*
 * all_init.h
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */

#ifndef CODE_ALL_INIT_H_
#define CODE_ALL_INIT_H_
#include "motor.h"

#define PIT_60_0_PERIOD 50 // 50ms 编码器、电机
#define PIT_60_1_PERIOD 10 // 10ms imu
#define PIT_61_0_PERIOD 50 // 50ms 读取调试值
//--------datatype declaration---------
#define LED1 (P33_9) // 驱动红外灯
#define SWITCH2 (P33_12)
//---------function declaration----------
extern void all_init(void);


#endif /* CODE_ALL_INIT_H_ */
