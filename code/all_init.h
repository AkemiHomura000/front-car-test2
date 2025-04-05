/*
 * all_init.h
 *
 *  Created on: 2025年3月1日
 *      Author: Night
 */

#ifndef CODE_ALL_INIT_H_
#define CODE_ALL_INIT_H_
#include "motor.h"
//--------datatype declaration---------
#define LED1 (P33_9) //驱动红外灯

//---------function declaration----------
extern void all_init(void);

//---------data declaration------------
extern PID_Datatypedef sptr_l,sptr_r;

#endif /* CODE_ALL_INIT_H_ */
