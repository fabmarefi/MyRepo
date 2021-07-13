/*
 * SCHEDULLER.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_SCHEDULLER_H_
#define INC_SCHEDULLER_H_

#include "stm32f1xx_hal.h"

#define FALSE                          0
#define TRUE                           1

typedef struct Scheduler
{
    uint8_t  program;
    uint32_t target_time;
}sched_var;

void Task_Fast(void);
void Task_Medium(void);
void Task_Slow(void);

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos);

#endif /* INC_SCHEDULLER_H_ */
