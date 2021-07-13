/*
 * SCHEDULLER.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_SCHEDULLER_H_
#define INC_SCHEDULLER_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"

typedef struct Scheduler
{
    uint8_t  program;
    uint32_t target_time;
}sched_var;

#ifndef SCHEDULLER_c
sched_var array_sched_var[3];
#endif

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos);

/*
#ifndef SCHEDULLER_c
#define SCOPE   extern

SCOPE sched_var array_sched_var[3];

SCOPE void Task_Fast(void);
SCOPE void Task_Medium(void);
SCOPE void Task_Slow(void);

SCOPE void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos);

#else
#define SCOPE
#endif

SCOPE sched_var array_sched_var[3];

SCOPE void Task_Fast(void);
SCOPE void Task_Medium(void);
SCOPE void Task_Slow(void);

SCOPE void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos);

#undef SCOPE
*/
#endif /* INC_SCHEDULLER_H_ */

