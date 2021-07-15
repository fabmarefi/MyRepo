/*
 * SCHEDULLER.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

//#define SCHEDULLER_c

#include "SCHEDULLER.h"

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos)
{
    volatile uint32_t counter;

    counter = HAL_GetTick();

    if(var[pos].program == FALSE)
    {
		var[pos].target_time = counter+period;
        var[pos].program = TRUE;
    }

    if(counter>=var[pos].target_time)
    {
        var[pos].program = FALSE;
        (*func)();
    }
}
