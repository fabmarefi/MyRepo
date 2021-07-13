/*
 * TIMER_FUNC.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */
#define TIMER_FUNC_c

#include "TIMER_FUNC.h"

void setTimeoutHookUp(timerSchedtype timer_list[],enum TimerID timer,uint32_t period,void (*func)(void))
{
		timer_list[timer].target_time=HAL_GetTick()+period;
	  timer_list[timer].func_pointer=*func;
	  timer_list[timer].output=FALSE;
}

uint8_t checkTimeoutHookUp(timerSchedtype timer_list[],enum TimerID timer)
{
    uint8_t TempResp=FALSE;

    if(timer_list[timer].output==TRUE)
    {
        timer_list[timer].output=FALSE;
        timer_list[timer].target_time=0;
        timer_list[timer].func_pointer=0;
        TempResp=TRUE;
    }

    return (TempResp);
}

void TimerListManagement(timerSchedtype timer_list[])
{
    uint8_t line;

    for(line=0;line<nTimer;line++)
    {
        if(timer_list[line].target_time!=0)
        {
            if(HAL_GetTick()>=timer_list[line].target_time)
            {
								timer_list[line].target_time=0;
                timer_list[line].output=TRUE;
                timer_list[line].func_pointer();
            }
        }
    }
}

uint8_t Timeout_ms(uint8_t Condition,uint32_t *timer,uint32_t period)
{
    uint8_t TimeoutResp=FALSE;

    if(Condition==TRUE)
    {
        if(*timer==0)
        {
            *timer=HAL_GetTick()+period;
        }

        if(HAL_GetTick()>=*timer)
        {
            TimeoutResp=TRUE;
        }
    }
    else
    {
				*timer=0;
    }

    return (TimeoutResp);
}
