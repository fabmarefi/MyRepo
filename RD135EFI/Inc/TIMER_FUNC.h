/*
 * TIMER_FUNC.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_TIMER_FUNC_H_
#define INC_TIMER_FUNC_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"

#define nTimer                        16

enum TimerID{Timer0,Timer1,Timer2,Timer3,Timer4,Timer5,Timer6,Timer7,Timer8,Timer9,Timer10,Timer11,Timer12,Timer13,Timer14,Timer15};// TimerNumb;

typedef struct TimerStruct
{
    uint32_t target_time;
    uint8_t  output;
		void (*func_pointer)();
}timerSchedtype;

#ifndef TIMER_FUNC_c
timerSchedtype timerList[nTimer];
uint8_t Cond0=0,Cond1=0,Cond2=0,Cond3=0,Cond4=0,Cond5=0,Cond6=0,Cond8=0;
uint32_t Counter0=0,Counter1=0,Counter2=0,Counter3=0,Counter4=0,Counter5=0,Counter6=0,Counter7=0,Counter8=0,Counter9=0,Counter10=0;
#endif

void setTimeoutHookUp(timerSchedtype timer_list[],enum TimerID timer,uint32_t period,void (*func)(void));
uint8_t checkTimeoutHookUp(timerSchedtype timer_list[],enum TimerID timer);
void TimerListManagement(timerSchedtype timer_list[]);
uint8_t Timeout_ms(uint8_t Condition,uint32_t *timer,uint32_t period);

#endif /* INC_TIMER_FUNC_H_ */
