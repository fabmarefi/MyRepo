/*
 * ENGINE_STATE.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_ENGINE_STATE_H_
#define INC_ENGINE_STATE_H_

#include "stm32f1xx_hal.h"
#include "TIMER_FUNC.h"
#include "IO_CONTROL.h"
#include "IDLE_CONTROL.h"

#define TMR2_16bits               65536u
#define EngineSpeedPeriod_Min    785455u     //100rpm
#define EngineSpeedPeriod_Max      5236u     //15000rpm
#define RPM_const              78545455u
#define accel_rate                   150
#define decel_rate                  -150
#define rpm_max                    11500
#define hyst                         300
#define rpm_stopped                  300

extern timerSchedtype timerList[nTimer];
extern uint8_t Cond0,Cond2,Cond3,Cond4,Cond5,Cond8;
extern uint32_t Counter0,Counter2,Counter3,Counter4,Counter8;

void EngineSpeedCalculation(void);
void Engine_STOP_test(void);
uint32_t PrimerPulse(uint8_t temp);
void AccelDer(void);
void Eng_State(void);

#endif /* INC_ENGINE_STATE_H_ */
