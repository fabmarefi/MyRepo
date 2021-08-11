/*
 * LAMBDA_CONTROL.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_LAMBDA_CONTROL_H_
#define INC_LAMBDA_CONTROL_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"
#include "TIMER_FUNC.h"
#include "SENSORS.h"

#define increment                      1
#define decrement                      1
#define Lambda_Stoichiometric        100
#define lambdaVoltLeanTheshould       30
#define lambdaVoltRichTheshould       70

extern uint8_t Cond5,Cond6;

void LambdaCorrectionFunc(uint8_t lambdaRequested,uint8_t lambdaMeasured);

#endif /* INC_LAMBDA_CONTROL_H_ */
