/*
 * VRS_TREATMENT.h
 *
 *  Created on: 24-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_VRS_TREATMENT_H_
#define INC_VRS_TREATMENT_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"
#include "TIMER_FUNC.h"

extern TIM_HandleTypeDef htim2;

void PulseDetection(void);
void Rising_Edge_Event(void);
void Falling_Edge_Event(void);

#endif /* INC_VRS_TREATMENT_H_ */
