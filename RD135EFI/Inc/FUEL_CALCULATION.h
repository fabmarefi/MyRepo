/*
 * FUEL_CALCULATION.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_FUEL_CALCULATION_H_
#define INC_FUEL_CALCULATION_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"
#include "SENSORS.h"
#include "IO_CONTROL.h"
#include "TOOLS.h"
#include "TPS_TREATMENT.h"
#include "LAMBDA_CONTROL.h"

#define InjectorMaxTime              800   //80% time
#define maxPedalOnCrank               70

void InjectorDeadTimeCalc(void);
void FuelCalc(void);

#endif /* INC_FUEL_CALCULATION_H_ */
