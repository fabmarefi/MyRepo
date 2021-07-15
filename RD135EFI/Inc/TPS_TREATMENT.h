/*
 * TPS_TREATMENT.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_TPS_TREATMENT_H_
#define INC_TPS_TREATMENT_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"
#include "IO_CONTROL.h"
#include "SENSORS.h"
#include "TIMER_FUNC.h"
#include "IDLE_CONTROL.h"

#define tps_min                       30
#define tps_max                      190
#define quickCmdPos                   30
#define quickCmdNeg                  -30
#define tfastenrich                 1000
#define tfastenleanment             1000
#define tps_min_cutoff                15
#define TimeToGetInCutoff           1000

extern uint8_t Cond5,Cond6;
extern uint32_t Counter5,Counter6,Counter7;

extern void TPS_Treatment(void);
uint8_t funcfastEnrichment(uint8_t TPS);
uint8_t funcfastEnleanment(int8_t TPS);

#endif /* INC_TPS_TREATMENT_H_ */
