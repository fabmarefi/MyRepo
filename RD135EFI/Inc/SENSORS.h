/*
 * SENSORS.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stm32f1xx_hal.h"
#include "TOOLS.h"

#define V25                          180
#define Avg_Slope                      5

typedef struct sensors_measurement
{
		uint16_t VBatRaw;                      //0
    uint16_t VBatFilt;                     //0
    uint8_t  VBat;                         //0

    uint16_t VLambdaRaw;                   //0
    uint16_t VLambdaFilt;                  //0
    uint8_t  Lambda;                       //0

    uint16_t TempBoardRaw;                 //0
    uint16_t TempBoardFilt;                //0
    uint8_t  TempBoard;                    //0

		uint16_t TPSRaw;                       //0
    uint16_t TPSFilt;                      //0
    uint8_t  TPS;                          //0

    uint16_t PMapRaw;                      //0
    uint16_t PMapFilt;                     //0
    uint8_t  PMap;                         //0

    uint16_t TairRaw;                      //0
    uint16_t TairFilt;                     //0
    uint8_t  Tair;                         //0

    uint16_t EngineTempRaw;                //0
    uint16_t EngineTempFilt;               //0
    uint8_t  EngineTemp;                   //0
}sensors_measur;

extern volatile sensors_measur sensors;	

extern uint16_t adcArray[7];

extern void Read_Analog_Sensors(void);
extern void TPSLinearization(void);
extern void TairLinearization(void);
extern void MAPLinearization(void);
extern void EngineTempLinearization(void);
extern void VBatLinearization(void);
extern void LambdaLinearization(void);
extern void Board_Temp(void);

#endif /* INC_SENSORS_H_ */
