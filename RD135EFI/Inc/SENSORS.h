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
		uint16_t VBatRaw;                      
    uint16_t VBatFilt;                     
    uint8_t  VBat;                         

    uint16_t VLambdaRaw;                   
    uint16_t VLambdaFilt;                  
    uint8_t  Lambda;                       

    uint16_t TempBoardRaw;                 
    uint16_t TempBoardFilt;                
    uint8_t  TempBoard;                    

		uint16_t TPSRaw;                       
    uint16_t TPSFilt;                      
    uint8_t  TPS;                          

    uint16_t PMapRaw;                      
    uint16_t PMapFilt;                     
    uint8_t  PMap;                         

    uint16_t TairRaw;                      
    uint16_t TairFilt;                     
    uint8_t  Tair;                         

    uint16_t EngineTempRaw;                
    uint16_t EngineTempFilt;               
    uint8_t  EngineTemp;                   
}sensors_measur;

extern volatile sensors_measur sensors;	
extern uint16_t adcArray[7];

extern void Read_Analog_Sensors(void);
extern void LearnTPSLimits(void);
extern void TPSLinearization(void);
extern void TairLinearization(void);
extern void MAPLinearization(void);
extern void EngineTempLinearization(void);
extern void VBatLinearization(void);
extern void LambdaLinearization(void);
extern void Board_Temp(void);

#endif /* INC_SENSORS_H_ */
