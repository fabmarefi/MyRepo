/*
 * GEN_DEF.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_GEN_DEF_H_
#define INC_GEN_DEF_H_

#include "stm32f1xx_hal.h"

#define FALSE                          0
#define TRUE                           1
#define OFF                            0
#define ON                             1
#define AFR_Bensine                  132
#define AFR_Ethanol                   90
#define R_div_Mair                    29

enum Accel{ACCEL,DECEL,STABLE};
enum EngineState{WAKEUP,PRIMERINJ,STOP,CRANK,STALL,IDLE,CRUISE,OVERSPEED};  
enum Lambda{RICH,LEAN,INACTIVE};

//Calibration values
typedef struct Calibration
{
	uint16_t Max_Engine_Speed;	
  uint16_t BP_Engine_Speed[8];
  uint8_t BP_Engine_Temperature[8];
	uint8_t BP_Delta_TPS[8];
	uint8_t BP_MAP[8];
	uint8_t BP_AirTemp[8];
	uint16_t TB_Cycles[8];
	uint16_t TB_InjectorDeadTime[8];
	uint8_t TB_WarmUp[8];
	uint8_t TB_Volef[8][8];
	uint8_t TB_LambdaTarget[8][8];
	uint8_t TB_Crank[8];
}struct_Calibration;   

typedef struct system_info
{
		enum EngineState Engine_State; 
		enum Accel Acceleration;          
		uint8_t  EnabLambdaCtrl;               //1    
    uint16_t Engine_Speed_old;             //0
    uint16_t Engine_Speed;                 //0
	  uint8_t  pulseDetected;                //0
    int16_t  deltaEngineSpeed;             //0
    uint16_t engineSpeedPred;              //0
    uint16_t engineSpeedFiltered;          //0
    uint16_t avarageEngineSpeed;           //0
    uint32_t Measured_Period;              //0

    uint32_t TDuty_Input_Signal;           //0
    uint32_t tdutyInputSignalPred;         //0
    uint32_t tdutyInputSignalPredLinear;   //0

    int8_t   deltaTPS;                     //0
    uint8_t  TPS_old;                      //0
	  uint16_t TPS_min;                      //150
	  uint16_t TPS_max;                      //0
	  uint8_t  TPS_learned;                  //0
    enum Lambda LambDir;                   //RICH
    uint8_t  Update_calc;                  //TRUE
    uint8_t  LambdaRequested;              //100
	  uint8_t  LambdaRequestedCrank;         //100

	  uint32_t Airmass;                      //0
    uint8_t  Voleff;                       //100
		uint8_t  VoleffCrank;                  //40
    uint8_t  Displacement;                 //135
    uint8_t  AFRstoich;                    //132
		uint16_t InjectorDeadTime;             //50  (1ms)
    uint32_t Injectormassflow;             //0
    uint32_t PW_us;                        //0
		uint32_t PW_percent;                   //0
    uint32_t Fuelmass;                     //0

    uint32_t nOverflow;                    //0
    uint8_t  EngineDiedCounter;            //0
    uint32_t counterCycles;                //0
    uint8_t  counterPos;                   //10
    uint8_t  counterNeg;                   //10
    uint32_t nOverflow_RE;                 //0
    uint32_t nOverflow_FE;                 //0
    uint32_t Rising_Edge_Counter;          //0

    uint8_t  cuttOffTerm;                  //1
    uint8_t  fastEnrichmentTerm;           //100
    uint8_t  warmUpTerm;                   //100
    uint8_t  overspeedTerm;                //100
    uint8_t  crankTerm;                    //100
    uint8_t  lambdaCorrectTerm;            //100
		uint16_t TotalTerm;                    //100    
}system_vars;

extern volatile struct_Calibration Calibration_RAM;
extern volatile system_vars scenario;
	
#endif /* INC_GEN_DEF_H_ */
