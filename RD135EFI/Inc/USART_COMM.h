/*
 * USART_COMM.h
 *
 *  Created on: 31-Aug-2021
 *      Author: Jerena
 */

#ifndef INC_USART_COMM_H_
#define INC_USART_COMM_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"

//typedef struct
//{
//    uint8_t  sensorAngDisplecement;
//    uint16_t Max_Engine_Speed;
//    uint16_t BP_Engine_Speed[12];
//    uint8_t  BP_Timing_Advance[12];
//    uint8_t  alpha;
//    uint8_t  beta;
//    uint8_t  gamma;
//}dataCalibration;

//#define blockSize sizeof (dataCalibration)

//typedef union 
//{
//    dataCalibration Calibration_RAM;
//    uint32_t array_Calibration_RAM[blockSize>>2];   //Divided in 4 (32/4 = 8 byte)	  
//    uint8_t array_Calibration_RAM_UART[blockSize];
//}calibrationBlock;

//calibrationBlock calibFlashBlock;

//static const calibrationBlock Initial_Calibration = { 28, 7500,
//	                                            ////The first Engine Speed value in the array needs to be 1200 or greater than mandatory
//                                              { 1300, 2000, 2500, 3000, 3500, 4000, 4500, 7000, 8000, 9000,12000,15000},
//                                              //{  64,   64,   64,   64,   64,   64,   64,   64,   64,   64,    64,    64}, 90, 80, 10};
//                                              //{  48,   48,   48,   48,   48,   48,   48,   48,   48,   48,    48,    48}, 90, 80, 10};
//                                              //{  32,   32,   32,   32,   32,   32,   32,   32,   32,   32,    32,    32}, 90, 80, 10};
//                                              //{  16,   16,   16,   16,   16,   16,   16,   16,   16,   16,    16,    16}, 90, 80, 10};
//                                              //{   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0}, 90, 80, 10};
//                                              //{  64,   54,   44,   39,   36,   32,   32,   36,   40,   45,     55,     64}, 90, 80, 10};
//                                              {   64,   58,   48,   38,   25,   15,    0,    0,   40,   45,   55,   64}, 90, 80, 10};
//                                              //64 -> 18 degree, calib_table = 64-ang_obj+18 <-> ang_obj = 64-calib_table+				

//enum Transmission_Status {TRANSMITING, TRANSMISSION_DONE} transmstatus;
//enum Reception_Status {DATA_AVAILABLE_RX_BUFFER, RECEPTION_DONE} receptstatus;
//uint8_t flgTransmition = OFF;

////UART Communication
//uint8_t UART3_txBuffer[blockSize+2];
//uint8_t UART3_rxBuffer[blockSize+2];
//uint8_t UART3_rxBufferAlt[11];


#endif /* INC_USART_COMM_H_ */
